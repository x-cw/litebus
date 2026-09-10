//============================================================================
// Filename    : lb_tniu_ext_bca.v
// Author      : litebus
// Description : TNIU external half, bca form (Slave side, hardenable)
// Date        : 2026-08-06
// Revision    : 1.0 initial -- per-round changes are in doc/HISTORY.md
//----------------------------------------------------------------------------
// lb_tniu_ext_core wrapped in the async bridge:
//   - forward REQ_R / REQ_W bca_mst (clk_s read side), one bridge each
//   - reverse RSP_RD/RSP_WR bca_slv (clk_s write side)
// Only async bca border signals connect to the internal half lb_tniu_int_bca.
// Domain = Slave, and the bca write-side halves belong to it -- which is why
// the bridge lives inside this module rather than at the top: PD gets one block.
//
// Two forms exist and the MODULE NAME selects between them:
//   lb_tniu_ext_bca       this file -- halves in different clock domains, bca
//                     bridges them, border is b_* (4 wires per channel)
//   lb_tniu_ext_core  both halves on one clock, border is a plain valid-ready
//                     vr_* group, no bca at all
// Not a parameter: the two borders are different sets of PORTS, and a V2001
// port list is fixed at elaboration. Not a `define either -- that is global to
// the compilation unit, while one top can hold NIUs of both forms. Wiring the
// halves to mismatched forms is a COMPILE error, not a silent deadlock.
//
// Parameter naming convention (CODING_STYLE 1.4): EXT_* are external (Slave-IP
// side) interface widths, INT_* are in-network quantities; structural parameters
// (BCA_DEPTH, SYNC_STAGES) carry no prefix. This half holds no pipe and no
// context table, so it takes neither pipe enables nor EXT_PENDING_TRANS.
//============================================================================
`include "lb_defines.vh"

module lb_tniu_ext_bca #(
    parameter EXT_PROTOCOL         = 0,   // 0 = litebus (feed-through), 1 = AXI, 2 = APB
    parameter EXT_DATA_WIDTH       = 64,  // Slave-side data width
    parameter EXT_LEN_WIDTH        = 4,   // Slave-side len width (beat count - 1)
    parameter EXT_USER_WIDTH_CMD   = 8,   // CMD user sideband width
    parameter EXT_USER_WIDTH_RSP_RD    = 8,   // RSP_RD user sideband width
    parameter EXT_USER_WIDTH_RSP_WR    = 8,   // RSP_WR user sideband width
    parameter EXT_TXNID_WIDTH      = 3,   // Slave-side external transaction ID width
    parameter EXT_MOD_W            = 0,   // REQ_W atomic-modifier pin width, 0 = none (Adv 15)
    parameter EXT_QOS_W            = 0,   // REQ qos pin width, 0 = no pin (rides inside the CMD payload)
    parameter INT_ADDR_LOCAL_WIDTH = 32,  // in-network local (rebased) address width
    parameter BCA_DEPTH            = 8,   // bca async FIFO depth (power of 2)
    parameter SYNC_STAGES          = 2,   // bca gray-code synchronizer stages (>=2)
    // ---- derived parameters; used in the port list, so they must stay in the
    // ---- parameter list (V2001 body localparams cannot size ports). Not to be
    // ---- overridden externally -- the defaults are the correct values.
    parameter EXT_STRB_WIDTH = EXT_DATA_WIDTH/8,
    // qos (if any) leads the CMD payload; it is opaque to this half throughout.
    parameter EXT_CMD_W  = EXT_QOS_W + `LB_OPCODE_WIDTH + INT_ADDR_LOCAL_WIDTH + EXT_LEN_WIDTH +
                           EXT_TXNID_WIDTH + EXT_USER_WIDTH_CMD,
    parameter EXT_WD_W   = EXT_DATA_WIDTH + EXT_STRB_WIDTH + 1 + EXT_TXNID_WIDTH,
    // external REQ = {cmd field (high) | modifier (Adv 15, often absent) | wd field (low)}
    parameter EXT_REQ_W  = EXT_CMD_W + EXT_MOD_W + EXT_WD_W,
    parameter EXT_RSP_RD_W   = EXT_TXNID_WIDTH + `LB_RESP_WIDTH + EXT_USER_WIDTH_RSP_RD + 1 + EXT_DATA_WIDTH,
    parameter EXT_RSP_WR_W   = EXT_TXNID_WIDTH + `LB_RESP_WIDTH + EXT_USER_WIDTH_RSP_WR,
    // user pin widths floored at 1, same reason as in lb_tniu_ext_core: a width-0
    // user cannot delete a V2001 port, so the stub stays and the top omits the pin.
    parameter EXT_USER_RSP_RD_PW = (EXT_USER_WIDTH_RSP_RD < 1) ? 1 : EXT_USER_WIDTH_RSP_RD,
    parameter EXT_USER_RSP_WR_PW = (EXT_USER_WIDTH_RSP_WR < 1) ? 1 : EXT_USER_WIDTH_RSP_WR,
    parameter AW_BCA     = (BCA_DEPTH <= 2)   ? 1 :   // bca pointer width = log2(BCA_DEPTH)
                           (BCA_DEPTH <= 4)   ? 2 :
                           (BCA_DEPTH <= 8)   ? 3 :
                           (BCA_DEPTH <= 16)  ? 4 :
                           (BCA_DEPTH <= 32)  ? 5 : 6
) (
    //--------- inputs: clock / reset (Slave clock domain) ---------
    input wire                             clk_s,            // Slave-side clock
    input wire                             srst_n,           // Slave-side async reset, active low
    //--------- inputs: external interface (Slave side, Valid-Ready) ---------
    input wire                             req_r_ready,      // REQ_R ready (backpressure from Slave)
    input wire                             req_w_ready,      // REQ_W ready (backpressure from Slave)
    input wire  [EXT_DATA_WIDTH-1:0]       rsp_rd_data,      // RSP_RD data from Slave
    input wire                             rsp_rd_last,      // RSP_RD last beat
    input wire  [`LB_RESP_WIDTH-1:0]       rsp_rd_resp,      // RSP_RD response code
    input wire  [EXT_TXNID_WIDTH-1:0]      rsp_rd_ext_txnid, // RSP_RD Slave-side transaction ID
    input wire  [EXT_USER_RSP_RD_PW-1:0]       rsp_rd_user,  // RSP_RD user sideband (tied off at width 0)
    input wire                             rsp_rd_valid,     // RSP_RD valid
    input wire  [`LB_RESP_WIDTH-1:0]       rsp_wr_resp,      // RSP_WR response code
    input wire  [EXT_TXNID_WIDTH-1:0]      rsp_wr_ext_txnid, // RSP_WR Slave-side transaction ID
    input wire  [EXT_USER_RSP_WR_PW-1:0]       rsp_wr_user,  // RSP_WR user sideband (tied off at width 0)
    input wire                             rsp_wr_valid,     // RSP_WR valid
    //--------- outputs: external interface (Slave side, Valid-Ready) ---------
    // Two request channels to the Slave: REQ_R is a bare CMD, REQ_W is
    // {cmd field | wd field} on one beat.
    output wire [EXT_CMD_W-1:0]            req_r_data,       // REQ_R packed payload (CMD only)
    output wire                            req_r_valid,      // REQ_R valid
    output wire [EXT_REQ_W-1:0]            req_w_data,       // REQ_W packed payload (CMD + WD)
    output wire                            req_w_valid,      // REQ_W valid
    output wire                            rsp_rd_ready,     // RSP_RD ready (backpressure to Slave)
    output wire                            rsp_wr_ready,     // RSP_WR ready (backpressure to Slave)
    //--------- inputs: bca async border (from lb_tniu_int_bca) ---------
    // forward REQ_R/REQ_W: this half is mst -> i_wrcnt / i_rdata in, o_rdcnt / o_rdptr out
    // reverse RSP_RD/RSP_WR: this half is slv -> o_wrcnt / o_rdata out, i_rdcnt / i_rdptr in
    input wire  [AW_BCA:0]                 b_req_r_wc,       // REQ_R bca write count, gray coded
    input wire  [EXT_CMD_W-1:0]            b_req_r_bd,       // REQ_R bca read data
    input wire  [AW_BCA:0]                 b_req_w_wc,       // REQ_W bca write count, gray coded
    input wire  [EXT_REQ_W-1:0]            b_req_w_bd,       // REQ_W bca read data
    input wire  [AW_BCA:0]                 b_rsp_rd_rc,      // RSP_RD  bca read count, gray coded
    input wire  [AW_BCA-1:0]               b_rsp_rd_rp,      // RSP_RD  bca read pointer
    input wire  [AW_BCA:0]                 b_rsp_wr_rc,      // RSP_WR  bca read count, gray coded
    input wire  [AW_BCA-1:0]               b_rsp_wr_rp,      // RSP_WR  bca read pointer
    //--------- outputs: bca async border (to lb_tniu_int_bca) ---------
    output wire [AW_BCA:0]                 b_req_r_rc,       // REQ_R bca read count, gray coded
    output wire [AW_BCA-1:0]               b_req_r_rp,       // REQ_R bca read pointer
    output wire [AW_BCA:0]                 b_req_w_rc,       // REQ_W bca read count, gray coded
    output wire [AW_BCA-1:0]               b_req_w_rp,       // REQ_W bca read pointer
    output wire [AW_BCA:0]                 b_rsp_rd_wc,      // RSP_RD  bca write count, gray coded
    output wire [EXT_RSP_RD_W-1:0]             b_rsp_rd_bd,  // RSP_RD  bca read data
    output wire [AW_BCA:0]                 b_rsp_wr_wc,      // RSP_WR  bca write count, gray coded
    output wire [EXT_RSP_WR_W-1:0]             b_rsp_wr_bd   // RSP_WR  bca read data
);
    //------------------------------------------------------------------------
    // Declarations (all up front, one per line)
    //------------------------------------------------------------------------
    // forward REQ_R / REQ_W out of bca, into the core
    wire [EXT_CMD_W-1:0]        sd_rq_r_pk;       // slave-decode side REQ_R packed payload
    wire                        sd_rq_r_v;        // slave-decode side REQ_R valid
    wire                        sd_rq_r_r;        // slave-decode side REQ_R ready (backpressure)
    wire [EXT_REQ_W-1:0]        sd_rq_w_pk;       // slave-decode side REQ_W packed payload
    wire                        sd_rq_w_v;        // slave-decode side REQ_W valid
    wire                        sd_rq_w_r;        // slave-decode side REQ_W ready (backpressure)
    // reverse RSP_RD/RSP_WR out of the core, into bca
    wire [EXT_RSP_RD_W-1:0]         sr_rsp_rd_pk; // slave-response side RSP_RD packed payload
    wire                        sr_rsp_rd_v;      // slave-response side RSP_RD valid
    wire                        sr_rsp_rd_r;      // slave-response side RSP_RD ready (backpressure)
    wire [EXT_RSP_WR_W-1:0]         sr_rsp_wr_pk; // slave-response side RSP_WR packed payload
    wire                        sr_rsp_wr_v;      // slave-response side RSP_WR valid
    wire                        sr_rsp_wr_r;      // slave-response side RSP_WR ready (backpressure)

    //------------------------------------------------------------------------
    // forward REQ_R / REQ_W: one bca_mst each (clk_s read side). Two bridges is
    // what keeps the two request channels independent across the crossing too.
    //------------------------------------------------------------------------
    lb_bca_mst #(
        .WIDTH        (EXT_CMD_W),
        .DEPTH        (BCA_DEPTH),
        .SYNC_STAGES  (SYNC_STAGES)
    ) u10_mst_req_r (
        .clk_r         (clk_s),
        .rrst_n       (srst_n),
        .r_ready      (sd_rq_r_r),
        .i_wrcnt_gray (b_req_r_wc),
        .i_rdata      (b_req_r_bd),
        .r_data       (sd_rq_r_pk),
        .r_valid      (sd_rq_r_v),
        .o_rdcnt_gray (b_req_r_rc),
        .o_rdptr      (b_req_r_rp)
    );

    lb_bca_mst #(
        .WIDTH        (EXT_REQ_W),
        .DEPTH        (BCA_DEPTH),
        .SYNC_STAGES  (SYNC_STAGES)
    ) u11_mst_req_w (
        .clk_r         (clk_s),
        .rrst_n       (srst_n),
        .r_ready      (sd_rq_w_r),
        .i_wrcnt_gray (b_req_w_wc),
        .i_rdata      (b_req_w_bd),
        .r_data       (sd_rq_w_pk),
        .r_valid      (sd_rq_w_v),
        .o_rdcnt_gray (b_req_w_rc),
        .o_rdptr      (b_req_w_rp)
    );

    //------------------------------------------------------------------------
    // reverse RSP_RD/RSP_WR: bca_slv (clk_s write side)
    //------------------------------------------------------------------------
    lb_bca_slv #(
        .WIDTH        (EXT_RSP_RD_W),
        .DEPTH        (BCA_DEPTH),
        .SYNC_STAGES  (SYNC_STAGES)
    ) u20_slv_rsp_rd (
        .clk_w         (clk_s),
        .wrst_n       (srst_n),
        .w_data       (sr_rsp_rd_pk),
        .w_valid      (sr_rsp_rd_v),
        .i_rdcnt_gray (b_rsp_rd_rc),
        .i_rdptr      (b_rsp_rd_rp),
        .w_ready      (sr_rsp_rd_r),
        .o_wrcnt_gray (b_rsp_rd_wc),
        .o_rdata      (b_rsp_rd_bd)
    );

    lb_bca_slv #(
        .WIDTH        (EXT_RSP_WR_W),
        .DEPTH        (BCA_DEPTH),
        .SYNC_STAGES  (SYNC_STAGES)
    ) u21_slv_rsp_wr (
        .clk_w         (clk_s),
        .wrst_n       (srst_n),
        .w_data       (sr_rsp_wr_pk),
        .w_valid      (sr_rsp_wr_v),
        .i_rdcnt_gray (b_rsp_wr_rc),
        .i_rdptr      (b_rsp_wr_rp),
        .w_ready      (sr_rsp_wr_r),
        .o_wrcnt_gray (b_rsp_wr_wc),
        .o_rdata      (b_rsp_wr_bd)
    );

    //------------------------------------------------------------------------
    // Core: cmd/wd split, adaptor, recombine. Everything that is not the bridge.
    //------------------------------------------------------------------------
    lb_tniu_ext_core #(
        .EXT_PROTOCOL         (EXT_PROTOCOL),
        .EXT_DATA_WIDTH       (EXT_DATA_WIDTH),
        .EXT_LEN_WIDTH        (EXT_LEN_WIDTH),
        .EXT_USER_WIDTH_CMD   (EXT_USER_WIDTH_CMD),
        .EXT_USER_WIDTH_RSP_RD    (EXT_USER_WIDTH_RSP_RD),
        .EXT_USER_WIDTH_RSP_WR    (EXT_USER_WIDTH_RSP_WR),
        .EXT_TXNID_WIDTH      (EXT_TXNID_WIDTH),
        .EXT_MOD_W            (EXT_MOD_W),
        .EXT_QOS_W            (EXT_QOS_W),
        .INT_ADDR_LOCAL_WIDTH (INT_ADDR_LOCAL_WIDTH)
    ) u30_core (
        .clk_s                 (clk_s),
        .srst_n               (srst_n),
        .req_r_ready          (req_r_ready),
        .req_w_ready          (req_w_ready),
        .rsp_rd_data              (rsp_rd_data),
        .rsp_rd_last              (rsp_rd_last),
        .rsp_rd_resp              (rsp_rd_resp),
        .rsp_rd_ext_txnid         (rsp_rd_ext_txnid),
        .rsp_rd_user              (rsp_rd_user),
        .rsp_rd_valid             (rsp_rd_valid),
        .rsp_wr_resp              (rsp_wr_resp),
        .rsp_wr_ext_txnid         (rsp_wr_ext_txnid),
        .rsp_wr_user              (rsp_wr_user),
        .rsp_wr_valid             (rsp_wr_valid),
        .req_r_data           (req_r_data),
        .req_r_valid          (req_r_valid),
        .req_w_data           (req_w_data),
        .req_w_valid          (req_w_valid),
        .rsp_rd_ready             (rsp_rd_ready),
        .rsp_wr_ready             (rsp_wr_ready),
        .vr_req_r_data        (sd_rq_r_pk),
        .vr_req_r_valid       (sd_rq_r_v),
        .vr_req_w_data        (sd_rq_w_pk),
        .vr_req_w_valid       (sd_rq_w_v),
        .vr_rsp_rd_ready          (sr_rsp_rd_r),
        .vr_rsp_wr_ready          (sr_rsp_wr_r),
        .vr_req_r_ready       (sd_rq_r_r),
        .vr_req_w_ready       (sd_rq_w_r),
        .vr_rsp_rd_data           (sr_rsp_rd_pk),
        .vr_rsp_rd_valid          (sr_rsp_rd_v),
        .vr_rsp_wr_data           (sr_rsp_wr_pk),
        .vr_rsp_wr_valid          (sr_rsp_wr_v)
    );

endmodule
