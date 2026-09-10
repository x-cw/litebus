//============================================================================
// Filename    : lb_iniu_ext_bca.v
// Author      : litebus
// Description : INIU external half, bca form (Master side, hardenable)
// Date        : 2026-08-06
// Revision    : 1.0 initial -- per-round changes are in doc/HISTORY.md
//----------------------------------------------------------------------------
// lb_iniu_ext_core wrapped in the async bridge:
//   - forward REQ_R / REQ_W bca_slv (clk_m write side), one bridge each
//   - reverse RSP_RD/RSP_WR bca_mst (clk_m read side)
// Only async bca border signals connect to the internal half lb_iniu_int_bca
// (4 per channel x 4 channels). This half belongs to the Master clock domain
// and can be hardened independently -- including the bca write-side halves,
// which is why the bridge lives inside this module rather than at the top.
//
// Two forms exist and the MODULE NAME selects between them:
//   lb_iniu_ext_bca   this file -- ext and int halves sit in different clock
//                     domains, bca bridges them, border is b_* (4 wires/channel)
//   lb_iniu_ext_core  both halves on one clock, border is a plain valid-ready
//                     vr_* group, no bca at all
// Not a parameter: the two borders are different sets of PORTS, and a V2001
// port list is fixed at elaboration. Not a `define either -- that is global to
// the compilation unit, while one top can hold NIUs of both forms.
//
// Wiring the two halves to mismatched forms is a COMPILE error (the top would
// drive b_req_w_wc on a module that has no such port), not a silent deadlock.
//
// Parameter naming convention: EXT_* are external (IP-side) interface widths.
// The bca depth / synchronizer depth are structural, not interface widths.
//============================================================================
`include "lb_defines.vh"

module lb_iniu_ext_bca #(
    parameter EXT_PROTOCOL       = 0,   // 0 = litebus (feed-through), 1 = AXI, 2 = APB
    parameter EXT_ADDR_WIDTH     = 32,  // external global address width
    parameter EXT_DATA_WIDTH     = 64,  // external (MST-side) data width
    parameter EXT_LEN_WIDTH      = 4,   // external len width (beat count - 1)
    parameter EXT_TXNID_WIDTH    = 8,   // external transaction ID width
    parameter EXT_USER_WIDTH_CMD = 8,   // external CMD user sideband width
    parameter EXT_USER_WIDTH_RSP_RD  = 8,   // external RSP_RD user sideband width
    parameter EXT_USER_WIDTH_RSP_WR  = 8,   // external RSP_WR user sideband width
    parameter EXT_MOD_W          = 0,   // REQ_W atomic-modifier pin width, 0 = none (Adv 15)
    parameter EXT_QOS_W          = 0,   // REQ qos pin width, 0 = none (rides inside the CMD payload)
    parameter BCA_DEPTH          = 8,   // bca async FIFO depth (power of 2)
    parameter SYNC_STAGES        = 2,   // bca gray-code synchronizer stages (>=2)
    // ---- derived parameters; used in the port list, so they must stay in the
    // ---- parameter list (V2001 body localparams cannot size ports). Not to be
    // ---- overridden externally -- the defaults are the correct values.
    parameter EXT_CMD_W = EXT_QOS_W + `LB_OPCODE_WIDTH + EXT_ADDR_WIDTH + EXT_LEN_WIDTH +
                          EXT_TXNID_WIDTH + EXT_USER_WIDTH_CMD,
    parameter EXT_WD_W  = EXT_DATA_WIDTH + EXT_DATA_WIDTH/8 + 1 + EXT_TXNID_WIDTH,
    // external REQ = {cmd field (high) | modifier (Adv 15, often absent) | wd field (low)}
    parameter EXT_REQ_W = EXT_CMD_W + EXT_MOD_W + EXT_WD_W,
    parameter EXT_RSP_RD_W  = EXT_TXNID_WIDTH + `LB_RESP_WIDTH + EXT_USER_WIDTH_RSP_RD + 1 + EXT_DATA_WIDTH,
    parameter EXT_RSP_WR_W  = EXT_TXNID_WIDTH + `LB_RESP_WIDTH + EXT_USER_WIDTH_RSP_WR,
    // user pin widths floored at 1, same reason as in lb_iniu_ext_core: a width-0
    // user cannot delete a V2001 port, so the stub stays and the top omits the pin.
    parameter EXT_USER_RSP_RD_PW = (EXT_USER_WIDTH_RSP_RD < 1) ? 1 : EXT_USER_WIDTH_RSP_RD,
    parameter EXT_USER_RSP_WR_PW = (EXT_USER_WIDTH_RSP_WR < 1) ? 1 : EXT_USER_WIDTH_RSP_WR,
    parameter AW_BCA    = (BCA_DEPTH <= 2)   ? 1 :   // bca pointer width = log2(BCA_DEPTH)
                          (BCA_DEPTH <= 4)   ? 2 :
                          (BCA_DEPTH <= 8)   ? 3 :
                          (BCA_DEPTH <= 16)  ? 4 :
                          (BCA_DEPTH <= 32)  ? 5 : 6
) (
    //--------- inputs: clock / reset (IP clock domain) ---------
    input wire                          clk_m,            // IP-side clock
    input wire                          mrst_n,           // IP-side async reset, active low
    //--------- inputs: external interface (Master side, Valid-Ready) ---------
    // Two forward channels: REQ_R carries the CMD half alone (EXT_CMD_W), REQ_W
    // carries CMD + WD on one beat (EXT_REQ_W). Independent handshakes.
    input wire  [EXT_CMD_W-1:0]         req_r_data,       // REQ_R packed payload (CMD only)
    input wire                          req_r_valid,      // REQ_R valid
    input wire  [EXT_REQ_W-1:0]         req_w_data,       // REQ_W packed payload (CMD + WD)
    input wire                          req_w_valid,      // REQ_W valid
    input wire                          rsp_rd_ready,     // RSP_RD ready (backpressure from IP)
    input wire                          rsp_wr_ready,     // RSP_WR ready (backpressure from IP)
    //--------- outputs: external interface (Master side, Valid-Ready) ---------
    output wire                         req_r_ready,      // REQ_R ready (backpressure to IP)
    output wire                         req_w_ready,      // REQ_W ready (backpressure to IP)
    output wire [EXT_DATA_WIDTH-1:0]    rsp_rd_data,      // RSP_RD data
    output wire                         rsp_rd_last,      // RSP_RD last beat
    output wire [`LB_RESP_WIDTH-1:0]    rsp_rd_resp,      // RSP_RD response code
    output wire [EXT_TXNID_WIDTH-1:0]   rsp_rd_ext_txnid, // RSP_RD external transaction ID
    output wire [EXT_USER_RSP_RD_PW-1:0]    rsp_rd_user,  // RSP_RD user sideband (zero stub at width 0)
    output wire                         rsp_rd_valid,     // RSP_RD valid
    output wire [`LB_RESP_WIDTH-1:0]    rsp_wr_resp,      // RSP_WR response code
    output wire [EXT_TXNID_WIDTH-1:0]   rsp_wr_ext_txnid, // RSP_WR external transaction ID
    output wire [EXT_USER_RSP_WR_PW-1:0]    rsp_wr_user,  // RSP_WR user sideband (zero stub at width 0)
    output wire                         rsp_wr_valid,     // RSP_WR valid
    //--------- inputs: bca async border (from lb_iniu_int_bca) ---------
    // forward REQ_R/REQ_W: this half is slv -> o_wrcnt / o_rdata out, i_rdcnt / i_rdptr in
    // reverse RSP_RD/RSP_WR: this half is mst -> i_wrcnt / i_rdata in, o_rdcnt / o_rdptr out
    input wire  [AW_BCA:0]              b_req_r_rc,       // REQ_R bca read count, gray coded
    input wire  [AW_BCA-1:0]            b_req_r_rp,       // REQ_R bca read pointer
    input wire  [AW_BCA:0]              b_req_w_rc,       // REQ_W bca read count, gray coded
    input wire  [AW_BCA-1:0]            b_req_w_rp,       // REQ_W bca read pointer
    input wire  [AW_BCA:0]              b_rsp_rd_wc,      // RSP_RD  bca write count, gray coded
    input wire  [EXT_RSP_RD_W-1:0]          b_rsp_rd_bd,  // RSP_RD  bca read data
    input wire  [AW_BCA:0]              b_rsp_wr_wc,      // RSP_WR  bca write count, gray coded
    input wire  [EXT_RSP_WR_W-1:0]          b_rsp_wr_bd,  // RSP_WR  bca read data
    //--------- outputs: bca async border (to lb_iniu_int_bca) ---------
    output wire [AW_BCA:0]              b_req_r_wc,       // REQ_R bca write count, gray coded
    output wire [EXT_CMD_W-1:0]         b_req_r_bd,       // REQ_R bca read data
    output wire [AW_BCA:0]              b_req_w_wc,       // REQ_W bca write count, gray coded
    output wire [EXT_REQ_W-1:0]         b_req_w_bd,       // REQ_W bca read data
    output wire [AW_BCA:0]              b_rsp_rd_rc,      // RSP_RD  bca read count, gray coded
    output wire [AW_BCA-1:0]            b_rsp_rd_rp,      // RSP_RD  bca read pointer
    output wire [AW_BCA:0]              b_rsp_wr_rc,      // RSP_WR  bca read count, gray coded
    output wire [AW_BCA-1:0]            b_rsp_wr_rp       // RSP_WR  bca read pointer
);
    //------------------------------------------------------------------------
    // Declarations (all up front, one per line)
    //------------------------------------------------------------------------
    wire [EXT_CMD_W-1:0] ad_rq_r_pk;       // adaptor side REQ_R packed payload
    wire                 ad_rq_r_v;        // adaptor side REQ_R valid
    wire                 ad_rq_r_r;        // adaptor side REQ_R ready (backpressure)
    wire [EXT_REQ_W-1:0] ad_rq_w_pk;       // adaptor side REQ_W packed payload
    wire                 ad_rq_w_v;        // adaptor side REQ_W valid
    wire                 ad_rq_w_r;        // adaptor side REQ_W ready (backpressure)
    wire [EXT_RSP_RD_W-1:0]  ad_rsp_rd_pk; // adaptor side RSP_RD packed payload
    wire                 ad_rsp_rd_v;      // adaptor side RSP_RD valid
    wire                 ad_rsp_rd_r;      // adaptor side RSP_RD ready (backpressure)
    wire [EXT_RSP_WR_W-1:0]  ad_rsp_wr_pk; // adaptor side RSP_WR packed payload
    wire                 ad_rsp_wr_v;      // adaptor side RSP_WR valid
    wire                 ad_rsp_wr_r;      // adaptor side RSP_WR ready (backpressure)

    //------------------------------------------------------------------------
    // Core: adaptor + reverse-payload unpack. Everything that is not the bridge.
    //------------------------------------------------------------------------
    lb_iniu_ext_core #(
        .EXT_PROTOCOL       (EXT_PROTOCOL),
        .EXT_ADDR_WIDTH     (EXT_ADDR_WIDTH),
        .EXT_DATA_WIDTH     (EXT_DATA_WIDTH),
        .EXT_LEN_WIDTH      (EXT_LEN_WIDTH),
        .EXT_TXNID_WIDTH    (EXT_TXNID_WIDTH),
        .EXT_USER_WIDTH_CMD (EXT_USER_WIDTH_CMD),
        .EXT_USER_WIDTH_RSP_RD  (EXT_USER_WIDTH_RSP_RD),
        .EXT_USER_WIDTH_RSP_WR  (EXT_USER_WIDTH_RSP_WR),
        .EXT_MOD_W          (EXT_MOD_W),
        .EXT_QOS_W          (EXT_QOS_W)
    ) u10_core (
        .clk_m               (clk_m),
        .mrst_n             (mrst_n),
        .req_r_data         (req_r_data),
        .req_r_valid        (req_r_valid),
        .req_w_data         (req_w_data),
        .req_w_valid        (req_w_valid),
        .rsp_rd_ready           (rsp_rd_ready),
        .rsp_wr_ready           (rsp_wr_ready),
        .req_r_ready        (req_r_ready),
        .req_w_ready        (req_w_ready),
        .rsp_rd_data            (rsp_rd_data),
        .rsp_rd_last            (rsp_rd_last),
        .rsp_rd_resp            (rsp_rd_resp),
        .rsp_rd_ext_txnid       (rsp_rd_ext_txnid),
        .rsp_rd_user            (rsp_rd_user),
        .rsp_rd_valid           (rsp_rd_valid),
        .rsp_wr_resp            (rsp_wr_resp),
        .rsp_wr_ext_txnid       (rsp_wr_ext_txnid),
        .rsp_wr_user            (rsp_wr_user),
        .rsp_wr_valid           (rsp_wr_valid),
        .vr_req_r_ready     (ad_rq_r_r),
        .vr_req_w_ready     (ad_rq_w_r),
        .vr_rsp_rd_data         (ad_rsp_rd_pk),
        .vr_rsp_rd_valid        (ad_rsp_rd_v),
        .vr_rsp_wr_data         (ad_rsp_wr_pk),
        .vr_rsp_wr_valid        (ad_rsp_wr_v),
        .vr_req_r_data      (ad_rq_r_pk),
        .vr_req_r_valid     (ad_rq_r_v),
        .vr_req_w_data      (ad_rq_w_pk),
        .vr_req_w_valid     (ad_rq_w_v),
        .vr_rsp_rd_ready        (ad_rsp_rd_r),
        .vr_rsp_wr_ready        (ad_rsp_wr_r)
    );

    //------------------------------------------------------------------------
    // forward REQ_R / REQ_W -> one bca_slv each. Two bridges rather than one is
    // the cost of the split on this side, and it is what keeps the two channels
    // independent across the domain crossing as well: a full REQ_W bridge must
    // not stall a read.
    //------------------------------------------------------------------------
    lb_bca_slv #(
        .WIDTH        (EXT_CMD_W),
        .DEPTH        (BCA_DEPTH),
        .SYNC_STAGES  (SYNC_STAGES)
    ) u20_slv_req_r (
        .clk_w         (clk_m),
        .wrst_n       (mrst_n),
        .w_data       (ad_rq_r_pk),
        .w_valid      (ad_rq_r_v),
        .i_rdcnt_gray (b_req_r_rc),
        .i_rdptr      (b_req_r_rp),
        .w_ready      (ad_rq_r_r),
        .o_wrcnt_gray (b_req_r_wc),
        .o_rdata      (b_req_r_bd)
    );

    lb_bca_slv #(
        .WIDTH        (EXT_REQ_W),
        .DEPTH        (BCA_DEPTH),
        .SYNC_STAGES  (SYNC_STAGES)
    ) u21_slv_req_w (
        .clk_w         (clk_m),
        .wrst_n       (mrst_n),
        .w_data       (ad_rq_w_pk),
        .w_valid      (ad_rq_w_v),
        .i_rdcnt_gray (b_req_w_rc),
        .i_rdptr      (b_req_w_rp),
        .w_ready      (ad_rq_w_r),
        .o_wrcnt_gray (b_req_w_wc),
        .o_rdata      (b_req_w_bd)
    );

    //------------------------------------------------------------------------
    // reverse RSP_RD/RSP_WR -> bca_mst (clk_m read side)
    //------------------------------------------------------------------------
    lb_bca_mst #(
        .WIDTH        (EXT_RSP_RD_W),
        .DEPTH        (BCA_DEPTH),
        .SYNC_STAGES  (SYNC_STAGES)
    ) u30_mst_rsp_rd (
        .clk_r         (clk_m),
        .rrst_n       (mrst_n),
        .r_ready      (ad_rsp_rd_r),
        .i_wrcnt_gray (b_rsp_rd_wc),
        .i_rdata      (b_rsp_rd_bd),
        .r_data       (ad_rsp_rd_pk),
        .r_valid      (ad_rsp_rd_v),
        .o_rdcnt_gray (b_rsp_rd_rc),
        .o_rdptr      (b_rsp_rd_rp)
    );

    lb_bca_mst #(
        .WIDTH        (EXT_RSP_WR_W),
        .DEPTH        (BCA_DEPTH),
        .SYNC_STAGES  (SYNC_STAGES)
    ) u31_mst_rsp_wr (
        .clk_r         (clk_m),
        .rrst_n       (mrst_n),
        .r_ready      (ad_rsp_wr_r),
        .i_wrcnt_gray (b_rsp_wr_wc),
        .i_rdata      (b_rsp_wr_bd),
        .r_data       (ad_rsp_wr_pk),
        .r_valid      (ad_rsp_wr_v),
        .o_rdcnt_gray (b_rsp_wr_rc),
        .o_rdptr      (b_rsp_wr_rp)
    );

endmodule
