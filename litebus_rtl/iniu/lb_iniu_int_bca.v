//============================================================================
// Filename    : lb_iniu_int_bca.v
// Author      : litebus
// Description : INIU internal half, bca form (network side, hardenable)
// Date        : 2026-08-06
// Revision    : 1.0 initial -- per-round changes are in doc/HISTORY.md
//----------------------------------------------------------------------------
// lb_iniu_int_core wrapped in the async bridge:
//   - forward REQ_R / REQ_W bca_mst (clk read side), one bridge each
//   - reverse RSP_RD/RSP_WR bca_slv (clk write side)
// Only async bca border signals connect to the external half lb_iniu_ext_bca.
// Domain = network, and the bca halves facing it belong here -- which is why
// the bridge lives inside this module rather than at the top: PD gets one block.
//
// Two forms exist and the MODULE NAME selects between them:
//   lb_iniu_int_bca       this file -- ext and int halves sit in different clock
//                     domains, bca bridges them, border is b_* (4 wires/channel)
//   lb_iniu_int_core  both halves on one clock, border is a plain valid-ready
//                     vr_* group, no bca at all
// Not a parameter: the two borders are different sets of PORTS, and a V2001
// port list is fixed at elaboration. Not a `define either -- that is global to
// the compilation unit, while one top can hold NIUs of both forms. Wiring the
// halves to mismatched forms is a COMPILE error, not a silent deadlock.
//
// Parameter naming convention: EXT_* are external (IP-side) interface widths,
// INT_* are in-network (fabric-side) interface widths and the credit/pipe
// configuration of that interface. NIU_ID / REGION_* / BCA_* are structural.
//============================================================================
`include "lb_defines.vh"

module lb_iniu_int_bca #(
    // ==== external (IP-side) interface widths ====
    parameter EXT_ADDR_WIDTH       = 32,  // external global address width
    parameter EXT_DATA_WIDTH       = 64,  // external (MST-side) data width
    parameter EXT_LEN_WIDTH        = 4,   // external len width (beat count - 1)
    parameter EXT_TXNID_WIDTH      = 8,   // external transaction ID width
    parameter EXT_USER_WIDTH_CMD   = 8,   // external CMD user sideband width
    parameter EXT_USER_WIDTH_RSP_RD    = 8,   // external RSP_RD user sideband width
    parameter EXT_USER_WIDTH_RSP_WR    = 8,   // external RSP_WR user sideband width
    parameter EXT_MOD_W            = 0,   // REQ_W atomic-modifier pin width, 0 = none (Adv 15)
    parameter EXT_QOS_W            = 0,   // REQ qos pin width, 0 = no pin on this Master
    // ==== in-network (fabric-side) interface widths ====
    // Top injects the global maxima; narrower externals are zero-extended.
    parameter INT_ADDR_LOCAL_WIDTH = 32,  // in-network local (rebased) address width
    parameter INT_DEST_ID_WIDTH    = 4,   // = log2(NUM_SLAVES), injected by top
    parameter INT_SRC_ID_WIDTH     = 4,   // = log2(NUM_MASTERS), injected by top
    parameter INT_ID_WIDTH         = 8,   // in-network unified int_id width (>= EXT_TXNID_WIDTH)
    parameter INT_USER_WIDTH_CMD   = 8,   // in-network CMD user width (>= EXT_USER_WIDTH_CMD)
    parameter INT_USER_WIDTH_RSP_RD    = 8,   // in-network RSP_RD  user width (>= EXT_USER_WIDTH_RSP_RD)
    parameter INT_USER_WIDTH_RSP_WR    = 8,   // in-network RSP_WR  user width (>= EXT_USER_WIDTH_RSP_WR)
    parameter INT_TOTBYTES_W       = 16,  // in-network unified total_bytes width
    parameter INT_LANE_W           = 7,   // addr_lane width (carried in the RSP_RD flit)
    parameter INT_MOD_W            = 0,   // REQ_W modifier segment width (bus max), 0 = absent (Adv 15)
    parameter INT_QOS_W            = 0,   // QoS segment width (bus max), 0 = absent; tops the CMD flit
    // ==== per-channel pipe enables (four channels, independently configurable) ====
    parameter EXT_PIPE_REQ_R       = 1,   // ext_pipe (valid-ready skid) on forward REQ_R
    parameter EXT_PIPE_REQ_W       = 1,   // ext_pipe (valid-ready skid) on forward REQ_W
    parameter EXT_PIPE_RSP_RD          = 1,   // ext_pipe (valid-ready skid) on reverse RSP_RD
    parameter EXT_PIPE_RSP_WR          = 1,   // ext_pipe (valid-ready skid) on reverse RSP_WR
    parameter INT_PIPE_REQ_R       = 1,   // int_pipe (credit register) on forward REQ_R
    parameter INT_PIPE_REQ_W       = 1,   // int_pipe (credit register) on forward REQ_W
    parameter INT_PIPE_RSP_RD          = 1,   // int_pipe (credit register) on reverse RSP_RD
    parameter INT_PIPE_RSP_WR          = 1,   // int_pipe (credit register) on reverse RSP_WR
    // ==== credit configuration of the internal interface ====
    parameter INT_REQ_R_CREDIT     = 4,   // forward REQ_R egress credit = peer ingress FIFO depth
    parameter INT_REQ_W_CREDIT     = 4,   // forward REQ_W egress credit = peer ingress FIFO depth
    parameter INT_RSP_RD_CREDIT_FIFO   = 4,   // reverse RSP_RD ingress FIFO depth = upstream credit budget
    parameter INT_RSP_WR_CREDIT_FIFO   = 4,   // reverse RSP_WR ingress FIFO depth = upstream credit budget
    // ==== identity and memory map (structural, generator-filled) ====
    parameter NIU_ID               = 0,   // this INIU id value
    parameter NUM_REGION           = 1,   // number of memory-map regions
    parameter REGION_BASE = {(NUM_REGION*EXT_ADDR_WIDTH){1'b0}},     // region base table, flattened
    parameter REGION_MASK = {(NUM_REGION*EXT_ADDR_WIDTH){1'b1}},     // region mask table, flattened
    parameter REGION_DEST = {(NUM_REGION*INT_DEST_ID_WIDTH){1'b0}},  // region DestID table, flattened
    // ==== bca (async border) structure ====
    parameter BCA_DEPTH            = 8,   // bca async FIFO depth (power of 2)
    parameter SYNC_STAGES          = 2,   // bca gray-code synchronizer stages (>=2)
    // ---- derived parameters; used in the port list, so they must stay in the
    // ---- parameter list (V2001 body localparams cannot size ports). Not to be
    // ---- overridden externally -- the defaults are the correct values.
    parameter EXT_CMD_W = EXT_QOS_W + `LB_OPCODE_WIDTH + EXT_ADDR_WIDTH + EXT_LEN_WIDTH +
                          EXT_TXNID_WIDTH + EXT_USER_WIDTH_CMD,
    parameter EXT_WD_W  = EXT_DATA_WIDTH + EXT_DATA_WIDTH/8 + 1 + EXT_TXNID_WIDTH,
    parameter EXT_REQ_W = EXT_CMD_W + EXT_MOD_W + EXT_WD_W,
    parameter EXT_RSP_RD_W  = EXT_TXNID_WIDTH + `LB_RESP_WIDTH + EXT_USER_WIDTH_RSP_RD + 1 + EXT_DATA_WIDTH,
    parameter EXT_RSP_WR_W  = EXT_TXNID_WIDTH + `LB_RESP_WIDTH + EXT_USER_WIDTH_RSP_WR,
    // CMD flit uses in-network widths (INT_ID / INT_USER / INT_TOTBYTES)
    parameter INT_CMD_FLIT_W = INT_QOS_W + `LB_OPCODE_WIDTH + INT_ADDR_LOCAL_WIDTH + INT_TOTBYTES_W +
                               INT_USER_WIDTH_CMD + INT_ID_WIDTH + INT_DEST_ID_WIDTH + INT_SRC_ID_WIDTH,
    parameter INT_WD_FLIT_W  = 1 + (EXT_DATA_WIDTH/8) + EXT_DATA_WIDTH +
                               INT_DEST_ID_WIDTH + INT_SRC_ID_WIDTH,
    // REQ_W channel = CMD field + WD field concatenated (parallel, same beat).
    // REQ_R carries the CMD field alone, so its flit width IS INT_CMD_FLIT_W.
    parameter INT_REQ_FLIT_W = INT_CMD_FLIT_W + INT_MOD_W + INT_WD_FLIT_W,
    // RSP_RD flit (low->high): data, last, trans_last, user, resp, int_id,
    //                      total_bytes, addr_lane, src_id  (in-network widths)
    parameter INT_RSP_RD_FLIT_W  = INT_SRC_ID_WIDTH + INT_QOS_W + INT_LANE_W + INT_TOTBYTES_W + INT_ID_WIDTH +
                               `LB_RESP_WIDTH + INT_USER_WIDTH_RSP_RD + 1 + 1 + EXT_DATA_WIDTH,
    // RSP_WR flit (low->high): user, resp, int_id, src_id
    parameter INT_RSP_WR_FLIT_W  = INT_SRC_ID_WIDTH + INT_QOS_W + INT_ID_WIDTH +
                               `LB_RESP_WIDTH + INT_USER_WIDTH_RSP_WR,
    parameter AW_BCA = (BCA_DEPTH <= 2)   ? 1 :   // bca pointer width = log2(BCA_DEPTH)
                       (BCA_DEPTH <= 4)   ? 2 :
                       (BCA_DEPTH <= 8)   ? 3 :
                       (BCA_DEPTH <= 16)  ? 4 :
                       (BCA_DEPTH <= 32)  ? 5 : 6
) (
    //--------- inputs: clock / reset (network clock domain) ---------
    input wire                        clk,                 // network-side clock
    input wire                        rst_n,               // async reset, active low
    //--------- inputs: internal interface (fabric side, packed flit + credit) ---------
    input wire                        o_req_r_credit_ret,  // REQ_R credit returned by the peer
    input wire                        o_req_w_credit_ret,  // REQ_W credit returned by the peer
    input wire  [INT_RSP_RD_FLIT_W-1:0]   i_rsp_rd_flit,   // RSP_RD flit in
    input wire                        i_rsp_rd_valid,      // RSP_RD flit valid
    input wire  [INT_RSP_WR_FLIT_W-1:0]   i_rsp_wr_flit,   // RSP_WR flit in
    input wire                        i_rsp_wr_valid,      // RSP_WR flit valid
    //--------- outputs: internal interface (fabric side, packed flit + credit) ---------
    output wire [INT_CMD_FLIT_W-1:0]  o_req_r_flit,        // REQ_R flit out (CMD only)
    output wire                       o_req_r_valid,       // REQ_R flit valid
    output wire [INT_REQ_FLIT_W-1:0]  o_req_w_flit,        // REQ_W flit out (CMD + WD)
    output wire                       o_req_w_valid,       // REQ_W flit valid
    output wire                       i_rsp_rd_credit_ret, // RSP_RD credit returned to the peer
    output wire                       i_rsp_wr_credit_ret, // RSP_WR credit returned to the peer
    //--------- inputs: bca async border (from lb_iniu_ext_bca) ---------
    // forward REQ_R/REQ_W: this half is mst -> i_wrcnt / i_rdata in, o_rdcnt / o_rdptr out
    // reverse RSP_RD/RSP_WR: this half is slv -> o_wrcnt / o_rdata out, i_rdcnt / i_rdptr in
    input wire  [AW_BCA:0]            b_req_r_wc,          // REQ_R bca write count, gray coded
    input wire  [EXT_CMD_W-1:0]       b_req_r_bd,          // REQ_R bca read data
    input wire  [AW_BCA:0]            b_req_w_wc,          // REQ_W bca write count, gray coded
    input wire  [EXT_REQ_W-1:0]       b_req_w_bd,          // REQ_W bca read data
    input wire  [AW_BCA:0]            b_rsp_rd_rc,         // RSP_RD  bca read count, gray coded
    input wire  [AW_BCA-1:0]          b_rsp_rd_rp,         // RSP_RD  bca read pointer
    input wire  [AW_BCA:0]            b_rsp_wr_rc,         // RSP_WR  bca read count, gray coded
    input wire  [AW_BCA-1:0]          b_rsp_wr_rp,         // RSP_WR  bca read pointer
    //--------- outputs: bca async border (to lb_iniu_ext_bca) ---------
    output wire [AW_BCA:0]            b_req_r_rc,          // REQ_R bca read count, gray coded
    output wire [AW_BCA-1:0]          b_req_r_rp,          // REQ_R bca read pointer
    output wire [AW_BCA:0]            b_req_w_rc,          // REQ_W bca read count, gray coded
    output wire [AW_BCA-1:0]          b_req_w_rp,          // REQ_W bca read pointer
    output wire [AW_BCA:0]            b_rsp_rd_wc,         // RSP_RD  bca write count, gray coded
    output wire [EXT_RSP_RD_W-1:0]        b_rsp_rd_bd,     // RSP_RD  bca read data
    output wire [AW_BCA:0]            b_rsp_wr_wc,         // RSP_WR  bca write count, gray coded
    output wire [EXT_RSP_WR_W-1:0]        b_rsp_wr_bd      // RSP_WR  bca read data
);
    //------------------------------------------------------------------------
    // Declarations (all up front, one per line)
    //------------------------------------------------------------------------
    wire [EXT_CMD_W-1:0]        fwd_rq_r_pk;       // REQ_R packed payload out of bca
    wire                        fwd_rq_r_v;        // REQ_R valid out of bca
    wire                        fwd_rq_r_r;        // REQ_R ready (backpressure) into bca
    wire [EXT_REQ_W-1:0]        fwd_rq_w_pk;       // REQ_W packed payload out of bca
    wire                        fwd_rq_w_v;        // REQ_W valid out of bca
    wire                        fwd_rq_w_r;        // REQ_W ready (backpressure) into bca
    wire [EXT_RSP_RD_W-1:0]         rev_rsp_rd_pk; // reverse direction RSP_RD packed payload
    wire                        rev_rsp_rd_v;      // reverse direction RSP_RD valid
    wire                        rev_rsp_rd_r;      // reverse direction RSP_RD ready (backpressure)
    wire [EXT_RSP_WR_W-1:0]         rev_rsp_wr_pk; // reverse direction RSP_WR packed payload
    wire                        rev_rsp_wr_v;      // reverse direction RSP_WR valid
    wire                        rev_rsp_wr_r;      // reverse direction RSP_WR ready (backpressure)

    //------------------------------------------------------------------------
    // forward REQ_R / REQ_W: one bca_mst each (clk read side). Two bridges is
    // what keeps the two request channels independent across the crossing too.
    //------------------------------------------------------------------------
    lb_bca_mst #(
        .WIDTH        (EXT_CMD_W),
        .DEPTH        (BCA_DEPTH),
        .SYNC_STAGES  (SYNC_STAGES)
    ) u10_mst_req_r (
        .clk_r         (clk),
        .rrst_n       (rst_n),
        .r_ready      (fwd_rq_r_r),
        .i_wrcnt_gray (b_req_r_wc),
        .i_rdata      (b_req_r_bd),
        .r_data       (fwd_rq_r_pk),
        .r_valid      (fwd_rq_r_v),
        .o_rdcnt_gray (b_req_r_rc),
        .o_rdptr      (b_req_r_rp)
    );

    lb_bca_mst #(
        .WIDTH        (EXT_REQ_W),
        .DEPTH        (BCA_DEPTH),
        .SYNC_STAGES  (SYNC_STAGES)
    ) u11_mst_req_w (
        .clk_r         (clk),
        .rrst_n       (rst_n),
        .r_ready      (fwd_rq_w_r),
        .i_wrcnt_gray (b_req_w_wc),
        .i_rdata      (b_req_w_bd),
        .r_data       (fwd_rq_w_pk),
        .r_valid      (fwd_rq_w_v),
        .o_rdcnt_gray (b_req_w_rc),
        .o_rdptr      (b_req_w_rp)
    );

    //------------------------------------------------------------------------
    // Core: the whole forward and reverse pipeline. Everything but the bridge.
    //------------------------------------------------------------------------
    lb_iniu_int_core #(
        .EXT_ADDR_WIDTH       (EXT_ADDR_WIDTH),
        .EXT_DATA_WIDTH       (EXT_DATA_WIDTH),
        .EXT_LEN_WIDTH        (EXT_LEN_WIDTH),
        .EXT_TXNID_WIDTH      (EXT_TXNID_WIDTH),
        .EXT_USER_WIDTH_CMD   (EXT_USER_WIDTH_CMD),
        .EXT_USER_WIDTH_RSP_RD    (EXT_USER_WIDTH_RSP_RD),
        .EXT_USER_WIDTH_RSP_WR    (EXT_USER_WIDTH_RSP_WR),
        .EXT_MOD_W            (EXT_MOD_W),
        .EXT_QOS_W            (EXT_QOS_W),
        .INT_ADDR_LOCAL_WIDTH (INT_ADDR_LOCAL_WIDTH),
        .INT_DEST_ID_WIDTH    (INT_DEST_ID_WIDTH),
        .INT_SRC_ID_WIDTH     (INT_SRC_ID_WIDTH),
        .INT_ID_WIDTH         (INT_ID_WIDTH),
        .INT_USER_WIDTH_CMD   (INT_USER_WIDTH_CMD),
        .INT_USER_WIDTH_RSP_RD    (INT_USER_WIDTH_RSP_RD),
        .INT_USER_WIDTH_RSP_WR    (INT_USER_WIDTH_RSP_WR),
        .INT_TOTBYTES_W       (INT_TOTBYTES_W),
        .INT_LANE_W           (INT_LANE_W),
        .INT_MOD_W            (INT_MOD_W),
        .INT_QOS_W            (INT_QOS_W),
        .EXT_PIPE_REQ_R       (EXT_PIPE_REQ_R),
        .EXT_PIPE_REQ_W       (EXT_PIPE_REQ_W),
        .EXT_PIPE_RSP_RD          (EXT_PIPE_RSP_RD),
        .EXT_PIPE_RSP_WR          (EXT_PIPE_RSP_WR),
        .INT_PIPE_REQ_R       (INT_PIPE_REQ_R),
        .INT_PIPE_REQ_W       (INT_PIPE_REQ_W),
        .INT_PIPE_RSP_RD          (INT_PIPE_RSP_RD),
        .INT_PIPE_RSP_WR          (INT_PIPE_RSP_WR),
        .INT_REQ_R_CREDIT     (INT_REQ_R_CREDIT),
        .INT_REQ_W_CREDIT     (INT_REQ_W_CREDIT),
        .INT_RSP_RD_CREDIT_FIFO   (INT_RSP_RD_CREDIT_FIFO),
        .INT_RSP_WR_CREDIT_FIFO   (INT_RSP_WR_CREDIT_FIFO),
        .NIU_ID               (NIU_ID),
        .NUM_REGION           (NUM_REGION),
        .REGION_BASE          (REGION_BASE),
        .REGION_MASK          (REGION_MASK),
        .REGION_DEST          (REGION_DEST)
    ) u20_core (
        .clk                  (clk),
        .rst_n                (rst_n),
        .o_req_r_credit_ret   (o_req_r_credit_ret),
        .o_req_w_credit_ret   (o_req_w_credit_ret),
        .i_rsp_rd_flit            (i_rsp_rd_flit),
        .i_rsp_rd_valid           (i_rsp_rd_valid),
        .i_rsp_wr_flit            (i_rsp_wr_flit),
        .i_rsp_wr_valid           (i_rsp_wr_valid),
        .o_req_r_flit         (o_req_r_flit),
        .o_req_r_valid        (o_req_r_valid),
        .o_req_w_flit         (o_req_w_flit),
        .o_req_w_valid        (o_req_w_valid),
        .i_rsp_rd_credit_ret      (i_rsp_rd_credit_ret),
        .i_rsp_wr_credit_ret      (i_rsp_wr_credit_ret),
        .vr_req_r_data        (fwd_rq_r_pk),
        .vr_req_r_valid       (fwd_rq_r_v),
        .vr_req_w_data        (fwd_rq_w_pk),
        .vr_req_w_valid       (fwd_rq_w_v),
        .vr_rsp_rd_ready          (rev_rsp_rd_r),
        .vr_rsp_wr_ready          (rev_rsp_wr_r),
        .vr_req_r_ready       (fwd_rq_r_r),
        .vr_req_w_ready       (fwd_rq_w_r),
        .vr_rsp_rd_data           (rev_rsp_rd_pk),
        .vr_rsp_rd_valid          (rev_rsp_rd_v),
        .vr_rsp_wr_data           (rev_rsp_wr_pk),
        .vr_rsp_wr_valid          (rev_rsp_wr_v)
    );

    //------------------------------------------------------------------------
    // reverse RSP_RD/RSP_WR: bca_slv (clk write side)
    //------------------------------------------------------------------------
    lb_bca_slv #(
        .WIDTH        (EXT_RSP_RD_W),
        .DEPTH        (BCA_DEPTH),
        .SYNC_STAGES  (SYNC_STAGES)
    ) u30_slv_rsp_rd (
        .clk_w         (clk),
        .wrst_n       (rst_n),
        .w_data       (rev_rsp_rd_pk),
        .w_valid      (rev_rsp_rd_v),
        .i_rdcnt_gray (b_rsp_rd_rc),
        .i_rdptr      (b_rsp_rd_rp),
        .w_ready      (rev_rsp_rd_r),
        .o_wrcnt_gray (b_rsp_rd_wc),
        .o_rdata      (b_rsp_rd_bd)
    );

    lb_bca_slv #(
        .WIDTH        (EXT_RSP_WR_W),
        .DEPTH        (BCA_DEPTH),
        .SYNC_STAGES  (SYNC_STAGES)
    ) u31_slv_rsp_wr (
        .clk_w         (clk),
        .wrst_n       (rst_n),
        .w_data       (rev_rsp_wr_pk),
        .w_valid      (rev_rsp_wr_v),
        .i_rdcnt_gray (b_rsp_wr_rc),
        .i_rdptr      (b_rsp_wr_rp),
        .w_ready      (rev_rsp_wr_r),
        .o_wrcnt_gray (b_rsp_wr_wc),
        .o_rdata      (b_rsp_wr_bd)
    );

endmodule
