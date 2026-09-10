//============================================================================
// Filename    : lb_tniu_int_bca.v
// Author      : litebus
// Description : TNIU internal half, bca form (network side, hardenable)
// Date        : 2026-08-06
// Revision    : 1.0 initial -- per-round changes are in doc/HISTORY.md
//----------------------------------------------------------------------------
// lb_tniu_int_core wrapped in the async bridge:
//   - forward REQ_R / REQ_W bca_slv (clk write side), one bridge each
//   - reverse RSP_RD/RSP_WR bca_mst (clk read side)
// Only async bca border signals connect to the external half lb_tniu_ext_bca.
// Domain = network, and the bca halves facing it belong here -- which is why
// the bridge lives inside this module rather than at the top: PD gets one block.
//
// Two forms exist and the MODULE NAME selects between them:
//   lb_tniu_int_bca       this file -- halves in different clock domains, bca
//                     bridges them, border is b_* (4 wires per channel)
//   lb_tniu_int_core  both halves on one clock, border is a plain valid-ready
//                     vr_* group, no bca at all
// Not a parameter: the two borders are different sets of PORTS, and a V2001
// port list is fixed at elaboration. Not a `define either -- that is global to
// the compilation unit, while one top can hold NIUs of both forms. Wiring the
// halves to mismatched forms is a COMPILE error, not a silent deadlock.
//
// Parameter naming convention (CODING_STYLE 1.4): EXT_* are external (Slave-IP
// side) interface widths and knobs, INT_* are in-network (fabric-side) interface
// widths and the credit configuration; structural / topology parameters
// (WFRAG_DW, BCA_DEPTH, SYNC_STAGES) carry no prefix.
//============================================================================
`include "lb_defines.vh"

module lb_tniu_int_bca #(
    // ==== external interface (Slave IP side, Valid-Ready) ====
    parameter EXT_DATA_WIDTH       = 64,  // Slave-side data width
    parameter EXT_LEN_WIDTH        = 4,   // Slave-side len width (beat count - 1)
    parameter EXT_USER_WIDTH_CMD   = 8,   // CMD user sideband width
    parameter EXT_USER_WIDTH_RSP_RD    = 8,   // RSP_RD user sideband width
    parameter EXT_USER_WIDTH_RSP_WR    = 8,   // RSP_WR user sideband width
    parameter EXT_TXNID_WIDTH      = 3,   // Slave-side external transaction ID width
    parameter EXT_MOD_W            = 0,   // REQ_W atomic-modifier pin width, 0 = none (Adv 15)
    parameter EXT_QOS_W            = 0,   // REQ qos pin width, 0 = no pin on this Slave
    parameter EXT_PENDING_TRANS    = 8,   // outstanding transactions (cmd_table depth)
    // 1 = Slave IP returns read data interleaved -> insert lb_tniu_rsp_rd_frag
    parameter EXT_RSP_RD_INTERLEAVE    = 0,
    // 1 = atomic support (Adv 15): dual-response cmd_table slots for
    // ATOMIC_LOAD/SWAP/COMPARE; 0 elaborates today's logic bit for bit
    parameter EXT_ATOMIC_EN        = 0,
    // ==== per-channel pipe enables (eight, independently configurable) ====
    parameter EXT_PIPE_REQ_R       = 1,   // ext_pipe (valid-ready skid) on forward REQ_R
    parameter EXT_PIPE_REQ_W       = 1,   // ext_pipe (valid-ready skid) on forward REQ_W
    // Default 0 is safe HERE and only here: the bca read side below is itself the
    // register on the reverse border. The core form has no bridge, so it needs a
    // real 1 -- see lb_tniu_int_core.v and lb_ir._check_same_domain_pipes.
    parameter EXT_PIPE_RSP_RD          = 0,   // ext_pipe (valid-ready skid) on reverse RSP_RD
    parameter EXT_PIPE_RSP_WR          = 0,   // ext_pipe (valid-ready skid) on reverse RSP_WR
    parameter INT_PIPE_REQ_R       = 1,   // int_pipe (credit register) on forward REQ_R
    parameter INT_PIPE_REQ_W       = 1,   // int_pipe (credit register) on forward REQ_W
    parameter INT_PIPE_RSP_RD          = 1,   // int_pipe (credit register) on reverse RSP_RD
    parameter INT_PIPE_RSP_WR          = 1,   // int_pipe (credit register) on reverse RSP_WR
    // ==== credit configuration of the internal interface ====
    parameter INT_REQ_R_CREDIT_FIFO = 4,  // forward REQ_R ingress FIFO depth = upstream credit budget
    parameter INT_REQ_W_CREDIT_FIFO = 4,  // forward REQ_W ingress FIFO depth = upstream credit budget
    parameter INT_RSP_RD_CREDIT        = 4,   // reverse RSP_RD egress credit = peer ingress FIFO depth
    parameter INT_RSP_WR_CREDIT        = 4,   // reverse RSP_WR egress credit = peer ingress FIFO depth
    // ==== internal interface (fabric side, credit) ====
    parameter INT_ADDR_LOCAL_WIDTH = 32,  // in-network local (rebased) address width
    parameter INT_DEST_ID_WIDTH    = 4,   // = log2(NUM_SLAVES), injected by top
    parameter INT_SRC_ID_WIDTH     = 4,   // = log2(NUM_MASTERS), injected by top
    parameter INT_ID_WIDTH         = 8,   // in-network unified int_id width
    parameter INT_USER_WIDTH_CMD   = 8,   // in-network CMD user width (>= EXT_USER_WIDTH_CMD)
    parameter INT_USER_WIDTH_RSP_RD    = 8,   // in-network RSP_RD  user width (>= EXT_USER_WIDTH_RSP_RD)
    parameter INT_USER_WIDTH_RSP_WR    = 8,   // in-network RSP_WR  user width (>= EXT_USER_WIDTH_RSP_WR)
    parameter INT_TOTBYTES_W       = 16,  // in-network unified total_bytes width
    parameter INT_LANE_W           = 7,   // addr_lane width (carried in the RSP_RD flit)
    parameter INT_MOD_W            = 0,   // REQ_W modifier segment width (bus max), 0 = absent (Adv 15)
    parameter INT_QOS_W            = 0,   // QoS segment width (bus max), 0 = absent; tops the CMD flit
    // ==== structural / topology ====
    parameter WFRAG_DW             = EXT_DATA_WIDTH,  // W_frag: max data_width among Masters on this Slave
    parameter BCA_DEPTH            = 8,   // bca async FIFO depth (power of 2)
    parameter SYNC_STAGES          = 2,   // bca gray-code synchronizer stages (>=2)
    // ---- derived parameters; used in the port list, so they must stay in the
    // ---- parameter list (V2001 body localparams cannot size ports). Not to be
    // ---- overridden externally -- the defaults are the correct values.
    parameter EXT_STRB_WIDTH = EXT_DATA_WIDTH/8,
    // CMD flit uses in-network widths (INT_ID / INT_USER / INT_TOTBYTES)
    parameter INT_CMD_FLIT_W = INT_QOS_W + `LB_OPCODE_WIDTH + INT_ADDR_LOCAL_WIDTH + INT_TOTBYTES_W +
                               INT_USER_WIDTH_CMD + INT_ID_WIDTH + INT_DEST_ID_WIDTH + INT_SRC_ID_WIDTH,
    parameter INT_WD_FLIT_W  = 1 + EXT_STRB_WIDTH + EXT_DATA_WIDTH +
                               INT_DEST_ID_WIDTH + INT_SRC_ID_WIDTH,
    // REQ_W = CMD + MOD (Adv 15, absent at INT_MOD_W = 0) + WD; REQ_R carries
    // the CMD field alone (INT_CMD_FLIT_W).
    parameter INT_REQ_FLIT_W = INT_CMD_FLIT_W + INT_MOD_W + INT_WD_FLIT_W,
    // RSP_RD flit (low->high): data, last, trans_last, user, resp, int_id,
    //                      total_bytes, addr_lane, qos, src_id
    parameter INT_RSP_RD_FLIT_W  = INT_SRC_ID_WIDTH + INT_QOS_W + INT_LANE_W + INT_TOTBYTES_W + INT_ID_WIDTH +
                               `LB_RESP_WIDTH + INT_USER_WIDTH_RSP_RD + 1 + 1 + EXT_DATA_WIDTH,
    // RSP_WR flit (low->high): user, resp, int_id, qos, src_id
    parameter INT_RSP_WR_FLIT_W  = INT_SRC_ID_WIDTH + INT_QOS_W + INT_ID_WIDTH +
                               `LB_RESP_WIDTH + INT_USER_WIDTH_RSP_WR,
    // external CMD payload leads with this Slave's own qos slice (EXT_QOS_W).
    parameter EXT_CMD_W  = EXT_QOS_W + `LB_OPCODE_WIDTH + INT_ADDR_LOCAL_WIDTH + EXT_LEN_WIDTH +
                           EXT_TXNID_WIDTH + EXT_USER_WIDTH_CMD,
    parameter EXT_WD_W   = EXT_DATA_WIDTH + EXT_STRB_WIDTH + 1 + EXT_TXNID_WIDTH,
    parameter EXT_REQ_W  = EXT_CMD_W + EXT_MOD_W + EXT_WD_W,
    parameter EXT_RSP_RD_W   = EXT_TXNID_WIDTH + `LB_RESP_WIDTH + EXT_USER_WIDTH_RSP_RD + 1 + EXT_DATA_WIDTH,
    parameter EXT_RSP_WR_W   = EXT_TXNID_WIDTH + `LB_RESP_WIDTH + EXT_USER_WIDTH_RSP_WR,
    parameter AW_BCA     = (BCA_DEPTH <= 2)   ? 1 :   // bca pointer width = log2(BCA_DEPTH)
                           (BCA_DEPTH <= 4)   ? 2 :
                           (BCA_DEPTH <= 8)   ? 3 :
                           (BCA_DEPTH <= 16)  ? 4 :
                           (BCA_DEPTH <= 32)  ? 5 : 6
) (
    //--------- inputs: clock / reset (network clock domain) ---------
    input wire                        clk,                 // network-side clock
    input wire                        rst_n,               // async reset, active low
    //--------- inputs: internal interface (fabric side, packed flit + credit) ---------
    input wire  [INT_CMD_FLIT_W-1:0]  i_req_r_flit,        // REQ_R flit in (CMD only)
    input wire                        i_req_r_valid,       // REQ_R flit valid
    input wire  [INT_REQ_FLIT_W-1:0]  i_req_w_flit,        // REQ_W flit in (CMD + WD)
    input wire                        i_req_w_valid,       // REQ_W flit valid
    input wire                        o_rsp_rd_credit_ret, // RSP_RD credit returned by the peer
    input wire                        o_rsp_wr_credit_ret, // RSP_WR credit returned by the peer
    //--------- outputs: internal interface (fabric side, packed flit + credit) ---------
    output wire                       i_req_r_credit_ret,  // REQ_R credit returned to the peer
    output wire                       i_req_w_credit_ret,  // REQ_W credit returned to the peer
    output wire [INT_RSP_RD_FLIT_W-1:0]   o_rsp_rd_flit,   // RSP_RD flit out
    output wire                       o_rsp_rd_valid,      // RSP_RD flit valid
    output wire [INT_RSP_WR_FLIT_W-1:0]   o_rsp_wr_flit,   // RSP_WR flit out
    output wire                       o_rsp_wr_valid,      // RSP_WR flit valid
    //--------- inputs: bca async border (from lb_tniu_ext_bca) ---------
    // forward REQ_R/REQ_W: this half is slv -> o_wrcnt / o_rdata out, i_rdcnt / i_rdptr in
    // reverse RSP_RD/RSP_WR: this half is mst -> i_wrcnt / i_rdata in, o_rdcnt / o_rdptr out
    input wire  [AW_BCA:0]            b_req_r_rc,          // REQ_R bca read count, gray coded
    input wire  [AW_BCA-1:0]          b_req_r_rp,          // REQ_R bca read pointer
    input wire  [AW_BCA:0]            b_req_w_rc,          // REQ_W bca read count, gray coded
    input wire  [AW_BCA-1:0]          b_req_w_rp,          // REQ_W bca read pointer
    input wire  [AW_BCA:0]            b_rsp_rd_wc,         // RSP_RD  bca write count, gray coded
    input wire  [EXT_RSP_RD_W-1:0]        b_rsp_rd_bd,     // RSP_RD  bca read data
    input wire  [AW_BCA:0]            b_rsp_wr_wc,         // RSP_WR  bca write count, gray coded
    input wire  [EXT_RSP_WR_W-1:0]        b_rsp_wr_bd,     // RSP_WR  bca read data
    //--------- outputs: bca async border (to lb_tniu_ext_bca) ---------
    output wire [AW_BCA:0]            b_req_r_wc,          // REQ_R bca write count, gray coded
    output wire [EXT_CMD_W-1:0]       b_req_r_bd,          // REQ_R bca read data
    output wire [AW_BCA:0]            b_req_w_wc,          // REQ_W bca write count, gray coded
    output wire [EXT_REQ_W-1:0]       b_req_w_bd,          // REQ_W bca read data
    output wire [AW_BCA:0]            b_rsp_rd_rc,         // RSP_RD  bca read count, gray coded
    output wire [AW_BCA-1:0]          b_rsp_rd_rp,         // RSP_RD  bca read pointer
    output wire [AW_BCA:0]            b_rsp_wr_rc,         // RSP_WR  bca read count, gray coded
    output wire [AW_BCA-1:0]          b_rsp_wr_rp          // RSP_WR  bca read pointer
);
    //------------------------------------------------------------------------
    // Declarations (all up front, one per line)
    //------------------------------------------------------------------------
    wire [EXT_CMD_W-1:0]        ep_rq_r_pk;       // forward REQ_R out of the core, into bca
    wire                        ep_rq_r_v;        // forward REQ_R valid
    wire                        ep_rq_r_r;        // forward REQ_R ready (backpressure) from bca
    wire [EXT_REQ_W-1:0]        ep_req_pk;        // forward REQ_W out of the core, into bca
    wire                        ep_req_v;         // forward REQ_W valid
    wire                        ep_req_r;         // forward REQ_W ready (backpressure) from bca
    wire [EXT_RSP_RD_W-1:0]         cr_rsp_rd_pk; // reverse RSP_RD out of bca, into the core
    wire                        cr_rsp_rd_v;      // reverse RSP_RD valid
    wire                        cr_rsp_rd_r;      // reverse RSP_RD ready (backpressure) from the core
    wire [EXT_RSP_WR_W-1:0]         cr_rsp_wr_pk; // reverse RSP_WR out of bca, into the core
    wire                        cr_rsp_wr_v;      // reverse RSP_WR valid
    wire                        cr_rsp_wr_r;      // reverse RSP_WR ready (backpressure) from the core

    //------------------------------------------------------------------------
    // Core: the whole forward and reverse pipeline. Everything but the bridge.
    //------------------------------------------------------------------------
    lb_tniu_int_core #(
        .EXT_DATA_WIDTH       (EXT_DATA_WIDTH),
        .EXT_LEN_WIDTH        (EXT_LEN_WIDTH),
        .EXT_USER_WIDTH_CMD   (EXT_USER_WIDTH_CMD),
        .EXT_USER_WIDTH_RSP_RD    (EXT_USER_WIDTH_RSP_RD),
        .EXT_USER_WIDTH_RSP_WR    (EXT_USER_WIDTH_RSP_WR),
        .EXT_TXNID_WIDTH      (EXT_TXNID_WIDTH),
        .EXT_MOD_W            (EXT_MOD_W),
        .EXT_QOS_W            (EXT_QOS_W),
        .EXT_PENDING_TRANS    (EXT_PENDING_TRANS),
        .EXT_RSP_RD_INTERLEAVE    (EXT_RSP_RD_INTERLEAVE),
        .EXT_ATOMIC_EN        (EXT_ATOMIC_EN),
        .EXT_PIPE_REQ_R       (EXT_PIPE_REQ_R),
        .EXT_PIPE_REQ_W       (EXT_PIPE_REQ_W),
        .EXT_PIPE_RSP_RD          (EXT_PIPE_RSP_RD),
        .EXT_PIPE_RSP_WR          (EXT_PIPE_RSP_WR),
        .INT_PIPE_REQ_R       (INT_PIPE_REQ_R),
        .INT_PIPE_REQ_W       (INT_PIPE_REQ_W),
        .INT_PIPE_RSP_RD          (INT_PIPE_RSP_RD),
        .INT_PIPE_RSP_WR          (INT_PIPE_RSP_WR),
        .INT_REQ_R_CREDIT_FIFO (INT_REQ_R_CREDIT_FIFO),
        .INT_REQ_W_CREDIT_FIFO (INT_REQ_W_CREDIT_FIFO),
        .INT_RSP_RD_CREDIT        (INT_RSP_RD_CREDIT),
        .INT_RSP_WR_CREDIT        (INT_RSP_WR_CREDIT),
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
        .WFRAG_DW             (WFRAG_DW)
    ) u10_core (
        .clk                  (clk),
        .rst_n                (rst_n),
        .i_req_r_flit         (i_req_r_flit),
        .i_req_r_valid        (i_req_r_valid),
        .i_req_w_flit         (i_req_w_flit),
        .i_req_w_valid        (i_req_w_valid),
        .o_rsp_rd_credit_ret      (o_rsp_rd_credit_ret),
        .o_rsp_wr_credit_ret      (o_rsp_wr_credit_ret),
        .i_req_r_credit_ret   (i_req_r_credit_ret),
        .i_req_w_credit_ret   (i_req_w_credit_ret),
        .o_rsp_rd_flit            (o_rsp_rd_flit),
        .o_rsp_rd_valid           (o_rsp_rd_valid),
        .o_rsp_wr_flit            (o_rsp_wr_flit),
        .o_rsp_wr_valid           (o_rsp_wr_valid),
        .vr_req_r_ready       (ep_rq_r_r),
        .vr_req_w_ready       (ep_req_r),
        .vr_rsp_rd_data           (cr_rsp_rd_pk),
        .vr_rsp_rd_valid          (cr_rsp_rd_v),
        .vr_rsp_wr_data           (cr_rsp_wr_pk),
        .vr_rsp_wr_valid          (cr_rsp_wr_v),
        .vr_req_r_data        (ep_rq_r_pk),
        .vr_req_r_valid       (ep_rq_r_v),
        .vr_req_w_data        (ep_req_pk),
        .vr_req_w_valid       (ep_req_v),
        .vr_rsp_rd_ready          (cr_rsp_rd_r),
        .vr_rsp_wr_ready          (cr_rsp_wr_r)
    );

    //------------------------------------------------------------------------
    // forward REQ_R / REQ_W: one bca_slv each (clk write side). Two bridges is
    // what keeps the two request channels independent across the crossing too.
    //------------------------------------------------------------------------
    lb_bca_slv #(
        .WIDTH        (EXT_CMD_W),
        .DEPTH        (BCA_DEPTH),
        .SYNC_STAGES  (SYNC_STAGES)
    ) u20_slv_req_r (
        .clk_w         (clk),
        .wrst_n       (rst_n),
        .w_data       (ep_rq_r_pk),
        .w_valid      (ep_rq_r_v),
        .i_rdcnt_gray (b_req_r_rc),
        .i_rdptr      (b_req_r_rp),
        .w_ready      (ep_rq_r_r),
        .o_wrcnt_gray (b_req_r_wc),
        .o_rdata      (b_req_r_bd)
    );

    lb_bca_slv #(
        .WIDTH        (EXT_REQ_W),
        .DEPTH        (BCA_DEPTH),
        .SYNC_STAGES  (SYNC_STAGES)
    ) u21_slv_req_w (
        .clk_w         (clk),
        .wrst_n       (rst_n),
        .w_data       (ep_req_pk),
        .w_valid      (ep_req_v),
        .i_rdcnt_gray (b_req_w_rc),
        .i_rdptr      (b_req_w_rp),
        .w_ready      (ep_req_r),
        .o_wrcnt_gray (b_req_w_wc),
        .o_rdata      (b_req_w_bd)
    );

    //------------------------------------------------------------------------
    // reverse RSP_RD/RSP_WR: bca_mst (clk read side)
    //------------------------------------------------------------------------
    lb_bca_mst #(
        .WIDTH        (EXT_RSP_RD_W),
        .DEPTH        (BCA_DEPTH),
        .SYNC_STAGES  (SYNC_STAGES)
    ) u30_mst_rsp_rd (
        .clk_r         (clk),
        .rrst_n       (rst_n),
        .r_ready      (cr_rsp_rd_r),
        .i_wrcnt_gray (b_rsp_rd_wc),
        .i_rdata      (b_rsp_rd_bd),
        .r_data       (cr_rsp_rd_pk),
        .r_valid      (cr_rsp_rd_v),
        .o_rdcnt_gray (b_rsp_rd_rc),
        .o_rdptr      (b_rsp_rd_rp)
    );

    lb_bca_mst #(
        .WIDTH        (EXT_RSP_WR_W),
        .DEPTH        (BCA_DEPTH),
        .SYNC_STAGES  (SYNC_STAGES)
    ) u31_mst_rsp_wr (
        .clk_r         (clk),
        .rrst_n       (rst_n),
        .r_ready      (cr_rsp_wr_r),
        .i_wrcnt_gray (b_rsp_wr_wc),
        .i_rdata      (b_rsp_wr_bd),
        .r_data       (cr_rsp_wr_pk),
        .r_valid      (cr_rsp_wr_v),
        .o_rdcnt_gray (b_rsp_wr_rc),
        .o_rdptr      (b_rsp_wr_rp)
    );

endmodule
