//============================================================================
// Filename    : lb_link_async_down.v
// Author      : litebus
// Description : async LINK downstream half (clk_r domain, independently placeable)
// Date        : 2026-08-15
// Revision    : 1.0 initial -- per-round changes are in doc/HISTORY.md
//----------------------------------------------------------------------------
// The clk_r half of what lb_link_top builds when CREDIT_NODE=1 and ASYNC=1; see
// lb_link_async_up.v for the whole argument. Same four leaves, same instance
// numbers as lb_link_top's g_async branch.
//
// This half owns no storage: the bca regfile lives in lb_bca_slv on the up side,
// and lb_bca_mst only reads it combinationally through i_rdata. That asymmetry is
// inherited from the bca itself, not introduced by the split.
//
// The i_rdata / o_rdptr pair is the combinational loop that must settle inside one
// clk_r period even when the two halves sit in different hierarchy levels.
//============================================================================
`include "lb_defines.vh"

module lb_link_async_down #(
    parameter FLIT_W        = 64,            // flit width
    parameter EGRESS_CREDIT = 4,             // egress credit = downstream ingress FIFO depth
    parameter BCA_DEPTH     = 8,             // bca async FIFO depth (power of 2)
    parameter SYNC_STAGES   = 2,             // bca gray-code synchronizer stages
    // ---- derived parameters; used in the port list, so they must stay in the
    // ---- parameter list (V2001 body localparams cannot size ports). Not to be
    // ---- overridden externally -- the default is the correct value, and it must
    // ---- match lb_link_async_up's ladder exactly or the border truncates.
    parameter AW_BCA = (BCA_DEPTH <= 2)  ? 1 :   // bca pointer / gray width = log2(BCA_DEPTH)
                       (BCA_DEPTH <= 4)  ? 2 :
                       (BCA_DEPTH <= 8)  ? 3 :
                       (BCA_DEPTH <= 16) ? 4 :
                       (BCA_DEPTH <= 32) ? 5 : 6
) (
    // ---- inputs (downstream, clk_r domain) ----
    input wire                 clk_r,          // downstream clock
    input wire                 rrst_n,         // downstream async reset, active low
    input wire                 out_credit_ret, // credit return from downstream
    // ---- inputs (bca border, from lb_link_async_up) ----
    input wire  [AW_BCA:0]     i_wrcnt_gray,   // WrCnt gray code, up->down (true CDC, synced inside)
    input wire  [FLIT_W-1:0]   i_rdata,        // regfile read data, up->down (combinational)
    // ---- outputs (downstream, clk_r domain) ----
    output wire [FLIT_W-1:0]   out_flit,       // forward flit out
    output wire                out_valid,      // forward valid out
    // ---- outputs (bca border, to lb_link_async_up) ----
    output wire [AW_BCA:0]     o_rdcnt_gray,   // RdCnt gray code, down->up (true CDC)
    output wire [AW_BCA-1:0]   o_rdptr         // RdPtr, down->up (combinational, drives read mux)
);
    //------------------------------------------------------------------------
    // Declarations (all up front, one per line)
    //------------------------------------------------------------------------
    wire [FLIT_W-1:0] cross_data; // bca_mst output flit
    wire              cross_v;    // bca_mst output valid
    wire              cross_r;    // bca_mst output ready

    //------------------------------------------------------------------------
    // bca read side: no storage, reads the up half's regfile combinationally
    //------------------------------------------------------------------------
    lb_bca_mst #(
        .WIDTH         (FLIT_W),         // flit width
        .DEPTH         (BCA_DEPTH),      // bca depth
        .SYNC_STAGES   (SYNC_STAGES)     // sync stages
    ) u50_mst (
        .clk_r         (clk_r),          // read clock
        .rrst_n        (rrst_n),         // read reset
        .r_ready       (cross_r),        // read ready
        .i_wrcnt_gray  (i_wrcnt_gray),   // WrCnt gray in
        .i_rdata       (i_rdata),        // regfile data in
        .r_data        (cross_data),     // read payload
        .r_valid       (cross_v),        // read valid
        .o_rdcnt_gray  (o_rdcnt_gray),   // RdCnt gray out
        .o_rdptr       (o_rdptr)         // RdPtr out
    );

    //------------------------------------------------------------------------
    // egress on clk_r (meets downstream ingress)
    //------------------------------------------------------------------------
    lb_credit_egress #(
        .WIDTH         (FLIT_W),         // flit width
        .CREDIT_INIT   (EGRESS_CREDIT)   // initial credit
    ) u60_eg (
        .clk           (clk_r),          // clock
        .rst_n         (rrst_n),         // reset
        .in_data       (cross_data),     // input flit
        .in_valid      (cross_v),        // input valid
        .credit_return (out_credit_ret), // credit from downstream (clk_r local)
        .in_ready      (cross_r),        // input ready
        .out_data      (out_flit),       // output flit
        .out_valid     (out_valid)       // output valid
    );

endmodule
