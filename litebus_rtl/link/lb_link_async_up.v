//============================================================================
// Filename    : lb_link_async_up.v
// Author      : litebus
// Description : async LINK upstream half (clk_w domain, independently placeable)
// Date        : 2026-08-15
// Revision    : 1.0 initial -- per-round changes are in doc/HISTORY.md
//----------------------------------------------------------------------------
// The clk_w half of what lb_link_top builds when CREDIT_NODE=1 and ASYNC=1:
//
//   lb_link_top(ASYNC=1)  ==  lb_link_async_up  +  lb_link_async_down
//
// Same four leaves, same instance numbers, so this file and lb_link_async_down.v
// read side by side against lb_link_top.v's g_async branch. What the pair buys is
// PLACEMENT: the two halves can sit in different levels of the generated module
// hierarchy, which lb_link_top cannot express because a V2001 port list is fixed
// at elaboration (CODING_STYLE 4A.3 -- different ports means different module
// names, never a parameter).
//
// Only four signals cross to the down half, exactly the bca border:
//   o_wrcnt_gray : up->down, write-count gray code                    [true CDC]
//   i_rdcnt_gray : down->up, read-count gray code                     [true CDC]
//   i_rdptr      : down->up, read address, drives the regfile mux     [non-CDC*]
//   o_rdata      : up->down, regfile read data                        [non-CDC*]
//
// *i_rdptr / o_rdata are a COMBINATIONAL loop that now crosses a module boundary
//  and may cross a hierarchy level. It must still settle inside one clk_r period
//  (see lb_bca_slv.v). Never insert a synchronizer there. This is the one cost of
//  splitting the halves, and it is invisible in RTL simulation -- the generator
//  therefore lists every split link in ir_report.txt so it reaches STA.
//
// The storage (ingress FIFO and bca regfile) lives entirely on this side.
//============================================================================
`include "lb_defines.vh"

module lb_link_async_up #(
    parameter FLIT_W        = 64,            // flit width
    parameter INGRESS_DEPTH = 4,             // ingress FIFO depth = credit to upstream
    parameter BCA_DEPTH     = 8,             // bca async FIFO depth (power of 2)
    parameter SYNC_STAGES   = 2,             // bca gray-code synchronizer stages
    // ---- derived parameters; used in the port list, so they must stay in the
    // ---- parameter list (V2001 body localparams cannot size ports). Not to be
    // ---- overridden externally -- the default is the correct value. The ladder
    // ---- stops at 6 because config/schema.yaml caps bca_depth at 64; over that
    // ---- shared range it agrees with lb_bca_slv's own AW, which runs to 8.
    parameter AW_BCA = (BCA_DEPTH <= 2)  ? 1 :   // bca pointer / gray width = log2(BCA_DEPTH)
                       (BCA_DEPTH <= 4)  ? 2 :
                       (BCA_DEPTH <= 8)  ? 3 :
                       (BCA_DEPTH <= 16) ? 4 :
                       (BCA_DEPTH <= 32) ? 5 : 6
) (
    // ---- inputs (upstream, clk_w domain) ----
    input wire                 clk_w,         // upstream clock
    input wire                 wrst_n,        // upstream async reset, active low
    input wire  [FLIT_W-1:0]   in_flit,       // forward flit
    input wire                 in_valid,      // forward valid
    // ---- inputs (bca border, from lb_link_async_down) ----
    input wire  [AW_BCA:0]     i_rdcnt_gray,  // RdCnt gray code, down->up (true CDC, synced inside)
    input wire  [AW_BCA-1:0]   i_rdptr,       // RdPtr, down->up (combinational, drives read mux)
    // ---- outputs (upstream, clk_w domain) ----
    output wire                in_credit_ret, // credit return to upstream (clk_w local)
    // ---- outputs (bca border, to lb_link_async_down) ----
    output wire [AW_BCA:0]     o_wrcnt_gray,  // WrCnt gray code, up->down (true CDC)
    output wire [FLIT_W-1:0]   o_rdata        // regfile read data, up->down (combinational)
);
    //------------------------------------------------------------------------
    // Declarations (all up front, one per line)
    //------------------------------------------------------------------------
    wire [FLIT_W-1:0] ing_data; // ingress output flit
    wire              ing_v;    // ingress output valid
    wire              ing_r;    // ingress output ready

    //------------------------------------------------------------------------
    // ingress on clk_w (meets upstream egress). Instance number kept from
    // lb_link_top's g_node so the two files stay readable against each other.
    //------------------------------------------------------------------------
    lb_credit_ingress #(
        .WIDTH         (FLIT_W),         // flit width
        .DEPTH         (INGRESS_DEPTH)   // ingress FIFO depth
    ) u20_ing (
        .clk           (clk_w),          // clock
        .rst_n         (wrst_n),         // reset
        .in_data       (in_flit),        // input flit
        .in_valid      (in_valid),       // input valid
        .out_ready     (ing_r),          // output ready
        .out_data      (ing_data),       // output flit
        .out_valid     (ing_v),          // output valid
        .credit_return (in_credit_ret)   // credit to upstream (clk_w local)
    );

    //------------------------------------------------------------------------
    // bca write side: the regfile and the write pointer live here
    //------------------------------------------------------------------------
    lb_bca_slv #(
        .WIDTH         (FLIT_W),         // flit width
        .DEPTH         (BCA_DEPTH),      // bca depth
        .SYNC_STAGES   (SYNC_STAGES)     // sync stages
    ) u40_slv (
        .clk_w         (clk_w),          // write clock
        .wrst_n        (wrst_n),         // write reset
        .w_data        (ing_data),       // write payload
        .w_valid       (ing_v),          // write valid
        .i_rdcnt_gray  (i_rdcnt_gray),   // RdCnt gray in
        .i_rdptr       (i_rdptr),        // RdPtr in
        .w_ready       (ing_r),          // write ready
        .o_wrcnt_gray  (o_wrcnt_gray),   // WrCnt gray out
        .o_rdata       (o_rdata)         // regfile data out
    );

endmodule
