//============================================================================
// Filename    : lb_link_top.v
// Author      : litebus
// Description : LINK top (based function model, unified credit interface)
// Date        : 2026-08-06
// Revision    : 1.0 initial -- per-round changes are in doc/HISTORY.md
//----------------------------------------------------------------------------
// LINK is an in-network transport path speaking Litebus flit + credit:
//   forward  in_flit + in_valid ;  reverse in_credit_ret  (LINK -> upstream)
//   forward  out_flit + out_valid; reverse out_credit_ret (downstream -> LINK)
//
// Three capabilities (selected by CREDIT_NODE / ASYNC):
//   (a) CREDIT_NODE=0          : credit pipelining (like int_pipe). Forward
//       payload+valid pipelined, reverse credit_return pipelined; no buffer,
//       one end-to-end credit (initial credit must cover round-trip). Single clk.
//   (b) CREDIT_NODE=1, ASYNC=0 : credit node. ingress meets upstream egress
//       (take data, return credit), internal FIFO, egress sends downstream
//       (holds downstream credit). Two segmented credit loops, credit does not
//       cross domains. Single clk.
//   (c) CREDIT_NODE=1, ASYNC=1 : (b) + async. ingress on clk_w, egress on clk_r,
//       data crosses via bca(slv/mst); credit segment 1 on clk_w, segment 2 on
//       clk_r, each looping locally; only bca internals cross domains.
//
// Instantiates bca_slv + bca_mst directly (no bca_top wrapper). Synthesizable.
//============================================================================
`include "lb_defines.vh"

module lb_link_top #(
    parameter FLIT_W        = 64,            // flit width
    parameter CREDIT_NODE   = 0,             // 0 = pipe (a); 1 = credit node (b/c)
    parameter ASYNC         = 0,             // 0 = single clk; 1 = cross async (c)
    parameter IN_PIPE       = 1,             // pipeline stages for mode (a)
    parameter INGRESS_DEPTH = 4,             // ingress FIFO depth = credit to upstream
    parameter EGRESS_CREDIT = 4,             // egress credit = downstream ingress FIFO depth
    parameter BCA_DEPTH     = 8,             // bca async FIFO depth (power of 2)
    parameter SYNC_STAGES   = 2              // bca gray-code synchronizer stages
) (
    // ---- inputs (upstream, clk_w domain) ----
    input wire                 clk_w,          // upstream clock
    input wire                 wrst_n,         // upstream async reset, active low
    input wire  [FLIT_W-1:0]   in_flit,        // forward flit
    input wire                 in_valid,       // forward valid
    // ---- inputs (downstream, clk_r domain; tie clk_r=clk_w when ASYNC=0) ----
    input wire                 clk_r,          // downstream clock
    input wire                 rrst_n,         // downstream async reset, active low
    input wire                 out_credit_ret, // credit return from downstream
    // ---- outputs ----
    output wire                in_credit_ret,  // credit return to upstream (LINK -> upstream)
    output wire [FLIT_W-1:0]   out_flit,       // forward flit out
    output wire                out_valid       // forward valid out
);
    //------------------------------------------------------------------------
    // Derived local params (AW from BCA_DEPTH)
    //------------------------------------------------------------------------
    localparam AW = (BCA_DEPTH <= 2)  ? 1 :        // bca pointer/gray width = log2(BCA_DEPTH)
                    (BCA_DEPTH <= 4)  ? 2 :
                    (BCA_DEPTH <= 8)  ? 3 :
                    (BCA_DEPTH <= 16) ? 4 :
                    (BCA_DEPTH <= 32) ? 5 : 6;

    generate
    if (CREDIT_NODE == 0) begin : g_pipe
        //--------------------------------------------------------------------
        // (a) credit pipelining: forward payload+valid, reverse credit
        //--------------------------------------------------------------------
        lb_iniu_int_pipe #(
            .WIDTH             (FLIT_W),      // flit width
            .ENABLE            (1),           // enable pipeline
            .STAGES            (IN_PIPE)      // pipeline stages
        ) u10_pipe (
            .clk               (clk_w),        // clock
            .rst_n             (wrst_n),      // reset
            .in_data           (in_flit),     // forward flit
            .in_valid          (in_valid),    // forward valid
            .out_credit_return (out_credit_ret),  // credit from downstream
            .in_credit_return  (in_credit_ret),   // credit to upstream
            .out_data          (out_flit),    // forward flit out
            .out_valid         (out_valid)    // forward valid out
        );
    end else begin : g_node
        //--------------------------------------------------------------------
        // (b)/(c) credit node: ingress -> [bca] -> egress
        //--------------------------------------------------------------------
        wire [FLIT_W-1:0] ing_data; // ingress output flit
        wire              ing_v;    // ingress output valid
        wire              ing_r;    // ingress output ready

        // ingress on clk_w (meets upstream egress)
        lb_credit_ingress #(
            .WIDTH         (FLIT_W),         // flit width
            .DEPTH         (INGRESS_DEPTH)   // ingress FIFO depth
        ) u20_ing (
            .clk           (clk_w),           // clock
            .rst_n         (wrst_n),         // reset
            .in_data       (in_flit),        // input flit
            .in_valid      (in_valid),       // input valid
            .out_ready     (ing_r),          // output ready
            .out_data      (ing_data),       // output flit
            .out_valid     (ing_v),          // output valid
            .credit_return (in_credit_ret)   // credit to upstream (clk_w local)
        );

        if (ASYNC == 0) begin : g_sync
            //----------------------------------------------------------------
            // (b) same domain: ingress -> egress
            //----------------------------------------------------------------
            lb_credit_egress #(
                .WIDTH         (FLIT_W),         // flit width
                .CREDIT_INIT   (EGRESS_CREDIT)   // initial credit
            ) u30_eg (
                .clk           (clk_w),           // clock
                .rst_n         (wrst_n),         // reset
                .in_data       (ing_data),       // input flit
                .in_valid      (ing_v),          // input valid
                .credit_return (out_credit_ret), // credit from downstream (clk_w local)
                .in_ready      (ing_r),          // input ready
                .out_data      (out_flit),       // output flit
                .out_valid     (out_valid)       // output valid
            );
        end else begin : g_async
            //----------------------------------------------------------------
            // (c) cross async: ingress(clk_w) -> bca(slv|mst) -> egress(clk_r)
            //----------------------------------------------------------------
            wire [AW:0]       wrcnt_g;    // WrCnt gray (bca border)
            wire [AW:0]       rdcnt_g;    // RdCnt gray (bca border)
            wire [AW-1:0]     rdptr;      // RdPtr (bca border, combinational)
            wire [FLIT_W-1:0] bdata;      // regfile data (bca border, combinational)
            wire [FLIT_W-1:0] cross_data; // bca_mst output flit
            wire              cross_v;    // bca_mst output valid
            wire              cross_r;    // bca_mst output ready

            lb_bca_slv #(
                .WIDTH         (FLIT_W),         // flit width
                .DEPTH         (BCA_DEPTH),      // bca depth
                .SYNC_STAGES   (SYNC_STAGES)     // sync stages
            ) u40_slv (
                .clk_w          (clk_w),           // write clock
                .wrst_n        (wrst_n),         // write reset
                .w_data        (ing_data),       // write payload
                .w_valid       (ing_v),          // write valid
                .i_rdcnt_gray  (rdcnt_g),        // RdCnt gray in
                .i_rdptr       (rdptr),          // RdPtr in
                .w_ready       (ing_r),          // write ready
                .o_wrcnt_gray  (wrcnt_g),        // WrCnt gray out
                .o_rdata       (bdata)           // regfile data out
            );
            lb_bca_mst #(
                .WIDTH         (FLIT_W),         // flit width
                .DEPTH         (BCA_DEPTH),      // bca depth
                .SYNC_STAGES   (SYNC_STAGES)     // sync stages
            ) u50_mst (
                .clk_r          (clk_r),           // read clock
                .rrst_n        (rrst_n),         // read reset
                .r_ready       (cross_r),        // read ready
                .i_wrcnt_gray  (wrcnt_g),        // WrCnt gray in
                .i_rdata       (bdata),          // regfile data in
                .r_data        (cross_data),     // read payload
                .r_valid       (cross_v),        // read valid
                .o_rdcnt_gray  (rdcnt_g),        // RdCnt gray out
                .o_rdptr       (rdptr)           // RdPtr out
            );

            // egress on clk_r (meets downstream ingress)
            lb_credit_egress #(
                .WIDTH         (FLIT_W),         // flit width
                .CREDIT_INIT   (EGRESS_CREDIT)   // initial credit
            ) u60_eg (
                .clk           (clk_r),           // clock
                .rst_n         (rrst_n),         // reset
                .in_data       (cross_data),     // input flit
                .in_valid      (cross_v),        // input valid
                .credit_return (out_credit_ret), // credit from downstream (clk_r local)
                .in_ready      (cross_r),        // input ready
                .out_data      (out_flit),       // output flit
                .out_valid     (out_valid)       // output valid
            );
        end
    end
    endgenerate
endmodule
