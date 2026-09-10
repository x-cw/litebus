//============================================================================
// Filename    : lb_bca_slv.v
// Author      : litebus
// Description : BCA write side (RxUnit, single clk_w domain, hardenable)
// Date        : 2026-08-06
// Revision    : 1.0 initial -- per-round changes are in doc/HISTORY.md
//----------------------------------------------------------------------------
// Implements the RxUnit: an nWord regfile (write port on clk_w) + combinational
// read mux + write pointer + send permission (w_ready = !full).
// The regfile is encapsulated here and not exposed.
//
// Only four signals cross to the mst side:
//   o_wrcnt_gray : slv->mst, write-count gray code (mst syncs it to test empty)   [true CDC]
//   i_rdcnt_gray : mst->slv, read-count gray code  (synced here to test full)      [true CDC]
//   i_rdptr      : mst->slv, read address (combinationally drives regfile mux)   [non-CDC*]
//   o_rdata      : slv->mst, regfile read data (combinational)                   [non-CDC*]
//
// *RdPtr/Data: i_rdptr is an clk_r-domain signal that combinationally drives the
//  regfile read mux here, selecting o_rdata back to mst, sampled in the same
//  clk_r cycle. Not CDC, not MCP. Constrain the combinational delay < 1 clk_r
//  period; never insert a synchronizer here.
//
// Only WrCnt/RdCnt gray-coded counts truly cross the synchronizer (S).
//============================================================================
`include "lb_defines.vh"

module lb_bca_slv #(
    parameter WIDTH       = 32,              // data width of each FIFO word (payload bit width)
    parameter DEPTH       = 8,               // FIFO depth in words (must be power of 2)
    parameter SYNC_STAGES = 2,               // synchronizer flop stages for gray-code CDC (>=2)
    // ---- derived parameter; must be declared BEFORE the port list because the
    // ---- port widths reference it. Do not override it externally.
    parameter AW          = (DEPTH <= 2)   ? 1 :   // pointer / gray width = log2(DEPTH); counts use AW+1 bits
                            (DEPTH <= 4)   ? 2 :
                            (DEPTH <= 8)   ? 3 :
                            (DEPTH <= 16)  ? 4 :
                            (DEPTH <= 32)  ? 5 :
                            (DEPTH <= 64)  ? 6 :
                            (DEPTH <= 128) ? 7 : 8
) (
    // ---- inputs (write clock domain) ----
    input wire                clk_w,        // write-side clock
    input wire                wrst_n,       // write-side async reset, active low
    input wire  [WIDTH-1:0]   w_data,       // write payload
    input wire                w_valid,      // write valid (forward)
    // ---- inputs (boundary from mst) ----
    input wire  [AW:0]        i_rdcnt_gray, // RdCnt gray code, mst->slv (true CDC, synced here)
    input wire  [AW-1:0]      i_rdptr,      // RdPtr, mst->slv (combinational, drives read mux)
    // ---- outputs (write clock domain) ----
    output wire               w_ready,      // write ready (space available)
    // ---- outputs (boundary to mst) ----
    output wire [AW:0]        o_wrcnt_gray, // WrCnt gray code, slv->mst (true CDC)
    output wire [WIDTH-1:0]   o_rdata       // regfile read data, slv->mst (combinational)
);
    //------------------------------------------------------------------------
    // Declarations (all up front, one per line). Gray-code conversion is done
    // inline, no function (CODING_STYLE 6.1):
    //   bin -> gray : g = b ^ (b>>1)
    // There is no gray -> bin conversion any more; see the full test below.
    //------------------------------------------------------------------------
    integer          s;                                 // sync-loop index
    reg  [WIDTH-1:0] regfile [0:DEPTH-1];               // nWord regfile (storage lives in slv)
    reg  [AW:0]      wbin;                              // write pointer (binary, AW+1 bits with wrap)
    reg  [AW:0]      wgray;                             // write count in gray code (= WrCnt)
    reg  [AW:0]      rdcnt_gray_sync [0:SYNC_STAGES-1]; // RdCnt gray synchronized into write domain
    wire [AW:0]      rg;                                // last synchronizer stage, as a plain vector
    wire             full;                              // FIFO full (occ == DEPTH)
    wire             w_fire;                            // write handshake fire

    //------------------------------------------------------------------------
    // Combinational status
    //------------------------------------------------------------------------
    // A plain vector copy of the last sync stage: Verilog-2001 has no part-select
    // on an array element, and the full test below needs one.
    assign rg = rdcnt_gray_sync[SYNC_STAGES-1];

    //------------------------------------------------------------------------
    // Full, tested directly in the gray domain (the standard async-FIFO form).
    //
    // Full means the write count is exactly DEPTH ahead of the synced read count,
    // i.e. in binary  wbin[AW] != rbin[AW]  &&  wbin[AW-1:0] == rbin[AW-1:0].
    // Because gray[AW] = bin[AW] and gray[AW-1] = bin[AW]^bin[AW-1], that maps
    // bit-for-bit onto "the top TWO gray bits differ and the rest match" -- so the
    // comparison can be made without ever converting to binary. It is exactly the
    // same function, valid because DEPTH is a power of two (line 25).
    //
    // What this replaces: an AW-deep chained XOR (gray->bin) feeding an (AW+1)-bit
    // subtract feeding an equality compare, all of it in front of w_ready and
    // therefore in front of the upstream credit path. The mst side had always
    // derived `empty` from a single gray compare (lb_bca_mst.v:60); this brings
    // the slv side to the same footing. Measured 15 -> 8 levels at DEPTH=8
    // (tools/timing/logic_depth.py, probe lb_bca_slv).
    //------------------------------------------------------------------------
    generate
    if (AW == 1) begin : g_full_d2
        // DEPTH=2: the "rest" is empty, so both bits simply have to differ
        assign full = (wgray == ~rg);
    end else begin : g_full
        assign full = (wgray == {~rg[AW:AW-1], rg[AW-2:0]});
    end
    endgenerate

    // Ready when the FIFO is not full.
    assign w_ready        = !full;
    assign w_fire         = w_valid && w_ready;

    //------------------------------------------------------------------------
    // RdCnt gray-code synchronizer (receive side = slv)
    //------------------------------------------------------------------------
    always @(posedge clk_w or negedge wrst_n) begin
        if (!wrst_n) begin
            for (s=0; s<SYNC_STAGES; s=s+1) rdcnt_gray_sync[s] <= {(AW+1){1'b0}};
        end
        else begin
            rdcnt_gray_sync[0] <= i_rdcnt_gray;
            for (s=1; s<SYNC_STAGES; s=s+1) rdcnt_gray_sync[s] <= rdcnt_gray_sync[s-1];
        end
    end

    //------------------------------------------------------------------------
    // Write pointer + WrCnt gray code (bin->gray inline: g = b ^ (b>>1))
    //------------------------------------------------------------------------
    always @(posedge clk_w or negedge wrst_n) begin
        if (!wrst_n) begin
            wbin  <= 0;
            wgray <= 0;
        end else if (w_fire) begin
            wbin  <= wbin + 1'b1;
            wgray <= (wbin + 1'b1) ^ ((wbin + 1'b1) >> 1);
        end
    end
    assign o_wrcnt_gray = wgray;

    //------------------------------------------------------------------------
    // Regfile write port (clk_w) and combinational read mux (driven by mst RdPtr)
    //------------------------------------------------------------------------
    always @(posedge clk_w) if (w_fire) begin
        regfile[wbin[AW-1:0]] <= w_data;
    end
    assign o_rdata = regfile[i_rdptr];       // one clk_r-cycle combinational path segment
endmodule
