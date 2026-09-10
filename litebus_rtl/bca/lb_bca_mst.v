//============================================================================
// Filename    : lb_bca_mst.v
// Author      : litebus
// Description : BCA read side (TxUnit, single clk_r domain, hardenable)
// Date        : 2026-08-06
// Revision    : 1.0 initial -- per-round changes are in doc/HISTORY.md
//----------------------------------------------------------------------------
// Implements the TxUnit: read pointer + empty detection + read-side handshake.
// No storage (the regfile lives in slv).
//
// Only four signals cross to the slv side:
//   i_wrcnt_gray : slv->mst, write-count gray code (synced here to test empty)  [true CDC]
//   o_rdcnt_gray : mst->slv, read-count gray code  (slv syncs it for full)      [true CDC]
//   o_rdptr      : mst->slv, read address (combinationally drives slv read mux) [non-CDC]
//   i_rdata      : slv->mst, data (combinational, sampled here same cycle)      [non-CDC]
//
// RdPtr/Data form one clk_r-cycle combinational path (start = this module's rptr
// register, end = sampling here), borrowing slv's combinational mux; not CDC.
//============================================================================
`include "lb_defines.vh"

module lb_bca_mst #(
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
    // ---- inputs (read clock domain) ----
    input wire                clk_r,        // read-side clock
    input wire                rrst_n,       // read-side async reset, active low
    input wire                r_ready,      // read ready (downstream consumes)
    // ---- inputs (boundary from slv) ----
    input wire  [AW:0]        i_wrcnt_gray, // WrCnt gray code, slv->mst (true CDC, synced here)
    input wire  [WIDTH-1:0]   i_rdata,      // data, slv->mst (combinational, sampled same cycle)
    // ---- outputs (read clock domain) ----
    output wire [WIDTH-1:0]   r_data,       // read payload
    output wire               r_valid,      // read valid (not empty)
    // ---- outputs (boundary to slv) ----
    output wire [AW:0]        o_rdcnt_gray, // RdCnt gray code, mst->slv (true CDC)
    output wire [AW-1:0]      o_rdptr       // RdPtr, mst->slv (combinational, drives slv read mux)
);
    //------------------------------------------------------------------------
    // Declarations (all up front, one per line). Gray-code conversion is done
    // inline, no function (CODING_STYLE 6.1): bin -> gray is g = b ^ (b>>1).
    //------------------------------------------------------------------------
    integer     s;                                 // sync-loop index
    reg  [AW:0] rbin;                              // read pointer (binary, AW+1 bits with wrap)
    reg  [AW:0] rgray;                             // read count in gray code (= RdCnt)
    reg  [AW:0] wrcnt_gray_sync [0:SYNC_STAGES-1]; // WrCnt gray synchronized into read domain
    wire        empty;                             // FIFO empty (read gray == synced write gray)
    wire        r_fire;                            // read handshake fire

    //------------------------------------------------------------------------
    // Combinational status
    //------------------------------------------------------------------------
    assign empty   = (rgray == wrcnt_gray_sync[SYNC_STAGES-1]);
    assign r_valid = !empty;
    assign o_rdptr = rbin[AW-1:0];           // combinational drive of slv read mux
    assign r_data  = i_rdata;                // data returns combinationally, sampled this cycle
    assign r_fire  = r_valid && r_ready;

    //------------------------------------------------------------------------
    // WrCnt gray-code synchronizer (receive side = mst)
    //------------------------------------------------------------------------
    always @(posedge clk_r or negedge rrst_n) begin
        if (!rrst_n) begin
            for (s=0; s<SYNC_STAGES; s=s+1) wrcnt_gray_sync[s] <= {(AW+1){1'b0}};
        end
        else begin
            wrcnt_gray_sync[0] <= i_wrcnt_gray;
            for (s=1; s<SYNC_STAGES; s=s+1) wrcnt_gray_sync[s] <= wrcnt_gray_sync[s-1];
        end
    end

    //------------------------------------------------------------------------
    // Read pointer + RdCnt gray code
    //------------------------------------------------------------------------
    always @(posedge clk_r or negedge rrst_n) begin
        if (!rrst_n) begin
            rbin  <= 0;
            rgray <= 0;
        end else if (r_fire) begin
            rbin  <= rbin + 1'b1;
            rgray <= (rbin + 1'b1) ^ ((rbin + 1'b1) >> 1);
        end
    end
    assign o_rdcnt_gray = rgray;
endmodule
