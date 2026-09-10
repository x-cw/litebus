//============================================================================
// Filename    : lb_sw_out_fifo.v
// Author      : litebus
// Description : Switch output FIFO (Switch stage 4: OUT_FIFO)
// Date        : 2026-08-06
// Revision    : 1.0 initial -- per-round changes are in doc/HISTORY.md
//----------------------------------------------------------------------------
// One synchronous FIFO per out-port between the crossbar and the credit
// egress, absorbing bursts from output contention / downstream backpressure.
// Standard valid-ready synchronous FIFO.
//   WIDTH : flit width.
//   DEPTH : this out-port FIFO depth (OUT_FIFO[o], per-port configurable).
//============================================================================
`include "lb_defines.vh"

module lb_sw_out_fifo #(
    parameter WIDTH = 64,                    // flit width
    parameter DEPTH = 4                      // FIFO depth
) (
    // ---- inputs ----
    input wire                   clk,       // clock
    input wire                   rst_n,     // async reset, active low
    input wire      [WIDTH-1:0]  in_data,   // input flit
    input wire                   in_valid,  // input valid
    input wire                   out_ready, // downstream ready
    // ---- outputs ----
    output wire                  in_ready,  // input ready (not full)
    output wire     [WIDTH-1:0]  out_data,  // output flit
    output wire                  out_valid  // output valid (not empty)
);
    //------------------------------------------------------------------------
    // Local params
    //------------------------------------------------------------------------
    localparam AW = (DEPTH <= 2)  ? 1 :        // pointer width = log2(DEPTH), +1 wrap bit in ptr
                    (DEPTH <= 4)  ? 2 :
                    (DEPTH <= 8)  ? 3 :
                    (DEPTH <= 16) ? 4 :
                    (DEPTH <= 32) ? 5 : 6;

    //------------------------------------------------------------------------
    // Storage and pointers
    //------------------------------------------------------------------------
    reg  [WIDTH-1:0] mem [0:DEPTH-1]; // FIFO memory
    reg  [AW:0]      wptr;            // write pointer (with wrap bit)
    reg  [AW:0]      rptr;            // read pointer (with wrap bit)

    //------------------------------------------------------------------------
    // Status and handshake
    //------------------------------------------------------------------------
    wire full;                        // FIFO full
    wire empty;                       // FIFO empty
    wire do_wr;                       // write fire
    wire do_rd;                       // read fire
    wire [AW:0] wptr_nxt;             // write pointer, next state (wraps at DEPTH)
    wire [AW:0] rptr_nxt;             // read pointer, next state (wraps at DEPTH)

    assign full      = (wptr[AW] != rptr[AW]) && (wptr[AW-1:0] == rptr[AW-1:0]);
    assign empty     = (wptr == rptr);
    assign do_wr     = in_valid && !full;
    assign do_rd     = out_valid && out_ready;
    assign in_ready  = !full;
    assign out_valid = !empty;
    assign out_data  = mem[rptr[AW-1:0]];

    //------------------------------------------------------------------------
    // Pointer next state. Two forms, selected at ELABORATION by whether DEPTH is
    // a power of two -- not one form that a synthesizer might or might not fold.
    //
    // WHY A PLAIN `ptr + 1` IS NOT ENOUGH. The address field must stay inside
    // 0..DEPTH-1: `mem` is declared [0:DEPTH-1] and both ports index it with
    // ptr[AW-1:0]. Letting the field run its natural 2**AW cycle is correct only
    // when DEPTH == 2**AW. At DEPTH=5 (AW=3) the field reaches 5, 6, 7 and writes
    // past the end of mem, while `full` -- which tests "same address, other lap"
    // -- does not rise until 8 entries are outstanding. NOTHING catches that: the
    // ingress assertion in lb_credit_ingress fires on in_valid && full and full
    // never rises; the egress credit counter never exceeds CREDIT_INIT. A silent
    // out-of-range write. That is the whole reason gen/lb_topo.py's legal depths
    // were powers of two -- the restriction lived in this one expression.
    //
    // WHY THE GENERATE, instead of writing the modulo form unconditionally. The
    // modulo form IS algebraically equal to ptr+1 when DEPTH == 2**AW (the compare
    // is against all-ones and {~ptr[AW],0} is exactly the carry-out result), but
    // proving that needs case analysis over the whole address field, and yosys's
    // opt -full does not do it: measured, writing it unconditionally cost +12 cells
    // at DEPTH=4, +6 at 2, +18 at 8, +24 at 16 -- on every one of the ~5500 FIFO
    // instances a large bus carries. ltp did not move, so it was timing-free but
    // not area-free. Selecting the form on a localparam makes the power-of-two case
    // LITERALLY the old code, so it cannot cost anything, and only a depth that
    // actually needs the modulo pays for it. (CODING_STYLE 4A.1: same ports, one
    // parameter, generate -- the lb_link_top pattern.)
    //
    // Keep each arm to ONE ternary: two would trip style_audit S17 (multi-level ?:
    // is exempt only inside parameter/localparam constant evaluation, 9.2 (1)).
    //------------------------------------------------------------------------
    localparam POW2 = (DEPTH == (1 << AW));   // natural 2**AW wrap is already correct

    generate
    if (POW2) begin : g_ptr_nat
        assign wptr_nxt = wptr + 1'b1;
        assign rptr_nxt = rptr + 1'b1;
    end else begin : g_ptr_mod
        assign wptr_nxt = (wptr[AW-1:0] == DEPTH - 1) ? {~wptr[AW], {AW{1'b0}}}
                                                      : (wptr + 1'b1);
        assign rptr_nxt = (rptr[AW-1:0] == DEPTH - 1) ? {~rptr[AW], {AW{1'b0}}}
                                                      : (rptr + 1'b1);
    end
    endgenerate

    //------------------------------------------------------------------------
    // Pointer update
    //------------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wptr <= 0;
            rptr <= 0;
        end else begin
            if (do_wr) begin
                wptr <= wptr_nxt;
            end
            if (do_rd) begin
                rptr <= rptr_nxt;
            end
        end
    end

    //------------------------------------------------------------------------
    // Memory write
    //------------------------------------------------------------------------
    always @(posedge clk) if (do_wr) begin
        mem[wptr[AW-1:0]] <= in_data;
    end
    //------------------------------------------------------------------------
    // Simulation-only check (LB_NO_ASSERT switches it off for lint builds).
    //
    // WHY IT EXISTS: the address field must stay inside 0..DEPTH-1, because `mem`
    // is declared [0:DEPTH-1] and both ports index it with ptr[AW-1:0]. NOTHING
    // else in the design or in the flow catches a violation:
    //   - an out-of-range write to a Verilog memory is DISCARDED silently -- no
    //     error, no warning, not even a width mismatch;
    //   - `full` compares addresses, so it keeps behaving plausibly;
    //   - the egress credit counter never exceeds CREDIT_INIT either.
    // The flit simply never comes out the other side.
    //
    // This is exactly the invariant the pointer generate above exists to hold, so
    // it is CHECKED here instead of being argued about in a comment. The g_ptr_mod
    // arm is the one that can get it wrong, and until 2026-08-25 no shipped
    // configuration exercised that arm at all.
    //
    // Until then this module had NO runtime guard of any kind -- the only storage
    // primitive in the repository without one (lb_credit_ingress has had its
    // full-FIFO assertion since 2026-08-22).
    //------------------------------------------------------------------------
`ifndef LB_NO_ASSERT
    // synthesis translate_off
    always @(posedge clk) begin
        if (rst_n) begin
            if (wptr[AW-1:0] > DEPTH - 1) begin
                $display("[%0t] ERROR %m: write address %0d is outside mem[0:%0d] (DEPTH=%0d)",
                         $time, wptr[AW-1:0], DEPTH - 1, DEPTH);
                $display("       the write is DISCARDED silently -- that flit never comes out");
            end
            if (rptr[AW-1:0] > DEPTH - 1) begin
                $display("[%0t] ERROR %m: read address %0d is outside mem[0:%0d] (DEPTH=%0d)",
                         $time, rptr[AW-1:0], DEPTH - 1, DEPTH);
            end
        end
    end
    // synthesis translate_on
`endif

endmodule
