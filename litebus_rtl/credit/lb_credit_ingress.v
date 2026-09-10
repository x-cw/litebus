//============================================================================
// Filename    : lb_credit_ingress.v
// Author      : litebus
// Description : ingress credit adapter (credit-in -> valid-ready)
// Date        : 2026-08-06
// Revision    : 1.0 initial -- per-round changes are in doc/HISTORY.md
//----------------------------------------------------------------------------
// Upstream holds credit = this stage's FIFO depth DEPTH, so the write side
// needs no ready; each pop (downstream takes a flit) returns one credit to
// upstream (credit_return pulse).
//
// The FIFO body is implemented locally (standalone synchronous valid-ready
// FIFO), NOT reusing any other module, so this file is self-contained.
//   Write side : credit guarantees non-full, so in_valid is written directly.
//   Read  side : pop when out_valid && out_ready, emitting credit_return.
//   WIDTH : flit width.  DEPTH : FIFO depth = upstream credit budget.
//============================================================================
`include "lb_defines.vh"

module lb_credit_ingress #(
    parameter WIDTH = 32,                    // flit width
    parameter DEPTH = 4                      // FIFO depth = upstream credit budget
) (
    // ---- inputs ----
    input wire               clk,          // clock
    input wire               rst_n,        // async reset, active low
    input wire  [WIDTH-1:0]  in_data,      // upstream flit (credit guarantees space)
    input wire               in_valid,     // upstream valid
    input wire               out_ready,    // downstream ready
    // ---- outputs ----
    output wire [WIDTH-1:0]  out_data,     // downstream flit
    output wire              out_valid,    // downstream valid
    output wire              credit_return // one-cycle credit pulse per pop
);
    //------------------------------------------------------------------------
    // Derived local params (AW from DEPTH)
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
    wire do_rd;                       // read fire (pop)
    wire [AW:0] wptr_nxt;             // write pointer, next state (wraps at DEPTH)
    wire [AW:0] rptr_nxt;             // read pointer, next state (wraps at DEPTH)

    assign full          = (wptr[AW] != rptr[AW]) && (wptr[AW-1:0] == rptr[AW-1:0]);
    assign empty         = (wptr == rptr);
    assign do_wr         = in_valid && !full;    // credit guarantees non-full in normal use
    assign out_valid     = !empty;
    assign out_data      = mem[rptr[AW-1:0]];
    assign do_rd         = out_valid && out_ready;
    assign credit_return = do_rd;                // return one credit per pop

    //------------------------------------------------------------------------
    // Pointer next state. Two forms selected at elaboration: the natural 2**AW wrap
    // when DEPTH is a power of two (literally the pre-2026-08-25 code, so it cannot
    // cost anything), the modulo-DEPTH wrap otherwise. That is what lets DEPTH be
    // any integer. The full reasoning -- the silent out-of-range write it removes,
    // and the measured reason the form is selected on a localparam instead of
    // written unconditionally -- is in lb_sw_out_fifo.v at the same place.
    //
    // These two modules hold the same pointer scheme twice (this file's header says
    // why the body is local rather than shared), so they must not drift:
    // tools/check/check_fifo_ptr_parity.py compares them.
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
    // Simulation-only checks (LB_NO_ASSERT switches them off for lint builds).
    //
    // WHY THESE EXIST: the credit protocol's whole promise is "upstream holds
    // exactly DEPTH credits, so a flit can never arrive with no slot free".
    // Until 2026-08-22 nothing enforced it anywhere. The write side above is
    // `do_wr = in_valid && !full`, so a flit that arrives at a full FIFO is
    // DROPPED -- no error, no hang, not even a width warning, just a flit that
    // never comes out the other side. gen/lb_ir.py::_check_credits calls that
    // "the hardest failure mode there is", and it had no gate at RTL, product
    // or simulation level: rtl/credit, rtl/bca and rtl/pipe_vr held zero
    // assertions and zero unit benches.
    //
    // A check that lives in the hardware runs under EVERY configuration, EVERY
    // case and EVERY traffic pattern, which makes it the least demo-dependent
    // check in the repository -- deleting example configs cannot weaken it.
    //------------------------------------------------------------------------
`ifndef LB_NO_ASSERT
    // synthesis translate_off
    always @(posedge clk) begin
        if (rst_n) begin
            if (in_valid && full) begin
                $display("[%0t] ERROR %m: flit arrived at a FULL ingress FIFO (DEPTH=%0d) -- upstream",
                         $time, DEPTH);
                $display("       holds more credit than this stage can buffer; the flit is dropped SILENTLY");
            end
            // Address range. Same invariant, same reasoning, same wording as in
            // lb_sw_out_fifo.v -- the pointer scheme is duplicated here (the header
            // says why the body is local), so its guard is duplicated too, and
            // tools/check/check_fifo_ptr_parity.py compares the two.
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
