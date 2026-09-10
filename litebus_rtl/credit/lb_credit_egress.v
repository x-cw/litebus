//============================================================================
// Filename    : lb_credit_egress.v
// Author      : litebus
// Description : egress credit adapter (sender side)
// Date        : 2026-08-06
// Revision    : 1.0 initial -- per-round changes are in doc/HISTORY.md
//----------------------------------------------------------------------------
// The internal interface uses fixed credit flow control. As an "egress" for
// CMD/WD, this module is the credit holder/sender: initial credit = peer
// (LINK/Switch) ingress FIFO depth, guaranteeing every emitted flit has a
// buffer slot at the peer, so it can send continuously without a peer ready
// (credit is the proof that "the peer has space").
//
// Operation:
//   - After reset, credit = CREDIT_INIT (= peer ingress FIFO depth, injected
//     by the top after topology derivation).
//   - Each emitted flit  : credit - 1.
//   - Each credit_return : credit + 1 (peer popped one).
//   - credit == 0 : stop sending (out_valid low) -> flow control, no overflow.
//   WIDTH       : flit width (CMD/WD each pass their own).
//   CREDIT_INIT : initial credit (= peer ingress FIFO depth).
//============================================================================
`include "lb_defines.vh"

module lb_credit_egress #(
    parameter WIDTH       = 32,              // flit width
    parameter CREDIT_INIT = 4                // initial credit = peer ingress FIFO depth
) (
    // ---- inputs ----
    input wire               clk,           // clock
    input wire               rst_n,         // async reset, active low
    input wire  [WIDTH-1:0]  in_data,       // upstream (internal) flit
    input wire               in_valid,      // upstream valid
    input wire               credit_return, // credit returned by peer (pulse per pop)
    // ---- outputs ----
    output wire              in_ready,      // upstream ready (has credit)
    output wire [WIDTH-1:0]  out_data,      // downstream flit
    output wire              out_valid      // downstream valid (credit send, no ready)
);
    //------------------------------------------------------------------------
    // Local params
    //------------------------------------------------------------------------
    localparam CW = (CREDIT_INIT <= 1)   ? 1 :   // credit counter width (0..CREDIT_INIT)
                    (CREDIT_INIT <= 3)   ? 2 :
                    (CREDIT_INIT <= 7)   ? 3 :
                    (CREDIT_INIT <= 15)  ? 4 :
                    (CREDIT_INIT <= 31)  ? 5 :
                    (CREDIT_INIT <= 63)  ? 6 :
                    (CREDIT_INIT <= 127) ? 7 : 8;

    //------------------------------------------------------------------------
    // State and handshake
    //------------------------------------------------------------------------
    reg  [CW:0] credit;     // credit counter (1 extra bit for safe compare)
    wire        has_credit; // credit available
    wire        do_send;    // send this cycle

    assign has_credit = (credit != {(CW+1){1'b0}});
    assign do_send    = in_valid && has_credit;   // send only with data and credit
    assign in_ready   = has_credit;               // backpressure upstream when out of credit
    assign out_valid  = do_send;
    assign out_data   = in_data;

    //------------------------------------------------------------------------
    // Credit counter: same cycle may both send (-1) and receive return (+1)
    //------------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            credit <= CREDIT_INIT;
        end else begin
            case ({do_send, credit_return})
                2'b10:   credit <= credit - 1'b1;   // send only
                2'b01:   credit <= credit + 1'b1;   // return only
                default: credit <= credit;          // both (net 0) or neither
            endcase
        end
    end
    //------------------------------------------------------------------------
    // Simulation-only checks (LB_NO_ASSERT switches them off for lint builds).
    //
    // WHY THESE EXIST: see the same block in lb_credit_ingress.v. This is the
    // sender half of the same invariant. `credit` is an accounting of how much
    // room the peer still has, so two things must hold forever:
    //
    //   credit <= CREDIT_INIT          -- never hold more budget than granted
    //   credit == CREDIT_INIT => no return arriving
    //                                  -- with full budget nothing is
    //                                     outstanding, so nothing can be
    //                                     returned; a return here means the
    //                                     peer returned more than it received
    //                                     (its DEPTH is larger than our
    //                                     CREDIT_INIT), i.e. the pairing is
    //                                     broken in the OTHER direction and
    //                                     the counter would wrap past the
    //                                     peer's real capacity.
    //
    // Both are pure over-issue detectors: they cannot fire in a design whose
    // egress credit equals the downstream ingress depth, which is exactly the
    // property tools/check/check_credit_pairing.py checks statically. Static
    // and dynamic are the two faces of one invariant -- the static one reads
    // the emitted parameters, this one watches the traffic.
    //------------------------------------------------------------------------
`ifndef LB_NO_ASSERT
    // synthesis translate_off
    always @(posedge clk) begin
        if (rst_n) begin
            if (credit > CREDIT_INIT) begin
                $display("[%0t] ERROR %m: credit=%0d exceeds CREDIT_INIT=%0d -- more budget held",
                         $time, credit, CREDIT_INIT);
                $display("       than the peer ever granted; every flit past the peer's DEPTH is lost");
            end
            // `!do_send` is load-bearing: a ZERO-LATENCY return (the peer pulses
            // credit_return in the same cycle the flit leaves) is unusual but not a
            // protocol violation -- in that cycle there IS one outstanding flit, the
            // one being sent right now. verify/unit/tb_sw_port_depth.v drives exactly
            // that (`out_credit_ret = out_valid`, combinational), and without this
            // guard the check fired on it 8 times per run: a correct bench flagged by
            // an assertion that had encoded a stricter timing than the protocol asks
            // for. A genuine over-return still gets caught -- the SECOND bogus return
            // arrives in a cycle with no send, and `credit > CREDIT_INIT` above is the
            // backstop either way.
            if (credit_return && !do_send && (credit == CREDIT_INIT)) begin
                $display("[%0t] ERROR %m: credit returned while already at full budget %0d -- nothing",
                         $time, CREDIT_INIT);
                $display("       was outstanding, so the peer returned more credit than it received");
            end
        end
    end
    // synthesis translate_on
`endif

endmodule
