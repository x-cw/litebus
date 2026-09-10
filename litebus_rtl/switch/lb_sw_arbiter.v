//============================================================================
// Filename    : lb_sw_arbiter.v
// Author      : litebus
// Description : Switch output arbiter (Switch stage 3 arbitration)
// Date        : 2026-08-06
// Revision    : 1.0 initial -- per-round changes are in doc/HISTORY.md
//----------------------------------------------------------------------------
// When several inputs contend for the same output in one cycle, decides which
// input is granted. One arbiter is instantiated per out-port. Request/grant
// use flattened 1-D vectors (Verilog-2001 has no 2-D ports).
//
// Scheme (ARB_SCHEME):
//   0 = FIX : fixed priority, lowest index first.
//   1 = RR  : round-robin, starvation-free (remembers last grantee, searches
//             from the next index).
//   NUM_REQ    : number of requesters (= NUM_IN).
//   ARB_SCHEME : 0 = FIX / 1 = RR.
//
// Structure: two priority encoders and a mask, which is the textbook
// constant-depth round robin.
//
//   FIX      = lowest set bit of req.
//   RR       = lowest set bit of (req & mask), or, if that is empty, lowest set
//              bit of req -- where mask holds the indices strictly after the
//              previous grantee. That is exactly "search circularly from
//              rr_ptr+1", which is what this used to spell out as a loop.
//
// The round-robin state is the MASK itself, not a binary pointer. Keeping a
// pointer would cost a one-hot-to-binary encoder on the way in and a
// binary-to-thermometer decoder on the way out, both O(NUM_REQ), for a value
// nothing outside this module reads. lb_lsb_onehot hands back the "above the
// winner" mask for free, so the next mask is one 2:1 mux away.
//
// What this replaced, and why. The previous body was
//     for (i = 0; i < NUM_REQ; i = i + 1) begin
//         idx = (rr_ptr + 1 + i) % NUM_REQ;
//         if (req[idx] && !found) ...
//     end
// Every iteration was an adder, a run-time modulo (rr_ptr is a register, so it
// does not fold), an N:1 mux for req[idx] and an N-wide decoder for grant[idx],
// and each iteration waited on the previous one through `found`. Measured with
// tools/timing/logic_depth.py at NUM_REQ=8: 53 levels of logic and 630 cells,
// against 7 levels for the FIX arm right beside it. The whole switch closed
// timing at whatever this cost, because the crossbar's own critical path is
// route -> ARBITER -> flit mux and the arbiter was 90% of it. Behaviour is
// unchanged cycle for cycle; only the shape is different.
//============================================================================
`include "lb_defines.vh"

module lb_sw_arbiter #(
    parameter NUM_REQ    = 2,                // number of requesters
    // 2 = RR-QoS behaves as RR here on purpose: the QoS dominance filter lives
    // UPSTREAM in lb_sw_crossbar's g_qosf (this module sees pre-filtered req),
    // so use_hi's (ARB_SCHEME != 0) test needs no third arm.
    parameter ARB_SCHEME = 1                 // 0 = FIX, 1 = RR, 2 = RR-QoS
) (
    // ---- inputs ----
    input wire                     clk,        // clock
    input wire                     rst_n,      // async reset, active low
    input wire  [NUM_REQ-1:0]      req,        // per-input request for this output
    input wire                     grant_take, // grant consumed this cycle (advance RR mask)
    // ---- outputs ----
    output wire [NUM_REQ-1:0]      grant       // one-hot grant
);
    //------------------------------------------------------------------------
    // Declarations (all up front, one per line)
    //------------------------------------------------------------------------
    reg  [NUM_REQ-1:0] mask;     // RR state: indices strictly after the last grantee
    wire [NUM_REQ-1:0] req_hi;   // requests at or after the RR mask
    wire [NUM_REQ-1:0] gnt_hi;   // lowest set bit of req_hi
    wire [NUM_REQ-1:0] abv_hi;   // indices above gnt_hi
    wire [NUM_REQ-1:0] gnt_lo;   // lowest set bit of req (the wrap-around case, and FIX)
    wire [NUM_REQ-1:0] abv_lo;   // indices above gnt_lo
    wire               use_hi;   // this cycle's winner comes from the masked half
    wire [NUM_REQ-1:0] mask_nxt; // mask after this grant

    //------------------------------------------------------------------------
    // Two priority encoders over the same request vector
    //------------------------------------------------------------------------
    assign req_hi = req & mask;

    lb_lsb_onehot #(
        .N      (NUM_REQ)     // requester count
    ) u10_hi (
        .i_vec  (req_hi),     // requests after the last grantee
        .onehot (gnt_hi),     // winner among them
        .above  (abv_hi)      // indices above that winner
    );

    lb_lsb_onehot #(
        .N      (NUM_REQ)     // requester count
    ) u11_lo (
        .i_vec  (req),        // all requests
        .onehot (gnt_lo),     // lowest-index winner
        .above  (abv_lo)      // indices above that winner
    );

    //------------------------------------------------------------------------
    // Grant. FIX ignores the mask entirely, so both arms share u11_lo and the
    // ARB_SCHEME test folds away at elaboration.
    //------------------------------------------------------------------------
    assign use_hi   = (ARB_SCHEME != 0) && (|req_hi);
    assign grant    = use_hi ? gnt_hi : gnt_lo;
    assign mask_nxt = use_hi ? abv_hi : abv_lo;

    //------------------------------------------------------------------------
    // RR mask update: after a grant is consumed, restart after the grantee.
    //
    // The (|grant) guard is load-bearing and is NOT redundant with grant_take.
    // The old loop simply found no match when req was empty and left rr_ptr
    // alone; here, lb_lsb_onehot reports "above" as all ones for an empty input,
    // so an unguarded update on grant_take with no request would reset the mask
    // to all ones and quietly restart the rotation at index 0. Today's caller
    // ties grant_take to out_fire, which implies |grant -- but this module has
    // its own boundary and must not depend on that.
    //
    // Reset value: everything except index 0, which is what the old rr_ptr = 0
    // meant ("start the search at index 1"). Written as a shift rather than
    // {{(NUM_REQ-1){1'b1}}, 1'b0} so NUM_REQ = 1 does not become a zero-width
    // replication.
    //------------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            mask <= {NUM_REQ{1'b1}} << 1;
        end
        else if (grant_take && (|grant)) begin
            mask <= mask_nxt;
        end
    end
endmodule
