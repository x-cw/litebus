//============================================================================
// Filename    : lb_lsb_onehot.v
// Author      : litebus
// Description : lowest-set-bit isolation, log-depth (generic primitive)
// Date        : 2026-08-06
// Revision    : 1.0 initial -- per-round changes are in doc/HISTORY.md
//----------------------------------------------------------------------------
// Combinational. Given a request vector, produce the one-hot mask of its LOWEST
// set bit, plus the mask of all positions strictly ABOVE that bit.
//
//   i_vec  = 0110_1000  ->  onehot = 0000_1000
//                           above  = 1111_0000
//   i_vec  = 0000_0000  ->  onehot = 0000_0000
//                           above  = 1111_1111   (no winner: nothing is below one)
//
// Why this exists as a module rather than an expression. The idiomatic
// `i_vec & (~i_vec + 1)` is an N-bit carry chain, and the equally common
// `for (i...) if (i_vec[i] && !found) ...` is an N-deep chain of muxes serialised
// on `found`. Both are O(N) where the function is O(log N). The switch arbiter
// used the second form and measured 53 levels of logic at NUM_REQ=8 -- more than
// the entire rest of the switch put together. This builds the standard
// Kogge-Stone prefix-OR instead: depth is ceil(log2(N)) stages of one OR gate.
//
// Why `above` is an output rather than something the caller derives. A
// round-robin arbiter wants "the indices after the one just granted" to carry
// into the next cycle, and the obvious way to get it -- ~(onehot | (onehot-1))
// -- is another N-bit borrow chain, sitting on the register D-input right behind
// the arbiter it just paid for. Here it is already inside the prefix network:
// the "is any bit below me set" scan IS the answer, because the winner is the
// lowest set bit. Only the empty input needs a correction. See the derivation
// at the assign itself.
//
// No function/endfunction (CODING_STYLE 6.1); the prefix network is a generate
// tree over a flattened stage array, which is also how the rest of this RTL
// carries 2-D data (Verilog-2001 has no 2-D ports).
//============================================================================
`include "lb_defines.vh"

module lb_lsb_onehot #(
    parameter N   = 4,                       // vector width (number of requesters)
    // ---- derived: prefix-network stage count = ceil(log2(N)). Referenced by a
    // ---- declaration below, and kept here rather than in the body only for
    // ---- symmetry with the rest of the tree. Do not override (CODING_STYLE 4.1).
    // Runs to 16: the widest user is lb_tniu_cmd_table's free-entry select, whose
    // N is EXT_PENDING_TRANS = 2**ext_txnid_width, and schema.yaml lets that width
    // reach 16. Too few stages does not fail loudly -- the prefix tree just stops
    // propagating and "lowest set bit" comes out wrong for the high indices.
    parameter LGN = (N <= 1)     ? 0 :         // = clog2(N)
                    (N <= 2)     ? 1 :
                    (N <= 4)     ? 2 :
                    (N <= 8)     ? 3 :
                    (N <= 16)    ? 4 :
                    (N <= 32)    ? 5 :
                    (N <= 64)    ? 6 :
                    (N <= 128)   ? 7 :
                    (N <= 256)   ? 8 :
                    (N <= 512)   ? 9 :
                    (N <= 1024)  ? 10 :
                    (N <= 2048)  ? 11 :
                    (N <= 4096)  ? 12 :
                    (N <= 8192)  ? 13 :
                    (N <= 16384) ? 14 :
                    (N <= 32768) ? 15 : 16
) (
    // ---- inputs ----
    input wire      [N-1:0] i_vec,  // request vector
    // ---- outputs ----
    output wire     [N-1:0] onehot, // lowest set bit of i_vec, all zero when i_vec is zero
    output wire     [N-1:0] above   // positions strictly above the winner, all ones when i_vec is zero
);
    //------------------------------------------------------------------------
    // Declarations (all up front, one per line)
    //------------------------------------------------------------------------
    genvar               s;     // prefix stage index
    wire [(LGN+1)*N-1:0] sc;    // Kogge-Stone stages; stage k at sc[k*N +: N]
    wire [N-1:0]         incl;  // inclusive prefix OR: incl[i] = |i_vec[i:0]
    wire [N-1:0]         below; // exclusive prefix OR: below[i] = |i_vec[i-1:0]

    //------------------------------------------------------------------------
    // Stage 0 of the prefix network is the input itself
    //------------------------------------------------------------------------
    assign sc[0 +: N] = i_vec;

    //------------------------------------------------------------------------
    // Kogge-Stone inclusive prefix OR: stage s+1 folds in the bit 2^s below.
    //
    // Written as ONE vector operation per stage, not N per-bit assigns. The two
    // are the same function -- `x | (x << 2**s)` gives bit b of the next stage as
    // `x[b] | x[b-2**s]` for b >= 2**s, and `x[b] | 0` below that, which is
    // exactly what the per-bit form spelled out -- and they synthesise to the same
    // OR tree, because a shift by a constant is wiring.
    //
    // The per-bit form cost N*LGN separate continuous assigns per instance, and a
    // simulator schedules each net individually. At N=8 that is 24 nets and nobody
    // notices; at N=256 it is 2048 per instance with 16 TNIU cmd_tables plus every
    // switch arbiter, and simulation time exploded superlinearly -- measured on
    // bus_16x16s8 with five transactions per Master: depth 16 -> 3s, 32 -> 6s,
    // 64 -> 26s, 128 -> 249s, 256 -> did not finish in 420s. Every one of those
    // PASSED, so it was never a correctness problem, only an unusable one.
    // tools/timing/logic_depth.py confirms the levels are unchanged.
    //------------------------------------------------------------------------
    generate
    for (s = 0; s < LGN; s = s + 1) begin : g_stage
        assign sc[(s+1)*N +: N] = sc[s*N +: N] | (sc[s*N +: N] << (1 << s));
    end
    endgenerate

    assign incl = sc[LGN*N +: N];

    //------------------------------------------------------------------------
    // Shift the inclusive scan up by one to get "is any bit below me set".
    // Split on N because a width-1 vector has no bits to shift and the
    // part-select would be zero-width.
    //------------------------------------------------------------------------
    generate
    if (N == 1) begin : g_one
        assign below = 1'b0;
    end else begin : g_many
        assign below = {incl[N-2:0], 1'b0};
    end
    endgenerate

    //------------------------------------------------------------------------
    // Winner and the positions after it.
    //
    // `below` IS "strictly above the winner" already, for free: below[i] =
    // |i_vec[i-1:0], and the winner is the LOWEST set bit, so below[i] is 1
    // exactly for i > winner and 0 for i <= winner. Nothing further to compute.
    //
    // The only correction is the empty input. With i_vec == 0 the prefix OR is all
    // zero, so `below` is all zero -- but the contract at the top of this file,
    // and lb_sw_arbiter's `(|grant)` guard which cites it, want all ones for
    // "no winner: nothing is below one". incl[N-1] is the OR of every input
    // bit, so ~incl[N-1] is exactly "the input was empty", and replicating it
    // forces that case without touching the non-empty one.
    //
    // What this replaced: `~(below | onehot)`, which is the set of indices
    // strictly BELOW the winner -- the mirror of the documented function. It is
    // right at N == 1 and at i_vec == 0 and wrong everywhere else, which is how it
    // survived: lb_sw_arbiter's only symptom was that its RR mask collapsed to
    // zero within two grants, after which `grant` is permanently the lowest
    // requesting index. Round robin silently became fixed priority on every
    // switch of every generated bus, and no functional check anywhere can see
    // that -- fixed priority changes who wins, never the data. See
    // verify/unit/tb_lsb_onehot.v and verify/unit/tb_sw_arbiter_fair.v, which
    // exist because of this.
    //------------------------------------------------------------------------
    assign onehot = i_vec & ~below;
    assign above  = below | {N{~incl[N-1]}};
endmodule
