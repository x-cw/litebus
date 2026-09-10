//============================================================================
// Filename    : lb_msb_onehot.v
// Author      : litebus
// Description : highest-set-bit isolation, log-depth (generic primitive)
// Date        : 2026-08-24
// Revision    : 1.0 initial -- per-round changes are in doc/HISTORY.md
//----------------------------------------------------------------------------
// Combinational. Given a request vector, produce the one-hot mask of its HIGHEST
// set bit, plus the mask of all positions strictly BELOW that bit.
//
//   i_vec  = 0110_1000  ->  onehot = 0100_0000
//                           below  = 0011_1111
//   i_vec  = 0000_0000  ->  onehot = 0000_0000
//                           below  = 1111_1111   (no winner: nothing is above one)
//
// The mirror of lb_lsb_onehot, and it exists for ONE reason: two allocate ports
// that must never be handed the same index. lb_tniu_cmd_table gives the read port
// the lowest free entry and the write port the highest, so whenever two or more
// entries are free the two picks differ BY CONSTRUCTION and the two prefix
// networks run IN PARALLEL. The alternative -- second pick = lb_lsb_onehot(free &
// above_first) -- is the same function but puts the second prefix network in
// SERIES with the first, doubling that path (7 -> 14 levels at N=128). Exactly one
// free entry is the only colliding case, and there the caller's round-robin
// pointer has already decided which port is ready, so only one of them allocates.
//
// WHY A SIBLING MODULE AND NOT A PARAMETER ON lb_lsb_onehot
//
// CODING_STYLE 4A.1 says same-ports-two-forms is one module plus a parameter, and
// the ports here ARE identical. It is still two modules, because the parameter
// would leave every FROM_MSB=1 instance named `lb_lsb_onehot` -- a primitive whose
// name states the opposite of what that instance computes. The shared part is a
// generate loop of one line; the names are load-bearing at every call site.
//
// WHY BIT-REVERSAL WAS NOT USED
//
// reverse -> lb_lsb_onehot -> reverse is correct and free in gates (reversal is
// wiring), but it needs 2*N per-bit continuous assigns, and lb_lsb_onehot.v
// records what that costs a SIMULATOR rather than a synthesiser: the per-bit form
// of the prefix network went 16 -> 3s, 128 -> 249s, 256 -> did not finish in 420s
// on bus_16x16s8, every run PASSING. This file keeps the one-vector-op-per-stage
// shape for the same reason.
//
// No function/endfunction (CODING_STYLE 6.1).
//============================================================================
`include "lb_defines.vh"

module lb_msb_onehot #(
    parameter N   = 4,                       // vector width (number of requesters)
    // ---- derived: prefix-network stage count = ceil(log2(N)). Referenced by a
    // ---- declaration below, and kept here rather than in the body only for
    // ---- symmetry with lb_lsb_onehot. Do not override (CODING_STYLE 4.1).
    // Runs to 16 for the same reason as lb_lsb_onehot: the widest user is
    // lb_tniu_cmd_table's free-entry select, whose N is EXT_PENDING_TRANS =
    // 2**ext_txnid_width, and schema.yaml lets that width reach 16. Too few stages
    // does not fail loudly -- the prefix tree just stops propagating and "highest
    // set bit" comes out wrong for the low indices.
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
    output wire     [N-1:0] onehot, // highest set bit of i_vec, all zero when i_vec is zero
    output wire     [N-1:0] below   // positions strictly below the winner, all ones when i_vec is zero
);
    //------------------------------------------------------------------------
    // Declarations (all up front, one per line)
    //------------------------------------------------------------------------
    genvar               s;     // prefix stage index
    wire [(LGN+1)*N-1:0] sc;    // Kogge-Stone stages; stage k at sc[k*N +: N]
    wire [N-1:0]         incl;  // inclusive suffix OR: incl[i] = |i_vec[N-1:i]
    wire [N-1:0]         above; // exclusive suffix OR: above[i] = |i_vec[N-1:i+1]

    //------------------------------------------------------------------------
    // Stage 0 of the prefix network is the input itself
    //------------------------------------------------------------------------
    assign sc[0 +: N] = i_vec;

    //------------------------------------------------------------------------
    // Kogge-Stone inclusive SUFFIX OR: stage s+1 folds in the bit 2^s ABOVE.
    // The only difference from lb_lsb_onehot is the shift direction: `>>` instead
    // of `<<`, which makes the scan run from the top down. One vector operation
    // per stage, so depth is ceil(log2(N)) stages of one OR gate, same as the
    // sibling.
    //------------------------------------------------------------------------
    generate
    for (s = 0; s < LGN; s = s + 1) begin : g_stage
        assign sc[(s+1)*N +: N] = sc[s*N +: N] | (sc[s*N +: N] >> (1 << s));
    end
    endgenerate

    assign incl = sc[LGN*N +: N];

    //------------------------------------------------------------------------
    // Shift the inclusive scan DOWN by one to get "is any bit above me set".
    // Split on N because a width-1 vector has no bits to shift and the
    // part-select would be zero-width.
    //------------------------------------------------------------------------
    generate
    if (N == 1) begin : g_one
        assign above = 1'b0;
    end else begin : g_many
        assign above = {1'b0, incl[N-1:1]};
    end
    endgenerate

    //------------------------------------------------------------------------
    // Winner and the positions before it.
    //
    // `above` IS "strictly below the winner" already, mirroring the derivation in
    // lb_lsb_onehot: above[i] = |i_vec[N-1:i+1], and the winner is the HIGHEST set
    // bit, so above[i] is 1 exactly for i < winner and 0 for i >= winner.
    //
    // The only correction is the empty input. With i_vec == 0 the suffix OR is all
    // zero, so `above` is all zero -- but the contract at the top of this file
    // wants all ones for "no winner: nothing is above one". incl[0] is the OR of
    // every input bit here (the suffix scan accumulates downward), so ~incl[0] is
    // exactly "the input was empty".
    //
    // The mirror-image mistake to avoid is the one lb_lsb_onehot.v documents
    // paying for: taking ~(above | onehot), which is the set strictly ABOVE the
    // winner -- right at N == 1 and at i_vec == 0 and wrong everywhere else, so it
    // survives every smoke test. verify/unit/tb_msb_onehot.v checks the function
    // exhaustively at small N for that reason.
    //------------------------------------------------------------------------
    assign onehot = i_vec & ~above;
    assign below  = above | {N{~incl[0]}};
endmodule
