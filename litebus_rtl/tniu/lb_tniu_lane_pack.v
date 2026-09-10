//============================================================================
// Filename    : lb_tniu_lane_pack.v
// Author      : litebus
// Description : write-channel Slave strb generation (by addr + total_bytes)
// Date        : 2026-08-06
// Revision    : 1.0 initial -- per-round changes are in doc/HISTORY.md
//----------------------------------------------------------------------------
// TNIU write forward. The barrel shifter this module started out with is gone:
// with phase-aligned width conversion the write data reaching the TNIU is
// already at Slave width and at the correct phase, because byte A always sits in
// lane A % EXT_DATA_BYTES no matter how many conversions happened upstream. So
// no byte movement is required here.
//
// What remains is the valid-range strb, ANDed with the upstream strb:
//   the range decides whether a beat is SENT (done by the gearbox),
//   the strb decides whether a byte is WRITTEN (done here).
// Slave byte b of beat beat_idx has transaction index beat_idx*EXT_DATA_BYTES + b
// and is in range when that index falls inside [addr_lo, addr_lo+total_bytes).
// Symmetric to the read side's byte_valid generation.
//
// Pure combinational. See arch.html section 6.1.7 / TNIU SPEC section 3.1.4.
//
// Parameter naming (CODING_STYLE 1.4): the data path here is entirely on the
// external (Slave IP) side, hence EXT_*; total_bytes arrives from the fabric CMD
// flit, hence INT_TOTBYTES_W. Note the phase input is EXT_BYTES_PER_BEAT_LOG
// wide = log2(bytes per beat); it is NOT the fabric addr_lane width INT_LANE_W,
// which is a different quantity (the two used to share the name LANE_W).
//============================================================================
`include "lb_defines.vh"

module lb_tniu_lane_pack #(
    parameter EXT_DATA_WIDTH         = 64,    // Slave-side data width in bits
    parameter INT_TOTBYTES_W         = 16,    // total_bytes width (from the CMD flit)
    parameter EXT_BYTES_PER_BEAT_LOG = 7,     // = log2(EXT_DATA_BYTES), the lane-phase width
    // ---- derived parameters; used in the port list, so they must stay in the
    // ---- parameter list. Not to be overridden externally.
    parameter EXT_DATA_BYTES         = EXT_DATA_WIDTH/8,  // Slave beat size in bytes
    parameter BEAT_IDX_W             = 8                  // beat-index width (burst beat counter)
) (
    // ---- inputs ----
    input wire  [EXT_BYTES_PER_BEAT_LOG-1:0] addr_lo,     // start byte offset in the first beat (lane phase)
    input wire  [INT_TOTBYTES_W-1:0]         total_bytes, // valid byte count of the whole burst
    input wire  [BEAT_IDX_W-1:0]             beat_idx,    // index of this beat within the burst
    input wire  [EXT_DATA_WIDTH-1:0]         in_data,     // write data, already placed by address
    input wire  [EXT_DATA_BYTES-1:0]         in_strb,     // upstream strb (sparse write)
    // ---- outputs ----
    output wire [EXT_DATA_WIDTH-1:0]         out_data,    // feed-through (no byte reordering)
    output wire [EXT_DATA_BYTES-1:0]         out_strb     // Slave strb (range & upstream)
);
    //------------------------------------------------------------------------
    // Derived local params
    //------------------------------------------------------------------------
    localparam LB = EXT_BYTES_PER_BEAT_LOG;          // log2(bytes per beat)
    localparam PW = INT_TOTBYTES_W + BEAT_IDX_W;         // byte-index comparison space

    //------------------------------------------------------------------------
    // Declarations (all up front, one per line)
    //------------------------------------------------------------------------
    genvar          g;        // byte-lane index
    wire [PW-1:0]   hi;       // valid range high bound (exclusive), transaction byte index
    wire [PW-LB-1:0] hi_beat; // hi / bytes-per-beat: the beat hi falls in
    wire [LB-1:0]   hi_lane;  // hi % bytes-per-beat: the lane within that beat
    wire [PW-LB-1:0] beat_w;  // beat_idx widened to the comparison space
    wire            beat_nz;  // this is not the burst's first beat
    wire            beat_lt;  // this beat is entirely below hi
    wire            beat_eq;  // hi falls inside this beat

    //------------------------------------------------------------------------
    // Valid byte range of the burst, split at the beat boundary.
    //
    // The straightforward form is, per lane,
    //     pos = beat_idx*EXT_DATA_BYTES + g;  in_range = pos >= lo && pos < hi;
    // which is two PW-bit magnitude comparators per byte lane -- 128 of them at a
    // 512-bit Slave, all fed from one adder, all in the same cycle as the CMD
    // decode beside them.
    //
    // EXT_DATA_BYTES is a power of two, so pos is not an addition at all: it is
    // the concatenation {beat_idx, g}, with g occupying exactly the low LB bits.
    // That lets the comparison split into a part that does not depend on the lane
    // and a part that is a constant on each lane:
    //
    //   pos >= lo   <=>  beat_idx != 0  ||  g >= addr_lo     (lo is only LB bits)
    //   pos <  hi   <=>  beat_idx < hi_beat
    //                    || (beat_idx == hi_beat && g < hi_lane)
    //
    // The three wide terms are computed once for the whole beat; what is left per
    // lane is two comparisons of an LB-bit value against a constant, which across
    // the lanes is just a thermometer decode of addr_lo and hi_lane.
    //
    // Measured (tools/timing/logic_depth.py, probe lb_tniu_lane_pack, 512-bit
    // Slave): 1402 -> 1148 cells, and logic depth UNCHANGED at 22. Worth saying
    // plainly -- the 128 replicated comparators were an area problem, not the
    // critical path. What sets the depth is the `hi` adder and the beat compare
    // hanging off it, and that arithmetic is inherent: the range genuinely depends
    // on addr_lo + total_bytes. Cutting it would need a register, which this pass
    // is not allowed to add.
    //------------------------------------------------------------------------
    assign hi      = addr_lo + total_bytes;
    assign hi_beat = hi[PW-1:LB];
    assign hi_lane = hi[LB-1:0];
    assign beat_w  = {{(PW-LB-BEAT_IDX_W){1'b0}}, beat_idx};
    assign beat_nz = |beat_idx;
    assign beat_lt = (beat_w <  hi_beat);
    assign beat_eq = (beat_w == hi_beat);

    //------------------------------------------------------------------------
    // Per-lane strb: in range AND enabled upstream
    //------------------------------------------------------------------------
    generate for (g = 0; g < EXT_DATA_BYTES; g = g + 1) begin : g_strb
        wire ge_lo; // this lane is at or after the burst's first valid byte
        wire lt_hi; // this lane is before the burst's end
        assign ge_lo       = beat_nz || (g[LB-1:0] >= addr_lo);
        assign lt_hi       = beat_lt || (beat_eq && (g[LB-1:0] < hi_lane));
        assign out_strb[g] = ge_lo && lt_hi && in_strb[g];
    end endgenerate

`ifndef LB_NO_ASSERT
    // synthesis translate_off
    // The split above is only valid when a beat is a power of two bytes and LB
    // really is its log2. Both hold for every width the generator emits; state it
    // here so a future non-power-of-two width fails loudly instead of producing
    // subtly wrong strobes.
    initial begin
        if ((1 << LB) != EXT_DATA_BYTES) begin
            $display("ERROR %m: BYTES_PER_BEAT_LOG=%0d != log2(EXT_DATA_BYTES=%0d)",
                     LB, EXT_DATA_BYTES);
        end
        if (LB < 1) begin
            $display("ERROR %m: EXT_BYTES_PER_BEAT_LOG must be at least 1", LB);
        end
    end
    // synthesis translate_on
`endif

    //------------------------------------------------------------------------
    // Data is already lane-aligned by the phase-aligned conversion upstream
    //------------------------------------------------------------------------
    assign out_data = in_data;
endmodule
