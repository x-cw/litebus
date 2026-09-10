//============================================================================
// Filename    : lb_tniu_cmd_conv.v
// Author      : litebus
// Description : CMD flit -> Slave-side CMD payload (pure combinational)
// Date        : 2026-08-06
// Revision    : 1.0 initial -- per-round changes are in doc/HISTORY.md
//----------------------------------------------------------------------------
// One conversion step of the TNIU forward path, factored out of
// lb_tniu_int_core so that the REQ_R and REQ_W arms can share ONE copy of it.
//
// WHY IT IS A MODULE. Once REQ split into REQ_R and REQ_W the two arms each need
// this conversion, and the arithmetic below rests on a non-obvious identity
// (see beats_m1) plus a precondition on total_bytes. Two textual copies of that
// in one file is how the two arms would drift apart -- and a drift here is a
// wrong Slave-side len, which elaborates cleanly and simulates as a burst of the
// wrong length.
//
// What it does, in the order the fields are produced:
//   addr_lo   = the byte offset of the transaction inside ONE Slave beat. Both
//               arms need it: this one to reconstruct the beat count, the REQ_W
//               arm again to drive lb_tniu_lane_pack. Emitted rather than
//               re-sliced upstream so the two uses cannot disagree.
//   c_len     = Slave-side beat count - 1, rebuilt from (addr_lo, total_bytes).
//   cmd_ext_pk= the packed Slave-side CMD payload.
//
// Parameter naming convention (CODING_STYLE 1.4): EXT_* describe the Slave-IP
// side interface, INT_* the in-network CMD flit fields. INT_ADDR_LOCAL_WIDTH is
// INT_ despite only feeding EXT_CMD_W, because it is the width of the CMD flit's
// address field -- attribution follows the interface a signal physically sits
// on, not where its value ends up.
//
// No clock, no reset, no state: this is a pure function of its inputs.
//============================================================================
`include "lb_defines.vh"

module lb_tniu_cmd_conv #(
    // ==== external interface (Slave IP side) ====
    parameter EXT_DATA_WIDTH       = 64,  // Slave-side data width
    parameter EXT_LEN_WIDTH        = 4,   // Slave-side len width (beat count - 1)
    parameter EXT_TXNID_WIDTH      = 3,   // Slave-side external transaction ID width
    parameter EXT_USER_WIDTH_CMD   = 8,   // Slave-side CMD user sideband width
    // ==== in-network CMD flit field widths ====
    parameter INT_ADDR_LOCAL_WIDTH = 32,  // in-network local (rebased) address width
    parameter INT_USER_WIDTH_CMD   = 8,   // in-network CMD user width (>= EXT_USER_WIDTH_CMD)
    parameter INT_TOTBYTES_W       = 16,  // in-network unified total_bytes width
    // ---- derived parameters; used in the port list, so they must stay in the
    // ---- parameter list (V2001 body localparams cannot size ports). Not to be
    // ---- overridden externally -- the defaults are the correct values.
    parameter EXT_BYTES_PER_BEAT_LOG =    // = log2(SLV bytes per beat)
        (EXT_DATA_WIDTH/8 <=   1) ? 0 : (EXT_DATA_WIDTH/8 <=   2) ? 1 :
        (EXT_DATA_WIDTH/8 <=   4) ? 2 : (EXT_DATA_WIDTH/8 <=   8) ? 3 :
        (EXT_DATA_WIDTH/8 <=  16) ? 4 : (EXT_DATA_WIDTH/8 <=  32) ? 5 :
        (EXT_DATA_WIDTH/8 <=  64) ? 6 : (EXT_DATA_WIDTH/8 <= 128) ? 7 : 8,
    parameter EXT_CMD_W = `LB_OPCODE_WIDTH + INT_ADDR_LOCAL_WIDTH + EXT_LEN_WIDTH +
                          EXT_TXNID_WIDTH + EXT_USER_WIDTH_CMD,
    // c_user_i port width, floored at 1: on a bus where no NIU carries a CMD user
    // the in-network field does not exist, and a V2001 port list cannot drop a port
    // (CODING_STYLE 4A.4). [-1:0] would be a LEGAL 2-bit ascending range, so the
    // floor has to be explicit. At width 0 the caller ties this to zero.
    parameter INT_USER_CMD_PW = (INT_USER_WIDTH_CMD < 1) ? 1 : INT_USER_WIDTH_CMD
) (
    // ---- inputs: CMD flit fields (in-network widths) ----
    input wire  [`LB_OPCODE_WIDTH-1:0]      c_opcode,      // CMD opcode field, passed through
    input wire  [INT_ADDR_LOCAL_WIDTH-1:0]  c_addr,        // CMD local address field
    input wire  [INT_TOTBYTES_W-1:0]        c_total_bytes, // CMD total_bytes field
    input wire  [INT_USER_CMD_PW-1:0]       c_user_i,      // CMD user field, in-network width (tied off at 0)
    // ---- inputs: the LID this beat drives, already resolved by the caller ----
    input wire  [EXT_TXNID_WIDTH-1:0]       use_txnid,     // Slave-side txnid = LID zero-extended
    // ---- outputs ----
    output wire [EXT_BYTES_PER_BEAT_LOG-1:0] addr_lo,      // byte offset within one Slave beat
    output wire [EXT_CMD_W-1:0]             cmd_ext_pk     // packed Slave-side CMD payload
);
    //------------------------------------------------------------------------
    // Declarations (all up front, one per line)
    //------------------------------------------------------------------------
    wire                          tb_nz;    // total_bytes is non-zero
    wire [INT_TOTBYTES_W:0]       span_m1;  // addr_lo + total_bytes - 1 (may carry, +1 bit)
    wire [INT_TOTBYTES_W:0]       beats_m1; // span_m1 / SLV_BYTES = beat count - 1
    wire [EXT_LEN_WIDTH-1:0]      c_len;    // Slave-side len = beats - 1

    //------------------------------------------------------------------------
    // Slave-side beat-count reconstruction (first-beat offset + round up):
    //   addr_lo = addr mod SLV_BYTES (offset within one SLV beat)
    //   len     = ceil((addr_lo + total_bytes) / SLV_BYTES) - 1
    //   e.g. SLV=32B, addr_lo=29, total=31 -> ceil(60/32) = 2 beats -> len 1
    // A plain total>>log2 would miss the first-beat offset and undercount, hence
    // ceil.
    //
    // Written as ONE add rather than three. The direct transcription was
    //     span = addr_lo + total_bytes;  span_ceil = span + (SLV_BYTES-1);
    //     beats = span_ceil >> log2;     len = beats - 1;
    // which is three dependent carry chains at INT_TOTBYTES_W+1 bits, in series,
    // feeding the CMD flit that goes straight into the ext skid. For any x >= 1,
    //     ceil(x / B) - 1  ==  floor((x - 1) / B)
    // (write x-1 = qB + r with 0 <= r < B: the left side is q+1-1, the right is
    // q), so the round-up constant and the final decrement both disappear and
    // what is left is a single 3-input add plus a constant shift. The x >= 1
    // precondition is total_bytes >= 1, which is what tb_nz singles out -- and it
    // is an OR-reduce computed in parallel rather than a compare sitting behind
    // the subtract.
    //------------------------------------------------------------------------
    assign addr_lo  = c_addr[EXT_BYTES_PER_BEAT_LOG-1:0];
    assign tb_nz    = |c_total_bytes;
    assign span_m1  = addr_lo + c_total_bytes + {(INT_TOTBYTES_W+1){1'b1}};
    assign beats_m1 = span_m1 >> EXT_BYTES_PER_BEAT_LOG;
    // semantics: len = beats - 1 (symmetric to INIU, len=0 -> 1 beat)
    assign c_len    = tb_nz ? beats_m1[EXT_LEN_WIDTH-1:0] : {EXT_LEN_WIDTH{1'b0}};

    //------------------------------------------------------------------------
    // user truncated back to the Slave external width (symmetric to the INIU
    // entry zero-extend), then the payload packed MSB..LSB.
    // A Slave with no CMD user pin takes no user member in its payload (EXT_CMD_W
    // drops the addend by itself) and reads none from the flit -- the bus may still
    // carry one for the other NIUs. Keyed on EXT: the in-network width is the
    // bus-wide maximum, so EXT > 0 guarantees the else arm's slice is in range.
    //------------------------------------------------------------------------
    generate
    if (EXT_USER_WIDTH_CMD == 0) begin : g_cmd_nouser
        assign cmd_ext_pk = {c_opcode, c_addr, c_len, use_txnid};
    end
    else begin : g_cmd_user
        wire [EXT_USER_WIDTH_CMD-1:0] c_user; // user truncated to the Slave width
        assign c_user     = c_user_i[EXT_USER_WIDTH_CMD-1:0];
        assign cmd_ext_pk = {c_opcode, c_addr, c_len, use_txnid, c_user};
    end
    endgenerate

    //------------------------------------------------------------------------
    // Simulation-only guard: the in-network CMD user width must cover the
    // external one, or the truncation above is an out-of-range part-select that
    // iverilog accepts quietly enough to reach the waveform as corrupt data.
    // The parent checks this too; it is repeated here because this module is now
    // instantiated twice and a mismatch would be attributed to the wrong arm.
    //------------------------------------------------------------------------
`ifndef LB_NO_ASSERT
    // synthesis translate_off
    initial begin
        if (INT_USER_WIDTH_CMD < EXT_USER_WIDTH_CMD) begin
            $display("ERROR %m: INT_USER_WIDTH_CMD=%0d < EXT_USER_WIDTH_CMD=%0d",
                     INT_USER_WIDTH_CMD, EXT_USER_WIDTH_CMD);
        end
    end
    // synthesis translate_on
`endif

endmodule
