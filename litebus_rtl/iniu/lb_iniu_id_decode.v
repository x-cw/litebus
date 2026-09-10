//============================================================================
// Filename    : lb_iniu_id_decode.v
// Author      : litebus
// Description : ID Decode (SPEC stage 3)
// Date        : 2026-08-06
// Revision    : 1.0 initial -- per-round changes are in doc/HISTORY.md
//----------------------------------------------------------------------------
// ID Decode (SPEC stage 3)
//
// Purpose:
//   Convert/pack external CMD channel signals into the internal CMD flit fields.
//   - input  = external CMD signals: cmd_addr / cmd_len / cmd_ext_txnid /
//              cmd_user / cmd_opcode (+ vld-rdy).
//   - output = internal CMD flit (packed bit vector):
//              { opcode, addr_local, total_bytes, user, int_id, dest_id, src_id }.
//   Steps: (1) look up cmd_addr in the memory map -> dest_id and rebased addr_local;
//          (2) src_id = this INIU id (NIU_ID, injected by top);
//          (3) int_id = cmd_ext_txnid zero-extended to the in-network width;
//          (4) total_bytes = (len+1) x bytes_per_beat - first-beat byte offset.
//
// memory map injection (method A, compile-time hardcoded):
//   The "address region -> DestID" table is compile-time hardcoded. The generator
//   reads the MemoryMap sheet and fills BASE/MASK/DEST constants and NUM_REGION
//   into the parameters below. Hit: (addr & MASK)==BASE; addr_local = addr & ~MASK.
//
// Combinational-path structure (why it looks like this):
//   The regions are decoded FULLY IN PARALLEL. An earlier version accumulated the
//   result in an `always @(*)` for-loop of `if (hit) dest = ...` statements, which
//   is a PRIORITY chain: synthesis must honour "last match wins", so it builds
//   NUM_REGION 2:1 muxes back to back and the decode delay grows LINEARLY with
//   NUM_REGION (the ID-decode path is already the INIU forward critical path).
//   The structure below instead is a classic one-hot AND-OR mux:
//     step 1  region_hit[r] = ((addr & MASK[r]) == BASE[r])  -- all r in parallel
//     step 2  every output bit is ONE reduction-OR over (region_hit & const_col),
//             i.e. an OR tree of depth log2(NUM_REGION), no priority ordering.
//   Because MASK/BASE/DEST are elaboration constants, `region_hit & col` collapses
//   to a subset selection at elaboration, so step 2 costs only the OR tree.
//   Latency is unchanged: this stage stays purely combinational (no pipe added).
//
//   Precondition: the memory map regions must NOT overlap, i.e. region_hit is
//   at most one-hot. The generator emits one region per Slave from disjoint
//   (base,size) pairs, so this holds by construction. If two regions did overlap,
//   the AND-OR mux ORs the two constants together instead of taking the last
//   match; the simulation-only check at the bottom of this file flags that.
//
// Parameterization: see parameter comments below. Naming convention: EXT_* are
//   external (IP-side) interface widths, INT_* are in-network interface widths.
//   Note: INT_CMD_FLIT_W is a derived parameter (packed width); since it is used
//   in ports it stays in the parameter list (Verilog-2001 body localparams cannot
//   be used in port declarations); it must not be overridden -- the default is
//   the correct value.
//============================================================================
`include "lb_defines.vh"

module lb_iniu_id_decode #(
    parameter EXT_ADDR_WIDTH      = 32,   // cmd_addr width (external)
    parameter EXT_LEN_WIDTH       = 4,    // cmd_len width (external beat count)
    parameter EXT_TXNID_WIDTH     = 8,    // cmd_ext_txnid width (external transaction ID)
    parameter EXT_USER_WIDTH_CMD  = 8,    // cmd_user width (external)
    parameter EXT_DATA_WIDTH      = 64,   // external data width, for total_bytes = len x (EXT_DATA_WIDTH/8)
    parameter INT_DEST_ID_WIDTH   = 4,    // = log2(NUM_SLAVES), injected by top
    parameter INT_SRC_ID_WIDTH    = 4,    // = log2(NUM_MASTERS), injected by top
    parameter NIU_ID              = 0,    // this INIU id value, injected by top
    parameter INT_ADDR_LOCAL_WIDTH = 32,  // target local address width (in-network), from memory map
    parameter NUM_REGION          = 1,    // number of memory-map regions
    // flattened memory-map region table (compile-time constants, generator-filled)
    parameter REGION_BASE = {(NUM_REGION*EXT_ADDR_WIDTH){1'b0}},
    parameter REGION_MASK = {(NUM_REGION*EXT_ADDR_WIDTH){1'b1}},
    parameter REGION_DEST = {(NUM_REGION*INT_DEST_ID_WIDTH){1'b0}},
    // ==== in-network unified widths (top injects global max; narrow externals zero-extended) ====
    parameter INT_ID_WIDTH        = 8,    // in-network unified int_id width (>= EXT_TXNID_WIDTH)
    // This block only ever handles the CMD channel, so both user widths carry the
    // _CMD suffix: a bare INT_USER_WIDTH here would leave "which of the three is
    // this?" unanswerable at the instantiation site.
    parameter INT_USER_WIDTH_CMD  = 8,    // in-network CMD user width (>= EXT_USER_WIDTH_CMD)
    parameter INT_TOTBYTES_W      = 16,   // in-network unified total_bytes width (>= local)
    // QoS: EXT is this Master's own pin width, INT the bus-wide maximum; the
    // field TOPS the CMD flit (above opcode) and takes the same three shapes as
    // the user field below (absent / zero-filled / zero-extended).
    parameter EXT_QOS_W           = 0,    // cmd_qos pin width, 0 = no pin on this Master
    parameter INT_QOS_W           = 0,    // in-network QoS width (bus max), 0 = absent
    // derived: cmd_user pin width, floored at 1. A V2001 port list is fixed at
    // elaboration, so EXT_USER_WIDTH_CMD = 0 cannot delete this port (CODING_STYLE
    // 4A.4); the generated top omits the IP pin and the caller ties this one off.
    // Declaring the port [EXT_USER_WIDTH_CMD-1:0] at width 0 would read [-1:0],
    // which is a LEGAL 2-bit ascending range -- silently wrong rather than an error.
    parameter EXT_USER_CMD_PW = (EXT_USER_WIDTH_CMD < 1) ? 1 : EXT_USER_WIDTH_CMD,
    // derived: cmd_qos pin width, floored at 1 for the same V2001 reason as above.
    parameter EXT_QOS_PW = (EXT_QOS_W < 1) ? 1 : EXT_QOS_W,
    // derived: log2(bytes per external beat) = first-beat offset width
    parameter BYTES_PER_BEAT_LOG =
        (EXT_DATA_WIDTH/8 <=   1) ? 0 : (EXT_DATA_WIDTH/8 <=   2) ? 1 :
        (EXT_DATA_WIDTH/8 <=   4) ? 2 : (EXT_DATA_WIDTH/8 <=   8) ? 3 :
        (EXT_DATA_WIDTH/8 <=  16) ? 4 : (EXT_DATA_WIDTH/8 <=  32) ? 5 :
        (EXT_DATA_WIDTH/8 <=  64) ? 6 : (EXT_DATA_WIDTH/8 <= 128) ? 7 : 8,
    // derived: local total_bytes width = beat-count width + log2(bytes/beat); later zero-extended to INT
    parameter LOC_TOTBYTES_W = EXT_LEN_WIDTH + 1 + BYTES_PER_BEAT_LOG,  // +1 for (len+1) carry
    // derived: CMD flit packed width (in-network: INT_QOS head + INT_ID / INT_USER / INT_TOTBYTES)
    parameter INT_CMD_FLIT_W = INT_QOS_W + `LB_OPCODE_WIDTH + INT_ADDR_LOCAL_WIDTH + INT_TOTBYTES_W +
                               INT_USER_WIDTH_CMD + INT_ID_WIDTH + INT_DEST_ID_WIDTH + INT_SRC_ID_WIDTH
) (
    // ---- inputs: clock / reset ----
    // This stage is purely combinational (handshake feed-through); clk/rst_n are
    // carried for uniformity with the other INIU stages and for future pipelining.
    input wire                          clk,           // clock (unused, combinational stage)
    input wire                          rst_n,         // async reset, active low (unused)
    // ---- inputs: external CMD channel signals ----
    input wire  [`LB_OPCODE_WIDTH-1:0]  cmd_opcode,    // CMD opcode (bit3 = write)
    input wire  [EXT_ADDR_WIDTH-1:0]    cmd_addr,      // CMD global address
    input wire  [EXT_LEN_WIDTH-1:0]     cmd_len,       // CMD beat count - 1
    input wire  [EXT_TXNID_WIDTH-1:0]   cmd_ext_txnid, // CMD external transaction ID
    input wire  [EXT_USER_CMD_PW-1:0]   cmd_user,      // CMD user sideband (tied off at width 0)
    input wire  [EXT_QOS_PW-1:0]        cmd_qos,       // CMD qos priority (tied off at width 0)
    input wire                          cmd_valid,     // CMD valid
    input wire                          flit_ready,    // CMD flit ready (backpressure from downstream)
    // ---- outputs: internal CMD flit (packed) ----
    output wire                         cmd_ready,     // CMD ready (backpressure to upstream)
    output wire [INT_CMD_FLIT_W-1:0]    flit_data,     // CMD flit, packed
    output wire                         flit_valid     // CMD flit valid
);
    //------------------------------------------------------------------------
    // Derived local params
    //------------------------------------------------------------------------
    // Offset vector width: EXT_DATA_WIDTH=8 gives BYTES_PER_BEAT_LOG=0, and a
    // zero-width vector is illegal, so clamp the declared width to >= 1 and tie
    // the offset off in that case (1 byte per beat has no in-beat offset).
    localparam OFF_W = (BYTES_PER_BEAT_LOG < 1) ? 1 : BYTES_PER_BEAT_LOG;

    //------------------------------------------------------------------------
    // Declarations (all up front, one per line)
    //------------------------------------------------------------------------
    wire [NUM_REGION-1:0]           region_hit;      // per-region match, at most one-hot
    wire [INT_DEST_ID_WIDTH-1:0]    dest_id;         // decoded DestID
    wire [INT_ADDR_LOCAL_WIDTH-1:0] addr_local;      // address rebased into the region
    wire [INT_SRC_ID_WIDTH-1:0]     src_id;          // this INIU id
    wire [OFF_W-1:0]                first_beat_off;  // first-beat byte offset within a beat
    wire                            off_nz;          // first-beat offset is non-zero
    wire [EXT_LEN_WIDTH:0]          beats;           // beat count = len + 1
    wire [EXT_LEN_WIDTH:0]          beats_adj;       // beat count minus the offset borrow
    wire [OFF_W-1:0]                low_bytes;       // low part of total_bytes = 2^B - offset
    wire [LOC_TOTBYTES_W-1:0]       loc_total_bytes; // total_bytes at local width
    wire [INT_TOTBYTES_W-1:0]       total_bytes;     // total_bytes zero-extended to in-network width
    wire [INT_ID_WIDTH-1:0]         int_id;          // ext_txnid zero-extended to in-network width
    // the flit below the (optional) qos head; = the whole flit at INT_QOS_W = 0
    wire [INT_CMD_FLIT_W-INT_QOS_W-1:0] flit_base;   // CMD flit minus the qos head
    genvar                          r;               // region index
    genvar                          k;               // dest_id bit index
    genvar                          rk;              // region index inside the dest column
    genvar                          b;               // addr_local bit index
    genvar                          rb;              // region index inside the mask column
`ifndef LB_NO_ASSERT
    // synthesis translate_off
    integer                         hit_cnt; // number of regions matched (assertion use only)
    integer                         hc;      // count-loop index
    // synthesis translate_on
`endif

    //------------------------------------------------------------------------
    // Step 1: region match. All NUM_REGION comparators are independent, so they
    // evaluate in parallel; delay is one masked equality, not NUM_REGION of them.
    //------------------------------------------------------------------------
    generate
    for (r = 0; r < NUM_REGION; r = r + 1) begin : g_region_hit
        assign region_hit[r] =
            ((cmd_addr & REGION_MASK[r*EXT_ADDR_WIDTH +: EXT_ADDR_WIDTH]) ==
                         REGION_BASE[r*EXT_ADDR_WIDTH +: EXT_ADDR_WIDTH]);
    end
    endgenerate

    //------------------------------------------------------------------------
    // Step 2a: dest_id = one-hot AND-OR mux over REGION_DEST.
    // For output bit k, gather column k of the constant DEST table and reduce:
    //   dest_id[k] = |(region_hit & {REGION_DEST[r][k] for all r})
    // One reduction-OR per bit => OR tree of depth log2(NUM_REGION), no priority.
    //------------------------------------------------------------------------
    generate
    for (k = 0; k < INT_DEST_ID_WIDTH; k = k + 1) begin : g_dest_bit
        wire [NUM_REGION-1:0] col;                    // column k of the DEST table
        for (rk = 0; rk < NUM_REGION; rk = rk + 1) begin : g_dest_col
            assign col[rk] = REGION_DEST[rk*INT_DEST_ID_WIDTH + k];
        end
        assign dest_id[k] = |(region_hit & col);
    end
    endgenerate

    //------------------------------------------------------------------------
    // Step 2b: addr_local = cmd_addr & ~MASK[hit], same one-hot AND-OR mux.
    // The selected mask bit and the address bit meet in a single AND, so the
    // whole rebase is (OR tree) + 1 AND. Bits above EXT_ADDR_WIDTH are zero.
    //------------------------------------------------------------------------
    generate
    for (b = 0; b < INT_ADDR_LOCAL_WIDTH; b = b + 1) begin : g_local_bit
        if (b < EXT_ADDR_WIDTH) begin : g_in_range
            wire [NUM_REGION-1:0] mcol;               // column b of the ~MASK table
            for (rb = 0; rb < NUM_REGION; rb = rb + 1) begin : g_mask_col
                assign mcol[rb] = ~REGION_MASK[rb*EXT_ADDR_WIDTH + b];
            end
            assign addr_local[b] = cmd_addr[b] & (|(region_hit & mcol));
        end else begin : g_above_range
            assign addr_local[b] = 1'b0;              // local address wider than the global one
        end
    end
    endgenerate

    //------------------------------------------------------------------------
    // total_bytes = (len+1) x bytes_per_beat - first_beat_off
    //   semantics: beats = len + 1 (len=0 -> 1 beat, AXI style)
    //   e.g. EXT_DATA_WIDTH=32b(4B), len=7(8 beats), addr=0x1D
    //        -> 8x4 - (0x1D mod 4) = 32 - 1 = 31
    // Note the product (beats << B) has B zero LSBs, so the subtraction never
    // ripples: the low B bits are the two's complement of the offset and the
    // high part is just decremented when the offset is non-zero. That replaces a
    // full-width subtractor with a B-bit negate plus a (LEN_WIDTH+1)-bit
    // decrement -- one more reason the ID-decode path stays short.
    //------------------------------------------------------------------------
    generate
    if (BYTES_PER_BEAT_LOG < 1) begin : g_one_byte_beat
        assign first_beat_off  = 1'b0;                // 1 byte per beat: no in-beat offset
        assign low_bytes       = 1'b0;                // unused in this branch
        assign loc_total_bytes = beats;               // total_bytes = beat count
    end else begin : g_multi_byte_beat
        assign first_beat_off  = cmd_addr[OFF_W-1:0];
        assign low_bytes       = (~first_beat_off) + 1'b1;   // 2^B - offset (mod 2^B)
        assign loc_total_bytes = {beats_adj, low_bytes};
    end
    endgenerate

    assign off_nz    = |first_beat_off;
    assign beats     = {1'b0, cmd_len} + 1'b1;
    assign beats_adj = beats - off_nz;

    //------------------------------------------------------------------------
    // Field assembly (in-network widths; narrow externals zero-extended)
    // pack order (MSB..LSB): [qos,] opcode, addr_local, total_bytes, user,
    //                        int_id, dest_id, src_id
    //------------------------------------------------------------------------
    assign src_id      = NIU_ID[INT_SRC_ID_WIDTH-1:0];
    assign total_bytes = {{(INT_TOTBYTES_W-LOC_TOTBYTES_W){1'b0}}, loc_total_bytes};
    assign int_id      = {{(INT_ID_WIDTH-EXT_TXNID_WIDTH){1'b0}}, cmd_ext_txnid};

    //------------------------------------------------------------------------
    // The user field has three shapes, picked at elaboration -- the same three the
    // MOD segment takes in lb_iniu_int_core:
    //   INT_USER_WIDTH_CMD = 0    no CMD user field anywhere on this bus; the flit
    //                             is one field shorter and INT_CMD_FLIT_W says so
    //   EXT_USER_WIDTH_CMD = 0    this Master has no user pin, so its slot in the
    //                             bus-wide field is driven to zero
    //   otherwise                 the pin value, zero-extended to the field width
    // Only the third arm slices or repeats: INT is the bus-wide maximum over every
    // Master AND Slave, so EXT > 0 implies INT >= EXT > 0 and the repeat count
    // cannot go negative. That is why the first two arms are keyed on INT and EXT
    // in this order and not the other way round.
    //------------------------------------------------------------------------
    generate
    if (INT_USER_WIDTH_CMD == 0) begin : g_cmd_nouser
        assign flit_base = { cmd_opcode,
                             addr_local,
                             total_bytes,
                             int_id,
                             dest_id,
                             src_id };
    end
    else if (EXT_USER_WIDTH_CMD == 0) begin : g_cmd_userz
        assign flit_base = { cmd_opcode,
                             addr_local,
                             total_bytes,
                             {INT_USER_WIDTH_CMD{1'b0}},
                             int_id,
                             dest_id,
                             src_id };
    end
    else begin : g_cmd_user
        wire [INT_USER_WIDTH_CMD-1:0] int_user; // user zero-extended to in-network width
        assign int_user  = {{(INT_USER_WIDTH_CMD-EXT_USER_WIDTH_CMD){1'b0}}, cmd_user};
        assign flit_base = { cmd_opcode,
                             addr_local,
                             total_bytes,
                             int_user,
                             int_id,
                             dest_id,
                             src_id };
    end
    endgenerate

    //------------------------------------------------------------------------
    // QoS head: prepended as its own layer so the three user arms above stay
    // three (qos would otherwise double them). Same three shapes, same
    // INT-then-EXT keying argument as the user field.
    //------------------------------------------------------------------------
    generate
    if (INT_QOS_W == 0) begin : g_cmd_noqos
        assign flit_data = flit_base;
    end
    else if (EXT_QOS_W == 0) begin : g_cmd_qosz
        assign flit_data = { {INT_QOS_W{1'b0}}, flit_base };
    end
    else begin : g_cmd_qos
        wire [INT_QOS_W-1:0] int_qos; // qos zero-extended to in-network width
        assign int_qos   = {{(INT_QOS_W-EXT_QOS_W){1'b0}}, cmd_qos};
        assign flit_data = { int_qos, flit_base };
    end
    endgenerate

    // combinational decode, handshake feed-through (ID Decode has no internal buffer)
    assign flit_valid  = cmd_valid;
    assign cmd_ready   = flit_ready;

    //------------------------------------------------------------------------
    // Simulation-only guard: the one-hot AND-OR mux above assumes the memory map
    // regions are disjoint. Overlapping regions used to resolve as "last match
    // wins"; now they OR together, so make the violation loud instead of silent.
    // Not synthesized (LB_NO_ASSERT can switch it off for lint-clean builds).
    //------------------------------------------------------------------------
`ifndef LB_NO_ASSERT
    // synthesis translate_off
    always @(*) begin
        hit_cnt = 0;
        for (hc = 0; hc < NUM_REGION; hc = hc + 1)
            if (region_hit[hc]) begin
                hit_cnt = hit_cnt + 1;
            end
        if (cmd_valid && hit_cnt > 1) begin
            $display("[%0t] ERROR %m: addr %h hits %0d memory-map regions; regions must be disjoint",
                     $time, cmd_addr, hit_cnt);
        end
    end
    // synthesis translate_on
`endif

endmodule
