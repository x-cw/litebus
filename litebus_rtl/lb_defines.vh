//============================================================================
// Filename    : lb_defines.vh
// Author      : litebus
// Description : project-wide shared definitions (base function model)
// Date        : 2026-08-06
// Revision    : 1.0 initial -- per-round changes are in doc/HISTORY.md
//----------------------------------------------------------------------------
// Litebus interconnect -- project-wide shared definitions (base function model)
//
// Verilog-2001 has no package/struct, so constants, encodings and flit layout
// conventions that are shared across modules live in this header. Every module
// pulls it in with `include "lb_defines.vh".
// Field WIDTHS are themselves configurable/derived and are passed as module
// parameters (see each module header); this file only fixes constant VALUES and
// the field ORDER conventions.
`ifndef LB_DEFINES_VH
`define LB_DEFINES_VH

//--------------------------------------------------------------------------
// Fixed constants (not configurable)
//--------------------------------------------------------------------------
`define LB_OPCODE_WIDTH   4      // cmd_opcode is fixed at 4 bits
// opcode convention: bit[3]=1 write transaction (carries WD), =0 read transaction (no WD)
// It is also what picks the CHANNEL: a write goes on REQ_W, a read on REQ_R. The
// two NIUs assert on a mismatch rather than trusting it, because a request on the
// wrong channel completes silently and wrongly (a write with no data, or a read
// answered on RSP_WR).
`define LB_OPCODE_WR_BIT  3
`define LB_RESP_WIDTH     2      // rsp_rd_resp / rsp_wr_resp fixed at 2 bits (OK/FAIL)

// Response encoding (OK / FAIL, plus the atomic compare-failure code)
`define LB_RESP_OK        2'b00
`define LB_RESP_FAIL      2'b01
// Produced by the Slave IP on a failed ATOMIC_COMPARE (carried on the R response);
// the bus itself never generates or decodes it -- pure pass-through.
`define LB_RESP_ATOMIC_FAIL 2'b10

// Opcode encoding.
// EVERY write opcode MUST have bit[LB_OPCODE_WR_BIT] set: that single bit is what
// the RTL actually decodes (lb_tniu_int/lb_tniu_ext/lb_req_unify_adapt all test
// opcode[LB_OPCODE_WR_BIT] and nothing else). A write value with bit 3 clear is
// silently treated as a read all the way through the fabric.
`define LB_OP_WR          4'h8   // write        (1000: bit3 = 1)
`define LB_OP_RD          4'h1   // read         (0001: bit3 = 0)
// `define LB_OP_MC_WR    4'hA   // advanced: multicast write,  reserved (bit3 = 1)
// `define LB_OP_WR_REDUCE 4'hB  // advanced: write reduction,  reserved (bit3 = 1)
// `define LB_OP_RD_REDUCE 4'h4  // advanced: read reduction,   reserved (bit3 = 0)
// Atomics (Adv 15): write family, so all four set bit3 and ride REQ_W. STORE gets
// only the B response, like a plain write; LOAD/SWAP/COMPARE additionally return
// an R (both must be collected before the TNIU cmd_table entry is released).
// 4'h9 is kept spare for a future write-family code.
`define LB_OP_ATOMIC_STORE   4'hC   // atomic store  : B only        (1100: bit3 = 1)
`define LB_OP_ATOMIC_LOAD    4'hD   // atomic load   : B + R         (1101: bit3 = 1)
`define LB_OP_ATOMIC_SWAP    4'hE   // atomic swap   : B + R         (1110: bit3 = 1)
`define LB_OP_ATOMIC_COMPARE 4'hF   // atomic compare: B + R         (1111: bit3 = 1)

//--------------------------------------------------------------------------
// Internal flit layout conventions (base model)
//   Packing order is MSB..LSB; every field width comes from a module parameter.
//
//   CMD flit = { opcode, addr_local, total_bytes, user_cmd, int_id, dest_id, src_id }
//   WD  flit = { last, strb, data, dest_id, src_id }
//   RSP_RD  flit = { src_id, addr_lane, total_bytes, int_id, resp, user_rsp_rd,
//                trans_last, last, data }
//   RSP_WR  flit = { src_id, int_id, resp, user_rsp_wr }
//
//   REQ_W flit = { CMD flit, WD flit } -- one beat carries both halves, and both
//       are always real: a write request is the only thing on this channel.
//   REQ_R flit = the CMD flit ALONE. A read request has no write data, so it needs
//       no WD half -- which is why REQ_R has no data segment at all, one flit width
//       bus-wide, and no width conversion (nothing to convert).
//
//   The two used to be ONE channel whose flit was always { CMD, WD }, with a read
//   leaving the WD half a don't-care. That wasted the whole data segment on every
//   read, and worse: the switch burst-locks its output port on the WD `last` bit,
//   so a read command queued behind a write burst waited out the entire burst even
//   when it was headed somewhere else.
//
//   The RSP_RD flit has TWO last bits. They are NOT interchangeable:
//       last        FRAGMENT level. The Switch burst-locks on this one, so a
//                   fragment is the fabric's non-interruptible transport unit --
//                   that is what makes read interleaving affordable (see
//                   lb_tniu_rsp_rd_frag). Under interleaving with SLV width >= W_frag
//                   every beat is its own fragment, so this bit is 1 on EVERY beat.
//                   Internal only; it must not reach an IP.
//       trans_last  TRANSACTION level, asserted once per read. This is the ONLY one
//                   that leaves the fabric: lb_iniu_int_core taps it for the Master
//                   IP's rsp_rd_last, so the IP sees the same thing the Slave IP meant
//                   by its own rsp_rd_last. Without interleaving lb_tniu_int_core drives
//                   both bits from the Slave's rsp_rd_last, so the two coincide.
//   Invariant: trans_last implies last.
//       established by  lb_tniu_rsp_rd_frag  (o_trans_last = o_last && s_tl)
//       broken by       lb_unify_gearbox DOWN, where out_sb is held for all R
//                       sub-beats while out_last is gated by at_end -- trans_last
//                       rides sb[0], so it would land on every sub-beat
//       restored in     lb_rsp_rd_unify_adapt (o_tl = o_sb[0] && o_last)
//       checked by      lb_iniu_int_core, which shouts if a flit arrives with
//                       trans_last set and last clear (that beat would be masked
//                       away and the IP would never see the burst end)
//   src_id sits at the TOP of the RSP_RD / RSP_WR flits: those travel back towards the
//       source, so the field the switch routes on is in the same place a forward
//       switch finds dest_id.
//   The WD half's dest_id / src_id are tied to zero: the REQ_W flit is routed by
//       its CMD half, which carries the real dest_id.
//
//   Authority for each line, checked when this block is edited:
//       CMD    lb_iniu_id_decode.v   assign flit_data
//       WD     lb_iniu_int_core.v    assign wd_flit_in
//       REQ_R  lb_iniu_int_core.v    the REQ_R arm sends dec_rq_r_flit unwrapped,
//                                    so the CMD line above IS this line
//       REQ_W  lb_iniu_int_core.v    assign req_flit_in
//       RSP_RD     lb_tniu_int_core.v    assign rsp_rd_flit_in   (+ the RSP_RDF_*_LSB ladder in
//                                    lb_iniu_int_core.v, which unpacks it)
//       RSP_WR     lb_tniu_int_core.v    assign rsp_wr_flit_in   (+ the RSP_WRF_*_LSB ladder)
//   The generator emits one flit-width localparam per flit from the SAME ordered
//   field list (gen/lb_flit.py) and publishes it as the <bus>.structure report, so a
//   field added here has exactly one other place to be added.
//
//   Note: the generic downstream blocks (pipe / credit / bca) move the flit as an
//       opaque [W-1:0] bit vector and never decode fields. Only id_decode and the
//       NIU adaptors care about field meaning.
//--------------------------------------------------------------------------

// Clock / reset convention: active-low rst_n, asynchronous assert and synchronous
// release (implemented inside each module).

`endif // LB_DEFINES_VH
