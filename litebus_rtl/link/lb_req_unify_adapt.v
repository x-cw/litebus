//============================================================================
// Filename    : lb_req_unify_adapt.v
// Author      : litebus
// Description : REQ channel width-conversion adapter
// Date        : 2026-08-06
// Revision    : 1.0 initial -- per-round changes are in doc/HISTORY.md
//----------------------------------------------------------------------------
// Wraps lb_link_unify_data to adapt INIU/TNIU REQ flit layout to the gearbox.
//
// INIU/TNIU REQ flit layout: { CMD_FLIT (high) | WD_FLIT (low) }
//   CMD_FLIT = { opcode, addr_local, total_bytes, user, int_id, dest_id, src_id }
//              -- width-independent, passed through as passthru.
//   WD_FLIT  = { w_last, w_strb[DW/8], w_data[DW], dest_id, src_id }
//              -- w_data/w_strb are width-converted; w_last delimits burst;
//                 dest_id/src_id passed through as sideband.
//
// unify_data flit layout (low->high): { data, mask, last, sb, lane, passthru }
//   Direction (UP/DOWN) auto-detected from IN_DW vs OUT_DW.
// Unified credit interface on both sides.
//============================================================================
`include "lb_defines.vh"

module lb_req_unify_adapt #(
    parameter IN_DW         = 128,                                   // input WD data width
    parameter OUT_DW        = 256,                                   // output WD data width
    // CMD field width (both sides equal, passthru). On a bus with atomics the
    // emitter passes the COMBINED {CMD, MOD} width here: the modifier sideband
    // (Adv 15) sits directly below CMD and is as opaque to the gearbox as CMD
    // itself, so the pair travels as one pass-through segment. MOD_W below only
    // shifts the two field taps; at MOD_W = 0 everything reads as before.
    // 56, not 40. The default has to be consistent with the OTHER defaults in
    // this list, because a caller that forgets one parameter silently gets all
    // of them (CODING_STYLE 4A.5). With MOD_W=0 SRC_ID_W=2 DEST_ID_W=2
    // INT_ID_W=8 INT_USER_W_CMD=8 the layout puts CMD_TOTB_LSB at 20,
    // CMD_ADDR_LSB at 36, addr_local ends at 51 and the 4-bit opcode tops it
    // at 55 -- so the segment needs 56 bits. At 40 the addr tap read
    // cmd[51:36] out of a cmd[39:0], which iverilog silently returns X for and
    // SpyGlass rejects outright (SYNTH_5251, and the whole module then fails to
    // synthesise). Every generated instance passes CMD_FLIT_W explicitly, so no
    // product ever used the old value -- which is exactly why it survived.
    parameter CMD_FLIT_W    = 56,
    parameter MOD_W         = 0,                                     // modifier width inside the segment
    // QoS width inside the segment: the QoS field tops the CMD layout, so it is
    // as opaque to the gearbox as the rest of CMD and rides the same segment.
    // Its only effect here is shifting the opcode tap DOWN from the segment's
    // MSB (the two bottom-relative taps below never move); at QOS_W = 0
    // everything reads as before.
    parameter QOS_W         = 0,                                     // qos width inside the segment
    parameter DEST_ID_W     = 2,                                     // dest id width
    parameter SRC_ID_W      = 2,                                     // src id width
    parameter LANE_W        = 7,                                     // addr_lane width = address phase
    parameter TOTBYTES_W    = 16,                                    // total_bytes width
    // int_id width inside CMD (for field location)
    parameter INT_ID_W       = 8,
    parameter INT_USER_W_CMD = 8,                                    // in-network CMD user width
    parameter ADDR_LOCAL_W  = 16,                                    // addr_local width inside CMD
    // ingress FIFO depth = credit to upstream
    parameter INGRESS_DEPTH = 4,
    parameter EGRESS_CREDIT = 4,                                     // egress credit = downstream FIFO depth
    // ---- derived parameters; used in the port list, so they must stay in the
    // ---- parameter list. Not to be overridden externally.
    parameter IN_WD_W       = 1 + IN_DW/8  + IN_DW  + DEST_ID_W + SRC_ID_W,  // input WD segment width
    parameter OUT_WD_W      = 1 + OUT_DW/8 + OUT_DW + DEST_ID_W + SRC_ID_W,  // output WD segment width
    parameter IN_FLIT_W     = CMD_FLIT_W + IN_WD_W,                  // input REQ flit width
    parameter OUT_FLIT_W    = CMD_FLIT_W + OUT_WD_W,                 // output REQ flit width
    // One opaque carry field: the CMD segment rides along with dest/src rather
    // than in a separate passthru field. lb_link_unify_data used to take both and
    // immediately concatenate them, so the split only ever changed a bit position.
    parameter SB_W          = CMD_FLIT_W + DEST_ID_W + SRC_ID_W      // sideband = {cmd,dest,src}
) (
    // ---- inputs ----
    input wire                       clk,            // clock
    input wire                       rst_n,          // async reset, active low
    input wire      [IN_FLIT_W-1:0]  in_flit,        // input REQ flit (INIU side)
    input wire                       in_valid,       // input valid
    input wire                       out_credit_ret, // credit return from downstream
    // ---- outputs ----
    output wire                      in_credit_ret,  // credit return to upstream
    output wire     [OUT_FLIT_W-1:0] out_flit,       // output REQ flit (TNIU side)
    output wire                      out_valid       // output valid
);
    // ---- unify_data flit widths (derived) ----
    localparam U_IN_FLIT_W  = IN_DW  + IN_DW/8  + 1 + TOTBYTES_W + SB_W + LANE_W;
    localparam U_OUT_FLIT_W = OUT_DW + OUT_DW/8 + 1 + TOTBYTES_W + SB_W + LANE_W;
    // CMD field layout (MSB..LSB): opcode, addr_local, total_bytes, user, int_id,
    // dest_id, src_id -- then the MOD segment (if any) below the whole CMD field,
    // which is why both taps shift up by MOD_W from the segment's LSB.
    localparam CMD_TOTB_LSB = MOD_W + SRC_ID_W + DEST_ID_W + INT_ID_W + INT_USER_W_CMD;
    localparam CMD_ADDR_LSB = CMD_TOTB_LSB + TOTBYTES_W;

    // ---- input REQ flit field splits ----
    wire [CMD_FLIT_W-1:0]  cmd;         // CMD segment (high part of in_flit)
    wire [IN_WD_W-1:0]     wd;          // WD segment (low part of in_flit)
    wire                   w_last;      // WD last (burst delimiter)
    wire [IN_DW/8-1:0]     w_strb;      // WD write strobe (becomes mask)
    wire [IN_DW-1:0]       w_data;      // WD write data
    wire [DEST_ID_W-1:0]   w_dest;      // dest id (sideband)
    wire [SRC_ID_W-1:0]    w_src;       // src id (sideband)
    // ---- unify_data input assembly ----
    wire [SB_W-1:0]        u_sb;        // sideband to gearbox = {cmd,dest,src}
    wire [LANE_W-1:0]      u_lane;      // lane to gearbox (tied 0 for REQ)
    wire [U_IN_FLIT_W-1:0] u_in_flit;   // assembled unify input flit
    // ---- unify_data outputs ----
    wire [U_OUT_FLIT_W-1:0] u_out_flit; // unify output flit
    wire                    u_out_v;    // unify output valid
    wire                    u_out_crd;  // unify output credit return (driven)
    // ---- unify_data output field splits ----
    wire [OUT_DW-1:0]      o_data;      // converted data
    wire [OUT_DW/8-1:0]    o_mask;      // converted mask (strb)
    wire                   o_last;      // last
    wire [SB_W-1:0]        o_sb;        // sideband {cmd,dest,src}
    wire [LANE_W-1:0]      o_lane;      // lane (unused for REQ)
    wire [CMD_FLIT_W-1:0]  o_cmd;       // CMD segment, carried inside o_sb
    wire [DEST_ID_W-1:0]   o_dest;      // dest id restored
    wire [SRC_ID_W-1:0]    o_src;       // src id restored
    wire [OUT_WD_W-1:0]    o_wd;        // reassembled output WD segment
    // ---- (address phase, total_bytes) tapped out of the CMD pass-through field ----
    wire [TOTBYTES_W-1:0]   c_totb;     // total_bytes carried in the CMD segment
    wire [ADDR_LOCAL_W-1:0] c_addr;     // addr_local carried in the CMD segment
    wire                    is_wr;      // this request is a write (opcode RSP_WR bit)
    wire [TOTBYTES_W-1:0]   g_totb;     // total_bytes handed to the gearbox
    wire [LANE_W-1:0]       g_lane;     // address phase handed to the gearbox

    // ---- split input REQ flit ----
    assign cmd    = in_flit[IN_FLIT_W-1 -: CMD_FLIT_W];
    assign wd     = in_flit[0 +: IN_WD_W];
    assign w_last = wd[IN_WD_W-1];
    assign w_strb = wd[IN_WD_W-2 -: IN_DW/8];
    assign w_data = wd[SRC_ID_W+DEST_ID_W +: IN_DW];
    assign w_dest = wd[SRC_ID_W +: DEST_ID_W];
    assign w_src  = wd[0 +: SRC_ID_W];

    // ---- assemble unify input flit: {lane, sb, total_bytes, last, mask, data} ----
    // sb carries the whole CMD segment plus dest/src; all of it is opaque to the
    // gearbox, which latches sb once per burst.
    assign u_sb     = { cmd, w_dest, w_src };
    // The (address phase, total_bytes) pair needed for width conversion is tapped
    // directly out of the CMD pass-through field, so the REQ channel needs NO new
    // flit field at all (arch.html section 6.1.6).
    assign c_totb   = cmd[CMD_TOTB_LSB +: TOTBYTES_W];
    assign c_addr   = cmd[CMD_ADDR_LSB +: ADDR_LOCAL_W];
    // opcode sits QOS_W below the segment MSB: the QoS field (if any) tops CMD.
    assign is_wr    = cmd[CMD_FLIT_W-1-QOS_W -: `LB_OPCODE_WIDTH] >> `LB_OPCODE_WR_BIT;
    // --------------------------------------------------------------------
    // A read request is a single beat: its WD field is invalid and it does not
    // form a burst (arch.html section 3.2).
    //   If a read request were width-converted by the (addr, total_bytes) range,
    //   DOWN would expand it into N(OUT_DW) sub-beats. Because CMD rides along as
    //   pass-through, the SAME read request would then be DUPLICATED into several
    //   requests towards the TNIU (measured: 4 reads became 8 with MST=256b,
    //   SLV=128b).
    //   Therefore WD width conversion applies to WRITES only; a read request is
    //   forced to "1 byte, phase 0" so that N(W)=1 at any width and CMD is never
    //   replicated.
    // --------------------------------------------------------------------
    assign g_totb    = is_wr ? c_totb : {{(TOTBYTES_W-1){1'b0}}, 1'b1};
    assign g_lane    = is_wr ? c_addr[LANE_W-1:0] : {LANE_W{1'b0}};
    assign u_lane    = g_lane;
    assign u_in_flit = { u_lane, u_sb, g_totb, w_last, w_strb, w_data };

    lb_link_unify_data #(
        .IN_DATA_W     (IN_DW),          // input data width
        .OUT_DATA_W    (OUT_DW),         // output data width
        .SB_W          (SB_W),           // sideband width = {cmd,dest,src}
        .LANE_W        (LANE_W),         // lane width
        .TOTBYTES_W    (TOTBYTES_W),     // range criterion
        .HAS_MASK      (1),              // WD strb travels with the data
        .INGRESS_DEPTH (INGRESS_DEPTH),  // ingress FIFO depth
        .EGRESS_CREDIT (EGRESS_CREDIT)   // egress credit
    ) u10_uni (
        .clk            (clk),           // clock
        .rst_n          (rst_n),         // reset
        .in_flit        (u_in_flit),     // unify input flit
        .in_valid       (in_valid),      // input valid
        .out_credit_ret (u_out_crd),     // credit from downstream
        .in_credit_ret  (in_credit_ret), // credit to upstream
        .out_flit       (u_out_flit),    // unify output flit
        .out_valid      (u_out_v)        // output valid
    );

    // ---- split unify output flit: {lane, sb, total_bytes, last, mask, data} ----
    assign o_data = u_out_flit[0 +: OUT_DW];
    assign o_mask = u_out_flit[OUT_DW +: OUT_DW/8];
    assign o_last = u_out_flit[OUT_DW + OUT_DW/8];
    assign o_sb   = u_out_flit[OUT_DW + OUT_DW/8 + 1 + TOTBYTES_W +: SB_W];
    assign o_lane = u_out_flit[OUT_DW + OUT_DW/8 + 1 + TOTBYTES_W + SB_W +: LANE_W];
    // sb = {cmd, dest, src}, unpacked from the top down
    assign o_cmd  = o_sb[SB_W-1 -: CMD_FLIT_W];
    assign o_dest = o_sb[SRC_ID_W +: DEST_ID_W];
    assign o_src  = o_sb[0 +: SRC_ID_W];

    // ---- reassemble TNIU REQ flit: {CMD | {last,strb,data,dest,src}} ----
    assign o_wd      = { o_last, o_mask, o_data, o_dest, o_src };
    assign out_flit  = { o_cmd, o_wd };
    assign out_valid = u_out_v;
    assign u_out_crd = out_credit_ret;
endmodule
