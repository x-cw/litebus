//============================================================================
// Filename    : lb_rsp_rd_unify_adapt.v
// Author      : litebus
// Description : RSP_RD channel width-conversion adapter (DOWN typical)
// Date        : 2026-08-06
// Revision    : 1.0 initial -- per-round changes are in doc/HISTORY.md
//----------------------------------------------------------------------------
// Wraps lb_link_unify_data to adapt TNIU/INIU RSP_RD flit layout to the gearbox.
//
// RSP_RD flit layout (low->high), matching lb_defines.vh and lb_tniu_int_core's
// rsp_rd_flit_in -- this file used to carry a SECOND, older version of this list here
// (with a byte_valid field, without total_bytes / trans_last) and the correction
// underneath it, so a reader had to know which of the two to believe:
//   { data[DW], last, trans_last, user, resp, int_id, total_bytes, addr_lane, qos, src_id }
//   - data is what gets width-converted (reverse RSP_RD: network->MST, typically DOWN)
//   - last delimits the burst; trans_last (1 bit) marks the TRANSACTION-level last
//     beat under read interleaving, and equals last when nothing interleaves
//   - addr_lane (phase) -> unify lane segment (pass-through)
//   - total_bytes[TOTBYTES_W] is a per-burst constant replicated on every beat and
//     carried as sideband pass-through; it is the gearbox's range criterion
//   - { src_id, qos, int_id, resp, user, trans_last } -> sideband (reverse routing + meta)
//
// No byte_valid on RSP_RD: the receiver rebuilds the byte mask from
// (addr_lane, total_bytes, beat index). Saving vs carrying it: DW=256 -> 15 bit,
// DW=512 -> 47 bit, DW=1024 -> 111 bit, per beat and per link stage. That is why
// lb_link_unify_data is instantiated with HAS_MASK=0 below.
//
// unify_data flit layout (low->high): { data, mask, last, totbytes, sb, lane }
// Direction auto-detected from IN_DW vs OUT_DW.
//============================================================================
`include "lb_defines.vh"

module lb_rsp_rd_unify_adapt #(
    parameter IN_DW         = 256,                                    // input (network) data width
    parameter OUT_DW        = 128,                                    // output (MST) data width
    parameter SRC_ID_W      = 2,                                      // src id width
    // QoS width: the QoS field sits directly under src_id and, like src_id, is a
    // pure pass-through here -- the two travel as ONE opaque top segment of
    // SRC_ID_W + QOS_W bits (wires i_hi / o_hi below). At QOS_W = 0 everything
    // reads as before.
    parameter QOS_W         = 0,                                      // qos width under src_id
    parameter INT_ID_W      = 8,                                      // network int id width
    parameter INT_USER_W_RSP_RD = 8,                                      // in-network RSP_RD user width
    parameter LANE_W        = 7,                                      // addr_lane width
    parameter TOTBYTES_W    = 16,                                     // total_bytes width
    parameter INGRESS_DEPTH = 4,                                      // ingress FIFO depth
    parameter EGRESS_CREDIT = 4,                                      // egress credit
    // sideband = {src,qos,iid,resp,user,trans_last}
    parameter SB_W          = 1 + INT_USER_W_RSP_RD + `LB_RESP_WIDTH + INT_ID_W + QOS_W + SRC_ID_W,
    // RSP_RD flit (low->high): data, last, trans_last, user, resp, int_id,
    //                          total_bytes, addr_lane, qos, src_id
    //   no byte_valid: rebuilt by the receiver from (addr_lane, total_bytes, beat index)
    parameter IN_FLIT_W     = SRC_ID_W + QOS_W + LANE_W + TOTBYTES_W + INT_ID_W +
                              `LB_RESP_WIDTH + INT_USER_W_RSP_RD + 1 + 1 + IN_DW,
    parameter OUT_FLIT_W    = SRC_ID_W + QOS_W + LANE_W + TOTBYTES_W + INT_ID_W +
                              `LB_RESP_WIDTH + INT_USER_W_RSP_RD + 1 + 1 + OUT_DW
) (
    // ---- inputs ----
    input wire                       clk,            // clock
    input wire                       rst_n,          // async reset, active low
    input wire      [IN_FLIT_W-1:0]  in_flit,        // input RSP_RD flit (TNIU/network side)
    input wire                       in_valid,       // input valid
    input wire                       out_credit_ret, // credit return from downstream
    // ---- outputs ----
    output wire                      in_credit_ret,  // credit return to upstream
    output wire     [OUT_FLIT_W-1:0] out_flit,       // output RSP_RD flit (INIU/MST side)
    output wire                      out_valid       // output valid
);
    // ---- unify_data flit widths (derived) ----
    // HAS_MASK=0 -> 1-bit mask placeholder
    localparam U_IN_FLIT_W  = IN_DW  + 1 + 1 + TOTBYTES_W + SB_W + LANE_W;
    localparam U_OUT_FLIT_W = OUT_DW + 1 + 1 + TOTBYTES_W + SB_W + LANE_W;

    // ---- input RSP_RD flit field splits ----
    wire [IN_DW-1:0]          i_data;     // read data
    wire [TOTBYTES_W-1:0]     i_totb;     // total_bytes
    wire                      i_tl;       // trans_last
    wire                      i_last;     // last
    wire [`LB_RESP_WIDTH-1:0] i_resp;     // response code
    wire [INT_ID_W-1:0]       i_iid;      // network int id
    wire [LANE_W-1:0]         i_lane;     // addr_lane (phase)
    wire [SRC_ID_W+QOS_W-1:0] i_hi;       // {src_id, qos}: routing + priority, opaque pass-through
    // ---- unify_data input assembly ----
    wire [SB_W-1:0]           i_sb;       // sideband = {src,iid,resp,user}
    wire [U_IN_FLIT_W-1:0]    u_in_flit;  // assembled unify input flit
    // ---- unify_data outputs ----
    wire [U_OUT_FLIT_W-1:0]   u_out_flit; // unify output flit
    wire                      u_out_v;    // unify output valid
    wire                      u_out_crd;  // unify output credit return (driven)
    // ---- unify_data output field splits ----
    wire [OUT_DW-1:0]         o_data;     // converted data
    wire [TOTBYTES_W-1:0]     o_totb;     // total_bytes restored
    wire                      o_tl;       // trans_last restored
    wire                      o_last;     // last
    wire [SB_W-1:0]           o_sb;       // sideband
    wire [LANE_W-1:0]         o_lane;     // addr_lane restored
    wire [SRC_ID_W+QOS_W-1:0] o_hi;       // {src_id, qos} restored
    wire [INT_ID_W-1:0]       o_iid;      // int id restored
    wire [`LB_RESP_WIDTH-1:0] o_resp;     // resp restored

    // ---- split input RSP_RD flit ----
    assign i_data = in_flit[0 +: IN_DW];
    assign i_last = in_flit[IN_DW];
    assign i_tl   = in_flit[IN_DW + 1];
    assign i_resp = in_flit[IN_DW + 2 + INT_USER_W_RSP_RD +: `LB_RESP_WIDTH];
    assign i_iid  = in_flit[IN_DW + 2 + INT_USER_W_RSP_RD + `LB_RESP_WIDTH +: INT_ID_W];
    assign i_totb = in_flit[IN_DW + 2 + INT_USER_W_RSP_RD + `LB_RESP_WIDTH + INT_ID_W +: TOTBYTES_W];
    assign i_lane = in_flit[IN_DW + 2 + INT_USER_W_RSP_RD + `LB_RESP_WIDTH + INT_ID_W + TOTBYTES_W +: LANE_W];
    assign i_hi   = in_flit[IN_DW + 2 + INT_USER_W_RSP_RD + `LB_RESP_WIDTH + INT_ID_W
                            + TOTBYTES_W + LANE_W +: SRC_ID_W+QOS_W];

    // ---- assemble unify input flit: {data, mask, last, sb, lane} ----
    // INT_USER_W_RSP_RD = 0 means no NIU on this bus carries an RSP_RD user sideband, so
    // the flit has no such field and neither does the sideband bucket. SB_W drops
    // the addend by itself and stays non-zero: trans_last, resp and the two ids are
    // always there, so the gearbox below never sees an empty sideband.
    generate
    if (INT_USER_W_RSP_RD == 0) begin : g_in_nouser
        assign i_sb = { i_hi, i_iid, i_resp, i_tl };
    end
    else begin : g_in_user
        wire [INT_USER_W_RSP_RD-1:0] i_user; // user sideband
        assign i_user = in_flit[IN_DW + 2 +: INT_USER_W_RSP_RD];
        assign i_sb   = { i_hi, i_iid, i_resp, i_user, i_tl };
    end
    endgenerate
    assign u_in_flit = { i_lane, i_sb, i_totb, i_last, 1'b0, i_data };   // 1-bit mask placeholder

    lb_link_unify_data #(
        .IN_DATA_W     (IN_DW),          // input data width
        .OUT_DATA_W    (OUT_DW),         // output data width
        .SB_W          (SB_W),           // sideband width
        .LANE_W        (LANE_W),         // lane width
        .TOTBYTES_W    (TOTBYTES_W),     // range criterion
        .HAS_MASK      (0),              // RSP_RD carries no mask
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

    // ---- split unify output flit: {data, mask, last, sb, lane} ----
    assign o_data = u_out_flit[0 +: OUT_DW];
    assign o_last = u_out_flit[OUT_DW + 1];
    assign o_totb = u_out_flit[OUT_DW + 1 + 1 +: TOTBYTES_W];
    assign o_sb   = u_out_flit[OUT_DW + 1 + 1 + TOTBYTES_W +: SB_W];
    assign o_lane = u_out_flit[OUT_DW + 1 + 1 + TOTBYTES_W + SB_W +: LANE_W];
    assign o_hi   = o_sb[SB_W-1 -: SRC_ID_W+QOS_W];
    assign o_iid  = o_sb[1 + INT_USER_W_RSP_RD + `LB_RESP_WIDTH +: INT_ID_W];
    assign o_resp = o_sb[1 + INT_USER_W_RSP_RD +: `LB_RESP_WIDTH];
    // trans_last must keep implying last. lb_tniu_rsp_rd_frag establishes that
    // (o_trans_last = o_last && s_tl) but the gearbox DOWN path breaks it: there
    // `out_last` is gated by at_end while `out_sb` -- which is where trans_last
    // rides -- is a registered constant held for all R sub-beats of one input beat
    // (lb_unify_gearbox.v: out_sb = sb_q vs out_last = last_q && at_end). So a
    // transaction's final input beat would hand trans_last to every sub-beat it
    // expands into. Re-project it onto last here, at the one point where the
    // converted flit is reassembled: `last` has already been regenerated correctly
    // per sub-beat, so ANDing with it puts trans_last back on exactly one beat.
    // The projection is idempotent, so chaining adapters (bus_uchain runs three on
    // one RSP_RD path) composes. UP and RATIO==1 already preserve the invariant --
    // there sb_q and last_q are latched from the same input beat.
    assign o_tl   = o_sb[0] && o_last;

    // ---- reassemble INIU RSP_RD flit (low->high) ----
    // Same two shapes as the input split above; the field is present or absent for
    // the whole bus, so the two generates can never disagree.
    generate
    if (INT_USER_W_RSP_RD == 0) begin : g_out_nouser
        assign out_flit = { o_hi, o_lane, o_totb, o_iid, o_resp, o_tl, o_last, o_data };
    end
    else begin : g_out_user
        wire [INT_USER_W_RSP_RD-1:0] o_user; // user restored
        assign o_user   = o_sb[1 +: INT_USER_W_RSP_RD];
        assign out_flit = { o_hi, o_lane, o_totb, o_iid, o_resp, o_user, o_tl,
                            o_last, o_data };
    end
    endgenerate
    assign out_valid = u_out_v;
    assign u_out_crd = out_credit_ret;
endmodule
