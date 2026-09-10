//============================================================================
// Filename    : lb_link_unify_data.v
// Author      : litebus
// Description : data-channel width-conversion LINK (WD/RSP_RD, credit + gearbox)
// Date        : 2026-08-06
// Revision    : 1.0 initial -- per-round changes are in doc/HISTORY.md
//----------------------------------------------------------------------------
// UNIFY-type LINK for the WD and RSP_RD data channels (both move data identically;
// only the mask semantics differ: WD mask = strb write-enable, RSP_RD mask =
// byte_valid filter). For transport, mask is just a bit string packed/unpacked
// alongside data, so one implementation serves both. Mask meaning is applied at
// the producer (TNIU) / consumer (INIU), not here.
//
// Data flow:
//   credit_ingress (take upstream flit, return credit)
//   -> lb_unify_gearbox (data+mask pack/unpack)
//   -> credit_egress (hold downstream credit, send)
// Single clock domain. Unified credit interface (forward flit+valid, reverse
// credit_return).
//
// flit layout (flattened, unpacked internally):
//   { lane[LANE_W], sb[SB_W], total_bytes[TOTBYTES_W], last, mask[MW], data[DW] }
//   - data/mask go through the gearbox (UP widen / DOWN narrow)
//   - sb travels with the burst untouched; lane = addr_lane, pure passthru
//
// `sb` is the ONE opaque carry field. It used to be split in two -- SB_W plus a
// separate PASSTHRU_W sitting at the top of the flit -- but the two were handled
// identically: this module concatenated them into a single vector, handed that to
// the gearbox's single in_sb port, and split it back at the same offsets. The
// gearbox latches the whole thing once per burst and never looks inside. The RSP_RD
// adapter already proved one field is enough by packing five things
// ({src,iid,resp,user,trans_last}) into SB_W alone with PASSTHRU_W=0. So the
// split bought nothing but a different bit position, and is gone; the REQ adapter
// now packs {cmd, dest, src} into sb.
//
// GEAR_MODE UP narrow->wide / DOWN wide->narrow; RATIO = UNIFY_DATA/PORT_DATA.
//============================================================================
`include "lb_defines.vh"

module lb_link_unify_data #(
    parameter IN_DATA_W    = 128,            // input data width (direction auto by IN/OUT)
    parameter OUT_DATA_W   = 256,            // output data width
    parameter SB_W         = 8,              // sideband: everything carried untouched per burst
    parameter LANE_W       = 7,              // addr_lane width
    parameter TOTBYTES_W   = 16,             // total_bytes: range criterion for phase-aligned conversion
    // 1 = mask travels with data (WD strb); 0 = RSP_RD, rebuilt from range
    parameter HAS_MASK     = 1,
    parameter INGRESS_DEPTH = 4,             // ingress FIFO depth = credit to upstream
    parameter EGRESS_CREDIT = 4,             // egress credit = downstream ingress FIFO depth
    // ---- derived parameters; used in the port list, so they must stay in the
    // ---- parameter list. Not to be overridden externally.
    parameter IMW           = HAS_MASK ? IN_DATA_W/8  : 1,  // internal input mask field width
    parameter OMW           = HAS_MASK ? OUT_DATA_W/8 : 1,  // internal output mask field width
    parameter IN_FLIT_W     = IN_DATA_W  + IMW + 1 + TOTBYTES_W + SB_W + LANE_W,  // input flit width
    parameter OUT_FLIT_W    = OUT_DATA_W + OMW + 1 + TOTBYTES_W + SB_W + LANE_W   // output flit width
) (
    // ---- inputs ----
    input wire                    clk,            // clock
    input wire                    rst_n,          // async reset, active low
    input wire  [IN_FLIT_W-1:0]   in_flit,        // upstream flit (credit)
    input wire                    in_valid,       // upstream valid
    input wire                    out_credit_ret, // credit return from downstream
    // ---- outputs ----
    output wire                   in_credit_ret,  // credit return to upstream
    output wire [OUT_FLIT_W-1:0]  out_flit,       // downstream flit (credit)
    output wire                   out_valid       // downstream valid
);
    //------------------------------------------------------------------------
    // ingress-side wires
    //------------------------------------------------------------------------
    wire [IN_FLIT_W-1:0]   ig_flit; // ingress output flit
    wire                   ig_v;    // ingress output valid
    wire                   ig_r;    // ingress output ready (from gearbox)
    wire [IN_DATA_W-1:0]   i_data;  // input data segment
    wire [IMW-1:0]         i_mask;  // input mask segment
    wire [TOTBYTES_W-1:0]  i_totb;  // total_bytes segment
    wire                   i_last;  // input last
    wire [SB_W-1:0]        i_sb;    // input sideband
    wire [LANE_W-1:0]      i_lane;  // input lane

    //------------------------------------------------------------------------
    // gearbox-side wires
    //------------------------------------------------------------------------
    wire [OUT_DATA_W-1:0]   g_data; // gearbox output data
    wire [OUT_DATA_W/8-1:0] g_mask; // gearbox output mask (full width)
    wire [TOTBYTES_W-1:0]   g_totb; // total_bytes pass-through
    wire [LANE_W-1:0]       g_lane; // gearbox output lane
    wire                    g_last; // gearbox output last
    wire                    g_v;    // gearbox output valid
    wire                    g_r;    // gearbox output ready (from egress)
    wire [SB_W-1:0]         g_sb;   // gearbox output sideband
    wire [OUT_FLIT_W-1:0]   eg_in;  // assembled egress input flit

    //------------------------------------------------------------------------
    // ingress: credit-in -> valid-ready
    //------------------------------------------------------------------------
    lb_credit_ingress #(
        .WIDTH         (IN_FLIT_W),      // flit width
        .DEPTH         (INGRESS_DEPTH)   // ingress FIFO depth
    ) u10_ing (
        .clk           (clk),            // clock
        .rst_n         (rst_n),          // reset
        .in_data       (in_flit),        // input flit
        .in_valid      (in_valid),       // input valid
        .out_ready     (ig_r),           // output ready
        .out_data      (ig_flit),        // output flit
        .out_valid     (ig_v),           // output valid
        .credit_return (in_credit_ret)   // credit to upstream
    );

    //------------------------------------------------------------------------
    // Split ingress flit (input width)
    //------------------------------------------------------------------------
    assign i_data = ig_flit[0 +: IN_DATA_W];
    assign i_mask = ig_flit[IN_DATA_W +: IMW];
    assign i_last = ig_flit[IN_DATA_W + IMW];
    assign i_totb = ig_flit[IN_DATA_W + IMW + 1 +: TOTBYTES_W];
    assign i_sb   = ig_flit[IN_DATA_W + IMW + 1 + TOTBYTES_W +: SB_W];
    assign i_lane = ig_flit[IN_DATA_W + IMW + 1 + TOTBYTES_W + SB_W +: LANE_W];

    //------------------------------------------------------------------------
    // gearbox: data+mask pack/unpack, sb/passthru/lane pass-through
    //------------------------------------------------------------------------
    lb_unify_gearbox #(
        .IN_DATA_W     (IN_DATA_W),      // input data width
        .OUT_DATA_W    (OUT_DATA_W),     // output data width
        .SB_W          (SB_W),           // sideband width
        .LANE_W        (LANE_W),         // lane width = address phase
        .TOTBYTES_W    (TOTBYTES_W),     // range criterion
        .HAS_MASK      (HAS_MASK)        // 1 = mask travels; 0 = rebuilt from range
    ) u20_gear (
        .clk             (clk),          // clock
        .rst_n           (rst_n),        // reset
        .in_data         (i_data),       // input data
        .in_mask         ({{(IN_DATA_W/8-IMW){1'b0}}, i_mask}),  // input mask (zero-padded when HAS_MASK=0)
        .in_sb           (i_sb),         // input sideband
        .in_lane         (i_lane),       // input lane (= address phase)
        .in_total_bytes  (i_totb),       // total_bytes
        .in_last         (i_last),       // input last
        .in_valid        (ig_v),         // input valid
        .out_ready       (g_r),          // output ready
        .in_ready        (ig_r),         // input ready
        .out_data        (g_data),       // output data
        .out_mask        (g_mask),       // output mask
        .out_sb          (g_sb),         // output sideband
        .out_lane        (g_lane),       // output lane
        .out_total_bytes (g_totb),       // total_bytes pass-through
        .out_last        (g_last),       // output last
        .out_valid       (g_v)           // output valid
    );
    //------------------------------------------------------------------------
    // Assemble egress flit (output width): lane + sb + total_bytes + last + mask + data
    //------------------------------------------------------------------------
    assign eg_in = { g_lane, g_sb, g_totb, g_last, g_mask[OMW-1:0], g_data };

    //------------------------------------------------------------------------
    // egress: valid-ready in -> credit out
    //------------------------------------------------------------------------
    lb_credit_egress #(
        .WIDTH         (OUT_FLIT_W),     // flit width
        .CREDIT_INIT   (EGRESS_CREDIT)   // initial credit
    ) u30_eg (
        .clk           (clk),            // clock
        .rst_n         (rst_n),          // reset
        .in_data       (eg_in),          // input flit
        .in_valid      (g_v),            // input valid
        .credit_return (out_credit_ret), // credit from downstream
        .in_ready      (g_r),            // input ready
        .out_data      (out_flit),       // output flit
        .out_valid     (out_valid)       // output valid
    );
endmodule
