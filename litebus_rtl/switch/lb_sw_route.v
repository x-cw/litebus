//============================================================================
// Filename    : lb_sw_route.v
// Author      : litebus
// Description : Switch pure routing core (generic, one per channel)
// Date        : 2026-08-06
// Revision    : 1.0 initial -- per-round changes are in doc/HISTORY.md
//----------------------------------------------------------------------------
// Width conversion is decoupled to unify-LINK; the Switch only does routing +
// arbitration with equal port width. Per-input data flow:
//   credit_ingress -> crossbar (route + arbitrate) -> out_fifo -> credit_egress
// All transaction sideband (strb / byte_valid / addr_lane / total_bytes /
// dest ...) travels inside FLIT_W and is passed through untouched; route does
// not interpret it (dest feeds crossbar routing, at position DEST_LSB).
//
// Unified credit interface: input in_flit+in_valid returns in_credit_ret;
// output out_flit+out_valid takes out_credit_ret. Flattened 1-D buses +
// generate (the generator expands per concrete NUM_IN/NUM_OUT). One instance
// per channel CMD/WD/RSP_RD/RSP_WR (each with its own FLIT_W).
//============================================================================
`include "lb_defines.vh"

module lb_sw_route #(
    parameter NUM_IN            = 2,                          // number of inputs
    parameter NUM_OUT           = 3,                          // number of outputs
    parameter NUM_DEST          = 3,                          // number of destinations
    parameter FLIT_W            = 64,                         // flit width
    parameter DEST_ID_WIDTH     = 4,                          // dest id width
    parameter DEST_LSB          = 0,                          // dest id position in flit
    parameter ARB_SCHEME        = 1,                          // 1 = round-robin
    // ---- derived: out index width. Declared here (not a body localparam)
    // ---- because ROUTE_TABLE's default and the port widths reference it.
    // ---- Do not override it externally (CODING_STYLE 4.1/4.2).
    parameter OW                = (NUM_OUT <= 2)   ? 1 :        // = clog2(NUM_OUT)
                                  (NUM_OUT <= 4)   ? 2 :
                                  (NUM_OUT <= 8)   ? 3 :
                                  (NUM_OUT <= 16)  ? 4 :
                                  (NUM_OUT <= 32)  ? 5 :
                                  (NUM_OUT <= 64)  ? 6 :
                                  (NUM_OUT <= 128) ? 7 : 8,
    parameter ROUTE_TABLE       = {(NUM_DEST*OW){1'b0}},      // dest -> out mapping
    parameter CONNECTIVITY_MASK = {(NUM_IN*NUM_OUT){1'b1}},   // in-out connectivity
    // ---- per-port depths, packed one DEPTH_W-bit field per port, field p at
    // ---- [p*DEPTH_W +: DEPTH_W]. They are vectors rather than scalars because
    // ---- a port facing a short local link and one facing a long cross-die link
    // ---- need different credit budgets; forcing one value per switch made the
    // ---- whole switch pay for its worst port. The _VEC suffix is deliberate:
    // ---- a caller still passing a scalar would silently configure port 0 only.
    // bits per field (do not override; 16 covers the 1..256 range)
    parameter DEPTH_W           = 16,
    // per-input ingress FIFO depth = credit to upstream
    parameter [NUM_IN*DEPTH_W-1:0]  CREDIT_DEPTH_VEC  = {NUM_IN{{{(DEPTH_W-8){1'b0}}, 8'd4}}},
    // per-output out_fifo depth (contention buffer)
    parameter [NUM_OUT*DEPTH_W-1:0] OUT_DEPTH_VEC     = {NUM_OUT{{{(DEPTH_W-8){1'b0}}, 8'd4}}},
    // per-output egress credit = downstream ingress depth
    parameter [NUM_OUT*DEPTH_W-1:0] EGRESS_CREDIT_VEC = {NUM_OUT{{{(DEPTH_W-8){1'b0}}, 8'd4}}},
    // 1 = WD/RSP_RD (transaction lock); 0 = CMD/RSP_WR (single beat)
    parameter HAS_LAST          = 0,
    // last bit position in flit (valid when HAS_LAST=1)
    parameter LAST_POS          = 0,
    // ---- QoS (RR-QoS, ARB_SCHEME = 2 only): pure pass-through to the crossbar,
    // ---- which owns every QoS structure (head decode / crosspoint trackers /
    // ---- dominance filter). This module's FIFO and route decode never read the
    // ---- field. 0 = no QoS, everything folds away.
    parameter QOS_W             = 0,                          // qos field width in the flit; 0 = no QoS
    parameter QOS_LSB           = 0,                          // qos field position (valid when QOS_W>0)
    // ---- One FIFO per (input, output) crosspoint, inside the crossbar. 0 = none,
    // ---- structurally identical to the original design. Nonzero relieves
    // ---- head-of-line blocking: without it an input's single queue exposes one
    // ---- head that requests one output, so a blocked head freezes that input even
    // ---- while other outputs idle. It is a per-switch scalar rather than a packed
    // ---- per-port vector because a crosspoint is a PAIR of connections and the
    // ---- .topo depth keys hang on single connections -- there is no edge to put it
    // ---- on. Legal: 0 (bypass) or any integer 2..64; see lb_sw_out_fifo's pointers.
    parameter XBAR_FIFO_DEPTH   = 0                           // per-crosspoint FIFO depth; 0 = bypass
) (
    // ---- inputs ----
    input wire                          clk,            // clock
    input wire                          rst_n,          // async reset, active low
    input wire  [NUM_IN*FLIT_W-1:0]     in_flit_bus,    // flattened input flits
    input wire  [NUM_IN-1:0]            in_valid,       // input valids
    input wire  [NUM_OUT-1:0]           out_credit_ret, // output credit returns
    // ---- outputs ----
    output wire [NUM_IN-1:0]            in_credit_ret,  // input credit returns
    output wire [NUM_OUT*FLIT_W-1:0]    out_flit_bus,   // flattened output flits
    output wire [NUM_OUT-1:0]           out_valid       // output valids
);
    //------------------------------------------------------------------------
    // Generate vars
    //------------------------------------------------------------------------
    genvar i;                               // input index
    genvar o;                               // output index

    //------------------------------------------------------------------------
    // Ingress-side wires (credit_ingress -> crossbar)
    //------------------------------------------------------------------------
    wire [NUM_IN*FLIT_W-1:0]  ig_flit_bus;  // ingress output flits
    wire [NUM_IN*NUM_OUT-1:0] ig_routed_oh; // ingress output target, one-hot per input
    wire [NUM_IN*NUM_OUT-1:0] in_routed_oh; // route decode of the flit at the input pin
    wire [NUM_IN-1:0]         ig_valid;     // ingress output valids
    wire [NUM_IN-1:0]         ig_ready;     // ingress output readies (from crossbar)

    //------------------------------------------------------------------------
    // Route decode, on the ingress FIFO's WRITE side.
    //
    // This is the switch's address decode: take the DestID out of the flit, look
    // it up in ROUTE_TABLE (a NUM_DEST:1 mux over a constant), and turn the answer
    // into a one-hot over outputs. It used to sit inside the crossbar, reading the
    // FIFO's asynchronous read port -- which put it in series with arbitration and
    // the flit mux, all inside one clock.
    //
    // Here it reads in_flit_bus at the module pin instead, and the result is
    // written into the FIFO alongside its flit. Nothing about the handshake, the
    // depth, the pointers or the credit contract changes, so the flit still takes
    // exactly the same number of cycles; the decode simply moves to the side of
    // the register that had slack. Cost is NUM_OUT extra bits per FIFO entry
    // (8 x 4 = 32 flops per input port against a 646-bit flit).
    //------------------------------------------------------------------------
    generate for (i = 0; i < NUM_IN; i = i + 1) begin : g_route
        wire [DEST_ID_WIDTH-1:0] dest_i;     // DestID carried by this input's flit
        wire [OW-1:0]            routed_out; // target output index
        assign dest_i     = in_flit_bus[i*FLIT_W + DEST_LSB +: DEST_ID_WIDTH];
        assign routed_out = ROUTE_TABLE[dest_i*OW +: OW];
        for (o = 0; o < NUM_OUT; o = o + 1) begin : g_oh
            assign in_routed_oh[i*NUM_OUT + o] = (routed_out == o[OW-1:0]);
        end
    end endgenerate

    //------------------------------------------------------------------------
    // Crossbar-side wires (crossbar -> out_fifo)
    //------------------------------------------------------------------------
    wire [NUM_OUT*FLIT_W-1:0] xb_flit_bus; // crossbar output flits
    wire [NUM_OUT-1:0]        xb_valid;    // crossbar output valids
    wire [NUM_OUT-1:0]        xb_ready;    // crossbar output readies (from out_fifo)

    //------------------------------------------------------------------------
    // Per-input credit_ingress: credit-in -> valid-ready
    //------------------------------------------------------------------------
    // The FIFO carries {routed_oh, flit} as one opaque word: the decode has to
    // arrive at the crossbar in the same cycle as the flit it belongs to, and the
    // FIFO already provides exactly that ordering. Widening it is the whole
    // mechanism -- there is no second structure to keep in step.
    generate for (i = 0; i < NUM_IN; i = i + 1) begin : g_ing
        wire [FLIT_W+NUM_OUT-1:0] ig_wr; // {route one-hot, flit} into the FIFO
        wire [FLIT_W+NUM_OUT-1:0] ig_rd; // {route one-hot, flit} out of the FIFO
        assign ig_wr = {in_routed_oh[i*NUM_OUT +: NUM_OUT], in_flit_bus[i*FLIT_W +: FLIT_W]};
        lb_credit_ingress #(
            .WIDTH         (FLIT_W + NUM_OUT),                    // flit plus its route decode
            .DEPTH         (CREDIT_DEPTH_VEC[i*DEPTH_W +: DEPTH_W])  // this input's ingress FIFO depth
        ) u10_ing (
            .clk           (clk),            // clock
            .rst_n         (rst_n),          // reset
            .in_data       (ig_wr),          // input flit + route
            .in_valid      (in_valid[i]),    // input valid
            .out_ready     (ig_ready[i]),    // output ready
            .out_data      (ig_rd),          // output flit + route
            .out_valid     (ig_valid[i]),    // output valid
            .credit_return (in_credit_ret[i])  // credit to upstream
        );
        assign ig_flit_bus [i*FLIT_W  +: FLIT_W]  = ig_rd[0 +: FLIT_W];
        assign ig_routed_oh[i*NUM_OUT +: NUM_OUT] = ig_rd[FLIT_W +: NUM_OUT];
    end endgenerate

    //------------------------------------------------------------------------
    // Crossbar: routing + arbitration (channel-agnostic, sideband pass-through)
    //------------------------------------------------------------------------
    // The crossbar no longer owns the route table: NUM_DEST / DEST_ID_WIDTH /
    // DEST_LSB / ROUTE_TABLE / OW all stop at this level now, because the decode
    // they describe happens above, on the FIFO write side.
    lb_sw_crossbar #(
        .NUM_IN            (NUM_IN),             // inputs
        .NUM_OUT           (NUM_OUT),            // outputs
        .FLIT_W            (FLIT_W),             // flit width
        .ARB_SCHEME        (ARB_SCHEME),         // arbitration scheme
        .CONNECTIVITY_MASK (CONNECTIVITY_MASK),  // connectivity
        .HAS_LAST          (HAS_LAST),           // transaction lock enable
        .LAST_POS          (LAST_POS),           // last bit position
        .QOS_W             (QOS_W),              // qos field width (0 = no QoS)
        .QOS_LSB           (QOS_LSB),            // qos field position
        .XBAR_FIFO_DEPTH   (XBAR_FIFO_DEPTH)     // per-crosspoint FIFO depth
    ) u20_xbar (
        .clk               (clk),                // clock
        .rst_n             (rst_n),              // reset
        .in_flit_bus       (ig_flit_bus),        // ingress flits
        .in_routed_oh_bus  (ig_routed_oh),       // ingress route decode, one-hot
        .in_valid          (ig_valid),           // ingress valids
        .out_ready         (xb_ready),           // crossbar out readies
        .in_ready          (ig_ready),           // ingress readies
        .out_flit_bus      (xb_flit_bus),        // crossbar out flits
        .out_valid         (xb_valid)            // crossbar out valids
    );

    //------------------------------------------------------------------------
    // Per-output out_fifo -> credit_egress
    //------------------------------------------------------------------------
    generate for (o = 0; o < NUM_OUT; o = o + 1) begin : g_out
        wire [FLIT_W-1:0] of_data; // out_fifo output flit
        wire              of_v;    // out_fifo output valid
        wire              of_r;    // out_fifo output ready (from egress)
        lb_sw_out_fifo #(
            .WIDTH         (FLIT_W),                             // flit width
            .DEPTH         (OUT_DEPTH_VEC[o*DEPTH_W +: DEPTH_W]) // this output's fifo depth
        ) u30_of (
            .clk           (clk),            // clock
            .rst_n         (rst_n),          // reset
            .in_data       (xb_flit_bus[o*FLIT_W +: FLIT_W]),  // crossbar flit
            .in_valid      (xb_valid[o]),    // crossbar valid
            .out_ready     (of_r),           // fifo out ready
            .in_ready      (xb_ready[o]),    // crossbar ready
            .out_data      (of_data),        // fifo out flit
            .out_valid     (of_v)            // fifo out valid
        );
        lb_credit_egress #(
            .WIDTH         (FLIT_W),                                 // flit width
            .CREDIT_INIT   (EGRESS_CREDIT_VEC[o*DEPTH_W +: DEPTH_W]) // this output's initial credit
        ) u40_eg (
            .clk           (clk),            // clock
            .rst_n         (rst_n),          // reset
            .in_data       (of_data),        // fifo flit
            .in_valid      (of_v),           // fifo valid
            .credit_return (out_credit_ret[o]),  // credit from downstream
            .in_ready      (of_r),           // fifo ready
            .out_data      (out_flit_bus[o*FLIT_W +: FLIT_W]),  // egress out flit
            .out_valid     (out_valid[o])    // egress out valid
        );
    end endgenerate
endmodule
