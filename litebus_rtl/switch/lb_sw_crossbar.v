//============================================================================
// Filename    : lb_sw_crossbar.v
// Author      : litebus
// Description : switch core (Switch stage 3 crossbar: route + arbitrate)
// Date        : 2026-08-06
// Revision    : 1.0 initial -- per-round changes are in doc/HISTORY.md
//----------------------------------------------------------------------------
// Each of NUM_IN inputs looks up ROUTE_TABLE by the DestID inside its flit to
// get the target output, filtered by CONNECTIVITY_MASK for real links. When
// several inputs contend for one output, a per-out arbiter (lb_sw_arbiter)
// decides, switching the winning input's flit to that output.
//
// ARB_SCHEME = 2 (RR-QoS) adds a dominance filter in FRONT of the arbiter:
// only the requesters at the highest pending QoS reach it, equal QoS falls
// back to plain RR (the arbiter itself is untouched and treats 2 as RR).
// The compared value is the head flit's qos at XBAR_FIFO_DEPTH == 0, and the
// MAXIMUM over the crosspoint FIFO's valid entries at depth > 0 (tracked by
// lb_sw_qos_track) -- a queued high-QoS flit lifts its whole queue so its
// blockers drain first. A burst lock outranks QoS: the locked input keeps
// the output regardless (grant_r below), QoS only picks new burst heads.
// At QOS_W == 0 every arm folds away and the module is bit-identical to the
// QoS-free build (ARB_SCHEME = 2 then degrades to RR; the generator refuses
// to emit that combination in the first place).
//
// Verilog-2001 has no 2-D ports; per-port signals use flattened 1-D buses:
//   in_flit[i]  = in_flit_bus [i*FLIT_W +: FLIT_W]
//   out_flit[o] = out_flit_bus[o*FLIT_W +: FLIT_W]
//
// Route/connectivity tables (compile-time constants, injected by the top from
// the address map / connectivity / topology):
//   ROUTE_TABLE       : flat [NUM_DEST*OW-1:0], ROUTE_TABLE[dest] = out index.
//   CONNECTIVITY_MASK : flat [NUM_IN*NUM_OUT-1:0], [i*NUM_OUT+o]=1 means in i
//                       -> out o is a real link.
//   DestID position in flit: DEST_LSB +: DEST_ID_WIDTH.
//============================================================================
`include "lb_defines.vh"

module lb_sw_crossbar #(
    parameter NUM_IN            = 2,                          // number of inputs
    parameter NUM_OUT           = 3,                          // number of outputs
    parameter FLIT_W            = 64,                         // flit width
    parameter ARB_SCHEME        = 1,                          // arbitration scheme
    parameter CONNECTIVITY_MASK = {(NUM_IN*NUM_OUT){1'b1}},   // in-out connectivity
    parameter HAS_LAST          = 0,                          // 1 = WD/RSP_RD (txn lock); 0 = CMD/RSP_WR
    parameter LAST_POS          = 0,                          // last bit position (valid when HAS_LAST=1)
    // ---- QoS (RR-QoS, ARB_SCHEME = 2 only). QOS_W = 0 folds every QoS arm
    // ---- away; the top injects both only on an arb_scheme = 2 switch, so a
    // ---- QoS-free instance keeps its historical parameter list.
    parameter QOS_W             = 0,                          // qos field width in the flit; 0 = no QoS
    parameter QOS_LSB           = 0,                          // qos field position (valid when QOS_W>0)
    // ---- Crosspoint FIFO depth: one FIFO per (input, output) pair.
    // ---- 0 = none, a pure combinational bypass identical to the original
    // ---- structure. Nonzero relieves head-of-line blocking: without it an input
    // ---- has ONE queue whose head requests ONE output, so a blocked head stalls
    // ---- that input even while other outputs sit idle. Measured on bus_16x16:
    // ---- inputs blocked 31% of cycles while outputs were idle 52%.
    // ---- Legal: 0 (bypass) or any integer 2..64. The 64 is lb_sw_out_fifo's
    // ---- 6-bit pointer address; the "any integer" is 2026-08-25 (the power-of-two
    // ---- restriction lived in the pointer increment, and that arm now wraps at
    // ---- DEPTH -- see that file). Keep it LAST in this list and keep the default a bare
    // ---- integer -- lint_strict P2 reads defaults for forward references and its
    // ---- parser mis-splits a default containing a comma.
    parameter XBAR_FIFO_DEPTH   = 0                           // per-crosspoint FIFO depth; 0 = bypass
) (
    // ---- inputs ----
    input wire                        clk,              // clock
    input wire                        rst_n,            // async reset, active low
    input wire  [NUM_IN*FLIT_W-1:0]   in_flit_bus,      // flattened input flits
    input wire  [NUM_IN*NUM_OUT-1:0]  in_routed_oh_bus, // per-input target output, one-hot, [i*NUM_OUT+o]
    input wire  [NUM_IN-1:0]          in_valid,         // input valids
    input wire  [NUM_OUT-1:0]         out_ready,        // output readies
    // ---- outputs ----
    output wire [NUM_IN-1:0]          in_ready,         // input readies
    output wire [NUM_OUT*FLIT_W-1:0]  out_flit_bus,     // flattened output flits
    output wire [NUM_OUT-1:0]         out_valid         // output valids
);
    //------------------------------------------------------------------------
    // Generate vars and locals
    //------------------------------------------------------------------------
    localparam XPN = NUM_IN * NUM_OUT;       // crosspoint count; index = i*NUM_OUT + o
    // Thermometer levels of the QoS lattice, clamped so the vectors below are
    // declarable at QOS_W = 0 (where they are tied off and never read).
    localparam QLVL = (QOS_W < 1) ? 1 : (1 << QOS_W) - 1;

    genvar gi;                                   // input gen index
    genvar go;                                   // output gen index
    genvar gl;                                   // qos thermometer level gen index

    //------------------------------------------------------------------------
    // Declarations (all up front, one per line)
    //------------------------------------------------------------------------
    wire [XPN-1:0]     routed_oh;                // one-hot target output per input
    wire [NUM_IN-1:0]  grant      [0:NUM_OUT-1]; // final one-hot grant per output (incl. burst-lock)
    wire [NUM_OUT-1:0] out_fire;                 // output handshake fire

    // ---- crosspoint state, flat so the per-output blocks can read across inputs.
    // ---- Flat vectors rather than cross-generate hierarchical references, which
    // ---- are where simulators most often disagree.
    wire [XPN-1:0]        xp_head_v;             // crosspoint has a flit for its output
    wire [XPN-1:0]        xp_wr_rdy;             // crosspoint can accept the ingress head
    wire [XPN*FLIT_W-1:0] xp_head;               // crosspoint head flit (depth>0 only)

    // ---- QoS thermometers (all-zero unless ARB_SCHEME=2 needs them) ----
    wire [NUM_IN*QLVL-1:0] in_therm;             // per-input head-flit qos thermometer (depth 0)
    wire [XPN*QLVL-1:0]    xp_therm;             // per-crosspoint queue-max qos thermometer (depth>0)

    // The route decode used to live here: slice the DestID out of the head flit,
    // index ROUTE_TABLE by it (a NUM_DEST:1 mux on a parameter), compare the
    // result against every output index. All of that sat in the SAME cycle as the
    // arbitration and the flit mux, in front of them.
    //
    // It now happens on the ingress FIFO's WRITE side, in lb_sw_route, and rides
    // through the FIFO next to its flit -- so this module is handed the answer.
    // The move costs nothing in latency (the same FIFO, the same pointers) and
    // nothing in cycles: the write path was previously just pin-to-memory and had
    // most of a clock period spare, while this path had none.
    assign routed_oh = in_routed_oh_bus;

    //------------------------------------------------------------------------
    // Head-flit QoS thermometer decode (depth 0 only): at XBAR_FIFO_DEPTH == 0
    // the flit competing at an output IS the ingress head, so its qos is cut
    // straight out of in_flit_bus and decoded against each level -- constant
    // comparisons, no state. Garbage while in_valid is low is harmless: the
    // g_qosf filter masks every thermometer with req, which contains valid.
    // At depth > 0 the per-crosspoint trackers below own the answer instead.
    //------------------------------------------------------------------------
    generate
    if (QOS_W > 0 && XBAR_FIFO_DEPTH == 0) begin : g_qos_head
        for (gi = 0; gi < NUM_IN; gi = gi + 1) begin : g_qh_in
            for (gl = 1; gl <= QLVL; gl = gl + 1) begin : g_qh_lvl
                assign in_therm[gi*QLVL + gl-1] =
                    (in_flit_bus[gi*FLIT_W + QOS_LSB +: QOS_W] >= gl);
            end
        end
    end else begin : g_qos_head_off
        assign in_therm = {(NUM_IN*QLVL){1'b0}};
    end
    endgenerate

    //------------------------------------------------------------------------
    // Crosspoint FIFOs: the write side is a demux on the ingress read port (the
    // head flit is offered to exactly the FIFO its routed output owns), the read
    // side is what the per-output mux selects from.
    //
    // This is the whole point of the change. With XBAR_FIFO_DEPTH == 0 an input's
    // ready is the OUTPUT's grant, so a head flit bound for a busy output freezes
    // that input's queue and its later flits for idle outputs with it. With a
    // crosspoint FIFO the ready is only "this crosspoint has room", so the input
    // keeps draining until D flits have piled up for one destination.
    //
    // Depth 0 must stay bit-identical to the original: the g_bypass arm folds each
    // expression back to what it was, instantiates nothing, and holds no state.
    // The mux below likewise branches on the parameter rather than routing
    // everything through xp_head -- at depth 0 that would cost 16 x FLIT_W bits of
    // continuous-assign copying per switch for no benefit.
    //------------------------------------------------------------------------
    generate
    for (gi = 0; gi < NUM_IN; gi = gi + 1) begin : g_xp
        for (go = 0; go < NUM_OUT; go = go + 1) begin : g_xpo
            if (XBAR_FIFO_DEPTH == 0) begin : g_bypass
                assign xp_head_v[gi*NUM_OUT + go] = in_valid[gi]
                                                 && routed_oh[gi*NUM_OUT + go];
                assign xp_wr_rdy[gi*NUM_OUT + go] = grant[go][gi] && out_ready[go];
                // tied, never read: the mux and sel_last take the bypass arm too
                assign xp_head[(gi*NUM_OUT + go)*FLIT_W +: FLIT_W] = {FLIT_W{1'b0}};
                // tied, never read: the g_qosf filter takes in_therm at depth 0
                assign xp_therm[(gi*NUM_OUT + go)*QLVL +: QLVL] = {QLVL{1'b0}};
            end else begin : g_fifo
                wire              xwr;   // demuxed write enable
                wire              xrdy;  // this crosspoint not full
                wire              xvld;  // this crosspoint not empty
                wire [FLIT_W-1:0] xhead; // this crosspoint head flit
                assign xwr = in_valid[gi]
                          && routed_oh[gi*NUM_OUT + go]
                          && CONNECTIVITY_MASK[gi*NUM_OUT + go];
                lb_sw_out_fifo #(
                    .WIDTH     (FLIT_W),                   // flit width
                    .DEPTH     (XBAR_FIFO_DEPTH)           // this crosspoint's depth
                ) u10_xp (
                    .clk       (clk),                      // clock
                    .rst_n     (rst_n),                    // reset
                    .in_data   (in_flit_bus[gi*FLIT_W +: FLIT_W]),  // ingress head flit
                    .in_valid  (xwr),                      // demuxed write enable
                    .out_ready (grant[go][gi] && out_ready[go]),    // pop when taken
                    .in_ready  (xrdy),                     // not full
                    .out_data  (xhead),                    // head flit
                    .out_valid (xvld)                      // not empty
                );
                assign xp_head_v[gi*NUM_OUT + go] = xvld;
                // MASK gates the READY too, not just the write enable: a flit routed
                // to an unconnected output must wedge its input exactly as it does at
                // depth 0 (there grant can never be given, so in_ready stays 0).
                assign xp_wr_rdy[gi*NUM_OUT + go] = xrdy
                                                 && CONNECTIVITY_MASK[gi*NUM_OUT + go];
                assign xp_head[(gi*NUM_OUT + go)*FLIT_W +: FLIT_W] = xhead;
                // ---- queue-max QoS tracker: mirrors this crosspoint's write
                // ---- and read fires exactly (push = xwr && not-full is the
                // ---- FIFO's own do_wr; pop implies xvld because grant is a
                // ---- subset of req which contains xp_head_v).
                if (QOS_W > 0) begin : g_qt
                    lb_sw_qos_track #(
                        .QOS_W    (QOS_W),                     // qos field width
                        .DEPTH    (XBAR_FIFO_DEPTH)            // tracked FIFO depth
                    ) u11_qt (
                        .clk      (clk),                       // clock
                        .rst_n    (rst_n),                     // reset
                        .push     (xwr && xrdy),               // crosspoint write fire
                        .push_qos (in_flit_bus[gi*FLIT_W + QOS_LSB +: QOS_W]),  // written flit's qos
                        .pop      (grant[go][gi] && out_ready[go]),             // crosspoint read fire
                        .pop_qos  (xhead[QOS_LSB +: QOS_W]),   // head flit's qos
                        .occ_ge   (xp_therm[(gi*NUM_OUT + go)*QLVL +: QLVL])    // queue-max thermometer
                    );
                end else begin : g_qt_off
                    assign xp_therm[(gi*NUM_OUT + go)*QLVL +: QLVL] = {QLVL{1'b0}};
                end
            end
        end
    end
    endgenerate

    //------------------------------------------------------------------------
    // Input ready: the ingress head moves as soon as the crosspoint its routed
    // output owns has room. At depth 0 that ready IS the output's grant, which is
    // the original expression and the source of head-of-line blocking.
    //
    // Selecting that crosspoint with routed_oh rather than indexing by
    // routed_out replaces a NUM_OUT:1 mux -- whose SELECT was the route-table
    // lookup of the same cycle -- with an AND against a vector that already
    // exists, plus an OR reduction. Same function, log depth instead of a mux
    // hanging off a decode.
    //------------------------------------------------------------------------
    generate
    for (gi = 0; gi < NUM_IN; gi = gi + 1) begin : g_inrdy
        assign in_ready[gi] = |(xp_wr_rdy[gi*NUM_OUT +: NUM_OUT]
                              & routed_oh[gi*NUM_OUT +: NUM_OUT]);
    end
    endgenerate

    //------------------------------------------------------------------------
    // Per-output arbitration + switching
    //------------------------------------------------------------------------
    generate
    for (go = 0; go < NUM_OUT; go = go + 1) begin : g_out
        // ---- declarations (all up front, one per line) ----
        wire [NUM_IN-1:0]  req;       // per-input request for this output
        wire [NUM_IN-1:0]  req_arb;   // requests after the QoS dominance filter (= req unless RR-QoS)
        wire [NUM_IN-1:0]  arb_grant; // arbiter one-hot grant
        wire [NUM_IN-1:0]  grant_r;   // final grant (arbiter, or the locked input)
        wire               arb_take;  // the ARBITER's grant is what fired this cycle
        reg                locked;    // this output is locked to an input's burst
        reg  [NUM_IN-1:0]  lock_oh;   // which input it is locked to, one-hot
        wire [NUM_IN-1:0]  last_col;  // last bit of each input's candidate flit
        wire               sel_last;  // last bit of the flit emitted this cycle
        reg  [FLIT_W-1:0]  sel;       // flit selected onto this output
        integer            k;         // output-mux loop index

        // ---- request collect: this out's crosspoint from input i has a flit ----
        // At depth 0 xp_head_v folds back to "valid & routed here", so this is the
        // original expression; at depth>0 the request comes from the crosspoint
        // head, which is what decouples it from the ingress head.
        for (gi = 0; gi < NUM_IN; gi = gi + 1) begin : g_req
            assign req[gi] = xp_head_v[gi*NUM_OUT + go]
                           && CONNECTIVITY_MASK[gi*NUM_OUT + go];
        end

        // ---- QoS dominance filter (RR-QoS): only the top-QoS requesters reach
        // ---- the arbiter; equal QoS falls back to plain RR on the survivors.
        // Thermometers make this three vector operations: t is the OR of every
        // requester's thermometer (bits 1..max end up set), and a requester is
        // dominated exactly when t has a bit its own thermometer lacks. Three
        // provable properties hang off that algebra:
        //   grant <= req_arb <= req      no grant to a non-requester;
        //   req != 0 -> req_arb != 0     the max is attained by a requester, so
        //                                the filter is work-conserving -- and at
        //                                all-qos-0 t == 0, req_arb == req, which
        //                                is bit-for-bit today's behavior;
        //   equal-QoS fairness           survivors hit the untouched RR mask.
        // Burst locks outrank all of this: grant_r below takes (req & lock_oh)
        // while locked, from the UNfiltered req, so a locked low-QoS burst
        // finishes no matter who shows up (see g_deadlock's atomicity note).
        if (ARB_SCHEME == 2 && QOS_W > 0) begin : g_qosf
            wire [NUM_IN*QLVL-1:0] otherm;  // this output's per-input thermometer view
            wire [QLVL-1:0]        t;       // merged thermometer of this out's requesters
            wire [NUM_IN-1:0]      exceed;  // some requester's max is strictly above mine
            reg  [QLVL-1:0]        t_acc;   // OR accumulation (balances into a tree)
            integer                q;       // accumulation loop index
            for (gi = 0; gi < NUM_IN; gi = gi + 1) begin : g_qf_in
                if (XBAR_FIFO_DEPTH == 0) begin : g_qf_head
                    assign otherm[gi*QLVL +: QLVL] = in_therm[gi*QLVL +: QLVL];
                end else begin : g_qf_fifo
                    assign otherm[gi*QLVL +: QLVL] =
                        xp_therm[(gi*NUM_OUT + go)*QLVL +: QLVL];
                end
                assign exceed[gi] = |(t & ~otherm[gi*QLVL +: QLVL]);
            end
            // OR accumulation over disjoint terms, same idiom as the flit mux
            // below: no dependency between iterations, balances to log depth.
            always @(*) begin
                t_acc = {QLVL{1'b0}};
                for (q = 0; q < NUM_IN; q = q + 1)
                    t_acc = t_acc | (otherm[q*QLVL +: QLVL] & {QLVL{req[q]}});
            end
            assign t = t_acc;
            assign req_arb = req & ~exceed;
        end else begin : g_qosf_off
            assign req_arb = req;   // zero-cell alias: FIX/RR read exactly as before
        end

        // ---- did the arbiter's OWN decision get consumed this cycle? ----
        // grant_take must mean "the grant lb_sw_arbiter produced is the one that
        // moved", not "something moved". During a HAS_LAST burst lock the final
        // grant below is (req & lock_oh) -- the arbiter did not decide it, and
        // arb_grant is not even used -- yet the output still fires on every beat.
        // Feeding out_fire straight in advanced the round-robin state once per
        // BEAT instead of once per burst, so a B-beat burst skipped B-1
        // requesters. With gcd(B, NUM_IN) > 1 that is not unfairness, it is
        // starvation: at B=2, NUM_IN=4 the output alternates 1,3,1,3 and inputs
        // 0 and 2 never win. Guarded by the SAME expression as grant_r below, so
        // the two cannot disagree. HAS_LAST is a parameter, so this folds away
        // entirely on the RSP_WR switches. See verify/unit/tb_sw_arbiter_fair.v arm D.
        assign arb_take = out_fire[go] && !(HAS_LAST && locked);

        // ---- arbiter grant for a new burst head (used when unlocked) ----
        // Fed the FILTERED requests: at scheme 0/1 req_arb is a zero-cell alias
        // of req, at scheme 2 the arbiter (which treats 2 as RR internally)
        // round-robins over the top-QoS survivors only.
        lb_sw_arbiter #(
            .NUM_REQ    (NUM_IN),        // number of requesters
            .ARB_SCHEME (ARB_SCHEME)     // arbitration scheme
        ) u20_arb (
            .clk        (clk),           // clock
            .rst_n      (rst_n),         // reset
            .req        (req_arb),       // requests (QoS-filtered at scheme 2)
            .grant_take (arb_take),      // the arbiter's own grant was consumed
            .grant      (arb_grant)      // one-hot grant
        );

        // ---- final grant: locked -> only the locked input; else -> arbiter ----
        // Holding the lock as a one-hot vector instead of a binary index removes
        // three O(NUM_IN) structures at once: the NUM_IN:1 mux that read
        // req[lock_in], the decoder that wrote grant_r[lock_in], and the
        // one-hot-to-binary encoder on the register's D-input below. The masked
        // grant is now one AND row.
        assign grant_r       = (HAS_LAST && locked) ? (req & lock_oh) : arb_grant;
        assign grant[go]     = grant_r;
        assign out_valid[go] = |grant[go];
        assign out_fire[go]  = out_valid[go] && out_ready[go];

        // ---- whether the flit being emitted this cycle carries last ----
        // MUST read the same flit the mux below emits. At depth>0 that is the
        // crosspoint head, NOT the ingress head -- taking it from in_flit_bus
        // compiles and elaborates but unlocks the output on a DIFFERENT flit's last
        // bit, so bursts interleave on the output wire.
        for (gi = 0; gi < NUM_IN; gi = gi + 1) begin : g_lastcol
            if (XBAR_FIFO_DEPTH == 0) begin : g_lc_bypass
                assign last_col[gi] = in_flit_bus[gi*FLIT_W + LAST_POS];
            end else begin : g_lc_fifo
                assign last_col[gi] = xp_head[(gi*NUM_OUT + go)*FLIT_W + LAST_POS];
            end
        end
        assign sel_last = |(grant[go] & last_col);

        // ---- lock update (HAS_LAST only): last beat never stays locked ----
        // Key: only update lock state with a real grant (|grant[go], i.e.
        // out_valid); otherwise an out_fire combinational glitch (grant=0)
        // would wrongly lock lock_in to a stale value -> deadlock.
        always @(posedge clk or negedge rst_n) begin
            if (!rst_n) begin
                locked  <= 1'b0;
                lock_oh <= {NUM_IN{1'b0}};
            end else if (HAS_LAST) begin
                if (out_fire[go] && (|grant[go])) begin   // only with a real granted input
                    if (sel_last) begin
                        locked <= 1'b0;                    // last beat (incl. single beat): unlock
                    end else if (!locked) begin
                        locked  <= 1'b1;                   // head beat (non-last): lock granted input
                        lock_oh <= grant[go];              // grant is already one-hot; no encoder
                    end
                end
            end
        end
`ifdef LOCK_DBG // hand-defined debug trace: report burst-lock grabs on this output, sim only
        always @(posedge clk) if (rst_n && HAS_LAST && out_fire[go] && !locked && !sel_last) begin
            $display("[%0t] xbar out%0d LOCK grant=%b sel_last=%b", $time, go, grant[go], sel_last);
        end
`endif // LOCK_DBG

        // ---- switch: drive the granted crosspoint's flit to this output ----
        // One-hot AND-OR rather than a last-wins if-chain. The if-chain form is
        // NUM_IN cascaded FLIT_W-wide 2:1 muxes -- O(NUM_IN) deep on 646 bits at
        // the widest shipped config, and its select arrives from the arbiter in
        // this same cycle. An OR accumulation over disjoint terms has no such
        // dependency between iterations, so it balances into a tree of depth
        // log2(NUM_IN); this is the same idiom lb_iniu_id_decode already uses for
        // its one-hot region select.
        always @(*) begin
            sel = {FLIT_W{1'b0}};
            for (k = 0; k < NUM_IN; k = k + 1)
                if (XBAR_FIFO_DEPTH == 0) begin
                    sel = sel | (in_flit_bus[k*FLIT_W +: FLIT_W]
                                 & {FLIT_W{grant[go][k]}});
                end
                else begin
                    sel = sel | (xp_head[(k*NUM_OUT + go)*FLIT_W +: FLIT_W]
                                 & {FLIT_W{grant[go][k]}});
                end
        end
        assign out_flit_bus[go*FLIT_W +: FLIT_W] = sel;

`ifndef LB_NO_ASSERT
        // synthesis translate_off
        // The burst-lock's liveness rests on ONE property of everything upstream:
        // the flits of one burst are CONSECUTIVE in the input stream. RSP_RD gets it from
        // lb_tniu_rsp_rd_frag emitting a fragment atomically; REQ from one write burst per
        // master ext port with the CMD half repeated, so routed_out cannot move
        // mid-burst. Violate it and this output is locked to an input whose head has
        // gone elsewhere: the lock never releases. That deadlock exists at depth 0 too
        // -- crosspoint FIFOs do not create it -- but they hide the evidence, because
        // the stuck flit sits inside a crosspoint instead of at the ingress head where
        // a waveform shows it at once. So name it here, where it happens.
        // Fires ONLY on a split burst: crosspoint full -> req[locked one]=1, no
        // fire; ingress empty -> in_valid=0, no fire; normal transient ->
        // routed_out==go.
        // Written per input rather than as one check indexed by the lock, because
        // the lock is now held one-hot: lock_oh[gi] picks out the same input the
        // old lock_in named, without reintroducing the NUM_IN:1 muxes that
        // indexing it would need.
        for (gi = 0; gi < NUM_IN; gi = gi + 1) begin : g_deadlock
            always @(posedge clk)
                if (rst_n && HAS_LAST && locked && lock_oh[gi]
                          && !req[gi] && in_valid[gi]
                          && !routed_oh[gi*NUM_OUT + go]) begin
                    // A burst was split in the input stream, so this lock can
                    // never release: deadlock. %b is one-hot over the outputs.
                    $display("ERROR %m: out%0d locked to in%0d but in%0d's head routes to %b",
                             go, gi, gi, routed_oh[gi*NUM_OUT +: NUM_OUT]);
                end
        end
        // synthesis translate_on
`endif
    end
    endgenerate
endmodule
