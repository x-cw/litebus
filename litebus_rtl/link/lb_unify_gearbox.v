//============================================================================
// Filename    : lb_unify_gearbox.v
// Author      : litebus
// Description : phase-aligned data width converter
// Date        : 2026-08-06
// Revision    : 1.0 initial -- per-round changes are in doc/HISTORY.md
//----------------------------------------------------------------------------
// Spec: doc/html/arch.html section 6.1 (Link SPEC section 6).
//
// Axiom: beat boundaries are decided by the ADDRESS, not by the transaction
//        start. Byte at address A always lands in beat (A/W), lane (A%W),
//        where W is the data width in bytes. Placement is therefore a pure
//        function of the address and is independent of how many (and in which
//        order) width conversions happened upstream.
//
//   R1 placement : as above
//   R2 beat set  : beats k in [addr/W, (addr+tb-1)/W]; a beat that lies fully
//                  outside the valid range is dropped on the spot
//   R3 beat count: N(W) = (addr+tb-1)/W - addr/W + 1, i.e. always the minimum
//                  possible for that width
//   R4 validity  : lane j of beat k is valid <=> addr <= k*W+j < addr+tb
//
//   UP   (narrow->wide): input beat k_in goes to sub-slot j = k_in % RATIO of
//        output beat k_in/RATIO. The FIRST beat may have j != 0; slots below j
//        are tied to 0 with mask 0 (this is the "redundancy introduced by
//        narrow-to-wide"). Flush when j == RATIO-1 or on in_last.
//   DOWN (wide->narrow): the input beat expands into RATIO sub-beats; only the
//        ones intersecting [addr, addr+tb) are emitted. The criterion is the
//        RANGE, not whether the mask is non-zero: for a sparse write an all-zero
//        strb sub-beat is legal, and using the mask would move `last` onto the
//        wrong sub-beat.
//   RATIO == 1: feed-through.
//
// Notes:
//   * in_lane IS the address phase (addr[LANE_W-1:0]); no separate addr_lo port
//     is needed. in_total_bytes is a per-burst constant replicated on every beat.
//   * With HAS_MASK = 0 (the RSP_RD channel carries no byte_valid) the output mask is
//     rebuilt from the range, bit-identical to what the producer would generate.
//   * No function/endfunction is used (CODING_STYLE 6.1); the range mask is
//     produced by an always @(*) + for loop.
//
// Golden model: verify/models/phase_gearbox_model.py
//============================================================================
`include "lb_defines.vh"

module lb_unify_gearbox #(
    parameter IN_DATA_W  = 256,                                     // input data width in bits
    parameter OUT_DATA_W = 512,                                     // output data width in bits
    parameter SB_W       = 8,                                       // sideband width, travels with the burst
    parameter LANE_W     = 6,                                       // addr_lane width = clog2(max data bytes)
    parameter TOTBYTES_W = 16,                                      // total_bytes width
    // 1 = mask travels with data; 0 = rebuild from range
    parameter HAS_MASK   = 1,
    // ---- derived parameters below; must not be overridden externally ----
    parameter IN_MASK_W  = IN_DATA_W/8,                             // input mask width
    parameter OUT_MASK_W = OUT_DATA_W/8,                            // output mask width
    parameter IN_B       = IN_DATA_W/8,                             // input width in bytes
    parameter OUT_B      = OUT_DATA_W/8,                            // output width in bytes
    parameter IS_UP      = (IN_DATA_W < OUT_DATA_W) ? 1 : 0,        // 1 = narrow->wide
    parameter NARROW     = (IN_DATA_W < OUT_DATA_W) ? IN_DATA_W  : OUT_DATA_W,  // narrow side width
    parameter WIDE       = (IN_DATA_W < OUT_DATA_W) ? OUT_DATA_W : IN_DATA_W,   // wide side width
    parameter RATIO      = WIDE / NARROW,                           // conversion ratio
    parameter RW         = (RATIO <= 2)  ? 1 :                        // sub-slot index width
                           (RATIO <= 4)  ? 2 :
                           (RATIO <= 8)  ? 3 :
                           (RATIO <= 16) ? 4 : 5,
    // byte-offset / beat-index arithmetic width
    parameter OFS_W      = LANE_W + TOTBYTES_W + 1
) (
    // ---- inputs ----
    input wire                     clk,             // clock
    input wire                     rst_n,           // async reset, active low
    input wire  [IN_DATA_W-1:0]    in_data,         // input data
    input wire  [IN_MASK_W-1:0]    in_mask,         // input byte mask, valid when HAS_MASK=1
    input wire  [SB_W-1:0]         in_sb,           // input sideband, pure pass-through
    input wire  [LANE_W-1:0]       in_lane,         // addr[LANE_W-1:0], the address phase
    input wire  [TOTBYTES_W-1:0]   in_total_bytes,  // per-burst constant, valid byte count
    input wire                     in_last,         // input burst last beat
    input wire                     in_valid,        // input valid
    input wire                     out_ready,       // downstream ready
    // ---- outputs ----
    output wire                    in_ready,        // input ready (backpressure)
    output wire [OUT_DATA_W-1:0]   out_data,        // output data
    output wire [OUT_MASK_W-1:0]   out_mask,        // output byte mask
    output wire [SB_W-1:0]         out_sb,          // output sideband
    output wire [LANE_W-1:0]       out_lane,        // addr phase, passed through
    output wire [TOTBYTES_W-1:0]   out_total_bytes, // total_bytes, passed through
    output wire                    out_last,        // output burst last beat
    output wire                    out_valid        // output valid
);

    //------------------------------------------------------------------------
    // Derived local params. IN_B / OUT_B are powers of two for every width the
    // generator can emit, so every "/ IN_B", "* IN_B" and "% OUT_B" below is a
    // shift or a mask -- written as such rather than left to the synthesiser,
    // because a non-power-of-two width would otherwise infer a real divider and
    // nothing would say so. See the elaboration check at the end of the file.
    //------------------------------------------------------------------------
    localparam LIB = (IN_B <= 1)   ? 0 :        // = clog2(IN_B)
                     (IN_B <= 2)   ? 1 :
                     (IN_B <= 4)   ? 2 :
                     (IN_B <= 8)   ? 3 :
                     (IN_B <= 16)  ? 4 :
                     (IN_B <= 32)  ? 5 :
                     (IN_B <= 64)  ? 6 :
                     (IN_B <= 128) ? 7 : 8;
    localparam LOB = (OUT_B <= 1)   ? 0 :       // = clog2(OUT_B)
                     (OUT_B <= 2)   ? 1 :
                     (OUT_B <= 4)   ? 2 :
                     (OUT_B <= 8)   ? 3 :
                     (OUT_B <= 16)  ? 4 :
                     (OUT_B <= 32)  ? 5 :
                     (OUT_B <= 64)  ? 6 :
                     (OUT_B <= 128) ? 7 : 8;

    //------------------------------------------------------------------------
    // Declarations (all up front)
    //------------------------------------------------------------------------
    wire [OFS_W-1:0] lo;      // valid range low bound  (byte offset)
    wire [OFS_W-1:0] hi;      // valid range high bound (exclusive)
    wire [OFS_W-1:0] k_in;    // absolute input beat index of the current beat
    wire [OFS_W-1:0] in_base; // byte offset of the current input beat
    wire             in_fire; // input handshake
    reg  [OFS_W-1:0] beat_n;  // input beat index within the burst
    reg              first;   // current beat is the burst's first beat

    //------------------------------------------------------------------------
    // Valid byte range [lo, hi), relative to the LANE_W-aligned base address.
    //
    // in_base used to be built as lo_algn + beat_n*IN_B with lo_algn = (lo/IN_B)*IN_B
    // -- a divide feeding a multiply feeding an add, all OFS_W wide (24 bits at
    // the usual LANE_W/TOTBYTES_W), and all of it in front of the UP arm's barrel
    // shifter. It is the same value as ((lo >> LIB) + beat_n) << LIB, which is one
    // add between two wirings, and it exposes the absolute beat index k_in that
    // the UP arm needed anyway.
    //------------------------------------------------------------------------
    assign lo      = {{(OFS_W-LANE_W){1'b0}}, in_lane};
    assign hi      = lo + {{(OFS_W-TOTBYTES_W){1'b0}}, in_total_bytes};
    assign k_in    = (lo >> LIB) + beat_n;
    assign in_base = k_in << LIB;
    assign in_fire = in_valid && in_ready;

    //------------------------------------------------------------------------
    // Per-burst input beat counter. This is the ONLY transaction state a stage
    // needs; no transaction context is stored anywhere.
    //------------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            beat_n <= {OFS_W{1'b0}};
            first  <= 1'b1;
        end else if (in_fire) begin
            if (in_last) begin
                beat_n <= {OFS_W{1'b0}};
                first  <= 1'b1;
            end else begin
                beat_n <= beat_n + 1'b1;
                first  <= 1'b0;
            end
        end
    end

    generate
    //========================================================================
    // RATIO == 1 : feed-through
    //========================================================================
    if (RATIO == 1) begin : g_thru
        // ---- declarations ----
        reg  [OUT_MASK_W-1:0] rmask_t; // range-derived mask
        integer               bt;      // byte loop index

        // Deliberately NOT given the beat/lane split the DOWN and UP arms use
        // below. Measured, it costs two levels here (22 -> 24) while saving 18%
        // of the cells, and this pass trades area for depth, not the other way
        // round. The difference is where the inputs come from: on those arms the
        // range is already registered (lo_q / hi_q / base_q), so the split starts
        // at flops, whereas here it stacks on top of the live hi adder.
        always @(*) begin
            rmask_t = {OUT_MASK_W{1'b0}};
            for (bt = 0; bt < OUT_B; bt = bt + 1)
                if (((in_base + bt) >= lo) && ((in_base + bt) < hi)) begin
                    rmask_t[bt] = 1'b1;
                end
        end
        assign out_data        = in_data;
        assign out_mask        = HAS_MASK ? in_mask : rmask_t;
        assign out_sb          = in_sb;
        assign out_lane        = in_lane;
        assign out_total_bytes = in_total_bytes;
        assign out_last        = in_last;
        assign out_valid       = in_valid;
        assign in_ready        = out_ready;

    //========================================================================
    // DOWN : wide -> narrow. Expand into RATIO sub-beats, emit only the ones
    // intersecting the valid range (leading/trailing dead sub-beats dropped).
    //========================================================================
    end else if (!IS_UP) begin : g_dn
        // ---- declarations ----
        reg  [RW-1:0]         j_lo_c;  // first intersecting sub-beat
        reg  [RW-1:0]         j_hi_c;  // last  intersecting sub-beat
        reg                   seen_c;  // an intersecting sub-beat was found
        integer               jj;      // sub-beat loop index
        wire [OFS_W-LOB-1:0]  d_sub;   // sub-beat being emitted, in OUT_B units
        wire [OFS_W-LOB-1:0]  d_lob;   // sub-beat holding the range's first byte (registered)
        wire [LOB-1:0]        d_lol;   // lane of the range's first byte
        wire [OFS_W-LOB-1:0]  d_hib;   // sub-beat holding the range's end
        wire [LOB-1:0]        d_hil;   // lane of the range's end
        wire                  d_gt;    // this sub-beat is entirely at/after lo
        wire                  d_eqlo;  // lo falls inside this sub-beat
        wire                  d_lt;    // this sub-beat is entirely before hi
        wire                  d_eqhi;  // hi falls inside this sub-beat
        reg  [WIDE-1:0]       d_q;     // captured input data
        reg  [WIDE/8-1:0]     m_q;     // captured input mask
        reg  [SB_W-1:0]       sb_q;    // captured sideband
        reg  [LANE_W-1:0]     ln_q;    // captured addr phase
        reg  [TOTBYTES_W-1:0] tb_q;    // captured total_bytes
        reg  [OFS_W-1:0]      base_q;  // captured beat byte offset
        reg  [OFS_W-1:0]      lo_q;    // captured range low
        reg  [OFS_W-1:0]      hi_q;    // captured range high
        reg  [RW-1:0]         cnt;     // sub-beat being emitted
        reg  [RW-1:0]         j_hi_q;  // last sub-beat to emit
        reg                   last_q;  // captured in_last
        reg                   busy;    // holding a captured beat
        wire                  at_end;  // emitting the last sub-beat
        reg  [OUT_MASK_W-1:0] rmask_d; // range-derived mask
        integer               bd;      // byte loop index

        // ---- which sub-beats of this input beat intersect [lo,hi) ----
        // Left as the original scan on purpose. It looks like the worst construct
        // in the file -- an O(RATIO) if-ladder serialised on seen_c, with jj*OUT_B
        // recomputed each step -- and the closed form is easy to write and was
        // proven equivalent by exhaustion:
        //     lo>>LOB <= s <= (hi-1)>>LOB   over sub-beats s, clipped to this beat
        // But measured, the closed form is WORSE: 29 -> 40 levels. The reason is
        // scale. RATIO is 2 or 4 for every width pair the generator can emit, so
        // the ladder is two or four steps of compares against constants, while the
        // closed form needs (hi-1) -- a second OFS_W-wide carry chain behind the
        // hi adder, worth about eleven levels on its own, and the optimiser
        // re-serialises it however the source is arranged. O(RATIO) beats O(1)
        // when RATIO is 4 and the constant is 24 bits wide.
        always @(*) begin
            j_lo_c = {RW{1'b0}};
            j_hi_c = {RW{1'b0}};
            seen_c = 1'b0;
            for (jj = 0; jj < RATIO; jj = jj + 1) begin
                if (((in_base + jj*OUT_B) < hi) && ((in_base + (jj+1)*OUT_B) > lo)) begin
                    if (!seen_c) begin
                        j_lo_c = jj[RW-1:0];
                        seen_c = 1'b1;
                    end
                    j_hi_c = jj[RW-1:0];
                end
            end
        end

        // ---- range mask for the sub-beat currently being emitted ----
        // base_q is IN_B aligned and IN_B is a multiple of OUT_B, so the byte
        // being tested is {base_q/OUT_B + cnt, bd}: the low LOB bits are the lane
        // index and nothing else. That turns "OUT_B pairs of OFS_W-wide compares
        // against a run-time multiply-add" into one small add plus four compares
        // shared across the beat, with a constant compare left per lane.
        assign d_sub  = base_q[OFS_W-1:LOB] + cnt;
        assign d_lob  = lo_q[OFS_W-1:LOB];
        assign d_lol  = lo_q[LOB-1:0];
        assign d_hib  = hi_q[OFS_W-1:LOB];
        assign d_hil  = hi_q[LOB-1:0];
        assign d_gt   = (d_sub >  d_lob);
        assign d_eqlo = (d_sub == d_lob);
        assign d_lt   = (d_sub <  d_hib);
        assign d_eqhi = (d_sub == d_hib);
        always @(*) begin
            rmask_d = {OUT_MASK_W{1'b0}};
            for (bd = 0; bd < OUT_B; bd = bd + 1)
                if ((d_gt || (d_eqlo && (bd[LOB-1:0] >= d_lol)))
                 && (d_lt || (d_eqhi && (bd[LOB-1:0] <  d_hil)))) begin
                     rmask_d[bd] = 1'b1;
                end
        end

        assign at_end          = (cnt == j_hi_q);
        assign in_ready        = !busy || (out_ready && at_end);
        assign out_valid       = busy;
        assign out_data        = d_q[cnt*OUT_DATA_W +: OUT_DATA_W];
        assign out_mask        = HAS_MASK ? m_q[cnt*OUT_MASK_W +: OUT_MASK_W] : rmask_d;
        assign out_sb          = sb_q;
        assign out_lane        = ln_q;
        assign out_total_bytes = tb_q;
        assign out_last        = last_q && at_end;

        always @(posedge clk or negedge rst_n) begin
            if (!rst_n) begin
                busy   <= 1'b0;
                cnt    <= {RW{1'b0}};
                j_hi_q <= {RW{1'b0}};
                last_q <= 1'b0;
                d_q    <= {WIDE{1'b0}};
                m_q    <= {(WIDE/8){1'b0}};
                sb_q   <= {SB_W{1'b0}};
                ln_q   <= {LANE_W{1'b0}};
                tb_q   <= {TOTBYTES_W{1'b0}};
                base_q <= {OFS_W{1'b0}};
                lo_q   <= {OFS_W{1'b0}};
                hi_q   <= {OFS_W{1'b0}};
            end else begin
                if (busy && out_ready && !at_end) begin
                    cnt <= cnt + 1'b1;
                end
                if (in_fire) begin
                    d_q    <= in_data;
                    m_q    <= in_mask;
                    sb_q   <= in_sb;
                    ln_q   <= in_lane;
                    tb_q   <= in_total_bytes;
                    last_q <= in_last;
                    base_q <= in_base;
                    lo_q   <= lo;
                    hi_q   <= hi;
                    cnt    <= j_lo_c;
                    j_hi_q <= j_hi_c;
                    busy   <= 1'b1;
                end else if (busy && out_ready && at_end) begin
                    busy   <= 1'b0;
                end
            end
        end

    //========================================================================
    // UP : narrow -> wide. Place at j = k_in % RATIO; slots below the first
    // used slot are tied to 0.
    //========================================================================
    end else begin : g_up
        // ---- declarations ----
        // k_in is no longer local: it is the absolute input beat index, which the
        // shared in_base expression above now needs too, so it lives at module
        // scope and both readers get one adder instead of a divide each.
        wire [RW-1:0]         j;       // sub-slot inside the output beat
        wire                  new_ob;  // starting to fill a new output beat
        wire                  flush;   // output beat becomes complete
        reg  [WIDE-1:0]       d_q;     // output beat data accumulator
        reg  [WIDE/8-1:0]     m_q;     // output beat mask accumulator
        reg  [SB_W-1:0]       sb_q;    // captured sideband
        reg  [LANE_W-1:0]     ln_q;    // captured addr phase
        reg  [TOTBYTES_W-1:0] tb_q;    // captured total_bytes
        reg  [OFS_W-1:0]      base_q;  // byte offset of the output beat
        reg  [OFS_W-1:0]      lo_q;    // captured range low
        reg  [OFS_W-1:0]      hi_q;    // captured range high
        reg                   full;    // an output beat is ready
        reg                   last_q;  // captured in_last
        wire [WIDE-1:0]       cur_d;   // accumulator with new-beat clearing
        wire [WIDE/8-1:0]     cur_m;   // mask accumulator with clearing
        reg  [OUT_MASK_W-1:0] rmask_u; // range-derived mask
        integer               bu;      // byte loop index
        wire [OFS_W-LOB-1:0]  u_sub;   // this output beat's index, in OUT_B units
        wire [OFS_W-LOB-1:0]  u_lob;   // beat holding the range's first byte
        wire [LOB-1:0]        u_lol;   // lane of the range's first byte
        wire [OFS_W-LOB-1:0]  u_hib;   // beat holding the range's end
        wire [LOB-1:0]        u_hil;   // lane of the range's end
        wire                  u_gt;    // this beat is entirely at/after lo
        wire                  u_eqlo;  // lo falls inside this beat
        wire                  u_lt;    // this beat is entirely before hi
        wire                  u_eqhi;  // hi falls inside this beat

        assign j      = k_in[RW-1:0];
        assign new_ob = first || (j == {RW{1'b0}});
        assign flush  = in_fire && ((j == (RATIO-1)) || in_last);
        assign cur_d  = new_ob ? {WIDE{1'b0}}     : d_q;
        assign cur_m  = new_ob ? {(WIDE/8){1'b0}} : m_q;

        // base_q is the output beat's own start, hence OUT_B aligned, so the byte
        // under test is {base_q/OUT_B, bu} and the same beat/lane split as the
        // other two arms applies.
        assign u_sub  = base_q[OFS_W-1:LOB];
        assign u_lob  = lo_q[OFS_W-1:LOB];
        assign u_lol  = lo_q[LOB-1:0];
        assign u_hib  = hi_q[OFS_W-1:LOB];
        assign u_hil  = hi_q[LOB-1:0];
        assign u_gt   = (u_sub >  u_lob);
        assign u_eqlo = (u_sub == u_lob);
        assign u_lt   = (u_sub <  u_hib);
        assign u_eqhi = (u_sub == u_hib);
        always @(*) begin
            rmask_u = {OUT_MASK_W{1'b0}};
            for (bu = 0; bu < OUT_B; bu = bu + 1)
                if ((u_gt || (u_eqlo && (bu[LOB-1:0] >= u_lol)))
                 && (u_lt || (u_eqhi && (bu[LOB-1:0] <  u_hil)))) begin
                     rmask_u[bu] = 1'b1;
                end
        end

        assign in_ready        = !full || out_ready;
        assign out_valid       = full;
        assign out_data        = d_q;
        assign out_mask        = HAS_MASK ? m_q : rmask_u;
        assign out_sb          = sb_q;
        assign out_lane        = ln_q;
        assign out_total_bytes = tb_q;
        assign out_last        = last_q;

        always @(posedge clk or negedge rst_n) begin
            if (!rst_n) begin
                full   <= 1'b0;
                last_q <= 1'b0;
                d_q    <= {WIDE{1'b0}};
                m_q    <= {(WIDE/8){1'b0}};
                sb_q   <= {SB_W{1'b0}};
                ln_q   <= {LANE_W{1'b0}};
                tb_q   <= {TOTBYTES_W{1'b0}};
                base_q <= {OFS_W{1'b0}};
                lo_q   <= {OFS_W{1'b0}};
                hi_q   <= {OFS_W{1'b0}};
            end else begin
                if (full && out_ready) begin
                    full <= 1'b0;
                end
                if (in_fire) begin
                    d_q  <= cur_d | ({{(WIDE-NARROW){1'b0}},     in_data} << (j*NARROW));
                    m_q  <= cur_m | ({{((WIDE-NARROW)/8){1'b0}}, in_mask} << (j*(NARROW/8)));
                    sb_q <= in_sb;
                    ln_q <= in_lane;
                    tb_q <= in_total_bytes;
                    lo_q <= lo;
                    hi_q <= hi;
                    if (new_ob) begin
                        base_q <= in_base - j*IN_B;
                    end
                    if (flush) begin
                        full   <= 1'b1;
                        last_q <= in_last;
                    end
                end
            end
        end
    end
    endgenerate

`ifndef LB_NO_ASSERT
    // synthesis translate_off
    // Every shift and mask above stands in for a divide, multiply or modulo by
    // IN_B / OUT_B, and is only the same function when both are powers of two and
    // LIB / LOB are their logs. That holds for every width the generator emits.
    // Say so here, so a future odd width fails loudly instead of silently placing
    // bytes in the wrong lanes.
    initial begin
        if ((1 << LIB) != IN_B) begin
            $display("ERROR %m: IN_DATA_W=%0d gives IN_B=%0d, which is not a power of two",
                     IN_DATA_W, IN_B);
        end
        if ((1 << LOB) != OUT_B) begin
            $display("ERROR %m: OUT_DATA_W=%0d gives OUT_B=%0d, which is not a power of two",
                     OUT_DATA_W, OUT_B);
        end
        if (LOB < 1) begin
            $display("ERROR %m: OUT_B=%0d leaves no lane index bits", OUT_B);
        end
    end
    // synthesis translate_on
`endif

endmodule
