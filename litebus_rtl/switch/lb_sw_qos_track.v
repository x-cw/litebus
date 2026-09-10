//============================================================================
// Filename    : lb_sw_qos_track.v
// Author      : litebus
// Description : per-crosspoint QoS occupancy tracker (thermometer counters)
// Date        : 2026-08-20
// Revision    : 1.0 initial -- per-round changes are in doc/HISTORY.md
//----------------------------------------------------------------------------
// Tracks the QoS histogram of one crosspoint FIFO's valid entries so the
// per-output arbiter can see the queue's MAXIMUM pending QoS, not just the
// head flit's (RR-QoS, ARB_SCHEME = 2): a high-QoS flit queued behind
// low-QoS ones lifts the whole queue's priority, which is what drains the
// blockers and gets it to the front -- a FIFO cannot reorder.
//
// One up/down counter per thermometer level l (1..QLVL): cnt_ge holds how
// many queued flits carry qos >= l. push/pop mirror the tracked FIFO's
// write and read fires, so the counters rebuild the queue's QoS multiset
// exactly -- there is no "running max cannot decrease on pop" problem: a
// pop of the maximum decrements its levels and the max falls by itself.
//
// occ_ge is a THERMOMETER vector (bit l-1 = "some entry has qos >= l"),
// deliberately not a binary maximum: thermometers are ordered by
// construction, so downstream a cross-input maximum is a bitwise OR and
// "someone strictly above me" is |(t & ~mine) -- no priority encoder and
// no comparator tree (see lb_sw_crossbar's g_qosf arm).
//
// Cost scales with DEPTH only through the counter width, ceil(log2(D+1))
// bits per level, QLVL = 2**QOS_W - 1 levels. Never instantiated at
// QOS_W == 0 (a QoS-free switch folds the whole feature away).
//============================================================================

module lb_sw_qos_track #(
    parameter QOS_W = 1,                     // qos field width (>= 1)
    parameter DEPTH = 4,                     // tracked FIFO depth (counter ceiling)
    // ---- derived; used in the port list, so it must stay in the parameter
    // ---- list. Not to be overridden externally.
    parameter QLVL  = (1 << QOS_W) - 1       // thermometer levels
) (
    // ---- inputs ----
    input wire              clk,      // clock
    input wire              rst_n,    // async reset, active low
    input wire              push,     // tracked FIFO write fire
    input wire  [QOS_W-1:0] push_qos, // qos of the flit being written
    input wire              pop,      // tracked FIFO read fire
    input wire  [QOS_W-1:0] pop_qos,  // qos of the flit being read out
    // ---- outputs ----
    output wire [QLVL-1:0]  occ_ge    // bit l-1: some queued flit has qos >= l
);
    //------------------------------------------------------------------------
    // Counter width: counts 0..DEPTH inclusive, so log2(DEPTH) + 1 bits. Legal
    // depths are any integer 2..64 (gen/lb_topo.py's DEPTH_MIN/DEPTH_MAX).
    //
    // This ladder is an UPPER BOUND, not a modulus -- which is why this module
    // needed no change when the depth granularity opened up on 2026-08-25. It is a
    // counter, not an addressed memory: a non-power-of-two DEPTH just leaves the
    // top count unused. The modulus problem was confined to the two modules that
    // INDEX a mem with the pointer (lb_sw_out_fifo, lb_credit_ingress).
    //------------------------------------------------------------------------
    localparam QCNT_W = (DEPTH <= 2)  ? 2 :
                       (DEPTH <= 4)  ? 3 :
                       (DEPTH <= 8)  ? 4 :
                       (DEPTH <= 16) ? 5 :
                       (DEPTH <= 32) ? 6 : 7;

    genvar gl;                               // thermometer level gen index

    //------------------------------------------------------------------------
    // One counter per level. A push and a pop hitting the same level in one
    // cycle cancel. The tracked FIFO's own flow control rules out overflow
    // (never more than DEPTH entries) and underflow (a pop pops what was
    // pushed) -- both asserted below rather than assumed silently.
    //------------------------------------------------------------------------
    generate
    for (gl = 1; gl <= QLVL; gl = gl + 1) begin : g_lvl
        reg  [QCNT_W-1:0] cnt_ge; // queued flits with qos >= gl
        wire             pu;      // push hits this level
        wire             po;      // pop hits this level
        assign pu = push && (push_qos >= gl);
        assign po = pop && (pop_qos >= gl);
        always @(posedge clk or negedge rst_n) begin
            if (!rst_n) begin
                cnt_ge <= {QCNT_W{1'b0}};
            end else begin
                case ({pu, po})
                    2'b10:   cnt_ge <= cnt_ge + {{(QCNT_W-1){1'b0}}, 1'b1};
                    2'b01:   cnt_ge <= cnt_ge - {{(QCNT_W-1){1'b0}}, 1'b1};
                    default: cnt_ge <= cnt_ge;   // idle, or push and pop cancel
                endcase
            end
        end
        assign occ_ge[gl-1] = |cnt_ge;

`ifndef LB_NO_ASSERT
        // synthesis translate_off
        always @(posedge clk) begin
            if (rst_n && po && !pu && cnt_ge == {QCNT_W{1'b0}}) begin
                // A pop at a level nothing pushed: the multiset invariant broke.
                $display("ERROR %m: level %0d counter underflow", gl);
            end
            if (rst_n && cnt_ge > DEPTH) begin
                // More tracked entries than the FIFO holds: push without room.
                $display("ERROR %m: level %0d counter %0d exceeds DEPTH", gl, cnt_ge);
            end
        end
        // synthesis translate_on
`endif
    end
    endgenerate
endmodule
