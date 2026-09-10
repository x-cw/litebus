//============================================================================
// Filename    : lb_tniu_rsp_rd_frag.v
// Author      : litebus
// Description : read interleaving: beat level -> transaction level
// Date        : 2026-08-06
// Revision    : 1.0 initial -- per-round changes are in doc/HISTORY.md
//----------------------------------------------------------------------------
// Spec: doc/html/arch.html section 6.2. This block sits inside the TNIU and
// regroups the BEAT-level read data that the Slave IP returns interleaved into
// TRANSACTION-level fragments the fabric can consume, so that the Switch
// burst-lock logic needs no change at all (the Switch still sees one complete,
// non-interruptible burst at a time).
//
// Key concept
//   FB (fragment block) = max(SLV data width, W_frag)
//     - W_frag = max data_width among the Masters connected to this Slave
//     - SLV >= W_frag : one SLV beat already spans a whole aligned block,
//                       so each fragment is 1 beat and no buffering is needed
//     - SLV <  W_frag : R = W_frag/SLV consecutive beats must be gathered into
//                       one block, which is what the assembly buffer is for
//   Self-contained fragments: each fragment carries its own
//   (addr_f, total_bytes_f), so the phase-aligned gearbox downstream can
//   interpret it at any width without transaction context.
//
// Cost: assembly buffer = EXT_PENDING_TRANS x FB bytes (one entry per LID holding
//       a single block, NOT a whole transaction). This bound is what makes read
//       interleaving affordable; see section 6.2.3.
//
// Throughput: fragments hand over back to back, so with the downstream ready this
//       block sustains one beat per cycle regardless of RSPAN. That needs a few
//       LIDs in rotation (~4 with the s1 stage) -- one LID on its own cannot beat
//       one beat per four cycles, because its single buffer entry is not released
//       until its previous fragment's last beat has been sent. That limit is
//       inherent to one entry per LID (raising it costs area); the per-fragment
//       dead cycle that used to sit on top of it was not, and is gone. See the
//       send FSM comment below.
//
// Pipeline: an s1 register stage separates the per-beat LOOKUPS (cmd_table
//       context, bcnt) from the 18-bit interval arithmetic. Both used to share
//       one cycle and were the whole design's 2.5 GHz critical cone (txnid ->
//       256:1 mux -> add/min/sub -> bcnt/p_* setup, 23~27 levels, measured
//       -0.05 ns at the full-bus top). Costs one cycle of latency on the read
//       return (SPEC latency table carries it); throughput is unchanged -- s1
//       plus the arm's stage is a standard two-deep pipeline. The one hazard it
//       opens (same-LID back-to-back pre-reads a stale bcnt) is closed by the
//       lw_* write mirror; see the bypass comment at the declarations.
//
// Parameter naming (CODING_STYLE 1.4): the beat-level data path is on the
// external (Slave IP) side, hence EXT_*; the fragment descriptor fields
// (addr_lane, total_bytes) are in-network quantities, hence INT_*. WFRAG_DW is a
// topology constant (max Master width on this Slave) and carries no prefix.
//
// Not done here: no ROB. Adv 10 assumes the Master IP itself accepts ID-tagged
//       interleaved read returns, so the INIU reverse path only translates
//       int_id -> ext_txnid and does not de-interleave.
//
// Golden model: verify/models/phase_gearbox_model.py :: fragments()
//============================================================================
`include "lb_defines.vh"

module lb_tniu_rsp_rd_frag #(
    parameter EXT_DATA_WIDTH    = 128,  // Slave IP data width in bits
    parameter WFRAG_DW          = 256,  // W_frag in bits (max Master width on this Slave)
    parameter EXT_PENDING_TRANS = 8,    // outstanding read transactions = number of LID entries
    parameter EXT_LID_WIDTH     = 3,    // local id width, derived from EXT_PENDING_TRANS by the parent
    parameter INT_LANE_W        = 6,    // addr_lane (address phase) width
    parameter INT_TOTBYTES_W    = 16,   // total_bytes width
    parameter FSB_W             = 1,    // transaction sideband (int_id/src_id), snapshotted per fragment
                                        // so that cmd_table may free its entry early
    parameter BSB_W             = 1,    // beat sideband (resp/user), stored next to data in the buffer
    // ---- derived parameters; used in the port list, so they must stay in the
    // ---- parameter list (V2001 body localparams cannot size ports). Not to be
    // ---- overridden externally -- the defaults are the correct values.
    parameter EXT_DATA_BYTES = EXT_DATA_WIDTH/8,                          // Slave beat size in bytes
    parameter WF_B           = WFRAG_DW/8,                                // W_frag in bytes
    // fragment block size in bytes
    parameter FB_B           = (EXT_DATA_BYTES > WF_B) ? EXT_DATA_BYTES : WF_B,
    parameter RSPAN          = FB_B / EXT_DATA_BYTES,                     // max beats per fragment
    // beat-index width inside a fragment
    parameter RW             = (RSPAN <= 1)  ? 1 :
                               (RSPAN <= 2)  ? 1 :
                               (RSPAN <= 4)  ? 2 :
                               (RSPAN <= 8)  ? 3 :
                               (RSPAN <= 16) ? 4 : 5,
    parameter OFS_W          = INT_LANE_W + INT_TOTBYTES_W + 1            // byte-offset arithmetic width
) (
    // ---- inputs: clock / reset ----
    input wire                          clk,           // clock
    input wire                          rst_n,         // async reset, active low
    // ---- inputs: context of the current beat, looked up in cmd_table by i_lid ----
    input wire  [INT_LANE_W-1:0]        i_ctx_addr_lo, // transaction address phase
    input wire  [INT_TOTBYTES_W-1:0]    i_ctx_totb,    // transaction total_bytes
    input wire  [FSB_W-1:0]             i_ctx_fsb,     // transaction sideband (int_id/src_id)
    // ---- inputs: Slave side, beat level, may be interleaved ----
    input wire                          i_valid,       // beat valid
    input wire  [EXT_LID_WIDTH-1:0]     i_lid,         // local transaction id of this beat
    input wire  [EXT_DATA_WIDTH-1:0]    i_data,        // beat read data
    input wire  [BSB_W-1:0]             i_bsb,         // beat sideband (resp/user)
    // ---- inputs: fabric side backpressure ----
    input wire                          o_ready,       // fabric ready (backpressure)
    // ---- outputs: Slave side ----
    output wire                         i_ready,       // beat ready (backpressure to the Slave)
    // ---- outputs: fabric side, transaction-level fragments, one is atomic ----
    output wire                         o_valid,       // fragment beat valid
    output wire [EXT_DATA_WIDTH-1:0]    o_data,        // fragment beat data
    output wire [BSB_W-1:0]             o_bsb,         // beat sideband, replayed from the buffer
    output wire [EXT_LID_WIDTH-1:0]     o_lid,         // local transaction id of the fragment
    output wire [FSB_W-1:0]             o_fsb,         // transaction sideband snapshot
    output wire [INT_LANE_W-1:0]        o_addr_lo,     // this fragment's own address phase
    output wire [INT_TOTBYTES_W-1:0]    o_total_bytes, // this fragment's own byte count
    output wire                         o_last,        // fragment last beat (used by burst-lock)
    output wire                         o_trans_last   // transaction last beat
);
    //------------------------------------------------------------------------
    // Derived local params (completion-queue geometry follows the table depth)
    //------------------------------------------------------------------------
    localparam QD = EXT_PENDING_TRANS;       // completion queue depth = one slot per LID
    // Completion queue index width. This MUST cover the whole depth: the queue is
    // declared q[0:QD-1] but addressed q[q_wp[QW-1:0]], and the "it can never
    // overflow" argument (i_ready blocks a LID that already has a fragment
    // pending, so occupancy <= EXT_PENDING_TRANS) is only sound when QD slots are
    // actually reachable. This ladder used to stop at 5, so any depth above 32
    // addressed 32 slots while the pointers ran a 32-deep FIFO's arithmetic: push
    // 33 wrapped onto a slot whose LID had not been read, losing one LID (its
    // fdone stays set forever -> that LID's remaining beats are back-pressured
    // forever) and delivering another twice. verify/unit/tb_rd_frag_qdepth.v holds
    // it: at depth 64 the beat COUNT stayed right and 62 LIDs were wrong.
    // Runs to 16 because ext_txnid_width does (config/schema.yaml), and
    // EXT_PENDING_TRANS cannot exceed 2**ext_txnid_width.
    localparam QW = (QD <= 2)     ? 1 :        // = clog2(QD)
                    (QD <= 4)     ? 2 :
                    (QD <= 8)     ? 3 :
                    (QD <= 16)    ? 4 :
                    (QD <= 32)    ? 5 :
                    (QD <= 64)    ? 6 :
                    (QD <= 128)   ? 7 :
                    (QD <= 256)   ? 8 :
                    (QD <= 512)   ? 9 :
                    (QD <= 1024)  ? 10 :
                    (QD <= 2048)  ? 11 :
                    (QD <= 4096)  ? 12 :
                    (QD <= 8192)  ? 13 :
                    (QD <= 16384) ? 14 :
                    (QD <= 32768) ? 15 : 16;
    // Slots ACTUALLY declared for the completion queue. QD is the CAPACITY (one
    // slot per LID); QSLOTS is the STORAGE, rounded up to the pointer's natural
    // modulus. Two names because they are two different things -- do not merge.
    //
    // Why round the storage up instead of teaching the pointers to wrap at QD:
    // depths have granularity 1 now (Slaves.ext_pending_trans), so QD is usually
    // not a power of two, while q_wp/q_rp are a textbook "one extra bit + natural
    // wrap" FIFO pair (q_empty is q_wp == q_rp, the slot index is q_wp[QW-1:0]).
    // That arithmetic is only correct modulo 2**QW. Padding the array costs
    // (2**QW - QD) x EXT_LID_WIDTH flops -- 28 bits at QD=9 -- and leaves the
    // pointer logic BIT FOR BIT as it was; an explicit modulo wrap would save
    // those flops and reopen exactly the failure this queue has already been bitten
    // by (see the QW comment above: at depth 64 the beat count stayed right while
    // 62 LIDs were wrong). The padding is also bounded by the storage the depth
    // already bought: (2**QW - QD) x LID_W < QD x LID_W << QD x per-LID context.
    //
    // Overflow argument is unchanged and still only needs QD: a push always sets
    // fdone[lid], and s2_take requires !fdone[s1_lid], so a LID cannot be pushed
    // again while its fragment is queued -> occupancy <= EXT_PENDING_TRANS = QD
    // <= QSLOTS. The padded slots are never written before they are read, so they
    // deliberately have no reset loop (that would be pure area).
    localparam QSLOTS = (1 << QW);
    // log2 of the Slave beat size. Every "/ EXT_DATA_BYTES" and "* EXT_DATA_BYTES"
    // in the fragment arithmetic below is written as a shift by this instead, and
    // every "/ FB_B" as a mask -- see the elaboration check at the end of the file
    // for the power-of-two assumption both rely on.
    localparam LSB = (EXT_DATA_BYTES <= 1)   ? 0 :   // = clog2(EXT_DATA_BYTES)
                     (EXT_DATA_BYTES <= 2)   ? 1 :
                     (EXT_DATA_BYTES <= 4)   ? 2 :
                     (EXT_DATA_BYTES <= 8)   ? 3 :
                     (EXT_DATA_BYTES <= 16)  ? 4 :
                     (EXT_DATA_BYTES <= 32)  ? 5 :
                     (EXT_DATA_BYTES <= 64)  ? 6 :
                     (EXT_DATA_BYTES <= 128) ? 7 : 8;

    //------------------------------------------------------------------------
    // Declarations (all up front, one per line)
    //------------------------------------------------------------------------
    integer                   k;                             // reset-loop index
    // ---- per-LID state, SHARED by both arms ----
    // Both arms need it: every fragment carries its OWN (addr_lane, total_bytes),
    // and which beat of the transaction this is decides those.
    reg  [OFS_W-1:0]          bcnt  [0:EXT_PENDING_TRANS-1]; // beats received so far, per transaction
    // ---- context of the current input beat ----
    wire [INT_LANE_W-1:0]     c_lo_lane;                     // context address phase
    wire [INT_TOTBYTES_W-1:0] c_tb;                          // context total_bytes
    wire [OFS_W-1:0]          c_lo;                          // transaction valid range low bound
    wire [OFS_W-1:0]          c_hi;                          // transaction range high bound (excl)
    // ---- combinational derivation of the fragment this beat belongs to ----
    wire [OFS_W-1:0]          n_now;                         // beats already received for this LID
    wire [OFS_W-1:0]          base_beat;                     // absolute Slave-beat index of this beat
    wire [OFS_W-1:0]          hi_m1;                         // last valid byte of the transaction
    wire [OFS_W-1:0]          hi_beat;                       // Slave beat holding that last byte
    wire [OFS_W-1:0]          blk_beat;                      // first Slave beat of this block
    wire [OFS_W-1:0]          blk_last;                      // last Slave beat of this block
    wire [OFS_W-1:0]          last_beat;                     // last Slave beat of this fragment
    wire [OFS_W-1:0]          base;                          // byte offset of this beat
    wire [OFS_W-1:0]          blk_lo;                        // block low bound (byte offset)
    wire [OFS_W-1:0]          blk_hi;                        // block high bound (byte offset, excl)
    wire [OFS_W-1:0]          fr_a;                          // fragment start = max(c_lo, blk_lo)
    wire [OFS_W-1:0]          fr_e;                          // fragment end   = min(c_hi, blk_hi)
    wire [OFS_W-1:0]          fr_tb;                         // fragment byte count
    wire [OFS_W-1:0]          fr_b0;                         // first beat index of the fragment
    wire [OFS_W-1:0]          fr_bn;                         // beat count of the fragment
    wire [OFS_W-1:0]          idx_in;                        // position of this beat in the fragment
    wire                      frag_end;                      // this beat completes the fragment
    wire                      tr_last;                       // this fragment ends the transaction
    // ---- handshakes, shared ----
    wire                      i_fire;                        // Slave-side handshake fire
    wire                      o_fire;                        // fabric-side handshake fire
    // ---- s1 pipeline stage: the beat and everything looked up FOR it ----
    // The cycle a beat is accepted it only travels from the upstream skid through
    // the two 256-entry lookups (context memories in cmd_table, bcnt here) into
    // these registers; the interval arithmetic below runs one cycle later on the
    // registered copies. This is the deliberate timing cut for the 2.5 GHz wall:
    // txnid -> 256:1 mux -> 18-bit add/min/sub no longer shares one cycle. Costs
    // one cycle of beat-to-fabric latency (SPEC latency table updated with it) and
    // zero throughput: s1 + the arm's own stage form a standard two-deep pipeline.
    reg                       s1_valid;                      // s1 holds a beat
    reg  [EXT_DATA_WIDTH-1:0] s1_data;                       // its data
    reg  [BSB_W-1:0]          s1_bsb;                        // its beat sideband
    reg  [EXT_LID_WIDTH-1:0]  s1_lid;                        // its LID
    reg  [FSB_W-1:0]          s1_fsb;                        // its transaction sideband snapshot
    reg  [INT_LANE_W-1:0]     s1_lane;                       // its transaction address phase
    reg  [INT_TOTBYTES_W-1:0] s1_totb;                       // its transaction total_bytes
    reg  [OFS_W-1:0]          s1_n;                          // bcnt[i_lid] pre-read at accept time
    // ---- mirror of the most recent bcnt write: same-LID back-to-back bypass ----
    // s1 pre-reads bcnt in the same cycle the beat ahead of it may still be
    // WRITING bcnt (non-blocking, lands at the cycle edge), so a beat directly
    // behind another beat of the SAME LID captures a stale count. The beat ahead
    // is, by the pipe's single-stream order, also the most recent bcnt write --
    // so mirroring that one write is a complete fix. When the pre-read did see
    // the write, mirror and pre-read are equal and the bypass is idempotent;
    // there is deliberately no freshness tracking to get wrong.
    reg                       lw_valid;                      // any write since reset
    reg  [EXT_LID_WIDTH-1:0]  lw_lid;                        // LID of the last bcnt write
    reg  [OFS_W-1:0]          lw_next;                       // value that write stored
    wire [OFS_W-1:0]          n_eff;                         // s1_n corrected by the bypass
    // Consume strobe for s1, driven by whichever arm elaborates: the arm moves
    // s1's beat into its own stage this cycle. i_ready derives from it, so the
    // input stalls exactly when the arm's stage cannot advance.
    wire                      s2_take;                       // s1 -> arm transfer fires

    //------------------------------------------------------------------------
    // Context of the current s1 beat (registered copies of the lookup results)
    //------------------------------------------------------------------------
    assign c_lo_lane = s1_lane;
    assign c_tb      = s1_totb;
    assign c_lo      = {{(OFS_W-INT_LANE_W){1'b0}},     c_lo_lane};
    assign c_hi      = c_lo + {{(OFS_W-INT_TOTBYTES_W){1'b0}}, c_tb};

    //------------------------------------------------------------------------
    // Combinational derivation of the fragment this input beat belongs to
    //------------------------------------------------------------------------
    // Everything here is done in BEAT units rather than byte offsets, which is
    // what shortens it. The old form worked in bytes and paid a full OFS_W carry
    // chain for each step, with frag_end at the end of the queue:
    //   c_hi -> fr_e -> (fr_e-1) -> /SB -> -fr_b0 -> +1 -> -1 -> == idx_in
    // that is five dependent adders and two divides behind the c_hi adder, and
    // frag_end gates the write enable of nine registers. Measured at 83 levels,
    // the deepest combinational path in the design.
    //
    // In beat units the same question -- "is this the fragment's last beat?" --
    // is just "is this beat the last beat of the fragment", where the fragment
    // ends at whichever comes first, the transaction's last beat or the block's
    // last beat. EXT_DATA_BYTES and FB_B are powers of two, so block alignment is
    // a mask and the block's last beat is an OR, not an add.
    //
    // fr_bn / idx_in / fr_tb are still needed as stored descriptor fields, but
    // they are no longer in front of frag_end. Every identity below was proven by
    // exhaustion over all shipped (SLV, W_frag) pairs, every first-beat phase and
    // every beat of the burst, before the rewrite.
    assign n_eff     = (lw_valid && (lw_lid == s1_lid)) ? lw_next : s1_n;
    assign n_now     = n_eff;
    assign base_beat = (c_lo >> LSB) + n_now;
    assign base      = base_beat << LSB;
    assign hi_m1     = c_lo + c_tb + {OFS_W{1'b1}};   // c_lo + total_bytes - 1
    assign hi_beat   = hi_m1 >> LSB;
    assign blk_beat  = base_beat & ~(RSPAN-1);
    assign blk_last  = blk_beat | (RSPAN-1);
    assign blk_lo    = blk_beat << LSB;
    assign blk_hi    = blk_lo + FB_B;
    assign fr_a      = (c_lo > blk_lo) ? c_lo : blk_lo;
    assign fr_e      = (c_hi < blk_hi) ? c_hi : blk_hi;
    assign fr_tb     = fr_e - fr_a;
    assign fr_b0     = fr_a >> LSB;
    assign last_beat = (hi_beat < blk_last) ? hi_beat : blk_last;
    assign fr_bn     = last_beat - fr_b0 + 1'b1;
    assign idx_in    = base_beat - fr_b0;
    assign frag_end  = (base_beat == last_beat);
    assign tr_last   = (hi_beat <= blk_last);

    assign i_fire  = i_valid && i_ready;
    assign o_fire  = o_valid && o_ready;
    // Uniform for both arms now that acceptance only has to reach s1: accept
    // whenever s1 is empty or being drained by the arm this cycle. The per-LID
    // conditions that used to live here moved into the arms' s2_take, where the
    // LID is already registered -- see g_asm.
    assign i_ready = !s1_valid || s2_take;

    // s1 capture. bcnt[i_lid] is pre-read here so the arithmetic never touches
    // the 256:1 bcnt mux; the same-LID staleness this opens is closed by the
    // lw_* bypass above.
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            s1_valid <= 1'b0;
            s1_data  <= {EXT_DATA_WIDTH{1'b0}};
            s1_bsb   <= {BSB_W{1'b0}};
            s1_lid   <= {EXT_LID_WIDTH{1'b0}};
            s1_fsb   <= {FSB_W{1'b0}};
            s1_lane  <= {INT_LANE_W{1'b0}};
            s1_totb  <= {INT_TOTBYTES_W{1'b0}};
            s1_n     <= {OFS_W{1'b0}};
        end else begin
            if (i_fire) begin
                s1_valid <= 1'b1;
                s1_data  <= i_data;
                s1_bsb   <= i_bsb;
                s1_lid   <= i_lid;
                s1_fsb   <= i_ctx_fsb;
                s1_lane  <= i_ctx_addr_lo;
                s1_totb  <= i_ctx_totb;
                s1_n     <= bcnt[i_lid];
            end else if (s2_take) begin
                s1_valid <= 1'b0;      // drained and nothing arriving
            end
        end
    end

    // Beat counter per LID, shared by both arms; written when the arm consumes
    // the s1 beat, together with its lw_* mirror (the bypass source).
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (k = 0; k < EXT_PENDING_TRANS; k = k + 1)
                bcnt[k] <= {OFS_W{1'b0}};
            lw_valid <= 1'b0;
            lw_lid   <= {EXT_LID_WIDTH{1'b0}};
            lw_next  <= {OFS_W{1'b0}};
        end else if (s2_take) begin
            // clear at transaction end so the LID can be reused
            if (frag_end && tr_last) begin
                bcnt[s1_lid] <= {OFS_W{1'b0}};
                lw_next      <= {OFS_W{1'b0}};
            end
            else begin
                bcnt[s1_lid] <= n_now + 1'b1;
                lw_next      <= n_now + 1'b1;
            end
            lw_valid <= 1'b1;
            lw_lid   <= s1_lid;
        end
    end

generate
if (RSPAN == 1) begin : g_pass
    //========================================================================
    // RSPAN == 1: nothing to assemble, so nothing to buffer.
    //
    // RSPAN is FB_B/EXT_DATA_BYTES with FB_B = max(EXT_DATA_BYTES, W_frag bytes),
    // so RSPAN == 1 means no Master reaching this Slave is WIDER than the Slave --
    // every fragment is exactly one beat. The assembly buffer, the completion
    // queue and the send FSM all exist to gather R consecutive beats per LID and
    // hand them over atomically. With one beat per fragment there is nothing to
    // gather and nothing to re-order: beats leave in the order they arrived.
    //
    // The general arm below charged for all of it anyway -- abuf is
    // EXT_PENDING_TRANS x EXT_DATA_WIDTH and sits ON the data path even when
    // RSPAN == 1. That was tolerable while EXT_PENDING_TRANS was capped at 16; it
    // is not now that the cap is gone and the depth is 2**ext_txnid_width. For
    // bus_16x16s8 (16 Slaves, 512-bit data, 8-bit Slave txnid -> 256 entries) the
    // general arm would carry 2.1 Mbit of flops for a buffer it can never use.
    //
    // Still needed: bcnt (above), because each beat's OWN (addr_lane,
    // total_bytes) depend on where it sits in the transaction -- fragments are
    // self-contained. That is also exactly why this cannot be folded into
    // lb_tniu_int_core's non-interleaving feed-through arm, which sends the WHOLE
    // transaction's descriptor on every beat and only raises last at the end of
    // the transaction.
    //
    // This arm is now s1 + this registered stage: beat-to-fabric latency is 2
    // cycles (the s1 timing cut above pays one), throughput stays one beat per
    // cycle while o_ready holds -- a standard two-deep pipeline. What does NOT
    // come back: the general arm could absorb one beat per LID while o_ready was
    // low, purely as a side effect of the per-LID buffer. This arm back-pressures
    // after two in flight. That absorption was never the buffer's purpose -- the
    // RSP_RD channel's credit_egress is where downstream buffering belongs.
    //========================================================================
    reg                       p_valid; // the registered stage holds a beat
    reg  [EXT_DATA_WIDTH-1:0] p_data;  // its data
    reg  [BSB_W-1:0]          p_bsb;   // its beat sideband
    reg  [EXT_LID_WIDTH-1:0]  p_lid;   // its LID
    reg  [FSB_W-1:0]          p_fsb;   // its transaction sideband
    reg  [INT_LANE_W-1:0]     p_a;     // its own address phase
    reg  [INT_TOTBYTES_W-1:0] p_tb;    // its own byte count
    reg                       p_tl;    // it ends the transaction

    // Consume s1 whenever this stage is empty or is being emptied this cycle.
    assign s2_take       = s1_valid && (!p_valid || o_fire);
    assign o_valid       = p_valid;
    assign o_data        = p_data;
    assign o_bsb         = p_bsb;
    assign o_lid         = p_lid;
    assign o_fsb         = p_fsb;
    assign o_addr_lo     = p_a;
    assign o_total_bytes = p_tb;
    // One beat per fragment, so every beat is its fragment's last.
    assign o_last        = p_valid;
    assign o_trans_last  = p_valid && p_tl;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            p_valid <= 1'b0;
            p_data  <= {EXT_DATA_WIDTH{1'b0}};
            p_bsb   <= {BSB_W{1'b0}};
            p_lid   <= {EXT_LID_WIDTH{1'b0}};
            p_fsb   <= {FSB_W{1'b0}};
            p_a     <= {INT_LANE_W{1'b0}};
            p_tb    <= {INT_TOTBYTES_W{1'b0}};
            p_tl    <= 1'b0;
        end else begin
            if (s2_take) begin
                p_valid <= 1'b1;
                p_data  <= s1_data;
                p_bsb   <= s1_bsb;
                p_lid   <= s1_lid;
                p_fsb   <= s1_fsb;
                p_a     <= fr_a [INT_LANE_W-1:0];
                p_tb    <= fr_tb[INT_TOTBYTES_W-1:0];
                p_tl    <= tr_last;
            end else if (o_fire) begin
                p_valid <= 1'b0;        // drained and nothing arriving
            end
        end
    end
end else begin : g_asm
    //========================================================================
    // RSPAN > 1: gather RSPAN consecutive beats per LID into one block, then hand
    // the block over atomically so the Switch's burst-lock sees no interleaving.
    //========================================================================
    reg                       fdone [0:EXT_PENDING_TRANS-1];       // a complete fragment waits to be sent
    reg  [RW:0]               fnb   [0:EXT_PENDING_TRANS-1];       // number of beats in that fragment
    reg  [INT_LANE_W-1:0]     fa    [0:EXT_PENDING_TRANS-1];       // that fragment's address phase
    reg  [INT_TOTBYTES_W-1:0] ftb   [0:EXT_PENDING_TRANS-1];       // that fragment's byte count
    reg                       ftl   [0:EXT_PENDING_TRANS-1];       // that fragment is the transaction's last
    reg  [FSB_W-1:0]          ffsb  [0:EXT_PENDING_TRANS-1];       // transaction sideband snapshot
    // ---- assembly buffer: EXT_PENDING_TRANS entries x RSPAN beats ----
    reg  [EXT_DATA_WIDTH-1:0] abuf  [0:EXT_PENDING_TRANS*RSPAN-1]; // buffered beat data
    reg  [BSB_W-1:0]          absb  [0:EXT_PENDING_TRANS*RSPAN-1]; // buffered beat sideband
    // ---- completion queue: fragments are sent in the order they completed ----
    reg  [EXT_LID_WIDTH-1:0]  q     [0:QSLOTS-1];                  // completion queue (QD used, padded)
    reg  [QW:0]               q_wp;                                // queue write pointer
    reg  [QW:0]               q_rp;                                // queue read pointer
    // ---- send-side state ----
    reg                       sending;                             // a fragment is being sent
    reg  [EXT_LID_WIDTH-1:0]  s_lid;                               // LID of the fragment being sent
    reg  [RW:0]               s_cnt;                               // beat counter within the fragment
    reg  [RW:0]               s_nb;                                // number of beats in the fragment
    reg  [INT_LANE_W-1:0]     s_a;                                 // addr_lane of the fragment
    reg  [INT_TOTBYTES_W-1:0] s_tb;                                // total_bytes of the transaction
    reg                       s_tl;                                // fragment ends the transaction
    reg  [FSB_W-1:0]          s_fsb;                               // transaction-level sideband
    // ---- fragment hand-off ----
    wire                      q_empty;                             // completion queue empty
    wire                      ld_last;                             // fragment being sent ends now
    wire                      ld_now;                              // a fragment may be loaded this cycle
    wire [EXT_LID_WIDTH-1:0]  ld_lid;                              // LID at the head of the completion queue
    integer                   ak;                                  // reset-loop index

    //------------------------------------------------------------------------
    // Send side
    //------------------------------------------------------------------------
    assign q_empty       = (q_wp == q_rp);
    assign o_valid       = sending;
    assign o_data        = abuf[s_lid*RSPAN + s_cnt];
    assign o_bsb         = absb[s_lid*RSPAN + s_cnt];
    assign o_lid         = s_lid;
    assign o_fsb         = s_fsb;
    assign o_addr_lo     = s_a;
    assign o_total_bytes = s_tb;
    assign o_last        = sending && (s_cnt == (s_nb - 1));
    assign o_trans_last  = o_last && s_tl;

    // While a LID still has a fragment pending or in flight its further beats must
    // be back-pressured (there is exactly one buffer entry per LID). The check
    // lives on s1's CONSUME side now, indexed by the registered s1_lid -- the
    // blocked beat waits in s1 instead of in the upstream skid, which blocks the
    // same set of beats (one stream, no reordering) at the same net rate.
    // LOAD-BEARING for the fdone write conflict below: the (sending && s_lid ==
    // s1_lid) term is what makes the receive side unable to set fdone[k] in the
    // same cycle the send side clears it. Anyone relaxing s2_take must re-check.
    assign s2_take = s1_valid && !fdone[s1_lid] && !(sending && (s_lid == s1_lid));

    //------------------------------------------------------------------------
    // Fragment hand-off, back to back.
    //
    // The send FSM used to drop `sending` to 0 on a fragment's last beat and only
    // reload from the `if (!sending)` branch on the NEXT cycle, so every pair of
    // fragments was separated by one dead cycle. Throughput was therefore
    //     RSPAN / (RSPAN + 1)
    // and RSPAN=1 (Slave data width >= W_frag, i.e. one beat already spans a whole
    // block) is the worst case: every beat paid the bubble, capping this TNIU's read
    // return at 50%. That is what limited bus_16x16 read bandwidth, upstream of the
    // fabric -- the Slave IP held rsp_rd_valid high while ready toggled every other
    // cycle, with no backpressure from the switch at all.
    //
    // ld_last : the fragment currently being sent finishes on this cycle
    // ld_now  : a fragment may be loaded on this cycle -- either the FSM is idle, or
    //           it is finishing one right now and the completion queue has another
    //
    // Reading q[q_rp] on the same cycle a new entry is pushed is not possible: when
    // the queue is empty q_empty is true for the whole cycle (it compares the
    // current pointers), so ld_now is false and the push is only seen next cycle.
    //------------------------------------------------------------------------
    assign ld_last = sending && o_fire && (s_cnt == (s_nb - 1));
    assign ld_now  = !q_empty && (!sending || ld_last);
    assign ld_lid  = q[q_rp[QW-1:0]];

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            // `ak`, not the module-level `k`: bcnt's reset loop lives in the
            // shared always block above and would race this one for the index.
            for (ak = 0; ak < EXT_PENDING_TRANS; ak = ak + 1) begin
                fdone[ak] <= 1'b0;
                fnb  [ak] <= {(RW+1){1'b0}};
                fa   [ak] <= {INT_LANE_W{1'b0}};
                ftb  [ak] <= {INT_TOTBYTES_W{1'b0}};
                ftl  [ak] <= 1'b0;
                ffsb [ak] <= {FSB_W{1'b0}};
            end
            for (ak = 0; ak < EXT_PENDING_TRANS*RSPAN; ak = ak + 1) begin
                abuf[ak] <= {EXT_DATA_WIDTH{1'b0}};
                absb[ak] <= {BSB_W{1'b0}};
            end
            q_wp    <= 0;
            q_rp    <= 0;
            sending <= 1'b0;
            s_lid   <= 0;
            s_cnt   <= 0;
            s_nb    <= 0;
            s_a     <= 0;
            s_tb    <= 0;
            s_tl    <= 1'b0;
            s_fsb   <= {FSB_W{1'b0}};
        end else begin
            //---------------- receive: store into the assembly buffer ----------------
            if (s2_take) begin
                abuf[s1_lid*RSPAN + idx_in[RW-1:0]] <= s1_data;
                absb[s1_lid*RSPAN + idx_in[RW-1:0]] <= s1_bsb;
                if (frag_end) begin
                    fdone[s1_lid] <= 1'b1;
                    fnb  [s1_lid] <= fr_bn[RW:0];
                    fa   [s1_lid] <= fr_a[INT_LANE_W-1:0];
                    ftb  [s1_lid] <= fr_tb[INT_TOTBYTES_W-1:0];
                    ftl  [s1_lid] <= tr_last;
                    ffsb [s1_lid] <= s1_fsb;
                    q[q_wp[QW-1:0]] <= s1_lid;
                    q_wp <= q_wp + 1'b1;
                end
                // bcnt is updated by the shared always block above -- both arms
                // need it, so it must not be driven from two places.
            end

            //---------------- send: drain one fragment atomically ----------------
            // Releasing the finished fragment's buffer entry is independent of
            // whether another fragment is loaded this cycle. This assignment comes
            // after the receive block, so on a same-index collision it would win --
            // i_ready makes that collision unreachable (see its comment).
            if (ld_last) begin
                fdone[s_lid] <= 1'b0;
            end

            if (ld_now) begin
                // Start the next fragment. Reached both from idle and directly off
                // the previous fragment's last beat, so there is no bubble between
                // fragments. s_cnt/s_nb are non-blocking, so o_last stays correct for
                // the beat being handed over on this very cycle.
                s_lid   <= ld_lid;
                s_cnt   <= {(RW+1){1'b0}};
                s_nb    <= fnb [ld_lid];
                s_a     <= fa  [ld_lid];
                s_tb    <= ftb [ld_lid];
                s_tl    <= ftl [ld_lid];
                s_fsb   <= ffsb[ld_lid];
                q_rp    <= q_rp + 1'b1;
                sending <= 1'b1;
            end else if (ld_last) begin
                sending <= 1'b0;               // nothing queued: go idle
            end else if (sending && o_fire) begin
                s_cnt <= s_cnt + 1'b1;         // next beat of the same fragment
            end
        end
    end
end
endgenerate

`ifndef LB_NO_ASSERT
    // synthesis translate_off
    // The fragment arithmetic replaces every divide and multiply by a shift, and
    // the block alignment by a mask. That is only the same function when the beat
    // size and the block size are powers of two -- true for every width the
    // generator emits. State it here so an odd width fails loudly rather than
    // silently mis-slicing fragments.
    initial begin
        if ((1 << LSB) != EXT_DATA_BYTES) begin
            $display("ERROR %m: EXT_DATA_WIDTH=%0d gives EXT_DATA_BYTES=%0d, which is not a power of two",
                     EXT_DATA_WIDTH, EXT_DATA_BYTES);
        end
        if ((FB_B & (FB_B-1)) != 0) begin
            $display("ERROR %m: FB_B=%0d is not a power of two", FB_B);
        end
        if ((RSPAN * EXT_DATA_BYTES) != FB_B) begin
            $display("ERROR %m: RSPAN=%0d does not divide FB_B=%0d into whole beats",
                     RSPAN, FB_B);
        end
        // The queue's CAPACITY is QD but it is addressed with QW bits (storage is
        // QSLOTS = 2**QW, see there). Nothing else notices when QW is short: the
        // pointers keep comparing, the pushes keep landing, they just land on the
        // wrong slots. This is what was missing when the QW ladder stopped at 5 --
        // a whole class of depth went wrong in silence. Checked here rather than
        // trusted to the ladder because the ladder is the thing that was wrong.
        // Keep this even though QSLOTS is derived from QW: if the ladder is ever
        // short again, QSLOTS < QD and this line is still the only thing that says so.
        if ((1 << QW) < QD) begin
            // Pushes past the addressable slots overwrite live entries.
            $display("ERROR %m: QW=%0d addresses %0d of the queue's %0d slots",
                     QW, 1 << QW, QD);
        end
        // Same reason, one level up: the LID must index the whole table.
        if ((1 << EXT_LID_WIDTH) < EXT_PENDING_TRANS) begin
            $display("ERROR %m: EXT_LID_WIDTH=%0d cannot index EXT_PENDING_TRANS=%0d entries",
                     EXT_LID_WIDTH, EXT_PENDING_TRANS);
        end
    end
    // synthesis translate_on
`endif
endmodule
