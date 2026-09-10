//============================================================================
// Filename    : lb_tniu_cmd_table.v
// Author      : litebus
// Description : TNIU local transaction table
// Date        : 2026-08-06
// Revision    : 1.0 initial -- per-round changes are in doc/HISTORY.md
//----------------------------------------------------------------------------
// Shared core of ID-compress (forward) and Resp-ID-gen (reverse) in TNIU.
//   Forward  : allocate a free entry for a command, store {int_id, src_id,
//              addr_lane, total_bytes}; the entry index is the local Slave id.
//   Reverse  : response carries the local id back; look up to restore context
//              and free the entry.
//
// Independent read/write lookup ports (key):
//   RSP_RD response (multi-beat burst) and RSP_WR response (single beat) may return in
//   the same cycle. Sharing one lookup port + free_id selector lets the write
//   preempt the read's LID, corrupting the RSP_RD flit src_id (src_id changes mid
//   burst, breaking reverse routing / burst-lock). Hence read and write each
//   have their own lookup port (rsp_rd_lid / rsp_wr_lid) and their own free strobe
//   (rsp_rd_free / rsp_wr_free). For a NON-atomic entry they can never name the same
//   entry in one cycle (asserted below); a two-response atomic entry (Adv 15,
//   ATOMIC_EN = 1) is the one legal case -- its B and R may land in the same
//   cycle, and the release arbitration (rsp_rd_rel / rsp_wr_rel) turns however they
//   arrive into exactly one release of the entry.
//
// TWO INDEPENDENT ALLOCATE PORTS (the REQ_R / REQ_W split):
//   Read requests arrive on REQ_R and write requests on REQ_W, two channels with
//   their own switches and their own credit loops, so they can present a command
//   in the SAME cycle. One allocate port cannot serve that: alloc_id is the
//   lowest free index, so both would be handed the same LID and the second
//   context write would win -- the Slave would see two transactions with one
//   txnid and the responses would restore the wrong src_id / int_id. That is the
//   same failure the lookup ports above were split to avoid, moved to the
//   allocate side.
//
//   The entries are FIRST COME FIRST SERVED out of ONE shared free set. What
//   keeps the two ports from being handed the same index is the END they pick
//   from, not an ownership mask and not an arbiter:
//
//       read  takes the LOWEST  free entry   (lb_lsb_onehot)
//       write takes the HIGHEST free entry   (lb_msb_onehot)
//
//   With two or more entries free those are different indices BY CONSTRUCTION,
//   both ports allocate in the same cycle, and the two prefix networks run in
//   PARALLEL. Exactly one free entry is the only case where the picks coincide,
//   and there a round-robin pointer decides whose turn it is, so only one port is
//   ready and only one allocates. Nothing compares the two picks.
//
//   THE POINTER IS A REGISTER, AND THAT IS THE WHOLE TRICK
//
//   The reason this module used an ownership mask instead of an arbiter was that
//   arbitrating "would put the grant in the ready path of a channel". A grant
//   computed from the two requests would. This one is not:
//
//       alloc_r_ready = (cnt_free > 1) || (cnt_free == 1 && rr_ptr == READ)
//       alloc_w_ready = (cnt_free > 1) || (cnt_free == 1 && rr_ptr == WRITE)
//
//   Both are functions of REGISTERS ONLY -- a count and a pointer. They do not
//   read the free vector and they do not read the other channel's request, so
//   the ready path is SHORTER than the `|free_r` it replaces, not longer, and
//   there is no combinational path between the two channels at all.
//
//   alloc_r_req / alloc_w_req enter only the pointer's NEXT state: the pointer
//   flips when its owner takes the last entry, and ALSO when its owner is not
//   asking while the other is. That second rule is not an optimisation, it is
//   what makes register-only ready safe -- ready cannot see that the owner is
//   idle, so without the yield an idle channel parks on the last entry and the
//   busy one hangs behind it. The two decisions are one decision.
//
//   The price is one cycle: the handover lands next cycle rather than this one,
//   so offering the last entry to a side that was not asking wastes a beat.
//   cnt_free == 1 is a transient (a saturated table sits at 0, an idle one sits
//   high), so that is a bubble on one entry of a transient state. The variant
//   that avoids it -- also offering the last entry to whoever is asking -- was
//   weighed and rejected: it drags the write egress's ready into the read
//   channel's valid to buy back that one beat.
//
//   FAIRNESS, AND WHY THERE IS NO RESERVED FLOOR
//
//   While two or more entries are free NEITHER channel can be refused -- that is
//   stronger than any static per-channel reservation, and it costs no capacity.
//   At exactly one free entry the pointer alternates, so a channel can lose the
//   last entry at most once in a row. No channel can be starved by the other's
//   arrival rate, which is what a fully shared FCFS pool cannot promise on its
//   own and what the ownership mask promised but did not deliver: measured on
//   bus_64m80s under saturation, its migration escape hatch was gated on the
//   donor being idle and was therefore vetoed on essentially every cycle it was
//   needed. See doc/HISTORY.md.
//
//   alloc_r_req / alloc_w_req mean "would allocate if an id were available" --
//   the caller must already have folded in its downstream readiness. A channel
//   stalled on its external port must NOT keep claiming ids; with one shared set
//   that no longer strands a private stock, but it still costs the other channel
//   entries it could have used.
//----------------------------------------------------------------------------
// ID scope (see arch.html section 6.2.1 / TNIU SPEC section 4.1):
//   LID exists ONLY between TNIU and the Slave IP; it never enters the fabric.
//   Inside the fabric transactions are identified by {src_id, dest_id, int_id}.
//   This table stores {int_id, src_id, addr_lane, total_bytes} -- dest_id is NOT
//   stored, because this TNIU *is* the destination. What restores the return
//   context is the entry CONTENT, not the LID itself; the LID is just the index.
//
//   The LID is one index space across both directions, exactly as before: the
//   split is in which POOL an id currently sits in, not in the id space. So
//   EXT_LID_WIDTH, the Slave txnid mapping and EXT_PENDING_TRANS all keep their
//   meaning, and a read LID and a write LID are still drawn from one range.
//
//   The table index width EXT_LID_WIDTH must satisfy 2**EXT_LID_WIDTH >=
//   EXT_PENDING_TRANS, otherwise the free-entry index truncates and allocation
//   hands out an index that is already live. The parent derives EXT_LID_WIDTH
//   from EXT_PENDING_TRANS so the two can never disagree.
//
//   The depth is NOT a power of two in general: Slaves.ext_pending_trans has
//   granularity 1, so EXT_LID_WIDTH = ceil(log2(depth)) can leave LID codes
//   EXT_PENDING_TRANS .. 2**EXT_LID_WIDTH-1 naming nothing. Those holes are
//   expressible on the Slave txnid pins (the TNIU zero-extends a LID on the way
//   out and truncates the echo on the way back), so the response paths gate on
//   an explicit range test -- see PEND_POW2 / g_rng_chk below. Everything else in
//   this table already sizes off EXT_PENDING_TRANS rather than 2**EXT_LID_WIDTH:
//   the valid vector, both one-hot picks (lb_lsb_onehot / lb_msb_onehot are N
//   generic), the encoder, and every context memory.
//
// Uniqueness invariant (TNIU SPEC section 4.1):
//   At most ONE live entry may carry a given (int_id, src_id) per direction.
//   Serialising same-ID transactions is the INIU / Master-IP side's job; if two
//   same-ID transactions were outstanding here they would take two different
//   LIDs, the Slave IP would see two unrelated ids and could complete them in
//   either order, and same-ID ordering would be silently lost. The simulation-
//   only check at the bottom of this file flags a violation instead.
//
// The read direction always stores addr_lane / total_bytes, because the read
//   lookup port is what replays them (rsp_wr_* restores int_id and src_id alone).
//   A write allocation stores them ONLY when it is a two-response atomic
//   (alloc_w_atm, Adv 15) -- the atomic's R replays them through the same read
//   lookup port. A plain write-allocated entry never has them looked up, so at
//   ATOMIC_EN = 0 the two memories keep their single write port exactly as
//   before the feature existed.
//
// Read interleaving (Adv 10) is handled by lb_tniu_rsp_rd_frag, which owns the
// per-LID assembly buffer; this table only supplies the per-LID context.
//----------------------------------------------------------------------------
`include "lb_defines.vh"

module lb_tniu_cmd_table #(
    parameter EXT_PENDING_TRANS = 8,         // number of table entries (any >= 1)
    parameter EXT_LID_WIDTH     = 3,         // local id width = ceil_log2(EXT_PENDING_TRANS)
    parameter INT_ID_WIDTH      = 8,         // network int id width
    parameter INT_SRC_ID_WIDTH  = 4,         // src id width
    parameter INT_LANE_W        = 7,         // addr_lane (phase) width
    parameter INT_TOTBYTES_W    = 16,        // total_bytes width
    // QoS context width. The QoS of a request is latched per entry on allocate
    // (both directions -- same shape as int_id / src_id) and looked up when the
    // response returns, so the fabric's RSP flits can carry it back to the
    // reverse switches. 0 = no QoS on this bus: the qos ports below become
    // 1-bit stubs (a V2001 port cannot be 0 wide, hence the _PW floor) and the
    // g_qos generate folds the storage away entirely.
    parameter INT_QOS_W         = 0,         // qos context width; 0 = no QoS
    // ---- derived; used in the port list, so it must stay in the parameter
    // ---- list. Not to be overridden externally.
    parameter INT_QOS_PW        = (INT_QOS_W < 1) ? 1 : INT_QOS_W,
    // 1 = two-response atomic slots (Adv 15): an entry allocated with
    // alloc_w_atm frees only after BOTH its B (rsp_wr_free) and its R (rsp_rd_free)
    // arrived, in either order or the same cycle. 0 elaborates the historic
    // single-response release bit for bit -- no pending flags, no arbitration.
    parameter ATOMIC_EN         = 0
) (
    // ---- inputs (clock/reset) ----
    input wire                          clk,                 // clock
    input wire                          rst_n,               // async reset, active low
    // ---- inputs (read allocate port, REQ_R) ----
    input wire                          alloc_r_req,         // would allocate if an id were free
    input wire                          alloc_r_fire,        // allocate commit (handshake ok)
    input wire  [INT_ID_WIDTH-1:0]      alloc_r_int_id,      // int id to store
    input wire  [INT_SRC_ID_WIDTH-1:0]  alloc_r_src_id,      // src id to store
    input wire  [INT_LANE_W-1:0]        alloc_r_addr_lane,   // addr_lane to store (read only)
    input wire  [INT_TOTBYTES_W-1:0]    alloc_r_total_bytes, // total_bytes to store (read only)
    input wire  [INT_QOS_PW-1:0]        alloc_r_qos,         // qos to store (stub at INT_QOS_W=0)
    // ---- inputs (write allocate port, REQ_W) ----
    input wire                          alloc_w_req,         // would allocate if an id were free
    input wire                          alloc_w_fire,        // allocate commit (handshake ok)
    input wire  [INT_ID_WIDTH-1:0]      alloc_w_int_id,      // int id to store
    input wire  [INT_SRC_ID_WIDTH-1:0]  alloc_w_src_id,      // src id to store
    input wire                          alloc_w_atm,         // two-response atomic entry (Adv 15)
    input wire  [INT_LANE_W-1:0]        alloc_w_addr_lane,   // addr_lane to store (atomic only)
    input wire  [INT_TOTBYTES_W-1:0]    alloc_w_total_bytes, // total_bytes to store (atomic only)
    input wire  [INT_QOS_PW-1:0]        alloc_w_qos,         // qos to store (stub at INT_QOS_W=0)
    // ---- inputs (reverse read lookup port) ----
    input wire  [EXT_LID_WIDTH-1:0]     rsp_rd_lid,          // read local id to look up
    input wire                          rsp_rd_free,         // free entry at RSP_RD burst last beat
    // ---- inputs (reverse write lookup port) ----
    input wire  [EXT_LID_WIDTH-1:0]     rsp_wr_lid,          // write local id to look up
    input wire                          rsp_wr_free,         // free entry at RSP_WR response
    // ---- outputs (read allocate port) ----
    output wire                         alloc_r_ready,       // the read pool has a free entry
    output wire [EXT_LID_WIDTH-1:0]     alloc_r_id,          // entry index offered to REQ_R
    // ---- outputs (write allocate port) ----
    output wire                         alloc_w_ready,       // the write pool has a free entry
    output wire [EXT_LID_WIDTH-1:0]     alloc_w_id,          // entry index offered to REQ_W
    // ---- outputs (reverse read lookup port) ----
    output wire [INT_ID_WIDTH-1:0]      rsp_rd_int_id,       // restored int id (read)
    output wire [INT_SRC_ID_WIDTH-1:0]  rsp_rd_src_id,       // restored src id (read)
    output wire [INT_LANE_W-1:0]        rsp_rd_addr_lane,    // restored addr_lane (read)
    output wire [INT_TOTBYTES_W-1:0]    rsp_rd_total_bytes,  // restored total_bytes (read)
    output wire [INT_QOS_PW-1:0]        rsp_rd_qos,          // restored qos (read; stub at INT_QOS_W=0)
    // ---- outputs (reverse write lookup port) ----
    output wire [INT_ID_WIDTH-1:0]      rsp_wr_int_id,       // restored int id (write)
    output wire [INT_SRC_ID_WIDTH-1:0]  rsp_wr_src_id,       // restored src id (write)
    output wire [INT_QOS_PW-1:0]        rsp_wr_qos           // restored qos (write; stub at INT_QOS_W=0)
);
    //------------------------------------------------------------------------
    // Derived local params
    //------------------------------------------------------------------------
    // The free counter must be able to hold EXT_PENDING_TRANS itself (the whole
    // table free), so it is one bit wider than an index.
    localparam FREE_CNT_W  = EXT_LID_WIDTH + 1;
    // EXT_PENDING_TRANS carried at FREE_CNT_W bits, for the cnt_free reset value.
    // A SIZED localparam is the only portable V2001 way to narrow an untyped
    // integer parameter: bit-selecting a parameter (EXT_PENDING_TRANS[FREE_CNT_W-1:0])
    // is not portable, and assigning the bare parameter would be a 32-bit literal
    // into a narrower reg -- legal, silent, one truncation away from wrong.
    // The elaboration check at the bottom of this file rejects a depth that does
    // not fit, so the narrowing here can never lose a bit.
    localparam [FREE_CNT_W-1:0] PEND_CNT = EXT_PENDING_TRANS;
    // Does every LID code name an entry? Depths are no longer powers of two
    // (Slaves.ext_pending_trans has granularity 1), so codes
    // EXT_PENDING_TRANS .. 2**EXT_LID_WIDTH-1 are HOLES: expressible on the Slave
    // txnid pins, naming nothing here. The response paths gate on that (g_rng_chk
    // below); at a power-of-two depth there is no hole and the gate folds away.
    localparam PEND_POW2 = (EXT_PENDING_TRANS == (1 << EXT_LID_WIDTH));
    // Round-robin pointer encoding: which port owns the LAST free entry.
    localparam RR_R    = 1'b0;
    localparam RR_W    = 1'b1;
    // Index of each allocate port inside the two flattened select buses.
    localparam P_R    = 0;
    localparam P_W    = 1;

    //------------------------------------------------------------------------
    // Table storage. valid is a packed vector so the free-entry selects are
    // parallel bit tricks instead of priority chains; the context fields stay
    // unpacked arrays (they are only ever read one entry at a time).
    //------------------------------------------------------------------------
    reg  [EXT_PENDING_TRANS-1:0]   valid;                              // entry valid flags
    reg  [FREE_CNT_W-1:0]          cnt_free;                           // free entries, whole table
    reg                            rr_ptr;                             // who owns the last free entry
    reg  [INT_ID_WIDTH-1:0]        int_id_mem [0:EXT_PENDING_TRANS-1]; // int id per entry
    reg  [INT_SRC_ID_WIDTH-1:0]    src_id_mem [0:EXT_PENDING_TRANS-1]; // src id per entry
    reg  [INT_LANE_W-1:0]          lane_mem   [0:EXT_PENDING_TRANS-1]; // addr_lane per entry (read only)
    reg  [INT_TOTBYTES_W-1:0]      totb_mem   [0:EXT_PENDING_TRANS-1]; // total_bytes per entry (read only)

    wire [EXT_PENDING_TRANS-1:0]   free_all;                           // 1 = entry free
    wire [2*EXT_PENDING_TRANS-1:0] oh_bus;                             // pick per port, [p*N +: N]
    wire [2*EXT_LID_WIDTH-1:0]     idx_bus;                            // that one-hot encoded, [p*W +: W]
    wire [EXT_PENDING_TRANS-1:0]   oh_r;                               // lowest free entry, one-hot
    wire [EXT_PENDING_TRANS-1:0]   oh_w;                               // highest free entry, one-hot
    wire                           free_ge2;                           // two or more entries free
    wire                           free_eq1;                           // exactly one entry free
    wire                           rr_flip;                            // pointer changes hands next cycle
    wire [EXT_PENDING_TRANS-1:0]   rsp_rd_bit;                         // one-hot of rsp_rd_lid
    wire [EXT_PENDING_TRANS-1:0]   rsp_wr_bit;                         // one-hot of rsp_wr_lid
    wire                           rsp_rd_in_rng;                      // rsp_rd_lid names a real entry
    wire                           rsp_wr_in_rng;                      // rsp_wr_lid names a real entry
    wire                           rsp_rd_rel;                         // this rsp_rd_free releases its entry
    wire                           rsp_wr_rel;                         // this rsp_wr_free releases its entry
    wire [EXT_PENDING_TRANS-1:0]   val_set;                            // valid bits to set this cycle
    wire [EXT_PENDING_TRANS-1:0]   val_clr;                            // valid bits to clear this cycle
    wire [EXT_PENDING_TRANS-1:0]   wen_r;                              // context write enable, read alloc
    wire [EXT_PENDING_TRANS-1:0]   wen_w;                              // context write enable, write alloc
    wire [EXT_PENDING_TRANS-1:0]   wen_watm;                           // lane/totb enable, atomic write alloc
    wire [1:0]                     free_gain;                          // entries returned to the set now
    wire [1:0]                     free_lose;                          // entries taken from the set now

    integer                        i;                                  // reset / assertion loop index
    integer                        mi;                                 // context memory write loop index
    genvar                         p;                                  // allocate port index
    genvar                         k;                                  // encoded-bit index
    genvar                         e;                                  // entry index
`ifndef LB_NO_ASSERT
    // synthesis translate_off
    // direction shadow, assertion use only: 0 = read, 1 = write, 2 = atomic.
    // An atomic occupies BOTH directions of the (int_id, src_id) space -- its B
    // and its R each come back carrying that key -- so the uniqueness checks
    // below treat state 2 as a member of both.
    reg  [1:0] dir_mem [0:EXT_PENDING_TRANS-1]; // per-entry direction shadow (see above)
    reg  dup_hit;                               // a live entry already holds this key
    integer pc_free;                            // popcount of free_all, assertion use only
    // synthesis translate_on
`endif

    //------------------------------------------------------------------------
    // Combinational: ONE shared free set, picked from opposite ends.
    //
    // Both selects see the SAME vector. What makes their results different is the
    // end they scan from, so whenever two or more bits are set the two indices
    // differ by construction -- no comparison, no arbiter, and the two prefix
    // networks are PARALLEL rather than in series. Taking "lowest free" and then
    // "lowest free above that" is the same function but chains two networks
    // (7 -> 14 levels at EXT_PENDING_TRANS=128).
    //
    // Exactly one bit set is the only case where the picks coincide, and the
    // round-robin pointer below makes only one port ready there, so the
    // coincidence is never observable at the allocate outputs.
    //
    // Both are real log-depth prefix networks (the same lb_lsb_onehot the switch
    // arbiter uses, plus its mirror). Their second outputs are unused here
    // because allocation has no rotation, and optimise away.
    //------------------------------------------------------------------------
    assign free_all = ~valid;

    lb_lsb_onehot #(
        .N      (EXT_PENDING_TRANS)                     // table depth
    ) u10_free_sel_r (
        .i_vec  (free_all),                             // the shared free set
        .onehot (oh_bus[P_R*EXT_PENDING_TRANS +: EXT_PENDING_TRANS]),  // lowest free entry
        .above  ()                                      // unused: allocation has no rotation
    );

    lb_msb_onehot #(
        .N      (EXT_PENDING_TRANS)                     // table depth
    ) u11_free_sel_w (
        .i_vec  (free_all),                             // the same shared free set
        .onehot (oh_bus[P_W*EXT_PENDING_TRANS +: EXT_PENDING_TRANS]),  // highest free entry
        .below  ()                                      // unused: allocation has no rotation
    );

    //------------------------------------------------------------------------
    // One-hot to binary, once per allocate port. Written as a generate over the
    // port index rather than twice, so the two encoders cannot drift.
    //------------------------------------------------------------------------
    generate
    for (p = 0; p < 2; p = p + 1) begin : g_enc_port
        for (k = 0; k < EXT_LID_WIDTH; k = k + 1) begin : g_enc_bit
            wire [EXT_PENDING_TRANS-1:0] col;      // entries whose index has bit k set
            for (e = 0; e < EXT_PENDING_TRANS; e = e + 1) begin : g_enc_col
                assign col[e] = ((e >> k) & 1) ? oh_bus[p*EXT_PENDING_TRANS + e] : 1'b0;
            end
            assign idx_bus[p*EXT_LID_WIDTH + k] = |col;
        end
    end
    endgenerate

    //------------------------------------------------------------------------
    // Combinational: allocate outputs and independent read/write lookups
    //------------------------------------------------------------------------
    assign oh_r           = oh_bus[P_R*EXT_PENDING_TRANS +: EXT_PENDING_TRANS];
    assign oh_w           = oh_bus[P_W*EXT_PENDING_TRANS +: EXT_PENDING_TRANS];
    // Ready, and why it is built from the COUNT rather than from the vector.
    //
    // Not `|free_all`: that answers "is anything free", which is right for one
    // port and wrong for two -- with one entry left it makes BOTH ports ready and
    // both are handed that entry. The count distinguishes "at least two" from
    // "exactly one", which is the only distinction this module needs, and it is a
    // register, so free_ge2 / free_eq1 cost a comparator instead of an N-input OR
    // tree (8 levels at EXT_PENDING_TRANS=128). The popcount assertion at the
    // bottom is what makes reading the counter instead of the vector legitimate.
    //
    // The last entry goes to the pointer's owner, full stop. Ready therefore
    // reads REGISTERS ONLY -- a count and a pointer. It does not read the free
    // vector and it does not read the other channel's request, so there is no
    // combinational path between the two channels at all and this path is
    // SHORTER than the `|free_r` it replaces.
    //
    // The alternative was to also offer the last entry to whoever is asking when
    // the owner is not, which wastes no cycle but puts the other channel's request
    // in this expression. That was weighed and rejected: the request cone would
    // reach from the write egress's ready into the read channel's valid, and the
    // cost it buys back is one cycle on ONE entry of a transient state
    // (a saturated table sits at cnt_free 0, an idle one sits high).
    //
    // Because ready ignores the requests, the pointer MUST hand itself over when
    // its owner is idle -- see rr_flip. Those two decisions are one decision: take
    // register-only ready without the yield and an idle owner parks on the last
    // entry forever while the busy channel blocks on it.
    assign free_ge2       = (cnt_free > {{(FREE_CNT_W-1){1'b0}}, 1'b1});
    assign free_eq1       = (cnt_free == {{(FREE_CNT_W-1){1'b0}}, 1'b1});
    assign alloc_r_ready  = free_ge2 || (free_eq1 && (rr_ptr == RR_R));
    assign alloc_w_ready  = free_ge2 || (free_eq1 && (rr_ptr == RR_W));
    assign alloc_r_id     = idx_bus[P_R*EXT_LID_WIDTH +: EXT_LID_WIDTH];
    assign alloc_w_id     = idx_bus[P_W*EXT_LID_WIDTH +: EXT_LID_WIDTH];
    assign rsp_rd_int_id      = int_id_mem[rsp_rd_lid];   // read-port lookup
    assign rsp_rd_src_id      = src_id_mem[rsp_rd_lid];
    assign rsp_rd_addr_lane   = lane_mem[rsp_rd_lid];
    assign rsp_rd_total_bytes = totb_mem[rsp_rd_lid];
    assign rsp_wr_int_id      = int_id_mem[rsp_wr_lid];   // write-port lookup
    assign rsp_wr_src_id      = src_id_mem[rsp_wr_lid];

    //------------------------------------------------------------------------
    // Combinational: when the last-entry pointer changes hands.
    //
    // Two reasons, and the second one is NOT optional -- it is what makes
    // register-only ready safe:
    //   * the owner USED the last entry: the other side gets the next one;
    //   * the owner is NOT asking while the other side IS: ready cannot see the
    //     requests, so if the pointer did not move here the busy channel would
    //     wait forever on an entry its idle peer is holding. This is the yield,
    //     and removing it DEADLOCKS. It costs one cycle -- the handover happens
    //     next cycle rather than this one -- and that bubble is the price of
    //     keeping both channels' requests out of the ready path.
    //
    // Only the second reason reads alloc_*_req, and it feeds a REGISTER's D input,
    // never a ready output.
    //
    // Nothing fires unless free_eq1: with two or more entries free both ports are
    // served and the pointer is deciding nothing, so leaving it parked is correct
    // and keeps this out of the common case entirely.
    //------------------------------------------------------------------------
    assign rr_flip = free_eq1
                  && (((rr_ptr == RR_R) && (alloc_r_fire || (!alloc_r_req && alloc_w_req)))
                   || ((rr_ptr == RR_W) && (alloc_w_fire || (!alloc_w_req && alloc_r_req))));

    assign rsp_rd_bit  = {{(EXT_PENDING_TRANS-1){1'b0}}, 1'b1} << rsp_rd_lid;
    assign rsp_wr_bit  = {{(EXT_PENDING_TRANS-1){1'b0}}, 1'b1} << rsp_wr_lid;

    //------------------------------------------------------------------------
    // Does the LID a response names actually exist? Only at a non-power-of-two
    // depth can it not: codes EXT_PENDING_TRANS .. 2**EXT_LID_WIDTH-1 are holes.
    //
    // A hole value is a protocol violation by the Slave IP (it must echo the
    // txnid this TNIU issued, verbatim) and NOTHING here can repair it -- but the
    // table must not be made dishonest by it. Ungated, a hole response would
    // shift rsp_*_bit out of the vector (val_clr all zero, so no valid bit is
    // cleared) while free_gain still counts 1: cnt_free drifts UP, and after
    // enough of them the table degrades into exactly the over-allocation case
    // described at the cnt_free reset above. Gating the release keeps the free
    // count truthful and costs the entry instead -- a permanent capacity loss of
    // one slot, which is loud in the pending report and in the assertions below,
    // rather than silent data delivered to the wrong Master.
    //
    // Clamping the index instead would be strictly worse: it converts "a broken
    // Slave named nothing" into "we destroyed a GOOD entry", and does so silently.
    //
    // At a power-of-two depth there is no hole, so g_rng_all makes this the
    // constant 1 and the whole thing folds away -- that is what keeps the
    // power-of-two configurations bit-for-bit as they were.
    //------------------------------------------------------------------------
    generate
    if (PEND_POW2) begin : g_rng_all
        assign rsp_rd_in_rng = 1'b1;
        assign rsp_wr_in_rng = 1'b1;
    end
    else begin : g_rng_chk
        assign rsp_rd_in_rng = (rsp_rd_lid < EXT_PENDING_TRANS);
        assign rsp_wr_in_rng = (rsp_wr_lid < EXT_PENDING_TRANS);
    end
    endgenerate

    //------------------------------------------------------------------------
    // Release strobes (Adv 15). Everything below this block -- valid clear,
    // ownership, pool counters -- fires on rsp_rd_rel / rsp_wr_rel, "this free actually
    // returns the entry to a pool", NOT on the raw response strobes.
    //
    // ATOMIC_EN = 0: rel IS free, wire for wire; nothing else in the module
    // changes and no pending state exists.
    //
    // ATOMIC_EN = 1: a two-response atomic entry carries two pending flags, set
    // together at allocation. Each response clears its own flag; the entry
    // releases on the response that finds the OTHER flag already clear. The
    // flags are registers, so when both responses land in the SAME cycle
    // neither sees the other's clearing -- that is what the combinational
    // same_lid_both cross-term is for, and it is deliberately asymmetric:
    // folded into rsp_rd_rel with OR (that side releases) and into rsp_wr_rel with
    // AND-NOT (that side stands down). Give it to both and the entry is
    // credited to a pool twice (the counters drift, the popcount assertion
    // fires); give it to neither and the entry leaks forever, because no
    // further response will ever name it. Which side wins is an arbitrary
    // convention -- the requirement is exactly one.
    //
    // A non-atomic entry has both flags at 0, so rsp_rd_rel / rsp_wr_rel collapse onto
    // rsp_rd_free / rsp_wr_free (same_lid_both on such an entry is a protocol violation
    // and asserted against below, as it always was).
    //------------------------------------------------------------------------
    generate
    if (ATOMIC_EN == 0) begin : g_noatm
        assign rsp_rd_rel = rsp_rd_free && rsp_rd_in_rng;
        assign rsp_wr_rel = rsp_wr_free && rsp_wr_in_rng;
    end
    else begin : g_atm
        // one flag pair per entry: "this response is still owed"
        reg  [EXT_PENDING_TRANS-1:0] atm_pend_r;    // R (rsp_rd_free) still owed
        reg  [EXT_PENDING_TRANS-1:0] atm_pend_b;    // B (rsp_wr_free) still owed
        wire                         same_lid_both; // B and R name one LID now

        assign same_lid_both = rsp_rd_free && rsp_wr_free && (rsp_rd_lid == rsp_wr_lid);
        assign rsp_rd_rel = rsp_rd_free && rsp_rd_in_rng
                         && (!atm_pend_b[rsp_rd_lid] || same_lid_both);
        assign rsp_wr_rel = rsp_wr_free && rsp_wr_in_rng
                         && !atm_pend_r[rsp_wr_lid] && !same_lid_both;

        // Set together on an atomic allocation (oh_w is already one-hot, same
        // source as val_set); each response clears its own flag. A half-free
        // touches nothing but its flag -- valid / ownership / counters move
        // only on the actual release.
        always @(posedge clk or negedge rst_n) begin
            if (!rst_n) begin
                atm_pend_r <= {EXT_PENDING_TRANS{1'b0}};
                atm_pend_b <= {EXT_PENDING_TRANS{1'b0}};
            end
            else begin
                atm_pend_r <= (atm_pend_r & ~(rsp_rd_free ? rsp_rd_bit : {EXT_PENDING_TRANS{1'b0}}))
                            | ((alloc_w_fire && alloc_w_atm) ? oh_w : {EXT_PENDING_TRANS{1'b0}});
                atm_pend_b <= (atm_pend_b & ~(rsp_wr_free ? rsp_wr_bit : {EXT_PENDING_TRANS{1'b0}}))
                            | ((alloc_w_fire && alloc_w_atm) ? oh_w : {EXT_PENDING_TRANS{1'b0}});
            end
        end
    end
    endgenerate

    //------------------------------------------------------------------------
    // valid update, as a set mask and a clear mask.
    //
    // The set masks are the SELECT one-hots, not a re-decode of alloc_*_id.
    // Written as four `valid[<index>] <= ...` statements instead, yosys builds a
    // shift-based decoder from the binary index and then chains the four masked
    // updates in series: measured at EXT_PENDING_TRANS=8 that spelling cost 27
    // levels, of which about seven were the encode-then-decode round trip alone.
    // The one-hot is already there -- encoding it to binary and decoding it back
    // is pure waste, and the four updates are independent, so they belong in one
    // parallel expression.
    //
    // rsp_rd_bit / rsp_wr_bit do need a decoder: those indices arrive as binary from the
    // response side, so there is no one-hot to reuse.
    //------------------------------------------------------------------------
    assign val_set = (alloc_r_fire ? oh_r   : {EXT_PENDING_TRANS{1'b0}})
                   | (alloc_w_fire ? oh_w   : {EXT_PENDING_TRANS{1'b0}});
    assign val_clr = (rsp_rd_rel       ? rsp_rd_bit : {EXT_PENDING_TRANS{1'b0}})
                   | (rsp_wr_rel       ? rsp_wr_bit : {EXT_PENDING_TRANS{1'b0}});

    //------------------------------------------------------------------------
    // Free-set occupancy: one gain and one loss term, mirroring val_clr /
    // val_set exactly. The popcount assertion at the bottom is what proves the
    // counter tracks free_all -- and it has to, because the ready outputs are
    // computed from the counter and not from the vector.
    //
    // Both terms can be 2 in one cycle (both ports allocating, or an atomic's B
    // and R releasing two different entries), which is why they are 2 bits wide
    // and not 1. Sizing them 1 bit would silently drop the second event.
    //------------------------------------------------------------------------
    assign free_gain = rsp_rd_rel + rsp_wr_rel;
    assign free_lose = alloc_r_fire + alloc_w_fire;

    //------------------------------------------------------------------------
    // Sequential: valid set on either allocate, cleared on read/write RELEASE
    // (a half-freed atomic entry stays valid until its second response).
    // No bit can be in both masks: the two allocates pick from opposite ends of
    // the same free set and only one is ready when those ends coincide, and a
    // released entry is live while an allocated one is free.
    //------------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            valid <= {EXT_PENDING_TRANS{1'b0}};
        end else begin
            valid <= (valid & ~val_clr) | val_set;
        end
    end

    //------------------------------------------------------------------------
    // Sequential: the free counter and the last-entry token.
    //
    // The counter starts at the full table and moves with the same events as
    // `valid`, which is what lets the ready outputs read it instead of the
    // vector. The token's reset value is arbitrary -- it only decides who wins
    // the first contested last entry -- but it must be a constant, not X, or the
    // first contest is resolved by simulation luck and the two runs differ.
    //------------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            // The whole table is free at reset, so this is EXT_PENDING_TRANS --
            // carried at FREE_CNT_W bits by the sized localparam (see PEND_CNT for
            // why a sized localparam and not a bit-select or a bare parameter).
            //
            // This used to be spelled {1'b1, EXT_LID_WIDTH zeros}, i.e. 2**LID_W,
            // which was only equal to the depth while every depth was a power of
            // two. At EXT_PENDING_TRANS=5 that reset value is 8: the ready outputs
            // read cnt_free (not the vector, see the block comment above), so the
            // table would keep saying "ready" after all 5 entries were taken,
            // free_all would be 0, both one-hot picks would be 0, and BOTH allocate
            // ports would hand out index 0 -- overwriting a live entry. The
            // same-key assertion cannot see that; only the popcount assertion can,
            // and that one does not exist under LB_NO_ASSERT or at gate level.
            cnt_free <= PEND_CNT;
            rr_ptr   <= RR_R;
        end else begin
            // cnt_free + free_gain never exceeds EXT_PENDING_TRANS (releases only
            // happen on live entries, so gain and cnt_free cannot both be large),
            // and FREE_CNT_W holds 2*EXT_PENDING_TRANS-1, so the intermediate of
            // this left-to-right evaluation cannot wrap.
            cnt_free <= cnt_free + free_gain - free_lose;
            rr_ptr   <= rr_ptr ^ rr_flip;
        end
    end

    //------------------------------------------------------------------------
    // Context write enables: one bit per entry, taken straight off the select
    // one-hots. What matters here is what is NOT in them -- no binary index.
    //
    // `int_id_mem[alloc_r_id] <= x` reads like one assignment, but alloc_r_id is
    // oh_r ENCODED to binary, so the synthesiser has to DECODE it back into N
    // write enables -- a round trip whose input one-hot was in hand all along.
    // It is not free and it is not hypothetical: DC put it on the critical path.
    // The 2026-08-16 atomic top report (2.5 GHz) has
    //     u11_free_sel_w/onehot[188] -> oh_bus[444] -> ...12 gates... ->
    //     int_id_mem_reg[126][4:2]/D1
    // where entry 188's select bit reaches entry 126's write DATA. That can only
    // happen through the shared index -- one entry's one-hot bit has no other
    // way to touch a different entry. Same finding as the valid update above
    // (see the note there): the one-hot is already built, so use it.
    //
    // wen_r and wen_w can never overlap: the two picks come from opposite ends of
    // the free set, and when those ends coincide only one port is ready. The data
    // select below therefore resolves nothing real -- and because a collision
    // would corrupt an entry SILENTLY (read wins, no other symptom), it is
    // asserted rather than assumed, next to the free-count check.
    //------------------------------------------------------------------------
    assign wen_r    = alloc_r_fire ? oh_r : {EXT_PENDING_TRANS{1'b0}};
    assign wen_w    = alloc_w_fire ? oh_w : {EXT_PENDING_TRANS{1'b0}};
    assign wen_watm = (ATOMIC_EN != 0 && alloc_w_fire && alloc_w_atm)
                      ? oh_w : {EXT_PENDING_TRANS{1'b0}};

    //------------------------------------------------------------------------
    // Sequential: store context on allocate. int_id / src_id take both ports;
    // addr_lane / total_bytes are read-direction only, plus the Adv 15 atomic
    // replay -- a two-response atomic replays them on its R, so its WRITE
    // allocation stores them too. At ATOMIC_EN = 0 wen_watm is the constant 0,
    // the enable collapses to wen_r and synthesis prunes the second data arm.
    //
    // One always block with a procedural for, NOT N generate blocks: the
    // lb_lsb_onehot note explains what per-bit continuous assigns did to
    // simulation time at N = 256. A loop inside one process is not that.
    //------------------------------------------------------------------------
    always @(posedge clk) begin
        for (mi = 0; mi < EXT_PENDING_TRANS; mi = mi + 1) begin
            if (wen_r[mi] || wen_w[mi]) begin
                int_id_mem[mi] <= wen_r[mi] ? alloc_r_int_id : alloc_w_int_id;
                src_id_mem[mi] <= wen_r[mi] ? alloc_r_src_id : alloc_w_src_id;
            end
            if (wen_r[mi] || wen_watm[mi]) begin
                lane_mem[mi]   <= wen_r[mi] ? alloc_r_addr_lane   : alloc_w_addr_lane;
                totb_mem[mi]   <= wen_r[mi] ? alloc_r_total_bytes : alloc_w_total_bytes;
            end
        end
    end

    //------------------------------------------------------------------------
    // QoS context (only on a bus that carries the QOS segment). Same shape as
    // int_id / src_id above: BOTH directions store it on allocate and BOTH
    // lookups restore it -- a write response needs it as much as a read one
    // (the reverse switches arbitrate RSP_WR by it too), which is why it sits
    // in the wen_r|wen_w group and NOT in the read-only/atomic wen_watm group.
    // Same one-hot write enables, same single always with a procedural for --
    // see the two notes above for why neither is negotiable. Folded away as a
    // generate so a QoS-free table elaborates today's logic bit for bit.
    //------------------------------------------------------------------------
    generate
    if (INT_QOS_W > 0) begin : g_qos
        reg     [INT_QOS_W-1:0] qos_mem [0:EXT_PENDING_TRANS-1]; // qos per entry
        integer                 qi;                              // context write loop index
        always @(posedge clk) begin
            for (qi = 0; qi < EXT_PENDING_TRANS; qi = qi + 1) begin
                if (wen_r[qi] || wen_w[qi]) begin
                    qos_mem[qi] <= wen_r[qi] ? alloc_r_qos : alloc_w_qos;
                end
            end
        end
        assign rsp_rd_qos = qos_mem[rsp_rd_lid];   // read-port lookup
        assign rsp_wr_qos = qos_mem[rsp_wr_lid];   // write-port lookup
    end else begin : g_qos_off
        assign rsp_rd_qos = 1'b0;                  // no QoS on this bus: stubs tied low
        assign rsp_wr_qos = 1'b0;
    end
    endgenerate

    //------------------------------------------------------------------------
    // Simulation-only checks. Not synthesized (LB_NO_ASSERT can switch them off
    // for lint-clean builds). The direction shadow below exists so the
    // uniqueness check does not fire on a read and a write that legitimately
    // share an id (separate id spaces, separate response channels, separate
    // LIDs -- harmless).
    //------------------------------------------------------------------------
`ifndef LB_NO_ASSERT
    // synthesis translate_off
    always @(posedge clk) begin
        if (alloc_r_fire) begin
            dir_mem[alloc_r_id] <= 2'd0;
        end
        if (alloc_w_fire) begin
            // === so an undriven alloc_w_atm (a bench that predates the port)
            // degrades to the historic write direction instead of poisoning the
            // shadow with X and silently disabling the uniqueness checks
            dir_mem[alloc_w_id] <= (alloc_w_atm === 1'b1) ? 2'd2 : 2'd1;
        end
    end

    always @(posedge clk) begin
        // cnt_free is redundant with `valid`, and the READY OUTPUTS ARE COMPUTED
        // FROM IT rather than from the vector -- so this comparison is not a
        // nicety, it is the thing that makes reading the counter legitimate. Any
        // slip in the gain/loss terms is otherwise SILENT in two directions at
        // once: counting too low quietly holds fewer outstanding transactions,
        // counting too high hands out an entry that is still live.
        if (rst_n) begin
            pc_free = 0;
            for (i = 0; i < EXT_PENDING_TRANS; i = i + 1) begin
                if (free_all[i]) begin
                    pc_free = pc_free + 1;
                end
            end
            if (cnt_free !== pc_free[FREE_CNT_W-1:0]) begin
                $display({"[%0t] ERROR %m: free count drifted: cnt_free=%0d but %0d free",
                          " -- every allocate decision this cycle used a wrong count"},
                         $time, cnt_free, pc_free);
            end
            // The property the whole allocate side rests on: the two ports are
            // never handed the same entry. It is checked on the FIRES, not on the
            // picks, because the picks legitimately coincide when one entry is
            // left -- what must never happen is both firing on it. A violation
            // here is the failure the ownership mask was built to prevent, and it
            // is silent in exactly the same way it was then: the Slave sees two
            // transactions under one txnid and the responses restore the wrong
            // context.
            if (alloc_r_fire && alloc_w_fire && (alloc_r_id === alloc_w_id)) begin
                $display({"[%0t] ERROR %m: both allocate ports fired on LID %0d",
                          " (cnt_free=%0d rr_ptr=%0b) -- two transactions share one entry"},
                         $time, alloc_r_id, cnt_free, rr_ptr);
            end
            // Firing without being ready is not a table bug, but it produces the
            // same corruption, and the caller's gating is now spread over three
            // expressions in lb_tniu_int_core (:469, :620, :685). Catch it here.
            if ((alloc_r_fire && !alloc_r_ready) || (alloc_w_fire && !alloc_w_ready)) begin
                $display({"[%0t] ERROR %m: an allocate fired without ready",
                          " (r %0b/%0b w %0b/%0b) -- the entry it took was not offered"},
                         $time, alloc_r_fire, alloc_r_ready, alloc_w_fire, alloc_w_ready);
            end
        end
        // The two allocate ports must never hand out the same index. Structural,
        // but this is the property the whole ownership mask exists to provide, so
        // it is checked rather than assumed.
        if (rst_n && alloc_r_fire && alloc_w_fire && (alloc_r_id === alloc_w_id)) begin
            $display("[%0t] ERROR %m: both allocate ports took LID %0d; pools overlap",
                     $time, alloc_r_id);
        end
        // The two context write selects must never name the same entry. The data
        // mux in the store block resolves a collision in favour of the read port
        // and says nothing, so a slip in the allocate side would corrupt one
        // entry with no other symptom -- this is the check that makes that mux
        // safe to write. Distinct from the alloc_r_id/alloc_w_id check above:
        // that one guards the INDEX handed out, this one guards the array write.
        if (rst_n && ((wen_r & (wen_w | wen_watm)) !== {EXT_PENDING_TRANS{1'b0}})) begin
            $display("[%0t] ERROR %m: context write enables overlap; two allocates named one entry", $time);
        end
        // THE YIELD MUST HAPPEN. Ready is computed from registers alone, so a
        // channel blocked on the last entry has exactly one way out: the pointer
        // moving. If someone later "simplifies" rr_flip to fire only on an actual
        // allocation -- which reads like the tidier rule, and was in fact the
        // first thing tried here -- an idle owner parks on the last entry and the
        // busy channel HANGS. Not degrades: hangs, with cnt_free stuck at 1 and no
        // error anywhere. So the requirement is asserted as an implication rather
        // than trusted to the expression above.
        if (rst_n && free_eq1 && !rr_flip
            && (((rr_ptr == RR_R) && !alloc_r_req && alloc_w_req)
             || ((rr_ptr == RR_W) && !alloc_w_req && alloc_r_req))) begin
            $display({"[%0t] ERROR %m: last entry held by an idle port while the other",
                      " is asking, and the token is not moving (rr_ptr=%0b r_req=%0b w_req=%0b)"},
                     $time, rr_ptr, alloc_r_req, alloc_w_req);
        end
        // And the mirror: it must not move when nothing is deciding. Spending the
        // token on an uncontested cycle is not a functional failure -- it charges
        // a channel for a turn it never used -- so nothing else in this file can
        // see it and the only effect is a fairness skew under sustained pressure.
        // That is precisely the class lb_sw_arbiter's collapsed RR mask belonged
        // to, and that one stayed green for a whole release.
        if (rst_n && rr_flip && !free_eq1) begin
            $display({"[%0t] ERROR %m: the last-entry token moved with %0d entries free",
                      " -- it was not deciding anything"}, $time, cnt_free);
        end
        if (rst_n && alloc_r_fire) begin
            dup_hit = 1'b0;
            for (i = 0; i < EXT_PENDING_TRANS; i = i + 1)
                if (valid[i] && (int_id_mem[i] == alloc_r_int_id)
                             && (src_id_mem[i] == alloc_r_src_id)
                             && (dir_mem[i] != 2'd1)) begin
                        dup_hit = 1'b1;
                end
            // Same (int_id, src_id) twice in flight in the same direction. It must
            // be serialised UPSTREAM (INIU side): the Slave IP is free to reorder
            // two transactions carrying the same id, and then same-id ordering is
            // lost with nothing here able to restore it. A live ATOMIC (dir 2)
            // counts for BOTH directions -- its R shares the read-return path.
            if (dup_hit) begin
                $display("[%0t] ERROR %m: read (int_id=%0h, src_id=%0h) already outstanding",
                         $time, alloc_r_int_id, alloc_r_src_id);
            end
        end
        if (rst_n && alloc_w_fire) begin
            dup_hit = 1'b0;
            for (i = 0; i < EXT_PENDING_TRANS; i = i + 1)
                if (valid[i] && (int_id_mem[i] == alloc_w_int_id)
                             && (src_id_mem[i] == alloc_w_src_id)
                             && ((dir_mem[i] != 2'd0) || alloc_w_atm)) begin
                        dup_hit = 1'b1;
                end
            // Same rule as the read side above: serialise same-id upstream.
            // Any write-side allocation conflicts with a live write or atomic
            // (dir 1 / 2); an INCOMING atomic conflicts with a live read too,
            // because its R will share that key on the read-return path.
            if (dup_hit) begin
                $display("[%0t] ERROR %m: write (int_id=%0h, src_id=%0h) already outstanding",
                         $time, alloc_w_int_id, alloc_w_src_id);
            end
        end
        // A response must name a LID that is actually live, or it indexes stale
        // context and the read data goes back to the wrong Master.
        //
        // The range test comes FIRST and the two are chained with else-if, which
        // is load bearing: at a non-power-of-two depth a hole LID indexes valid[]
        // out of range, so `!valid[hole]` is X, `if (X)` is false, and the liveness
        // check below prints NOTHING (or worse, a misleading "not live" once the
        // tools disagree about X). Naming the two failures separately is the only
        // way "the Slave echoed a txnid we never issued" and "the Slave freed an
        // entry that was already free" stay distinguishable.
        if (rst_n && rsp_rd_free && (rsp_rd_lid >= EXT_PENDING_TRANS)) begin
            $display({"[%0t] ERROR %m: RSP_RD response named LID %0d but only %0d",
                      " entries exist -- the Slave echoed a txnid this table never",
                      " issued; the entry is NOT released (rsp_rd_in_rng) so the",
                      " free count stays honest, at the cost of one slot"},
                     $time, rsp_rd_lid, EXT_PENDING_TRANS);
        end
        else if (rst_n && rsp_rd_free && !valid[rsp_rd_lid]) begin
            $display("[%0t] ERROR %m: RSP_RD response freed LID %0d which is not live",
                     $time, rsp_rd_lid);
        end
        if (rst_n && rsp_wr_free && (rsp_wr_lid >= EXT_PENDING_TRANS)) begin
            $display({"[%0t] ERROR %m: RSP_WR response named LID %0d but only %0d",
                      " entries exist -- the Slave echoed a txnid this table never",
                      " issued; the entry is NOT released (rsp_wr_in_rng) so the",
                      " free count stays honest, at the cost of one slot"},
                     $time, rsp_wr_lid, EXT_PENDING_TRANS);
        end
        else if (rst_n && rsp_wr_free && !valid[rsp_wr_lid]) begin
            $display("[%0t] ERROR %m: RSP_WR response freed LID %0d which is not live",
                     $time, rsp_wr_lid);
        end
    end

    // The same-LID pair of checks depends on the atomic pending flags, which
    // only exist when the g_atm branch elaborated -- hence a generate pair
    // rather than one always with a dead hierarchical reference.
    generate
    if (ATOMIC_EN == 0) begin : g_chk_noatm
        always @(posedge clk) begin
            // one response per entry: both strobes naming one LID is a protocol
            // violation, exactly as before the atomic feature existed
            if (rst_n && rsp_rd_free && rsp_wr_free && (rsp_rd_lid == rsp_wr_lid)) begin
                $display("[%0t] ERROR %m: RSP_RD and RSP_WR responses freed the same LID %0d in one cycle",
                         $time, rsp_rd_lid);
            end
        end
    end
    else begin : g_chk_atm
        always @(posedge clk) begin
            // a same-cycle pair is legal ONLY on an atomic entry still owing
            // both halves; anything else is the historic protocol violation
            if (rst_n && rsp_rd_free && rsp_wr_free && (rsp_rd_lid == rsp_wr_lid)
                      && !(g_atm.atm_pend_r[rsp_rd_lid] && g_atm.atm_pend_b[rsp_rd_lid])) begin
                $display("[%0t] ERROR %m: RSP_RD and RSP_WR responses freed the same LID %0d in one cycle",
                         $time, rsp_rd_lid);
            end
            // duplicate halves: a second B while the R is still owed, or a
            // second R while the B is still owed, names a half already consumed
            if (rst_n && rsp_wr_free && valid[rsp_wr_lid]
                      && g_atm.atm_pend_r[rsp_wr_lid] && !g_atm.atm_pend_b[rsp_wr_lid]) begin
                $display("[%0t] ERROR %m: second B for atomic LID %0d (R still pending)",
                         $time, rsp_wr_lid);
            end
            if (rst_n && rsp_rd_free && valid[rsp_rd_lid]
                      && g_atm.atm_pend_b[rsp_rd_lid] && !g_atm.atm_pend_r[rsp_rd_lid]) begin
                $display("[%0t] ERROR %m: second R for atomic LID %0d (B still pending)",
                         $time, rsp_rd_lid);
            end
        end
    end
    endgenerate

    initial begin
        if ((1 << EXT_LID_WIDTH) < EXT_PENDING_TRANS) begin
            // A truncated free-entry index hands out a LID that is still in use.
            // This is also the guard that keeps PEND_CNT's narrowing lossless and
            // the hole set non-negative -- everything below assumes it held.
            $display("ERROR %m: EXT_LID_WIDTH=%0d cannot index %0d entries",
                     EXT_LID_WIDTH, EXT_PENDING_TRANS);
        end
        // MINIMALITY, not equality. Depths have granularity 1 now
        // (Slaves.ext_pending_trans), so EXT_PENDING_TRANS < 2**EXT_LID_WIDTH is
        // the normal case; what is still wrong is a LID width WIDER than the depth
        // needs, because then more than half the LID code space names nothing and
        // the parent must have bypassed its own clog2 ladder (lb_tniu_int_core
        // derives EXT_LID_WIDTH as a localparam precisely so nobody can inject it).
        if ((EXT_PENDING_TRANS * 2) <= (1 << EXT_LID_WIDTH)) begin
            $display({"ERROR %m: EXT_PENDING_TRANS=%0d needs only %0d LID bits but",
                      " EXT_LID_WIDTH=%0d was injected; %0d of the %0d LID codes",
                      " name no entry at all"},
                     EXT_PENDING_TRANS, (EXT_LID_WIDTH - 1), EXT_LID_WIDTH,
                     ((1 << EXT_LID_WIDTH) - EXT_PENDING_TRANS), (1 << EXT_LID_WIDTH));
        end
        // Non-power-of-two depths are legal and expected; this line is not a
        // complaint, it is the only record in the log that THIS instance has holes
        // and that the out-of-range gate (g_rng_chk) is the thing standing between
        // a mis-echoed txnid and a drifting free count. Whoever reads a waveform
        // or a lint log later needs that sentence to exist.
        if (!PEND_POW2) begin
            $display({"NOTE %m: EXT_PENDING_TRANS=%0d is not a power of two -- LID",
                      " codes %0d..%0d name no entry; a Slave that echoes one is",
                      " caught by the out-of-range check on the response path, not",
                      " by the liveness check"},
                     EXT_PENDING_TRANS, EXT_PENDING_TRANS, ((1 << EXT_LID_WIDTH) - 1));
        end
        // Two entries is the smallest depth at which the two allocate ports can
        // ever fire together. At one entry the design still WORKS -- every cycle
        // is a contest and the token alternates -- but a bus configured that way
        // has serialised its read and write command paths, and that is worth
        // saying out loud rather than letting it look like a throughput bug.
        if (EXT_PENDING_TRANS < 2) begin
            $display({"NOTE %m: EXT_PENDING_TRANS=%0d -- read and write commands",
                      " are fully serialised at this depth"}, EXT_PENDING_TRANS);
        end
    end
    // synthesis translate_on
`endif

endmodule
