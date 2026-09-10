//============================================================================
// Filename    : lb_tniu_int_core.v
// Author      : litebus
// Description : TNIU internal half, core (network side, single clk domain)
// Date        : 2026-08-06
// Revision    : 1.0 initial -- per-round changes are in doc/HISTORY.md
//----------------------------------------------------------------------------
// Everything the TNIU internal half does EXCEPT crossing a clock domain:
//   - forward REQ_R and REQ_W, each: int_pipe -> credit_ingress -> ID compress
//     (its own cmd_table allocate port) -> ext_pipe -> border
//   - reverse RSP_RD/RSP_WR: border -> ext_pipe -> Resp ID gen (cmd_table free) ->
//     credit_egress -> int_pipe
// cmd_table alloc/free are in the clk domain, all in this half.
//
// THE TWO FORWARD CHAINS SHARE EXACTLY ONE THING: the cmd_table -- and since
// 2026-08-24 they share it as ONE POOL, not two. lb_tniu_cmd_table.v builds a
// single free set (free_all = ~valid); the read port takes the lowest free entry
// and the write port the highest, and BOTH alloc_*_ready hang off the same
// cnt_free. So reads and writes DO compete for table capacity.
//
// Be precise about which kind of independence survives, because the two get
// confused and the confusion costs real debugging time: neither ready below
// contains a COMBINATIONAL term from the other channel (they read registered
// state only, see lb_tniu_cmd_table.v's ready section), but the cnt_free they
// read is decremented by BOTH channels' allocates. Independent timing, shared
// capacity. A write burst that fills the table drives alloc_r_ready low:
//     ci_req_r_r = alloc_r_ready && ep_req_r_in_ready
//     ci_req_w_r = (is_first_beat ? alloc_w_ready : 1'b1) && ep_req_w_in_ready
// The is_first_beat guard on the write arm is NOT about the split -- it predates
// it. Gating later WD beats on alloc_w_ready would stall mid-burst whenever a
// burst took the last table entry.
//
// The two arms drive the table's alloc_*_req with their downstream readiness
// folded in ("would allocate if an id were free"). That is load bearing, but
// NOT for the reason this comment used to give: the background migration it
// described was measured useless under saturation and removed (the why is kept
// in lb_tniu_cmd_table.v's header). What the folded-in readiness feeds now is
// the LAST-ENTRY round robin -- when exactly one entry is free the table hands
// it to whichever arm the token points at, and an arm that keeps claiming ids
// it cannot actually consume would win that entry and then sit on it while
// stalled on its external port.
//
// The CMD-to-Slave conversion is one module, lb_tniu_cmd_conv, instantiated once
// per arm -- the beat-count arithmetic in it rests on a non-obvious identity and
// two textual copies of that is how the arms would drift apart.
//
// The border towards lb_tniu_ext_bca is a plain valid-ready interface (vr_*). See
// lb_tniu_int_bca.v for why the two forms are two MODULES rather than one module
// with a parameter or a `define.
//
// Parameter naming convention (CODING_STYLE 1.4): EXT_* are external (Slave-IP
// side) interface widths and knobs, INT_* are in-network (fabric-side) interface
// widths and the credit configuration; structural / topology parameters
// (WFRAG_DW) carry no prefix. No bca depth / synchronizer depth here -- this
// core has no bca.
//
// Two pipe flavours, one per interface style (same split as the INIU):
//   ext_pipe = lb_iniu_pipe      valid-ready skid buffer (bca / Slave side)
//   int_pipe = lb_iniu_int_pipe  credit pipeline register (fabric side)
// Eight enables in total -- four channels (REQ_R / REQ_W / RSP_RD / RSP_WR) x two
// interface styles (EXT_PIPE_* and INT_PIPE_*) -- configured independently.
//
// EXT_PIPE_RSP_RD / EXT_PIPE_RSP_WR still DEFAULT to 0, but that default is only safe in
// the bca form. It used to be justified by "the bca read side already registers
// the reverse path, so the stage is redundant" -- and that argument dies with the
// bca. In this core there is no bridge: with ext_pipe 0 the Slave's rsp_rd_valid /
// rsp_rd_data run combinationally through the cmd_table lookup, the fragmenter and
// credit_egress, with rsp_rd_ready coming back the same way. Every same-domain config
// therefore sets these to 1, and lb_ir._check_same_domain_pipes refuses to
// generate one that does not. The defaults stay at 0 so the bca form, which is
// what this module's parameter block is shared with, keeps its old behaviour.
//
// LID width: the local transaction id is the cmd_table entry index, so its width
// is DERIVED from EXT_PENDING_TRANS (EXT_LID_WIDTH below) and is never injected.
// It is zero-extended to EXT_TXNID_WIDTH on the Slave pins. Deriving it here is
// what keeps the table index and the table depth from ever disagreeing.
//============================================================================
`include "lb_defines.vh"

module lb_tniu_int_core #(
    // ==== external interface (Slave IP side, Valid-Ready) ====
    parameter EXT_DATA_WIDTH       = 64,  // Slave-side data width
    parameter EXT_LEN_WIDTH        = 4,   // Slave-side len width (beat count - 1)
    parameter EXT_USER_WIDTH_CMD   = 8,   // CMD user sideband width
    parameter EXT_USER_WIDTH_RSP_RD    = 8,   // RSP_RD user sideband width
    parameter EXT_USER_WIDTH_RSP_WR    = 8,   // RSP_WR user sideband width
    parameter EXT_TXNID_WIDTH      = 3,   // Slave-side external transaction ID width
    parameter EXT_MOD_W            = 0,   // REQ_W atomic-modifier pin width, 0 = none (Adv 15)
    parameter EXT_QOS_W            = 0,   // REQ qos pin width, 0 = no pin on this Slave
    parameter EXT_PENDING_TRANS    = 8,   // outstanding transactions (cmd_table depth)
    // 1 = Slave IP returns read data interleaved -> insert lb_tniu_rsp_rd_frag
    parameter EXT_RSP_RD_INTERLEAVE    = 0,
    // 1 = atomic support (Adv 15): ATOMIC_LOAD/SWAP/COMPARE slots collect both a
    // B and an R before their cmd_table entry frees; 0 elaborates today's logic
    parameter EXT_ATOMIC_EN        = 0,
    // ==== per-channel pipe enables (eight, independently configurable) ====
    parameter EXT_PIPE_REQ_R       = 1,   // ext_pipe (valid-ready skid) on forward REQ_R
    parameter EXT_PIPE_REQ_W       = 1,   // ext_pipe (valid-ready skid) on forward REQ_W
    parameter EXT_PIPE_RSP_RD          = 0,   // ext_pipe (valid-ready skid) on reverse RSP_RD
    parameter EXT_PIPE_RSP_WR          = 0,   // ext_pipe (valid-ready skid) on reverse RSP_WR
    parameter INT_PIPE_REQ_R       = 1,   // int_pipe (credit register) on forward REQ_R
    parameter INT_PIPE_REQ_W       = 1,   // int_pipe (credit register) on forward REQ_W
    parameter INT_PIPE_RSP_RD          = 1,   // int_pipe (credit register) on reverse RSP_RD
    parameter INT_PIPE_RSP_WR          = 1,   // int_pipe (credit register) on reverse RSP_WR
    // ==== credit configuration of the internal interface ====
    parameter INT_REQ_R_CREDIT_FIFO = 4,  // forward REQ_R ingress FIFO depth = upstream credit budget
    parameter INT_REQ_W_CREDIT_FIFO = 4,  // forward REQ_W ingress FIFO depth = upstream credit budget
    parameter INT_RSP_RD_CREDIT        = 4,   // reverse RSP_RD egress credit = peer ingress FIFO depth
    parameter INT_RSP_WR_CREDIT        = 4,   // reverse RSP_WR egress credit = peer ingress FIFO depth
    // ==== internal interface (fabric side, credit) ====
    parameter INT_ADDR_LOCAL_WIDTH = 32,  // in-network local (rebased) address width
    parameter INT_DEST_ID_WIDTH    = 4,   // = log2(NUM_SLAVES), injected by top
    parameter INT_SRC_ID_WIDTH     = 4,   // = log2(NUM_MASTERS), injected by top
    parameter INT_ID_WIDTH         = 8,   // in-network unified int_id width
    // One in-network user width PER CHANNEL, not one for the bus -- see the INIU
    // side. Each must be >= the matching EXT_USER_WIDTH_* (checked below).
    parameter INT_USER_WIDTH_CMD   = 8,   // in-network CMD user width (>= EXT_USER_WIDTH_CMD)
    parameter INT_USER_WIDTH_RSP_RD    = 8,   // in-network RSP_RD  user width (>= EXT_USER_WIDTH_RSP_RD)
    parameter INT_USER_WIDTH_RSP_WR    = 8,   // in-network RSP_WR  user width (>= EXT_USER_WIDTH_RSP_WR)
    parameter INT_TOTBYTES_W       = 16,  // in-network unified total_bytes width
    parameter INT_LANE_W           = 7,   // addr_lane width (carried in the RSP_RD flit)
    parameter INT_MOD_W            = 0,   // REQ_W modifier segment width (bus max), 0 = absent (Adv 15)
    parameter INT_QOS_W            = 0,   // QoS segment width (bus max), 0 = absent; tops the CMD flit
    // ==== structural / topology ====
    parameter WFRAG_DW             = EXT_DATA_WIDTH,  // W_frag: max data_width among Masters on this Slave
    // ---- derived parameters; used in the port list, so they must stay in the
    // ---- parameter list (V2001 body localparams cannot size ports). Not to be
    // ---- overridden externally -- the defaults are the correct values.
    parameter EXT_STRB_WIDTH = EXT_DATA_WIDTH/8,
    parameter EXT_BYTES_PER_BEAT_LOG =    // = log2(SLV bytes per beat)
        (EXT_DATA_WIDTH/8 <=   1) ? 0 : (EXT_DATA_WIDTH/8 <=   2) ? 1 :
        (EXT_DATA_WIDTH/8 <=   4) ? 2 : (EXT_DATA_WIDTH/8 <=   8) ? 3 :
        (EXT_DATA_WIDTH/8 <=  16) ? 4 : (EXT_DATA_WIDTH/8 <=  32) ? 5 :
        (EXT_DATA_WIDTH/8 <=  64) ? 6 : (EXT_DATA_WIDTH/8 <= 128) ? 7 : 8,
    parameter INT_CMD_FLIT_W = INT_QOS_W + `LB_OPCODE_WIDTH + INT_ADDR_LOCAL_WIDTH + INT_TOTBYTES_W +
                               INT_USER_WIDTH_CMD + INT_ID_WIDTH + INT_DEST_ID_WIDTH + INT_SRC_ID_WIDTH,
    parameter INT_WD_FLIT_W  = 1 + EXT_STRB_WIDTH + EXT_DATA_WIDTH +
                               INT_DEST_ID_WIDTH + INT_SRC_ID_WIDTH,
    // REQ_W channel = CMD field + MOD segment (Adv 15, absent at INT_MOD_W = 0)
    // + WD field concatenated (symmetric to INIU). REQ_R carries the CMD field
    // alone, so its flit width IS INT_CMD_FLIT_W.
    parameter INT_REQ_FLIT_W = INT_CMD_FLIT_W + INT_MOD_W + INT_WD_FLIT_W,
    // RSP_RD flit (low->high): data, last, trans_last, user, resp, int_id,
    //                      total_bytes, addr_lane, qos, src_id
    parameter INT_RSP_RD_FLIT_W  = INT_SRC_ID_WIDTH + INT_QOS_W + INT_LANE_W + INT_TOTBYTES_W + INT_ID_WIDTH +
                               `LB_RESP_WIDTH + INT_USER_WIDTH_RSP_RD + 1 + 1 + EXT_DATA_WIDTH,
    // RSP_WR flit (low->high): user, resp, int_id, qos, src_id
    parameter INT_RSP_WR_FLIT_W  = INT_SRC_ID_WIDTH + INT_QOS_W + INT_ID_WIDTH +
                               `LB_RESP_WIDTH + INT_USER_WIDTH_RSP_WR,
    // external CMD payload leads with this Slave's own qos slice (EXT_QOS_W,
    // often 0); the response payloads never carry qos at the IP border.
    parameter EXT_CMD_W  = EXT_QOS_W + `LB_OPCODE_WIDTH + INT_ADDR_LOCAL_WIDTH + EXT_LEN_WIDTH +
                           EXT_TXNID_WIDTH + EXT_USER_WIDTH_CMD,
    parameter EXT_WD_W   = EXT_DATA_WIDTH + EXT_STRB_WIDTH + 1 + EXT_TXNID_WIDTH,
    // external REQ = {cmd field (high) | modifier (Adv 15, often absent) | wd field (low)}
    parameter EXT_REQ_W  = EXT_CMD_W + EXT_MOD_W + EXT_WD_W,
    parameter EXT_RSP_RD_W   = EXT_TXNID_WIDTH + `LB_RESP_WIDTH + EXT_USER_WIDTH_RSP_RD + 1 + EXT_DATA_WIDTH,
    parameter EXT_RSP_WR_W   = EXT_TXNID_WIDTH + `LB_RESP_WIDTH + EXT_USER_WIDTH_RSP_WR
) (
    //--------- inputs: clock / reset (network clock domain) ---------
    input wire                        clk,                 // network-side clock
    input wire                        rst_n,               // async reset, active low
    //--------- inputs: internal interface (fabric side, packed flit + credit) ---------
    input wire  [INT_CMD_FLIT_W-1:0]  i_req_r_flit,        // REQ_R flit in (CMD only)
    input wire                        i_req_r_valid,       // REQ_R flit valid
    input wire  [INT_REQ_FLIT_W-1:0]  i_req_w_flit,        // REQ_W flit in (CMD + WD)
    input wire                        i_req_w_valid,       // REQ_W flit valid
    input wire                        o_rsp_rd_credit_ret, // RSP_RD credit returned by the peer
    input wire                        o_rsp_wr_credit_ret, // RSP_WR credit returned by the peer
    //--------- outputs: internal interface (fabric side, packed flit + credit) ---------
    output wire                       i_req_r_credit_ret,  // REQ_R credit returned to the peer
    output wire                       i_req_w_credit_ret,  // REQ_W credit returned to the peer
    output wire [INT_RSP_RD_FLIT_W-1:0]   o_rsp_rd_flit,   // RSP_RD flit out
    output wire                       o_rsp_rd_valid,      // RSP_RD flit valid
    output wire [INT_RSP_WR_FLIT_W-1:0]   o_rsp_wr_flit,   // RSP_WR flit out
    output wire                       o_rsp_wr_valid,      // RSP_WR flit valid
    //--------- inputs: direct valid-ready border (towards lb_tniu_ext_bca) ---------
    input wire                        vr_req_r_ready,      // REQ_R ready from the external half
    input wire                        vr_req_w_ready,      // REQ_W ready from the external half
    input wire  [EXT_RSP_RD_W-1:0]        vr_rsp_rd_data,  // RSP_RD  packed payload from the external half
    input wire                        vr_rsp_rd_valid,     // RSP_RD  valid
    input wire  [EXT_RSP_WR_W-1:0]        vr_rsp_wr_data,  // RSP_WR  packed payload from the external half
    input wire                        vr_rsp_wr_valid,     // RSP_WR  valid
    //--------- outputs: direct valid-ready border (towards lb_tniu_ext_bca) ---------
    output wire [EXT_CMD_W-1:0]       vr_req_r_data,       // REQ_R packed payload to the external half
    output wire                       vr_req_r_valid,      // REQ_R valid
    output wire [EXT_REQ_W-1:0]       vr_req_w_data,       // REQ_W packed payload to the external half
    output wire                       vr_req_w_valid,      // REQ_W valid
    output wire                       vr_rsp_rd_ready,     // RSP_RD  ready to the external half
    output wire                       vr_rsp_wr_ready      // RSP_WR  ready to the external half
);
    //------------------------------------------------------------------------
    // Derived local params
    //------------------------------------------------------------------------
    // LID = cmd_table entry index, so its width follows the table depth. Kept a
    // localparam (not a parameter) so no top can inject a value that disagrees
    // with EXT_PENDING_TRANS.
    // Runs to 16 because ext_txnid_width does (config/schema.yaml) and the table
    // depth is now 2**ext_txnid_width with no cap. A ladder that stops early does
    // not fail loudly -- it silently under-widths an index; see the sibling ladder
    // in lb_tniu_rsp_rd_frag.v, which stopped at 5 and scrambled LIDs above depth 32.
    localparam EXT_LID_WIDTH = (EXT_PENDING_TRANS <= 2)     ? 1 :  // = clog2(EXT_PENDING_TRANS)
                               (EXT_PENDING_TRANS <= 4)     ? 2 :
                               (EXT_PENDING_TRANS <= 8)     ? 3 :
                               (EXT_PENDING_TRANS <= 16)    ? 4 :
                               (EXT_PENDING_TRANS <= 32)    ? 5 :
                               (EXT_PENDING_TRANS <= 64)    ? 6 :
                               (EXT_PENDING_TRANS <= 128)   ? 7 :
                               (EXT_PENDING_TRANS <= 256)   ? 8 :
                               (EXT_PENDING_TRANS <= 512)   ? 9 :
                               (EXT_PENDING_TRANS <= 1024)  ? 10 :
                               (EXT_PENDING_TRANS <= 2048)  ? 11 :
                               (EXT_PENDING_TRANS <= 4096)  ? 12 :
                               (EXT_PENDING_TRANS <= 8192)  ? 13 :
                               (EXT_PENDING_TRANS <= 16384) ? 14 :
                               (EXT_PENDING_TRANS <= 32768) ? 15 : 16;
    // qos joins the transaction-level sideband (it is per-transaction context,
    // NOT per-beat): {qos, int_id, src_id}, qos on top so the two taps below
    // reduce to their historical expressions at INT_QOS_W = 0.
    localparam RSP_RD_FSB_W = INT_QOS_W + INT_ID_WIDTH + INT_SRC_ID_WIDTH;  // transaction-level sideband
    localparam RSP_RD_BSB_W = `LB_RESP_WIDTH + EXT_USER_WIDTH_RSP_RD;   // beat-level sideband
    // user widths floored at 1 for the wires that must exist in BOTH arms of a
    // generate (they are unpacked in one place and consumed in another, too far
    // apart to live inside one block). At width 0 they carry a constant zero that
    // no elaborated arm reads. RSP_RD_BSB_W above needs no floor: `LB_RESP_WIDTH is 2,
    // so the beat-level sideband is never empty.
    localparam EXT_USER_RSP_RD_PW  = (EXT_USER_WIDTH_RSP_RD  < 1) ? 1 : EXT_USER_WIDTH_RSP_RD;
    localparam EXT_USER_RSP_WR_PW  = (EXT_USER_WIDTH_RSP_WR  < 1) ? 1 : EXT_USER_WIDTH_RSP_WR;
    localparam INT_USER_CMD_PW = (INT_USER_WIDTH_CMD < 1) ? 1 : INT_USER_WIDTH_CMD;
    localparam INT_QOS_PW      = (INT_QOS_W < 1) ? 1 : INT_QOS_W;
    // CMD payload the shared cmd_conv packs -- everything but the qos head.
    // The full EXT_CMD_W payload is assembled outside it (g_rq_r_eqos /
    // g_cmd_eqos), so cmd_conv stays untouched by QoS.
    localparam EXT_CMDB_W      = EXT_CMD_W - EXT_QOS_W;

    //------------------------------------------------------------------------
    // Declarations (all up front, one per line)
    //------------------------------------------------------------------------
    // (A) REQ_R arm: int_pipe -> credit_ingress -> cmd_conv -> ext_pipe
    wire [INT_CMD_FLIT_W-1:0]       ip_rq_r_data;        // int-pipe side REQ_R flit
    wire                            ip_rq_r_v;           // int-pipe side REQ_R valid
    wire                            cig_rq_r_crd;        // credit-ingress REQ_R credit return
    wire [INT_CMD_FLIT_W-1:0]       ci_rq_r_data;        // credit-ingress output REQ_R flit (a CMD flit)
    wire                            ci_rq_r_v;           // credit-ingress output REQ_R valid
    wire                            ci_rq_r_r;           // credit-ingress output REQ_R ready (backpressure)
    // rc_* = REQ_R CMD fields. Deliberately not r_*: in this file r_* already
    // means "field of an RSP_RD response" (r_txnid / r_resp / r_data ...), and reusing
    // that prefix for a forward-path field is how the two would be confused.
    wire [`LB_OPCODE_WIDTH-1:0]     rc_opcode;           // REQ_R CMD opcode field
    wire [INT_ADDR_LOCAL_WIDTH-1:0] rc_addr;             // REQ_R CMD local address field
    wire [INT_TOTBYTES_W-1:0]       rc_total_bytes;      // REQ_R CMD total_bytes field
    wire [INT_USER_CMD_PW-1:0]      rc_user_i;           // REQ_R CMD user field (in-network width)
    wire [INT_ID_WIDTH-1:0]         rc_int_id;           // REQ_R CMD int_id field
    wire [INT_DEST_ID_WIDTH-1:0]    rc_dest;             // REQ_R CMD dest_id field (unused here)
    wire [INT_SRC_ID_WIDTH-1:0]     rc_src;              // REQ_R CMD src_id field
    wire [EXT_BYTES_PER_BEAT_LOG-1:0] rc_addr_lo;        // REQ_R offset within one SLV beat (unused here)
    wire [EXT_CMDB_W-1:0]           rc_ext_pk;           // REQ_R Slave-side CMD payload, minus the qos head
    wire [EXT_CMD_W-1:0]            rc_ext_full;         // REQ_R Slave-side CMD payload, qos head included
    wire [INT_QOS_PW-1:0]           rc_qos_i;            // REQ_R qos field (stub at INT_QOS_W=0)
    wire [EXT_TXNID_WIDTH-1:0]      rc_txnid;            // REQ_R LID zero-extended to the Slave txnid width
    wire                            ep_rq_r_in_ready;    // REQ_R ext-pipe input ready (backpressure)
    wire                            ep_rq_r_in_v;        // REQ_R ext-pipe input valid
    wire                            ep_rq_r_v;           // REQ_R ext-pipe output valid
    wire                            ep_rq_r_r;           // REQ_R ext-pipe output ready (backpressure)
    // (A) REQ_W arm: int_pipe -> credit_ingress -> split CMD/WD -> cmd_conv -> ext_pipe
    wire [INT_REQ_FLIT_W-1:0]       ip_req_data;         // int-pipe side REQ_W flit
    wire                            ip_req_v;            // int-pipe side REQ_W valid
    wire                            cig_req_crd;         // credit-ingress REQ_W credit return
    wire [INT_REQ_FLIT_W-1:0]       ci_req_data;         // credit-ingress output REQ_W flit
    wire                            ci_req_v;            // credit-ingress output REQ_W valid
    wire                            ci_req_r;            // credit-ingress output REQ_W ready (backpressure)
    wire [INT_CMD_FLIT_W-1:0]       ci_cmd_data;         // CMD field of the REQ_W flit
    wire [INT_WD_FLIT_W-1:0]        ci_wd_data;          // WD field of the REQ_W flit
    // (B) REQ_W CMD flit fields (in-network widths)
    wire [`LB_OPCODE_WIDTH-1:0]     c_opcode;            // CMD opcode field
    wire [INT_ADDR_LOCAL_WIDTH-1:0] c_addr;              // CMD local address field
    wire [INT_TOTBYTES_W-1:0]       c_total_bytes;       // CMD total_bytes field
    wire [INT_USER_CMD_PW-1:0]      c_user_i;            // CMD user field (in-network width)
    wire [INT_ID_WIDTH-1:0]         c_int_id;            // CMD int_id field
    wire [INT_DEST_ID_WIDTH-1:0]    c_dest;              // CMD dest_id field
    wire [INT_SRC_ID_WIDTH-1:0]     c_src;               // CMD src_id field
    // (B) local transaction ID (LID) allocation, one port per request channel
    wire                            alloc_r_ready;       // the read pool has a free entry
    wire [EXT_LID_WIDTH-1:0]        alloc_r_id;          // LID offered to REQ_R
    wire                            alloc_r_req;         // REQ_R would allocate if an id were free
    wire                            alloc_r_fire;        // REQ_R LID allocation fire
    wire                            alloc_w_ready;       // the write pool has a free entry
    wire [EXT_LID_WIDTH-1:0]        alloc_w_id;          // LID offered to REQ_W
    wire                            alloc_w_req;         // REQ_W would allocate if an id were free
    wire                            alloc_w_fire;        // REQ_W LID allocation fire
    wire                            w_is_atm2;           // this REQ_W is a two-response atomic (Adv 15)
    wire [INT_LANE_W-1:0]           alloc_w_lane;        // addr_lane stored at a write allocation
    reg  [7:0]                      wd_beat_idx;         // WD beat index within the burst (0 = first)
    reg  [EXT_LID_WIDTH-1:0]        burst_lid;           // latched first-beat LID, reused by later beats
    wire                            is_first_beat;       // wd_beat_idx == 0
    wire [EXT_LID_WIDTH-1:0]        use_lid;             // LID actually driven this beat
    wire [EXT_TXNID_WIDTH-1:0]      use_txnid;           // use_lid zero-extended to the Slave txnid width
    wire [EXT_CMDB_W-1:0]           cmd_ext_pk;          // REQ_W Slave-side CMD payload, minus the qos head
    wire [EXT_CMD_W-1:0]            cmd_ext_full;        // REQ_W Slave-side CMD payload, qos head included
    wire [INT_QOS_PW-1:0]           c_qos_i;             // REQ_W CMD qos field (stub at INT_QOS_W=0)
    // (B) WD flit fields and write-channel lane alignment
    wire                            w_last;              // WD last beat
    wire [EXT_STRB_WIDTH-1:0]       w_strb;              // WD write strobe (in-network lane order)
    wire [EXT_DATA_WIDTH-1:0]       w_data;              // WD write data (in-network lane order)
    wire [INT_DEST_ID_WIDTH-1:0]    w_dest;              // WD dest_id field (unused here)
    wire [INT_SRC_ID_WIDTH-1:0]     w_src;               // WD src_id field (unused here)
    wire [EXT_BYTES_PER_BEAT_LOG-1:0] wr_addr_lo;        // write address offset within one SLV beat
    wire                            wd_fire;             // WD handshake fire
    wire [EXT_DATA_WIDTH-1:0]       w_data_aligned;      // write data aligned to Slave byte lanes
    wire [EXT_STRB_WIDTH-1:0]       w_strb_slv;          // write strobe regenerated for the Slave
    wire [EXT_WD_W-1:0]             wd_ext_pk;           // Slave-side WD packed payload
    wire                            ep_req_in_ready;     // REQ_W ext-pipe input ready (backpressure)
    // (C) ext_pipe -> bca_slv
    wire [EXT_REQ_W-1:0]            ep_req_in;           // ext-pipe input REQ_W packed payload
    wire                            ep_req_in_v;         // ext-pipe input REQ_W valid
    wire                            ep_req_v;            // ext-pipe output REQ_W valid
    wire                            ep_req_r;            // ext-pipe output REQ_W ready (backpressure)
    // (D) reverse bca_mst outputs
    wire [EXT_RSP_RD_W-1:0]             cr_rsp_rd_pk;    // clock-crossing read side RSP_RD packed payload
    wire                            cr_rsp_rd_v;         // clock-crossing read side RSP_RD valid
    wire                            cr_rsp_rd_r;         // clock-crossing read side RSP_RD ready
    wire [EXT_RSP_WR_W-1:0]             cr_rsp_wr_pk;    // clock-crossing read side RSP_WR packed payload
    wire                            cr_rsp_wr_v;         // clock-crossing read side RSP_WR valid
    wire                            cr_rsp_wr_r;         // clock-crossing read side RSP_WR ready
    // (D) reverse ext_pipe outputs (EXT_PIPE_RSP_RD / EXT_PIPE_RSP_WR)
    wire [EXT_RSP_RD_W-1:0]             rp_rsp_rd_pk;    // reverse ext-pipe output RSP_RD packed payload
    wire                            rp_rsp_rd_v;         // reverse ext-pipe output RSP_RD valid
    wire                            rp_rsp_rd_r;         // reverse ext-pipe output RSP_RD ready
    wire [EXT_RSP_WR_W-1:0]             rp_rsp_wr_pk;    // reverse ext-pipe output RSP_WR packed payload
    wire                            rp_rsp_wr_v;         // reverse ext-pipe output RSP_WR valid
    wire                            rp_rsp_wr_r;         // reverse ext-pipe output RSP_WR ready
    // (D) reverse response fields and cmd_table free ports
    wire [EXT_TXNID_WIDTH-1:0]      r_txnid;             // RSP_RD response txnid as returned by the Slave
    wire [EXT_LID_WIDTH-1:0]        r_lid;               // RSP_RD response LID (low bits of the Slave txnid)
    wire [`LB_RESP_WIDTH-1:0]       r_resp;              // RSP_RD response code
    wire [EXT_USER_RSP_RD_PW-1:0]       r_user;          // RSP_RD user sideband (zero at width 0)
    wire                            r_last;              // RSP_RD last beat
    wire [EXT_DATA_WIDTH-1:0]       r_data;              // RSP_RD data
    wire [EXT_TXNID_WIDTH-1:0]      b_txnid;             // RSP_WR response txnid as returned by the Slave
    wire [EXT_LID_WIDTH-1:0]        b_lid;               // RSP_WR response LID (low bits of the Slave txnid)
    wire [`LB_RESP_WIDTH-1:0]       b_resp;              // RSP_WR response code
    wire [EXT_USER_RSP_WR_PW-1:0]       b_user;          // RSP_WR user sideband (zero at width 0)
    wire                            rsp_rd_fire;         // RSP_RD handshake fire
    wire                            rsp_wr_fire;         // RSP_WR handshake fire
    wire                            rsp_rd_free;         // release the cmd_table entry (read)
    wire                            rsp_wr_free;         // release the cmd_table entry (write)
    wire [INT_ID_WIDTH-1:0]         free_int_id;         // int_id read back for the RSP_RD response
    wire [INT_SRC_ID_WIDTH-1:0]     free_src_id;         // src_id read back for the RSP_RD response
    wire [INT_LANE_W-1:0]           free_lane;           // addr_lane read back for the RSP_RD response
    wire [INT_TOTBYTES_W-1:0]       free_total_bytes;    // total_bytes read back for the RSP_RD response
    wire [INT_ID_WIDTH-1:0]         rsp_wr_int_id;       // int_id read back for the RSP_WR response
    wire [INT_SRC_ID_WIDTH-1:0]     rsp_wr_src_id;       // src_id read back for the RSP_WR response
    wire [INT_LANE_W-1:0]           alloc_lane;          // addr_lane (phase) stored at allocation
    // (E) read fragmenter interface
    wire [EXT_DATA_WIDTH-1:0]       fr_data;             // fragmenter output data
    wire [RSP_RD_BSB_W-1:0]             fr_bsb;          // fragmenter output beat-level sideband
    wire [RSP_RD_FSB_W-1:0]             fr_fsb;          // fragmenter output transaction-level sideband
    wire [INT_LANE_W-1:0]           fr_lane;             // fragmenter output addr_lane
    wire [INT_TOTBYTES_W-1:0]       fr_totb;             // fragmenter output total_bytes
    wire                            fr_last;             // fragmenter output fragment last beat
    wire                            fr_tlast;            // fragmenter output transaction last beat
    wire                            fr_valid;            // fragmenter output valid
    wire                            fr_ready;            // fragmenter output ready (backpressure)
    wire [RSP_RD_BSB_W-1:0]             rsp_rd_bsb_in;   // beat-level sideband into the fragmenter
    wire [RSP_RD_FSB_W-1:0]             rsp_rd_fsb_in;   // transaction-level sideband into the fragmenter
    wire [`LB_RESP_WIDTH-1:0]       fr_resp;             // resp unpacked from fr_bsb
    wire [INT_ID_WIDTH-1:0]         fr_iid;              // int_id unpacked from fr_fsb
    wire [INT_SRC_ID_WIDTH-1:0]     fr_src;              // src_id unpacked from fr_fsb
    wire [INT_QOS_PW-1:0]           free_qos;            // qos restored by the read lookup (stub at 0)
    wire [INT_QOS_PW-1:0]           b_qos;               // qos restored by the write lookup (stub at 0)
    wire [INT_SRC_ID_WIDTH+INT_QOS_W-1:0] fr_hi;         // {src_id, qos} top segment of the RSP_RD flit
    wire [INT_SRC_ID_WIDTH+INT_QOS_W-1:0] b_hi;          // {src_id, qos} top segment of the RSP_WR flit
    // (F) reverse flit assembly and credit egress
    wire [INT_RSP_RD_FLIT_W-1:0]        rsp_rd_flit_in;  // packed RSP_RD flit
    wire [INT_RSP_WR_FLIT_W-1:0]        rsp_wr_flit_in;  // packed RSP_WR flit
    wire [INT_RSP_RD_FLIT_W-1:0]        ceg_rsp_rd_data; // credit-egress output RSP_RD flit
    wire                            ceg_rsp_rd_v;        // credit-egress output RSP_RD valid
    wire                            ceg_rsp_rd_crd;      // credit-egress RSP_RD credit return
    wire [INT_RSP_WR_FLIT_W-1:0]        ceg_rsp_wr_data; // credit-egress output RSP_WR flit
    wire                            ceg_rsp_wr_v;        // credit-egress output RSP_WR valid
    wire                            ceg_rsp_wr_crd;      // credit-egress RSP_WR credit return

    //========================================================================
    // (A1) forward REQ_R: int_pipe -> credit_ingress -> cmd_conv -> ext_pipe.
    // The flit IS a CMD flit, so there is nothing to split and no WD arm.
    //========================================================================
    lb_iniu_int_pipe #(
        .WIDTH             (INT_CMD_FLIT_W),
        .ENABLE            (INT_PIPE_REQ_R)
    ) u10_ipipe_req_r (
        .clk               (clk),
        .rst_n             (rst_n),
        .in_data           (i_req_r_flit),
        .in_valid          (i_req_r_valid),
        .out_credit_return (cig_rq_r_crd),
        .in_credit_return  (i_req_r_credit_ret),
        .out_data          (ip_rq_r_data),
        .out_valid         (ip_rq_r_v)
    );

    lb_credit_ingress #(
        .WIDTH         (INT_CMD_FLIT_W),
        .DEPTH         (INT_REQ_R_CREDIT_FIFO)
    ) u20_cig_req_r (
        .clk           (clk),
        .rst_n         (rst_n),
        .in_data       (ip_rq_r_data),
        .in_valid      (ip_rq_r_v),
        .out_ready     (ci_rq_r_r),
        .out_data      (ci_rq_r_data),
        .out_valid     (ci_rq_r_v),
        .credit_return (cig_rq_r_crd)
    );

    // Keyed on INT: the flit layout is a property of the BUS, not of this Slave. A
    // bus on which no NIU carries a CMD user has no such field in its CMD flit, so
    // the unpack drops it and the pin-width stub goes to zero. A Slave that merely
    // opts out itself still sees the field here and discards it in lb_tniu_cmd_conv.
    // The qos field tops the CMD flit, so it is sliced positionally here and the
    // two user arms below keep reading the EXPLICIT low part -- at INT_QOS_W = 0
    // that slice is the whole flit and everything reads as before.
    generate
    if (INT_QOS_W > 0) begin : g_rq_r_qos
        assign rc_qos_i = ci_rq_r_data[INT_CMD_FLIT_W-1 -: INT_QOS_W];
    end
    else begin : g_rq_r_qos0
        assign rc_qos_i = 1'b0;
    end
    endgenerate

    generate
    if (INT_USER_WIDTH_CMD == 0) begin : g_rq_r_nouser
        assign {rc_opcode, rc_addr, rc_total_bytes, rc_int_id, rc_dest, rc_src} =
            ci_rq_r_data[INT_CMD_FLIT_W-INT_QOS_W-1:0];
        assign rc_user_i = {INT_USER_CMD_PW{1'b0}};
    end
    else begin : g_rq_r_user
        assign {rc_opcode, rc_addr, rc_total_bytes, rc_user_i, rc_int_id, rc_dest, rc_src} =
            ci_rq_r_data[INT_CMD_FLIT_W-INT_QOS_W-1:0];
    end
    endgenerate

    // A read is a single beat, so it is always its own first beat: it takes
    // alloc_r_id directly, with no burst_lid to latch and no beat index.
    assign rc_txnid = alloc_r_id;

    lb_tniu_cmd_conv #(
        .EXT_DATA_WIDTH       (EXT_DATA_WIDTH),
        .EXT_LEN_WIDTH        (EXT_LEN_WIDTH),
        .EXT_TXNID_WIDTH      (EXT_TXNID_WIDTH),
        .EXT_USER_WIDTH_CMD   (EXT_USER_WIDTH_CMD),
        .INT_ADDR_LOCAL_WIDTH (INT_ADDR_LOCAL_WIDTH),
        .INT_USER_WIDTH_CMD   (INT_USER_WIDTH_CMD),
        .INT_TOTBYTES_W       (INT_TOTBYTES_W)
    ) u30_cmd_conv_r (
        .c_opcode      (rc_opcode),
        .c_addr        (rc_addr),
        .c_total_bytes (rc_total_bytes),
        .c_user_i      (rc_user_i),
        .use_txnid     (rc_txnid),
        .addr_lo       (rc_addr_lo),
        .cmd_ext_pk    (rc_ext_pk)
    );

    // Slave-side payload leads with this Slave's OWN qos pin slice: a low-bit
    // truncation of the in-network field, the exact inverse of the INIU's
    // low-aligned zero-extension (same convention as the MOD segment). A Slave
    // with qos_width 0 ships the historical payload even on a QoS bus.
    generate
    if (EXT_QOS_W > 0) begin : g_rq_r_eqos
        assign rc_ext_full = { rc_qos_i[EXT_QOS_W-1:0], rc_ext_pk };
    end
    else begin : g_rq_r_eqos0
        assign rc_ext_full = rc_ext_pk;
    end
    endgenerate

    // No cross-channel term: this arm advances when the read pool has an id and
    // its own ext_pipe has room.
    assign alloc_r_req  = ci_rq_r_v && ep_rq_r_in_ready;
    assign ci_rq_r_r    = alloc_r_ready && ep_rq_r_in_ready;
    assign alloc_r_fire = ci_rq_r_v && ci_rq_r_r;
    assign ep_rq_r_in_v = ci_rq_r_v && alloc_r_ready;

    lb_iniu_pipe #(
        .WIDTH     (EXT_CMD_W),
        .ENABLE    (EXT_PIPE_REQ_R)
    ) u40_epipe_req_r (
        .clk       (clk),
        .rst_n     (rst_n),
        .in_data   (rc_ext_full),
        .in_valid  (ep_rq_r_in_v),
        .out_ready (ep_rq_r_r),
        .in_ready  (ep_rq_r_in_ready),
        .out_data  (vr_req_r_data),
        .out_valid (ep_rq_r_v)
    );

    assign vr_req_r_valid = ep_rq_r_v;
    assign ep_rq_r_r      = vr_req_r_ready;

    //========================================================================
    // (A2) forward REQ_W: int_pipe -> credit_ingress -> split CMD/WD
    //========================================================================
    lb_iniu_int_pipe #(
        .WIDTH             (INT_REQ_FLIT_W),
        .ENABLE            (INT_PIPE_REQ_W)
    ) u50_ipipe_req_w (
        .clk               (clk),
        .rst_n             (rst_n),
        .in_data           (i_req_w_flit),
        .in_valid          (i_req_w_valid),
        .out_credit_return (cig_req_crd),
        .in_credit_return  (i_req_w_credit_ret),
        .out_data          (ip_req_data),
        .out_valid         (ip_req_v)
    );

    lb_credit_ingress #(
        .WIDTH         (INT_REQ_FLIT_W),
        .DEPTH         (INT_REQ_W_CREDIT_FIFO)
    ) u60_cig_req_w (
        .clk           (clk),
        .rst_n         (rst_n),
        .in_data       (ip_req_data),
        .in_valid      (ip_req_v),
        .out_ready     (ci_req_r),
        .out_data      (ci_req_data),
        .out_valid     (ci_req_v),
        .credit_return (cig_req_crd)
    );

    // split REQ_W: high = CMD field, low = WD field
    assign ci_cmd_data = ci_req_data[INT_REQ_FLIT_W-1 -: INT_CMD_FLIT_W];
    assign ci_wd_data  = ci_req_data[0 +: INT_WD_FLIT_W];

    //========================================================================
    // (B) ID compress: allocate a local transaction ID (CMD flit is in-network width)
    //========================================================================
    // Same shapes as the REQ_R unpack above, on the CMD half of REQ_W: the qos
    // head is sliced positionally, the user arms read the explicit low part.
    generate
    if (INT_QOS_W > 0) begin : g_cmd_qos
        assign c_qos_i = ci_cmd_data[INT_CMD_FLIT_W-1 -: INT_QOS_W];
    end
    else begin : g_cmd_qos0
        assign c_qos_i = 1'b0;
    end
    endgenerate

    generate
    if (INT_USER_WIDTH_CMD == 0) begin : g_cmd_nouser
        assign {c_opcode, c_addr, c_total_bytes, c_int_id, c_dest, c_src} =
            ci_cmd_data[INT_CMD_FLIT_W-INT_QOS_W-1:0];
        assign c_user_i = {INT_USER_CMD_PW{1'b0}};
    end
    else begin : g_cmd_user
        assign {c_opcode, c_addr, c_total_bytes, c_user_i, c_int_id, c_dest, c_src} =
            ci_cmd_data[INT_CMD_FLIT_W-INT_QOS_W-1:0];
    end
    endgenerate

    // LID actually used: the first beat takes alloc_w_id, later beats reuse the
    // latched value (the Slave txnid is constant over a burst).
    assign is_first_beat = (wd_beat_idx == 8'd0);
    assign use_lid       = is_first_beat ? alloc_w_id : burst_lid;
    // zero-extend to the Slave txnid width (EXT_LID_WIDTH <= EXT_TXNID_WIDTH);
    // a plain assignment does the extension without a zero-width replication.
    assign use_txnid     = use_lid;

    // Slave-side CMD payload + beat count, in the module both arms share. It also
    // hands back addr_lo, which lb_tniu_lane_pack below needs: taking the slice
    // again here would be a second copy of the same statement.
    lb_tniu_cmd_conv #(
        .EXT_DATA_WIDTH       (EXT_DATA_WIDTH),
        .EXT_LEN_WIDTH        (EXT_LEN_WIDTH),
        .EXT_TXNID_WIDTH      (EXT_TXNID_WIDTH),
        .EXT_USER_WIDTH_CMD   (EXT_USER_WIDTH_CMD),
        .INT_ADDR_LOCAL_WIDTH (INT_ADDR_LOCAL_WIDTH),
        .INT_USER_WIDTH_CMD   (INT_USER_WIDTH_CMD),
        .INT_TOTBYTES_W       (INT_TOTBYTES_W)
    ) u70_cmd_conv_w (
        .c_opcode      (c_opcode),
        .c_addr        (c_addr),
        .c_total_bytes (c_total_bytes),
        .c_user_i      (c_user_i),
        .use_txnid     (use_txnid),
        .addr_lo       (wr_addr_lo),
        .cmd_ext_pk    (cmd_ext_pk)
    );

    assign {w_last, w_strb, w_data, w_dest, w_src} = ci_wd_data;

    assign wd_fire = ci_req_v && ci_req_r;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wd_beat_idx <= 8'd0;
        end
        else if (wd_fire) begin
            wd_beat_idx <= w_last ? 8'd0 : (wd_beat_idx + 8'd1);
        end
    end

    lb_tniu_lane_pack #(
        .EXT_DATA_WIDTH        (EXT_DATA_WIDTH),
        .INT_TOTBYTES_W        (INT_TOTBYTES_W),
        .EXT_BYTES_PER_BEAT_LOG(EXT_BYTES_PER_BEAT_LOG),
        .BEAT_IDX_W                (8)
    ) u80_lane_pack (
        .addr_lo               (wr_addr_lo),
        .total_bytes           (c_total_bytes),
        .beat_idx              (wd_beat_idx),
        .in_data               (w_data),
        .in_strb               (w_strb),
        .out_data              (w_data_aligned),
        .out_strb              (w_strb_slv)
    );

    assign wd_ext_pk = {w_data_aligned, w_strb_slv, w_last, use_txnid};

    // The opcode-driven write detect is gone: every flit on this arm is a write,
    // so CMD and WD are always both real and the beat is one stream with one
    // ready. What survives is the FIRST-BEAT rule -- only the first beat of a
    // burst needs a free table entry, and gating later WD beats on alloc_w_ready
    // would stall mid-burst whenever that burst took the last entry.
    //
    // Neither expression below mentions the read channel, and that is a statement
    // about TIMING, not about capacity: alloc_w_ready is registered, so no read
    // signal appears in this combinational path. It is NOT a property of a
    // "write pool" -- there is one shared pool (lb_tniu_cmd_table.v), so a read
    // burst that fills the table will drive alloc_w_ready low too.
    assign alloc_w_req  = ci_req_v && is_first_beat && ep_req_in_ready;
    assign ci_req_r     = (is_first_beat ? alloc_w_ready : 1'b1) && ep_req_in_ready;
    assign alloc_w_fire = ci_req_v && ci_req_r && is_first_beat;

    // Two-response atomics (Adv 15): LOAD/SWAP/COMPARE occupy the high quad of
    // the write family (4'hD/E/F -- bit3 and bit2 set with a nonzero low pair),
    // and their table entry must collect BOTH the B and the R before it frees.
    // ATOMIC_STORE (4'hC, low pair 00) deliberately misses this decode: it frees
    // on its B alone, exactly like a plain write. The reserved multicast /
    // reduction codes (4'hA/4'hB) have bit2 clear and also miss it. Constant 0
    // on a non-atomic TNIU, where the config guarantees no atomic ever arrives.
    assign w_is_atm2   = (EXT_ATOMIC_EN != 0)
                       && c_opcode[3] && c_opcode[2] && (c_opcode[1] || c_opcode[0]);
    // The R of an atomic replays addr_lane / total_bytes out of the table, so a
    // write allocation now stores them too -- mirroring the read arm's
    // alloc_lane below (the table only writes them when alloc_w_atm holds).
    assign alloc_w_lane = c_addr[INT_LANE_W-1:0];

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            burst_lid <= {EXT_LID_WIDTH{1'b0}};
        end
        else if (alloc_w_fire) begin
            burst_lid <= alloc_w_id;
        end
    end

    // The wd_beat_idx counter lives next to wd_fire / lane_pack above. A second,
    // byte-identical always block for it used to sit HERE -- a leftover of the
    // REQ_R/REQ_W split. Both blocks computed the same value every cycle, so
    // simulation resolved deterministically and every functional case stayed
    // green; DC's first run refused it as a multi-driven net (ELAB-366).
    // lint_strict P4 now guards this shape.

    //========================================================================
    // (C) REQ_W ext_pipe -> border: cmd/wd combined into one REQ_W beat. There is
    //     no read case to zero the WD half for any more.
    //
    // The modifier (Adv 15) is carved out of the in-network MOD segment and
    // truncated to this Slave's own pin width (EXT_MOD_W <= INT_MOD_W, checked
    // below; the widths need not be equal -- dropping high bits is the configured
    // semantics). A Slave with no modifier pin ships the historical payload even
    // on a bus whose other Slaves carry one.
    //========================================================================
    // The qos head takes the same shape as on the REQ_R arm (g_rq_r_eqos).
    generate
    if (EXT_QOS_W > 0) begin : g_cmd_eqos
        assign cmd_ext_full = { c_qos_i[EXT_QOS_W-1:0], cmd_ext_pk };
    end
    else begin : g_cmd_eqos0
        assign cmd_ext_full = cmd_ext_pk;
    end
    endgenerate

    generate
    if (EXT_MOD_W == 0) begin : g_ep_nomod
        assign ep_req_in = { cmd_ext_full, wd_ext_pk };
    end
    else begin : g_ep_mod
        wire [INT_MOD_W-1:0] ci_mod;  // MOD segment of the REQ_W flit
        wire [EXT_MOD_W-1:0] ext_mod; // truncated to this Slave's pin width
        assign ci_mod    = ci_req_data[INT_WD_FLIT_W +: INT_MOD_W];
        assign ext_mod   = ci_mod[EXT_MOD_W-1:0];
        assign ep_req_in = { cmd_ext_full, ext_mod, wd_ext_pk };
    end
    endgenerate
    assign ep_req_in_v = ci_req_v && (is_first_beat ? alloc_w_ready : 1'b1);

    lb_iniu_pipe #(
        .WIDTH     (EXT_REQ_W),
        .ENABLE    (EXT_PIPE_REQ_W)
    ) u90_epipe_req_w (
        .clk       (clk),
        .rst_n     (rst_n),
        .in_data   (ep_req_in),
        .in_valid  (ep_req_in_v),
        .out_ready (ep_req_r),
        .in_ready  (ep_req_in_ready),
        .out_data  (vr_req_w_data),
        .out_valid (ep_req_v)
    );

    // forward REQ_W leaves straight onto the valid-ready border
    assign vr_req_w_valid = ep_req_v;
    assign ep_req_r       = vr_req_w_ready;

    //========================================================================
    // (D) reverse: border -> ext_pipe -> Resp ID gen -> credit_egress -> int_pipe
    //========================================================================
    // reverse RSP_RD/RSP_WR arrive straight off the valid-ready border
    assign cr_rsp_rd_pk    = vr_rsp_rd_data;
    assign cr_rsp_rd_v     = vr_rsp_rd_valid;
    assign vr_rsp_rd_ready = cr_rsp_rd_r;
    assign cr_rsp_wr_pk    = vr_rsp_wr_data;
    assign cr_rsp_wr_v     = vr_rsp_wr_valid;
    assign vr_rsp_wr_ready = cr_rsp_wr_r;

    // Reverse ext_pipe: optional valid-ready skid between the bca read side and
    // the cmd_table lookup / fragmenter. Sits before the unpack so the whole
    // packed payload is registered as one vector.
    lb_iniu_pipe #(
        .WIDTH     (EXT_RSP_RD_W),
        .ENABLE    (EXT_PIPE_RSP_RD)
    ) u91_epipe_rsp_rd (
        .clk       (clk),
        .rst_n     (rst_n),
        .in_data   (cr_rsp_rd_pk),
        .in_valid  (cr_rsp_rd_v),
        .out_ready (rp_rsp_rd_r),
        .in_ready  (cr_rsp_rd_r),
        .out_data  (rp_rsp_rd_pk),
        .out_valid (rp_rsp_rd_v)
    );

    lb_iniu_pipe #(
        .WIDTH     (EXT_RSP_WR_W),
        .ENABLE    (EXT_PIPE_RSP_WR)
    ) u92_epipe_rsp_wr (
        .clk       (clk),
        .rst_n     (rst_n),
        .in_data   (cr_rsp_wr_pk),
        .in_valid  (cr_rsp_wr_v),
        .out_ready (rp_rsp_wr_r),
        .in_ready  (cr_rsp_wr_r),
        .out_data  (rp_rsp_wr_pk),
        .out_valid (rp_rsp_wr_v)
    );

    // Keyed on EXT: these come off the Slave's own pins, so it is this Slave's
    // width that decides whether the payload has a user member at all (EXT_RSP_RD_W /
    // EXT_RSP_WR_W drop the addend by themselves). The stub is driven to zero so the
    // consumers further down need no second condition.
    generate
    if (EXT_USER_WIDTH_RSP_RD == 0) begin : g_rp_rsp_rd_nouser
        assign {r_txnid, r_resp, r_last, r_data} = rp_rsp_rd_pk;
        assign r_user = {EXT_USER_RSP_RD_PW{1'b0}};
    end
    else begin : g_rp_rsp_rd_user
        assign {r_txnid, r_resp, r_user, r_last, r_data} = rp_rsp_rd_pk;
    end
    endgenerate

    generate
    if (EXT_USER_WIDTH_RSP_WR == 0) begin : g_rp_rsp_wr_nouser
        assign {b_txnid, b_resp} = rp_rsp_wr_pk;
        assign b_user = {EXT_USER_RSP_WR_PW{1'b0}};
    end
    else begin : g_rp_rsp_wr_user
        assign {b_txnid, b_resp, b_user} = rp_rsp_wr_pk;
    end
    endgenerate
    // The Slave returns the txnid verbatim; the LID is its low EXT_LID_WIDTH bits
    // (the table index). Upper bits are the zero padding added by use_txnid.
    assign r_lid = r_txnid[EXT_LID_WIDTH-1:0];
    assign b_lid = b_txnid[EXT_LID_WIDTH-1:0];

    assign rsp_rd_fire = rp_rsp_rd_v && rp_rsp_rd_r;
    assign rsp_wr_fire = rp_rsp_wr_v && rp_rsp_wr_r;
    // read and write free independently: a read frees on its last beat (burst
    // end), a write frees on its single response beat.
    assign rsp_rd_free = rsp_rd_fire && r_last;
    assign rsp_wr_free = rsp_wr_fire;
    // forward: take addr_lane (phase) from the REQ_R CMD address low bits and
    // store it in cmd_table; reverse: read it back into the RSP_RD flit. The read
    // arm always stores it; the write arm stores it only for a two-response
    // atomic (Adv 15), whose R replays it -- a plain write-allocated entry still
    // never has it looked up.
    assign alloc_lane = rc_addr[INT_LANE_W-1:0];

    lb_tniu_cmd_table #(
        .EXT_PENDING_TRANS  (EXT_PENDING_TRANS),
        .EXT_LID_WIDTH      (EXT_LID_WIDTH),
        .INT_ID_WIDTH       (INT_ID_WIDTH),
        .INT_SRC_ID_WIDTH   (INT_SRC_ID_WIDTH),
        .INT_LANE_W         (INT_LANE_W),
        .INT_TOTBYTES_W     (INT_TOTBYTES_W),
        .INT_QOS_W          (INT_QOS_W),
        .ATOMIC_EN          (EXT_ATOMIC_EN)
    ) u100_cmd_table (
        .clk                 (clk),
        .rst_n               (rst_n),
        // read allocate port, driven by the REQ_R arm
        .alloc_r_req         (alloc_r_req),
        .alloc_r_fire        (alloc_r_fire),
        .alloc_r_int_id      (rc_int_id),
        .alloc_r_src_id      (rc_src),
        .alloc_r_addr_lane   (alloc_lane),
        .alloc_r_total_bytes (rc_total_bytes),
        .alloc_r_qos         (rc_qos_i),
        // write allocate port, driven by the REQ_W arm
        .alloc_w_req         (alloc_w_req),
        .alloc_w_fire        (alloc_w_fire),
        .alloc_w_int_id      (c_int_id),
        .alloc_w_src_id      (c_src),
        .alloc_w_atm         (w_is_atm2),
        .alloc_w_addr_lane   (alloc_w_lane),
        .alloc_w_total_bytes (c_total_bytes),
        .alloc_w_qos         (c_qos_i),
        // read lookup port: keyed by the read-response LID (r_lid)
        .rsp_rd_lid              (r_lid),
        .rsp_rd_free             (rsp_rd_free),
        // write lookup port: keyed by the write-response LID (b_lid)
        .rsp_wr_lid              (b_lid),
        .rsp_wr_free             (rsp_wr_free),
        .alloc_r_ready       (alloc_r_ready),
        .alloc_r_id          (alloc_r_id),
        .alloc_w_ready       (alloc_w_ready),
        .alloc_w_id          (alloc_w_id),
        .rsp_rd_int_id           (free_int_id),
        .rsp_rd_src_id           (free_src_id),
        .rsp_rd_addr_lane        (free_lane),
        .rsp_rd_total_bytes      (free_total_bytes),
        .rsp_rd_qos              (free_qos),
        .rsp_wr_int_id           (rsp_wr_int_id),
        .rsp_wr_src_id           (rsp_wr_src_id),
        .rsp_wr_qos              (b_qos)
    );

    //========================================================================
    // (E) Read interleaving: beat level -> transaction level fragmentation.
    //   EXT_RSP_RD_INTERLEAVE=0: feed-through (the Slave guarantees the beats of one
    //                    transaction come back contiguously). Zero latency, zero area.
    //   EXT_RSP_RD_INTERLEAVE=1: insert lb_tniu_rsp_rd_frag, which regroups the interleaved
    //                    beat stream into transaction-level fragments so the
    //                    Switch burst-lock logic needs no change.
    //   See arch.html section 6.2.3.
    // byte_valid is not carried: the receiver rebuilds the byte mask from
    //   (addr_lane, total_bytes, beat index), bit-identical to what this side
    //   would produce (DV invariant I5). The RSP_RD flit instead carries total_bytes
    //   (a per-burst constant) plus trans_last.
    //========================================================================
    // RSP_RD_BSB_W already drops the user addend at width 0, so the sideband is just
    // the resp then -- never empty, because `LB_RESP_WIDTH is 2. That is why
    // lb_tniu_rsp_rd_frag needs no zero-width handling of its own.
    generate
    if (EXT_USER_WIDTH_RSP_RD == 0) begin : g_bsb_nouser
        assign rsp_rd_bsb_in = r_resp;
    end
    else begin : g_bsb_user
        assign rsp_rd_bsb_in = { r_resp, r_user };
    end
    endgenerate
    // qos is per-TRANSACTION context, so it rides the fsb (never the bsb): the
    // interleaving fragmenter replays it per fragment out of the same latch
    // that replays int_id / src_id. Two arms because a 1-bit stub must not be
    // concatenated in on a QoS-free bus.
    generate
    if (INT_QOS_W > 0) begin : g_fsb_qos
        assign rsp_rd_fsb_in = { free_qos, free_int_id, free_src_id };
    end
    else begin : g_fsb_noqos
        assign rsp_rd_fsb_in = { free_int_id, free_src_id };
    end
    endgenerate

    generate
    if (EXT_RSP_RD_INTERLEAVE) begin : g_ilv
        lb_tniu_rsp_rd_frag #(
            .EXT_DATA_WIDTH    (EXT_DATA_WIDTH),
            .WFRAG_DW          (WFRAG_DW),
            .EXT_PENDING_TRANS (EXT_PENDING_TRANS),
            .EXT_LID_WIDTH     (EXT_LID_WIDTH),
            .INT_LANE_W        (INT_LANE_W),
            .INT_TOTBYTES_W    (INT_TOTBYTES_W),
            .FSB_W             (RSP_RD_FSB_W),
            .BSB_W             (RSP_RD_BSB_W)
        ) u110_rsp_rd_frag (
            .clk               (clk),
            .rst_n             (rst_n),
            .i_ctx_addr_lo     (free_lane),
            .i_ctx_totb        (free_total_bytes),
            .i_ctx_fsb         (rsp_rd_fsb_in),
            .i_valid           (rp_rsp_rd_v),
            .i_lid             (r_lid),
            .i_data            (r_data),
            .i_bsb             (rsp_rd_bsb_in),
            .o_ready           (fr_ready),
            .i_ready           (rp_rsp_rd_r),
            .o_valid           (fr_valid),
            .o_data            (fr_data),
            .o_bsb             (fr_bsb),
            .o_lid             (),
            .o_fsb             (fr_fsb),
            .o_addr_lo         (fr_lane),
            .o_total_bytes     (fr_totb),
            .o_last            (fr_last),
            .o_trans_last      (fr_tlast)
        );
    end else begin : g_noilv
        assign fr_data  = r_data;
        assign fr_bsb   = rsp_rd_bsb_in;
        assign fr_fsb   = rsp_rd_fsb_in;
        assign fr_lane  = free_lane;
        assign fr_totb  = free_total_bytes;
        assign fr_last  = r_last;
        assign fr_tlast = r_last;      // non-interleaving: fragment last == transaction last
        assign fr_valid = rp_rsp_rd_v;
        assign rp_rsp_rd_r  = fr_ready;
    end
    endgenerate

    //========================================================================
    // (F) reverse flit assembly -> credit_egress -> int_pipe
    //========================================================================
    assign fr_resp  = fr_bsb[RSP_RD_BSB_W-1 -: `LB_RESP_WIDTH];
    // int_id sits INT_QOS_W below the fsb top (qos is the head); the expression
    // reduces to the historical tap at INT_QOS_W = 0.
    assign fr_iid   = fr_fsb[RSP_RD_FSB_W-1-INT_QOS_W -: INT_ID_WIDTH];
    assign fr_src   = fr_fsb[0 +: INT_SRC_ID_WIDTH];

    // {src_id, qos} top segment of both response flits: merging the two lets
    // the three user arms below stay three (qos would otherwise double them).
    generate
    if (INT_QOS_W > 0) begin : g_rsp_hi
        assign fr_hi = { fr_src, fr_fsb[RSP_RD_FSB_W-1 -: INT_QOS_W] };
        assign b_hi  = { rsp_wr_src_id, b_qos };
    end
    else begin : g_rsp_hi0
        assign fr_hi = fr_src;
        assign b_hi  = rsp_wr_src_id;
    end
    endgenerate
    // RSP_RD flit (low->high): data, last, trans_last, user, resp, int_id,
    //                      total_bytes, addr_lane, src_id
    // src_id sits at the top for reverse routing back to the source.
    //
    // The user field takes the same three shapes as the MOD segment does on REQ_W:
    //   INT = 0        no user field on this bus; the flit is one field shorter
    //   EXT = 0 < INT  this Slave has no user pin, so its slot is driven to zero
    //   otherwise      the pin value, zero-extended to the field width
    // Only the third arm slices or repeats: INT is the bus-wide maximum over every
    // Master AND Slave, so EXT > 0 implies INT >= EXT > 0.
    generate
    if (INT_USER_WIDTH_RSP_RD == 0) begin : g_rsp_rdf_nouser
        assign rsp_rd_flit_in = { fr_hi, fr_lane, fr_totb, fr_iid, fr_resp,
                              fr_tlast, fr_last, fr_data };
    end
    else if (EXT_USER_WIDTH_RSP_RD == 0) begin : g_rsp_rdf_userz
        assign rsp_rd_flit_in = { fr_hi, fr_lane, fr_totb, fr_iid, fr_resp,
                              {INT_USER_WIDTH_RSP_RD{1'b0}},
                              fr_tlast, fr_last, fr_data };
    end
    else begin : g_rsp_rdf_user
        wire [EXT_USER_WIDTH_RSP_RD-1:0] fr_user;  // user unpacked from fr_bsb
        wire [INT_USER_WIDTH_RSP_RD-1:0] r_user_i; // user zero-extended to in-network width
        assign fr_user    = fr_bsb[0 +: EXT_USER_WIDTH_RSP_RD];
        assign r_user_i   = {{(INT_USER_WIDTH_RSP_RD-EXT_USER_WIDTH_RSP_RD){1'b0}}, fr_user};
        assign rsp_rd_flit_in = { fr_hi, fr_lane, fr_totb, fr_iid, fr_resp, r_user_i,
                              fr_tlast, fr_last, fr_data };
    end
    endgenerate

    generate
    if (INT_USER_WIDTH_RSP_WR == 0) begin : g_rsp_wrf_nouser
        assign rsp_wr_flit_in = { b_hi, rsp_wr_int_id, b_resp };
    end
    else if (EXT_USER_WIDTH_RSP_WR == 0) begin : g_rsp_wrf_userz
        assign rsp_wr_flit_in = { b_hi, rsp_wr_int_id, b_resp,
                              {INT_USER_WIDTH_RSP_WR{1'b0}} };
    end
    else begin : g_rsp_wrf_user
        wire [INT_USER_WIDTH_RSP_WR-1:0] b_user_i; // user zero-extended to in-network width
        assign b_user_i   = {{(INT_USER_WIDTH_RSP_WR-EXT_USER_WIDTH_RSP_WR){1'b0}}, b_user};
        assign rsp_wr_flit_in = { b_hi, rsp_wr_int_id, b_resp, b_user_i };
    end
    endgenerate

    lb_credit_egress #(
        .WIDTH         (INT_RSP_RD_FLIT_W),
        .CREDIT_INIT   (INT_RSP_RD_CREDIT)
    ) u120_ceg_rsp_rd (
        .clk           (clk),
        .rst_n         (rst_n),
        .in_data       (rsp_rd_flit_in),
        .in_valid      (fr_valid),
        .credit_return (ceg_rsp_rd_crd),
        .in_ready      (fr_ready),
        .out_data      (ceg_rsp_rd_data),
        .out_valid     (ceg_rsp_rd_v)
    );

    lb_credit_egress #(
        .WIDTH         (INT_RSP_WR_FLIT_W),
        .CREDIT_INIT   (INT_RSP_WR_CREDIT)
    ) u121_ceg_rsp_wr (
        .clk           (clk),
        .rst_n         (rst_n),
        .in_data       (rsp_wr_flit_in),
        .in_valid      (rp_rsp_wr_v),
        .credit_return (ceg_rsp_wr_crd),
        .in_ready      (rp_rsp_wr_r),
        .out_data      (ceg_rsp_wr_data),
        .out_valid     (ceg_rsp_wr_v)
    );

    lb_iniu_int_pipe #(
        .WIDTH             (INT_RSP_RD_FLIT_W),
        .ENABLE            (INT_PIPE_RSP_RD)
    ) u130_ipipe_rsp_rd (
        .clk               (clk),
        .rst_n             (rst_n),
        .in_data           (ceg_rsp_rd_data),
        .in_valid          (ceg_rsp_rd_v),
        .out_credit_return (o_rsp_rd_credit_ret),
        .in_credit_return  (ceg_rsp_rd_crd),
        .out_data          (o_rsp_rd_flit),
        .out_valid         (o_rsp_rd_valid)
    );

    lb_iniu_int_pipe #(
        .WIDTH             (INT_RSP_WR_FLIT_W),
        .ENABLE            (INT_PIPE_RSP_WR)
    ) u131_ipipe_rsp_wr (
        .clk               (clk),
        .rst_n             (rst_n),
        .in_data           (ceg_rsp_wr_data),
        .in_valid          (ceg_rsp_wr_v),
        .out_credit_return (o_rsp_wr_credit_ret),
        .in_credit_return  (ceg_rsp_wr_crd),
        .out_data          (o_rsp_wr_flit),
        .out_valid         (o_rsp_wr_valid)
    );

    //------------------------------------------------------------------------
    // Simulation-only integration checks. Not synthesized (LB_NO_ASSERT can
    // switch them off for lint-clean builds).
    //------------------------------------------------------------------------
`ifndef LB_NO_ASSERT
    // synthesis translate_off
    initial begin
        if (EXT_LID_WIDTH > EXT_TXNID_WIDTH) begin
            // Fix by raising Slaves.ext_txnid_width or lowering EXT_PENDING_TRANS.
            $display("ERROR %m: %0d entries need a %0d-bit LID, txnid is %0d bits",
                     EXT_PENDING_TRANS, EXT_LID_WIDTH, EXT_TXNID_WIDTH);
        end
        // CMD was the one channel with no check, and it is the one that TRUNCATES
        // (c_user_i[EXT_USER_WIDTH_CMD-1:0] below): too narrow an in-network width
        // makes that an out-of-range part-select. RSP_RD/RSP_WR zero-extend instead, where
        // the same violation is a negative repeat count.
        if (INT_USER_WIDTH_CMD < EXT_USER_WIDTH_CMD) begin
            $display("ERROR %m: INT_USER_WIDTH_CMD=%0d < EXT_USER_WIDTH_CMD=%0d",
                     INT_USER_WIDTH_CMD, EXT_USER_WIDTH_CMD);
        end
        if (INT_USER_WIDTH_RSP_RD < EXT_USER_WIDTH_RSP_RD) begin
            $display("ERROR %m: INT_USER_WIDTH_RSP_RD=%0d < EXT_USER_WIDTH_RSP_RD=%0d",
                     INT_USER_WIDTH_RSP_RD, EXT_USER_WIDTH_RSP_RD);
        end
        if (INT_USER_WIDTH_RSP_WR < EXT_USER_WIDTH_RSP_WR) begin
            $display("ERROR %m: INT_USER_WIDTH_RSP_WR=%0d < EXT_USER_WIDTH_RSP_WR=%0d",
                     INT_USER_WIDTH_RSP_WR, EXT_USER_WIDTH_RSP_WR);
        end
        // The QoS segment is the bus-wide maximum too, so this Slave's pin slice
        // rc_qos_i[EXT_QOS_W-1:0] must be a genuine truncation (same argument as
        // the MOD check below).
        if (INT_QOS_W < EXT_QOS_W) begin
            $display("ERROR %m: INT_QOS_W=%0d < EXT_QOS_W=%0d",
                     INT_QOS_W, EXT_QOS_W);
        end
        // The atomic-modifier segment (Adv 15) is the bus-wide maximum, so this
        // Slave's pin slice ci_mod[EXT_MOD_W-1:0] must be a genuine truncation.
        if (INT_MOD_W < EXT_MOD_W) begin
            $display("ERROR %m: INT_MOD_W=%0d < EXT_MOD_W=%0d",
                     INT_MOD_W, EXT_MOD_W);
        end
        // A two-response atomic needs the dual-response cmd_table; the generator
        // only routes atomics to atomic-capable Slaves (lb_ir._check_atomic), so
        // a modifier pin on a non-atomic TNIU is a wiring-by-hand mistake.
        if (EXT_ATOMIC_EN == 0 && EXT_MOD_W != 0) begin
            $display("ERROR %m: EXT_MOD_W=%0d but EXT_ATOMIC_EN=0", EXT_MOD_W);
        end
    end

    //------------------------------------------------------------------------
    // A flit must arrive on the request channel that matches its opcode.
    //
    // On this side that means the FABRIC is wired wrong: a REQ_W flit reached a
    // REQ_R switch, or the reverse. The INIU has the same pair of checks at the
    // source; this one catches the case where the two channels' graphs got crossed
    // between there and here, which the INIU cannot see.
    //
    // Both failures are silent otherwise. A write arriving on REQ_R is handed a
    // read LID, forwarded to the Slave with no data, and answered on RSP_RD -- the
    // Master's write never completes. A read arriving on REQ_W takes a write LID
    // and is answered on RSP_WR, so the Master waits forever for read data.
    //
    // Atomics do NOT weaken either check: all four atomic opcodes are write
    // family (bit3 = 1) and ride REQ_W. What HAS changed is the reply-channel
    // reasoning above -- a two-response atomic is the one legal case of a REQ_W
    // transaction being answered on RSP_RD (as well as RSP_WR), with its write-pool LID
    // riding the rsp_rd_txnid pins. The reverse path handles that by storing
    // addr_lane / total_bytes at the write allocation (alloc_w_atm) and by the
    // cmd_table freeing such an entry only after both responses arrived.
    //------------------------------------------------------------------------
    always @(posedge clk) begin
        if (rst_n && ci_rq_r_v && rc_opcode[`LB_OPCODE_WR_BIT]) begin
            $display("ERROR %m: opcode %0h on REQ_R has the write bit set",
                     rc_opcode);
        end
        if (rst_n && ci_req_v && !c_opcode[`LB_OPCODE_WR_BIT]) begin
            $display("ERROR %m: opcode %0h on REQ_W has the write bit clear",
                     c_opcode);
        end
        // An atomic opcode must never reach a TNIU whose table cannot hold a
        // two-response slot -- the entry would free on whichever response lands
        // first and the other would restore stale context. The generator refuses
        // such a config (lb_ir._check_atomic); this catches hand-wired setups.
        if (rst_n && ci_req_v && (EXT_ATOMIC_EN == 0)
                  && c_opcode[3] && c_opcode[2] && (c_opcode[1] || c_opcode[0])) begin
            $display("ERROR %m: atomic opcode %0h but EXT_ATOMIC_EN=0", c_opcode);
        end
        // The Slave-side len must hold ceil((addr_lo + total_bytes) / SW) beats.
        // lb_tniu_cmd_conv computes exactly that and then keeps only the low
        // EXT_LEN_WIDTH bits, so a burst one beat too long goes out as len=0 with
        // 2^EXT_LEN_WIDTH+1 beats of data -- silently. It happens when a NARROWER
        // Master's maxburst equals this Slave's and the footprint is not aligned
        // to the Slave beat (2026-08-27: 4 of 218 write bursts on bus_16x16d_atomic,
        // found with a bound probe after five days of red VIP cases). The same
        // arithmetic is re-derived here on purpose rather than read out of the
        // cmd_conv instance: a check that reuses the value it checks proves nothing.
        // total_bytes is zero-extended by 16 bits so the sum itself cannot wrap --
        // the point of this check is to catch a truncation, not to perform one.
        if (rst_n && alloc_w_fire && (c_total_bytes != {INT_TOTBYTES_W{1'b0}})
                  && ((({{16{1'b0}}, c_total_bytes} + c_addr[EXT_BYTES_PER_BEAT_LOG-1:0] - 1)
                        >> EXT_BYTES_PER_BEAT_LOG) >= (1 << EXT_LEN_WIDTH))) begin
            $display("ERROR %m: REQ_W total_bytes=%0d at offset %0d needs > %0d beats; len truncates",
                     c_total_bytes, c_addr[EXT_BYTES_PER_BEAT_LOG-1:0], (1 << EXT_LEN_WIDTH));
        end
        if (rst_n && alloc_r_fire && (rc_total_bytes != {INT_TOTBYTES_W{1'b0}})
                  && ((({{16{1'b0}}, rc_total_bytes} + rc_addr[EXT_BYTES_PER_BEAT_LOG-1:0] - 1)
                        >> EXT_BYTES_PER_BEAT_LOG) >= (1 << EXT_LEN_WIDTH))) begin
            $display("ERROR %m: REQ_R total_bytes=%0d at offset %0d needs > %0d beats; len truncates",
                     rc_total_bytes, rc_addr[EXT_BYTES_PER_BEAT_LOG-1:0], (1 << EXT_LEN_WIDTH));
        end
        // The echoed LID must name an entry that exists. EXT_PENDING_TRANS has
        // granularity 1, so codes EXT_PENDING_TRANS .. 2**EXT_LID_WIDTH-1 name
        // nothing; a Slave that returns one is violating the echo-back contract.
        // Checked HERE because this is the single point where the returned txnid
        // enters the design: the same r_lid feeds the cmd_table lookup AND
        // lb_tniu_rsp_rd_frag's i_lid. Deliberately no in-band recovery on either
        // side -- every "safe" reaction (drop the beat, stall, clamp the index)
        // trades one loud X for a silent corruption: clamping would poison a GOOD
        // LID's assembly state, dropping would desynchronise the burst. The
        // cmd_table declines to release (its own range gate), which costs one slot
        // and keeps the free count honest; nothing here can repair a broken Slave.
        if (rst_n && rsp_rd_fire && (r_lid >= EXT_PENDING_TRANS)) begin
            $display({"ERROR %m: Slave echoed rsp_rd txnid %0h -> LID %0d, outside",
                      " the %0d entries this TNIU issues; cmd_table will not release",
                      " and lb_tniu_rsp_rd_frag indexes its per-LID state out of range"},
                     r_txnid, r_lid, EXT_PENDING_TRANS);
        end
        if (rst_n && rsp_wr_fire && (b_lid >= EXT_PENDING_TRANS)) begin
            $display({"ERROR %m: Slave echoed rsp_wr txnid %0h -> LID %0d, outside",
                      " the %0d entries this TNIU issues; cmd_table will not release"},
                     b_txnid, b_lid, EXT_PENDING_TRANS);
        end
    end
    // synthesis translate_on
`endif

endmodule
