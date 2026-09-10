//============================================================================
// Filename    : lb_iniu_int_core.v
// Author      : litebus
// Description : INIU internal half, core (network side, single clk domain)
// Date        : 2026-08-06
// Revision    : 1.0 initial -- per-round changes are in doc/HISTORY.md
//----------------------------------------------------------------------------
// Everything the INIU internal half does EXCEPT crossing a clock domain:
//   - ext_pipe / id_decode / credit_egress / int_pipe  (forward)
//   - int_pipe / credit_ingress / ext_pipe             (reverse)
//
// The border towards the external half is a plain valid-ready interface. See
// lb_iniu_int_bca.v for why the two forms are two MODULES rather than one module
// with a parameter or a `define.
//
// Two distinct pipe flavours are used, one per interface style (see the INIU SPEC
// section on sub-module design):
//   ext_pipe = lb_iniu_pipe      valid-ready skid buffer, for Valid-Ready
//                                interfaces (bca / IP side)
//   int_pipe = lb_iniu_int_pipe  plain pipeline register pair (payload+valid
//                                forward, credit_return reverse), for the
//                                credit-based fabric interface -- no ready, so
//                                no skid is possible or needed
// Each of the four channels REQ_R / REQ_W / RSP_RD / RSP_WR has its own enable
// (EXT_PIPE_{REQ_R,REQ_W,RSP_RD,RSP_WR} and INT_PIPE_{REQ_R,REQ_W,RSP_RD,RSP_WR}), so they are
// configured independently.
//
// THE FORWARD PATH IS TWO FULLY PARALLEL CHAINS. REQ_R carries read requests
// (the CMD flit alone) and REQ_W carries write requests (CMD + WD on one beat).
// Each has its own ext_pipe, its own lb_iniu_id_decode, its own credit_egress
// and its own int_pipe, and the two never meet -- no arbiter, no opcode
// demux, no shared ready. That is the whole point: a read request cannot be
// queued behind write data anywhere between the IP pins and the fabric.
//
// The cost is a second id_decode per INIU, i.e. the memory-map comparators
// duplicated. Sharing one and arbitrating would put the two channels back on a
// common resource, which is exactly what this split removes.
//
// Width symbols are named after the PAYLOAD, so the split introduces none:
//     REQ_R  ext EXT_CMD_W        int INT_CMD_FLIT_W
//     REQ_W  ext EXT_REQ_W        int INT_REQ_FLIT_W   (both unchanged in value)
//
// Parameter naming convention: EXT_* are external (IP-side) interface widths,
// INT_* are in-network (fabric-side) interface widths and the credit/pipe
// configuration of that interface. NIU_ID / REGION_* are structural. No bca
// depth / synchronizer depth here -- this core has no bca.
//============================================================================
`include "lb_defines.vh"

module lb_iniu_int_core #(
    // ==== external (IP-side) interface widths ====
    parameter EXT_ADDR_WIDTH       = 32,  // external global address width
    parameter EXT_DATA_WIDTH       = 64,  // external (MST-side) data width
    parameter EXT_LEN_WIDTH        = 4,   // external len width (beat count - 1)
    parameter EXT_TXNID_WIDTH      = 8,   // external transaction ID width
    parameter EXT_USER_WIDTH_CMD   = 8,   // external CMD user sideband width
    parameter EXT_USER_WIDTH_RSP_RD    = 8,   // external RSP_RD user sideband width
    parameter EXT_USER_WIDTH_RSP_WR    = 8,   // external RSP_WR user sideband width
    parameter EXT_MOD_W            = 0,   // REQ_W atomic-modifier pin width, 0 = none (Adv 15)
    parameter EXT_QOS_W            = 0,   // REQ qos pin width, 0 = no pin on this Master
    // ==== in-network (fabric-side) interface widths ====
    // Top injects the global maxima; narrower externals are zero-extended.
    parameter INT_ADDR_LOCAL_WIDTH = 32,  // in-network local (rebased) address width
    parameter INT_DEST_ID_WIDTH    = 4,   // = log2(NUM_SLAVES), injected by top
    parameter INT_SRC_ID_WIDTH     = 4,   // = log2(NUM_MASTERS), injected by top
    parameter INT_ID_WIDTH         = 8,   // in-network unified int_id width (>= EXT_TXNID_WIDTH)
    // One in-network user width PER CHANNEL, not one for the bus. The three are
    // independent maxima over Masters+Slaves, so a wide rsp_rd_user_width no longer
    // fattens the CMD and RSP_WR flits as collateral. Each must be >= the matching
    // EXT_USER_WIDTH_* (checked below); mixing them up misplaces every field
    // above user in that flit.
    parameter INT_USER_WIDTH_CMD   = 8,   // in-network CMD user width (>= EXT_USER_WIDTH_CMD)
    parameter INT_USER_WIDTH_RSP_RD    = 8,   // in-network RSP_RD  user width (>= EXT_USER_WIDTH_RSP_RD)
    parameter INT_USER_WIDTH_RSP_WR    = 8,   // in-network RSP_WR  user width (>= EXT_USER_WIDTH_RSP_WR)
    parameter INT_TOTBYTES_W       = 16,  // in-network unified total_bytes width
    parameter INT_LANE_W           = 7,   // addr_lane width (carried in the RSP_RD flit)
    parameter INT_MOD_W            = 0,   // REQ_W modifier segment width (bus max), 0 = absent (Adv 15)
    parameter INT_QOS_W            = 0,   // QoS segment width (bus max), 0 = absent; tops the CMD flit
    // ==== per-channel pipe enables (four channels, independently configurable) ====
    parameter EXT_PIPE_REQ_R       = 1,   // ext_pipe (valid-ready skid) on forward REQ_R
    parameter EXT_PIPE_REQ_W       = 1,   // ext_pipe (valid-ready skid) on forward REQ_W
    parameter EXT_PIPE_RSP_RD          = 1,   // ext_pipe (valid-ready skid) on reverse RSP_RD
    parameter EXT_PIPE_RSP_WR          = 1,   // ext_pipe (valid-ready skid) on reverse RSP_WR
    parameter INT_PIPE_REQ_R       = 1,   // int_pipe (credit register) on forward REQ_R
    parameter INT_PIPE_REQ_W       = 1,   // int_pipe (credit register) on forward REQ_W
    parameter INT_PIPE_RSP_RD          = 1,   // int_pipe (credit register) on reverse RSP_RD
    parameter INT_PIPE_RSP_WR          = 1,   // int_pipe (credit register) on reverse RSP_WR
    // ==== credit configuration of the internal interface ====
    parameter INT_REQ_R_CREDIT     = 4,   // forward REQ_R egress credit = peer ingress FIFO depth
    parameter INT_REQ_W_CREDIT     = 4,   // forward REQ_W egress credit = peer ingress FIFO depth
    parameter INT_RSP_RD_CREDIT_FIFO   = 4,   // reverse RSP_RD ingress FIFO depth = upstream credit budget
    parameter INT_RSP_WR_CREDIT_FIFO   = 4,   // reverse RSP_WR ingress FIFO depth = upstream credit budget
    // ==== identity and memory map (structural, generator-filled) ====
    parameter NIU_ID               = 0,   // this INIU id value
    parameter NUM_REGION           = 1,   // number of memory-map regions
    parameter REGION_BASE = {(NUM_REGION*EXT_ADDR_WIDTH){1'b0}},     // region base table, flattened
    parameter REGION_MASK = {(NUM_REGION*EXT_ADDR_WIDTH){1'b1}},     // region mask table, flattened
    parameter REGION_DEST = {(NUM_REGION*INT_DEST_ID_WIDTH){1'b0}},  // region DestID table, flattened
    // ---- derived parameters; used in the port list, so they must stay in the
    // ---- parameter list (V2001 body localparams cannot size ports). Not to be
    // ---- overridden externally -- the defaults are the correct values.
    // qos (if any) leads the CMD payload / flit; the response payloads never
    // carry qos at the IP border.
    parameter EXT_CMD_W = EXT_QOS_W + `LB_OPCODE_WIDTH + EXT_ADDR_WIDTH + EXT_LEN_WIDTH +
                          EXT_TXNID_WIDTH + EXT_USER_WIDTH_CMD,
    parameter EXT_WD_W  = EXT_DATA_WIDTH + EXT_DATA_WIDTH/8 + 1 + EXT_TXNID_WIDTH,
    parameter EXT_REQ_W = EXT_CMD_W + EXT_MOD_W + EXT_WD_W,
    parameter EXT_RSP_RD_W  = EXT_TXNID_WIDTH + `LB_RESP_WIDTH + EXT_USER_WIDTH_RSP_RD + 1 + EXT_DATA_WIDTH,
    parameter EXT_RSP_WR_W  = EXT_TXNID_WIDTH + `LB_RESP_WIDTH + EXT_USER_WIDTH_RSP_WR,
    // CMD flit uses in-network widths (INT_QOS head + INT_ID / INT_USER / INT_TOTBYTES)
    parameter INT_CMD_FLIT_W = INT_QOS_W + `LB_OPCODE_WIDTH + INT_ADDR_LOCAL_WIDTH + INT_TOTBYTES_W +
                               INT_USER_WIDTH_CMD + INT_ID_WIDTH + INT_DEST_ID_WIDTH + INT_SRC_ID_WIDTH,
    parameter INT_WD_FLIT_W  = 1 + (EXT_DATA_WIDTH/8) + EXT_DATA_WIDTH +
                               INT_DEST_ID_WIDTH + INT_SRC_ID_WIDTH,
    // REQ_W channel = CMD field + MOD segment (Adv 15, absent at INT_MOD_W = 0)
    // + WD field concatenated (parallel, same beat). REQ_R carries the CMD field
    // alone, so its flit width IS INT_CMD_FLIT_W -- no separate symbol for it.
    parameter INT_REQ_FLIT_W = INT_CMD_FLIT_W + INT_MOD_W + INT_WD_FLIT_W,
    // RSP_RD flit (low->high): data, last, trans_last, user, resp, int_id,
    //                      total_bytes, addr_lane, qos, src_id  (in-network widths)
    // src_id and qos at the top are routing/arbitration fields for the reverse
    // switches; this INIU never reads either, so the bottom-anchored ladder of
    // RSP_RDF_*_LSB offsets below is untouched by their widths.
    parameter INT_RSP_RD_FLIT_W  = INT_SRC_ID_WIDTH + INT_QOS_W + INT_LANE_W + INT_TOTBYTES_W + INT_ID_WIDTH +
                               `LB_RESP_WIDTH + INT_USER_WIDTH_RSP_RD + 1 + 1 + EXT_DATA_WIDTH,
    // RSP_WR flit (low->high): user, resp, int_id, qos, src_id
    parameter INT_RSP_WR_FLIT_W  = INT_SRC_ID_WIDTH + INT_QOS_W + INT_ID_WIDTH +
                               `LB_RESP_WIDTH + INT_USER_WIDTH_RSP_WR
) (
    //--------- inputs: clock / reset (network clock domain) ---------
    input wire                        clk,                 // network-side clock
    input wire                        rst_n,               // async reset, active low
    //--------- inputs: internal interface (fabric side, packed flit + credit) ---------
    input wire                        o_req_r_credit_ret,  // REQ_R credit returned by the peer
    input wire                        o_req_w_credit_ret,  // REQ_W credit returned by the peer
    input wire  [INT_RSP_RD_FLIT_W-1:0]   i_rsp_rd_flit,   // RSP_RD flit in
    input wire                        i_rsp_rd_valid,      // RSP_RD flit valid
    input wire  [INT_RSP_WR_FLIT_W-1:0]   i_rsp_wr_flit,   // RSP_WR flit in
    input wire                        i_rsp_wr_valid,      // RSP_WR flit valid
    //--------- outputs: internal interface (fabric side, packed flit + credit) ---------
    output wire [INT_CMD_FLIT_W-1:0]  o_req_r_flit,        // REQ_R flit out (CMD only)
    output wire                       o_req_r_valid,       // REQ_R flit valid
    output wire [INT_REQ_FLIT_W-1:0]  o_req_w_flit,        // REQ_W flit out (CMD + WD)
    output wire                       o_req_w_valid,       // REQ_W flit valid
    output wire                       i_rsp_rd_credit_ret, // RSP_RD credit returned to the peer
    output wire                       i_rsp_wr_credit_ret, // RSP_WR credit returned to the peer
    //--------- inputs: direct valid-ready border (towards the external half) ---------
    input wire  [EXT_CMD_W-1:0]       vr_req_r_data,       // REQ_R packed payload from the external half
    input wire                        vr_req_r_valid,      // REQ_R valid
    input wire  [EXT_REQ_W-1:0]       vr_req_w_data,       // REQ_W packed payload from the external half
    input wire                        vr_req_w_valid,      // REQ_W valid
    input wire                        vr_rsp_rd_ready,     // RSP_RD  ready from the external half
    input wire                        vr_rsp_wr_ready,     // RSP_WR  ready from the external half
    //--------- outputs: direct valid-ready border (towards the external half) ---------
    output wire                       vr_req_r_ready,      // REQ_R ready to the external half
    output wire                       vr_req_w_ready,      // REQ_W ready to the external half
    output wire [EXT_RSP_RD_W-1:0]        vr_rsp_rd_data,  // RSP_RD  packed payload to the external half
    output wire                       vr_rsp_rd_valid,     // RSP_RD  valid
    output wire [EXT_RSP_WR_W-1:0]        vr_rsp_wr_data,  // RSP_WR  packed payload to the external half
    output wire                       vr_rsp_wr_valid      // RSP_WR  valid
);
    //------------------------------------------------------------------------
    // Derived local params
    //------------------------------------------------------------------------
    localparam EXT_STRB_WIDTH   = EXT_DATA_WIDTH/8;         // external write-strobe width
    // CMD user width for the two id_decode instances, floored at 1: their cmd_user
    // port cannot be zero-width (see EXT_USER_CMD_PW in lb_iniu_id_decode). At
    // EXT_USER_WIDTH_CMD = 0 the wires below carry a constant zero that id_decode's
    // g_cmd_nouser / g_cmd_userz arms never look at.
    localparam EXT_USER_CMD_PW  = (EXT_USER_WIDTH_CMD < 1) ? 1 : EXT_USER_WIDTH_CMD;
    // cmd_qos pin width for the two id_decode instances, floored for the same
    // V2001 reason; at EXT_QOS_W = 0 the wires carry a constant zero that
    // id_decode's g_cmd_noqos / g_cmd_qosz arms never look at.
    localparam EXT_QOS_PW       = (EXT_QOS_W < 1) ? 1 : EXT_QOS_W;
    // RSP_RD flit field offsets (low->high), so the field extraction below reads as
    // named slices instead of repeated additive bit arithmetic.
    localparam RSP_RDF_DATA_LSB  = 0;                                        // data
    // last: fragment level, Switch locks on it
    localparam RSP_RDF_LAST_LSB  = RSP_RDF_DATA_LSB  + EXT_DATA_WIDTH;
    // trans_last: transaction level, the one the IP gets
    localparam RSP_RDF_TLAST_LSB = RSP_RDF_LAST_LSB  + 1;
    localparam RSP_RDF_USER_LSB  = RSP_RDF_TLAST_LSB + 1;                        // user
    localparam RSP_RDF_RESP_LSB  = RSP_RDF_USER_LSB  + INT_USER_WIDTH_RSP_RD;        // resp
    localparam RSP_RDF_ID_LSB    = RSP_RDF_RESP_LSB  + `LB_RESP_WIDTH;           // int_id
    localparam RSP_RDF_TOTB_LSB  = RSP_RDF_ID_LSB    + INT_ID_WIDTH;             // total_bytes
    localparam RSP_RDF_LANE_LSB  = RSP_RDF_TOTB_LSB  + INT_TOTBYTES_W;           // addr_lane
    // RSP_WR flit field offsets (low->high)
    localparam RSP_WRF_USER_LSB  = 0;                                        // user
    localparam RSP_WRF_RESP_LSB  = RSP_WRF_USER_LSB  + INT_USER_WIDTH_RSP_WR;        // resp
    localparam RSP_WRF_ID_LSB    = RSP_WRF_RESP_LSB  + `LB_RESP_WIDTH;           // int_id

    //------------------------------------------------------------------------
    // Declarations (all up front, one per line)
    //------------------------------------------------------------------------
    // forward REQ_R: border -> ext_pipe -> id_decode -> credit_egress -> int_pipe
    wire [EXT_CMD_W-1:0]        ep_rq_r_pk;               // ext-pipe side REQ_R packed payload
    wire                        ep_rq_r_v;                // ext-pipe side REQ_R valid
    wire                        ep_rq_r_r;                // ext-pipe side REQ_R ready (backpressure)
    wire [`LB_OPCODE_WIDTH-1:0] dr_opcode;                // REQ_R CMD opcode field
    wire [EXT_ADDR_WIDTH-1:0]   dr_addr;                  // REQ_R CMD address field
    wire [EXT_LEN_WIDTH-1:0]    dr_len;                   // REQ_R CMD len field
    wire [EXT_TXNID_WIDTH-1:0]  dr_txnid;                 // REQ_R CMD external transaction ID field
    wire [EXT_USER_CMD_PW-1:0]  dr_user;                  // REQ_R CMD user field (zero at width 0)
    wire [EXT_QOS_PW-1:0]       dr_qos;                   // REQ_R CMD qos field (zero at width 0)
    wire [INT_CMD_FLIT_W-1:0]   dec_rq_r_flit;            // packed CMD flit out of the REQ_R id_decode
    wire                        dec_rq_r_v;               // REQ_R CMD flit valid
    wire                        dec_rq_r_r;               // REQ_R CMD flit ready (backpressure)
    wire [INT_CMD_FLIT_W-1:0]   ce_rq_r_data;             // credit-egress output REQ_R flit
    wire                        ce_rq_r_v;                // credit-egress output REQ_R valid
    wire                        ce_rq_r_crd;              // credit-egress REQ_R credit return
    // forward REQ_W: border -> ext_pipe -> split cmd/wd -> id_decode -> ...
    wire [EXT_REQ_W-1:0]        ep_rq_w_pk;               // ext-pipe side REQ_W packed payload
    wire                        ep_rq_w_v;                // ext-pipe side REQ_W valid
    wire                        ep_rq_w_r;                // ext-pipe side REQ_W ready (backpressure)
    wire [EXT_CMD_W-1:0]        ep_cmd_pk;                // CMD field of the REQ_W payload
    wire [EXT_WD_W-1:0]         ep_wd_pk;                 // WD field of the REQ_W payload
    wire [`LB_OPCODE_WIDTH-1:0] d_opcode;                 // REQ_W CMD opcode field
    wire [EXT_ADDR_WIDTH-1:0]   d_addr;                   // REQ_W CMD address field
    wire [EXT_LEN_WIDTH-1:0]    d_len;                    // REQ_W CMD len field
    wire [EXT_TXNID_WIDTH-1:0]  d_txnid;                  // REQ_W CMD external transaction ID field
    wire [EXT_USER_CMD_PW-1:0]  d_user;                   // REQ_W CMD user field (zero at width 0)
    wire [EXT_QOS_PW-1:0]       d_qos;                    // REQ_W CMD qos field (zero at width 0)
    wire [INT_CMD_FLIT_W-1:0]   dec_cmd_flit;             // packed CMD flit out of the REQ_W id_decode
    wire                        dec_cmd_v;                // REQ_W CMD flit valid
    wire                        dec_cmd_r;                // REQ_W CMD flit ready (backpressure)
    wire [EXT_DATA_WIDTH-1:0]   w_data;                   // WD write data
    wire [EXT_STRB_WIDTH-1:0]   w_strb;                   // WD write strobe
    wire                        w_last;                   // WD last beat
    wire [EXT_TXNID_WIDTH-1:0]  w_txnid;                  // WD external transaction ID
    wire [INT_WD_FLIT_W-1:0]    wd_flit_in;               // packed WD flit
    wire [INT_REQ_FLIT_W-1:0]   req_flit_in;              // packed REQ_W flit = {CMD | WD}
    wire                        req_in_valid;             // REQ_W flit valid
    wire                        req_in_ready;             // REQ_W flit ready (backpressure)
    wire                        req_fire;                 // REQ_W flit handshake fire
    wire [INT_REQ_FLIT_W-1:0]   ce_req_data;              // credit-egress output REQ_W flit
    wire                        ce_req_v;                 // credit-egress output REQ_W valid
    wire                        ce_req_crd;               // credit-egress REQ_W credit return
    // reverse: int_pipe -> credit_ingress
    wire [INT_RSP_RD_FLIT_W-1:0]    ip_rsp_rd_data;       // int-pipe side RSP_RD flit
    wire                        ip_rsp_rd_v;              // int-pipe side RSP_RD valid
    wire                        cig_rsp_rd_crd;           // credit-ingress RSP_RD credit return
    wire [INT_RSP_WR_FLIT_W-1:0]    ip_rsp_wr_data;       // int-pipe side RSP_WR flit
    wire                        ip_rsp_wr_v;              // int-pipe side RSP_WR valid
    wire                        cig_rsp_wr_crd;           // credit-ingress RSP_WR credit return
    wire [INT_RSP_RD_FLIT_W-1:0]    ci_rsp_rd_data;       // credit-ingress output RSP_RD flit
    wire                        ci_rsp_rd_v;              // credit-ingress output RSP_RD valid
    wire                        ci_rsp_rd_r;              // credit-ingress output RSP_RD ready (backpressure)
    wire [INT_RSP_WR_FLIT_W-1:0]    ci_rsp_wr_data;       // credit-ingress output RSP_WR flit
    wire                        ci_rsp_wr_v;              // credit-ingress output RSP_WR valid
    wire                        ci_rsp_wr_r;              // credit-ingress output RSP_WR ready (backpressure)
    // reverse: RSP_RD flit -> external RSP_RD payload
    wire [EXT_DATA_WIDTH-1:0]   ci_rsp_rd_data_d;         // RSP_RD flit data field
    wire                        rsp_rd_last;              // RSP_RD flit trans_last -> IP rsp_rd_last
    wire [`LB_RESP_WIDTH-1:0]   rsp_rd_resp;              // RSP_RD flit resp field
    wire [INT_ID_WIDTH-1:0]     rsp_rd_id_i;              // RSP_RD flit int_id field (in-network width)
    wire [EXT_TXNID_WIDTH-1:0]  rsp_rd_id_e;              // RSP_RD int_id truncated to external txnid
    wire [EXT_RSP_RD_W-EXT_DATA_WIDTH-1:0] ci_rsp_rd_hdr; // external RSP_RD header fields (above data)
    wire [EXT_RSP_RD_W-1:0]                ci_rsp_rd_ext; // external RSP_RD packed payload
    // reverse: RSP_WR flit -> external RSP_WR payload
    wire [`LB_RESP_WIDTH-1:0]   rsp_wr_resp;              // RSP_WR flit resp field
    wire [INT_ID_WIDTH-1:0]     rsp_wr_id_i;              // RSP_WR flit int_id field (in-network width)
    wire [EXT_TXNID_WIDTH-1:0]  rsp_wr_id_e;              // RSP_WR int_id truncated to external txnid
    wire [EXT_RSP_WR_W-1:0]         ci_rsp_wr_ext;        // external RSP_WR packed payload
    // reverse: ext_pipe -> bca_slv
    wire [EXT_RSP_RD_W-1:0]         rev_rsp_rd_pk;        // reverse direction RSP_RD packed payload
    wire                        rev_rsp_rd_v;             // reverse direction RSP_RD valid
    wire                        rev_rsp_rd_r;             // reverse direction RSP_RD ready (backpressure)
    wire [EXT_RSP_WR_W-1:0]         rev_rsp_wr_pk;        // reverse direction RSP_WR packed payload
    wire                        rev_rsp_wr_v;             // reverse direction RSP_WR valid
    wire                        rev_rsp_wr_r;             // reverse direction RSP_WR ready (backpressure)

    //========================================================================
    // Forward path, TWICE: border -> ext_pipe -> id_decode -> credit_egress ->
    // int_pipe, once for REQ_R and once for REQ_W. The two chains share nothing.
    //========================================================================

    //------------------------------------------------------------------------
    // REQ_R: ext_pipe (valid-ready skid). The payload is a bare CMD, so there is
    // no cmd/wd split on this arm and nothing to bind two fields to one beat.
    //------------------------------------------------------------------------
    lb_iniu_pipe #(
        .WIDTH     (EXT_CMD_W),
        .ENABLE    (EXT_PIPE_REQ_R)
    ) u10_epipe_req_r (
        .clk       (clk),
        .rst_n     (rst_n),
        .in_data   (vr_req_r_data),
        .in_valid  (vr_req_r_valid),
        .out_ready (ep_rq_r_r),
        .in_ready  (vr_req_r_ready),
        .out_data  (ep_rq_r_pk),
        .out_valid (ep_rq_r_v)
    );

    //------------------------------------------------------------------------
    // REQ_R ID Decode: memory-map decode + CMD flit packing (combinational)
    //------------------------------------------------------------------------
    // A Master with no CMD user sideband has no user member in its REQ payload
    // (EXT_CMD_W drops the addend by itself), so the unpack drops it too and the
    // pin-width stub is driven to zero. Keyed on EXT alone: the in-network width is
    // the bus-wide maximum, so EXT = 0 is the only way this payload can lack it.
    // The qos head (this Master's own width) is sliced positionally off the
    // payload top so the two user arms below stay two; they read the explicit
    // low part, which is the whole payload at EXT_QOS_W = 0.
    generate
    if (EXT_QOS_W > 0) begin : g_rq_r_qos
        assign dr_qos = ep_rq_r_pk[EXT_CMD_W-1 -: EXT_QOS_W];
    end
    else begin : g_rq_r_qos0
        assign dr_qos = 1'b0;
    end
    endgenerate

    generate
    if (EXT_USER_WIDTH_CMD == 0) begin : g_rq_r_nouser
        assign {dr_opcode, dr_addr, dr_len, dr_txnid} = ep_rq_r_pk[EXT_CMD_W-EXT_QOS_W-1:0];
        assign dr_user = {EXT_USER_CMD_PW{1'b0}};
    end
    else begin : g_rq_r_user
        assign {dr_opcode, dr_addr, dr_len, dr_txnid, dr_user} = ep_rq_r_pk[EXT_CMD_W-EXT_QOS_W-1:0];
    end
    endgenerate

    lb_iniu_id_decode #(
        .EXT_ADDR_WIDTH       (EXT_ADDR_WIDTH),
        .EXT_LEN_WIDTH        (EXT_LEN_WIDTH),
        .EXT_TXNID_WIDTH      (EXT_TXNID_WIDTH),
        .EXT_USER_WIDTH_CMD   (EXT_USER_WIDTH_CMD),
        .EXT_DATA_WIDTH       (EXT_DATA_WIDTH),
        .INT_DEST_ID_WIDTH    (INT_DEST_ID_WIDTH),
        .INT_SRC_ID_WIDTH     (INT_SRC_ID_WIDTH),
        .NIU_ID               (NIU_ID),
        .INT_ADDR_LOCAL_WIDTH (INT_ADDR_LOCAL_WIDTH),
        .NUM_REGION           (NUM_REGION),
        .REGION_BASE          (REGION_BASE),
        .REGION_MASK          (REGION_MASK),
        .REGION_DEST          (REGION_DEST),
        .INT_ID_WIDTH         (INT_ID_WIDTH),
        .INT_USER_WIDTH_CMD   (INT_USER_WIDTH_CMD),
        .INT_TOTBYTES_W       (INT_TOTBYTES_W),
        .EXT_QOS_W            (EXT_QOS_W),
        .INT_QOS_W            (INT_QOS_W)
    ) u20_id_decode_r (
        .clk                  (clk),
        .rst_n                (rst_n),
        .cmd_opcode           (dr_opcode),
        .cmd_addr             (dr_addr),
        .cmd_len              (dr_len),
        .cmd_ext_txnid        (dr_txnid),
        .cmd_user             (dr_user),
        .cmd_qos              (dr_qos),
        .cmd_valid            (ep_rq_r_v),
        .flit_ready           (dec_rq_r_r),
        .cmd_ready            (ep_rq_r_r),
        .flit_data            (dec_rq_r_flit),
        .flit_valid           (dec_rq_r_v)
    );

    //------------------------------------------------------------------------
    // REQ_R credit_egress -> int_pipe -> network. The flit IS the CMD flit.
    //------------------------------------------------------------------------
    lb_credit_egress #(
        .WIDTH         (INT_CMD_FLIT_W),
        .CREDIT_INIT   (INT_REQ_R_CREDIT)
    ) u30_ceg_req_r (
        .clk           (clk),
        .rst_n         (rst_n),
        .in_data       (dec_rq_r_flit),
        .in_valid      (dec_rq_r_v),
        .credit_return (ce_rq_r_crd),
        .in_ready      (dec_rq_r_r),
        .out_data      (ce_rq_r_data),
        .out_valid     (ce_rq_r_v)
    );

    lb_iniu_int_pipe #(
        .WIDTH             (INT_CMD_FLIT_W),
        .ENABLE            (INT_PIPE_REQ_R)
    ) u40_ipipe_req_r (
        .clk               (clk),
        .rst_n             (rst_n),
        .in_data           (ce_rq_r_data),
        .in_valid          (ce_rq_r_v),
        .out_credit_return (o_req_r_credit_ret),
        .in_credit_return  (ce_rq_r_crd),
        .out_data          (o_req_r_flit),
        .out_valid         (o_req_r_valid)
    );

    //------------------------------------------------------------------------
    // REQ_W: ext_pipe (valid-ready skid buffer)
    //------------------------------------------------------------------------
    lb_iniu_pipe #(
        .WIDTH     (EXT_REQ_W),
        .ENABLE    (EXT_PIPE_REQ_W)
    ) u50_epipe_req_w (
        .clk       (clk),
        .rst_n     (rst_n),
        .in_data   (vr_req_w_data),
        .in_valid  (vr_req_w_valid),
        .out_ready (ep_rq_w_r),
        .in_ready  (vr_req_w_ready),
        .out_data  (ep_rq_w_pk),
        .out_valid (ep_rq_w_v)
    );

    //------------------------------------------------------------------------
    // split REQ_W: high = cmd field, low = wd field. CMD/WD are bound to the
    // same beat, so ep_rq_w is consumed in one beat and they share one ready.
    //------------------------------------------------------------------------
    assign ep_cmd_pk = ep_rq_w_pk[EXT_REQ_W-1 -: EXT_CMD_W];
    assign ep_wd_pk  = ep_rq_w_pk[0 +: EXT_WD_W];

    //------------------------------------------------------------------------
    // REQ_W ID Decode: memory-map decode + CMD flit packing (combinational)
    //------------------------------------------------------------------------
    // Same shapes as the REQ_R unpack above, on the CMD half of REQ_W.
    generate
    if (EXT_QOS_W > 0) begin : g_rq_w_qos
        assign d_qos = ep_cmd_pk[EXT_CMD_W-1 -: EXT_QOS_W];
    end
    else begin : g_rq_w_qos0
        assign d_qos = 1'b0;
    end
    endgenerate

    generate
    if (EXT_USER_WIDTH_CMD == 0) begin : g_rq_w_nouser
        assign {d_opcode, d_addr, d_len, d_txnid} = ep_cmd_pk[EXT_CMD_W-EXT_QOS_W-1:0];
        assign d_user = {EXT_USER_CMD_PW{1'b0}};
    end
    else begin : g_rq_w_user
        assign {d_opcode, d_addr, d_len, d_txnid, d_user} = ep_cmd_pk[EXT_CMD_W-EXT_QOS_W-1:0];
    end
    endgenerate

    lb_iniu_id_decode #(
        .EXT_ADDR_WIDTH       (EXT_ADDR_WIDTH),
        .EXT_LEN_WIDTH        (EXT_LEN_WIDTH),
        .EXT_TXNID_WIDTH      (EXT_TXNID_WIDTH),
        .EXT_USER_WIDTH_CMD   (EXT_USER_WIDTH_CMD),
        .EXT_DATA_WIDTH       (EXT_DATA_WIDTH),
        .INT_DEST_ID_WIDTH    (INT_DEST_ID_WIDTH),
        .INT_SRC_ID_WIDTH     (INT_SRC_ID_WIDTH),
        .NIU_ID               (NIU_ID),
        .INT_ADDR_LOCAL_WIDTH (INT_ADDR_LOCAL_WIDTH),
        .NUM_REGION           (NUM_REGION),
        .REGION_BASE          (REGION_BASE),
        .REGION_MASK          (REGION_MASK),
        .REGION_DEST          (REGION_DEST),
        .INT_ID_WIDTH         (INT_ID_WIDTH),
        .INT_USER_WIDTH_CMD   (INT_USER_WIDTH_CMD),
        .INT_TOTBYTES_W       (INT_TOTBYTES_W),
        .EXT_QOS_W            (EXT_QOS_W),
        .INT_QOS_W            (INT_QOS_W)
    ) u60_id_decode_w (
        .clk                  (clk),
        .rst_n                (rst_n),
        .cmd_opcode           (d_opcode),
        .cmd_addr             (d_addr),
        .cmd_len              (d_len),
        .cmd_ext_txnid        (d_txnid),
        .cmd_user             (d_user),
        .cmd_qos              (d_qos),
        .cmd_valid            (ep_rq_w_v),
        .flit_ready           (dec_cmd_r),
        .cmd_ready            (ep_rq_w_r),
        .flit_data            (dec_cmd_flit),
        .flit_valid           (dec_cmd_v)
    );

    //------------------------------------------------------------------------
    // WD field -> WD flit. src_id/dest_id of the WD half are zero: the REQ_W flit
    // is routed by the CMD half, which carries the real dest_id.
    //------------------------------------------------------------------------
    assign {w_data, w_strb, w_last, w_txnid} = ep_wd_pk;
    assign wd_flit_in = { w_last,
                          w_strb,
                          w_data,
                          {INT_DEST_ID_WIDTH{1'b0}},
                          {INT_SRC_ID_WIDTH{1'b0}} };

    //------------------------------------------------------------------------
    // REQ_W flit = {CMD field (high) | WD field (low)}, one flit per beat.
    // CMD and WD are split from the same ep_rq_w_pk, hence naturally bound to the
    // same beat with no separate vld-ready: they enter and are consumed together.
    // A write burst sends one REQ_W flit per beat (CMD repeated, WD per beat) and
    // TNIU tracks the position with wd_beat_idx.
    //
    // There is no longer a read case here: a read request never reaches this arm,
    // so the WD half is always real data rather than a don't-care the TNIU had to
    // discard by opcode. That is the bandwidth half of what the split bought --
    // a read used to occupy a full CMD+WD flit all the way across the fabric.
    //
    // The MOD segment (atomic modifier, Adv 15) sits between the two, only on a
    // bus whose INT_MOD_W is nonzero. Three shapes, picked at elaboration:
    //   INT_MOD_W = 0             no segment; the concatenation reads as before
    //   EXT_MOD_W = 0 < INT_MOD_W this Master never issues atomics, so its slot
    //                             in the bus-wide segment is driven to zero
    //   otherwise                 the pin value, zero-extended to the segment
    //------------------------------------------------------------------------
    generate
    if (INT_MOD_W == 0) begin : g_req_nomod
        assign req_flit_in = { dec_cmd_flit, wd_flit_in };
    end
    else if (EXT_MOD_W == 0) begin : g_req_modz
        assign req_flit_in = { dec_cmd_flit, {INT_MOD_W{1'b0}}, wd_flit_in };
    end
    else begin : g_req_mod
        wire [EXT_MOD_W-1:0] ep_mod_pk; // MOD field of the REQ_W payload
        wire [INT_MOD_W-1:0] mod_zext;  // zero-extended to the segment width
        assign ep_mod_pk = ep_rq_w_pk[EXT_WD_W +: EXT_MOD_W];
        // plain assignment zero-extends (EXT_MOD_W <= INT_MOD_W, checked below)
        assign mod_zext  = ep_mod_pk;
        assign req_flit_in = { dec_cmd_flit, mod_zext, wd_flit_in };
    end
    endgenerate
    assign req_in_valid = dec_cmd_v;
    assign req_fire     = req_in_valid && req_in_ready;
    assign dec_cmd_r    = req_fire;

    //------------------------------------------------------------------------
    // REQ_W credit_egress -> int_pipe -> network
    //------------------------------------------------------------------------
    lb_credit_egress #(
        .WIDTH         (INT_REQ_FLIT_W),
        .CREDIT_INIT   (INT_REQ_W_CREDIT)
    ) u70_ceg_req_w (
        .clk           (clk),
        .rst_n         (rst_n),
        .in_data       (req_flit_in),
        .in_valid      (req_in_valid),
        .credit_return (ce_req_crd),
        .in_ready      (req_in_ready),
        .out_data      (ce_req_data),
        .out_valid     (ce_req_v)
    );

    lb_iniu_int_pipe #(
        .WIDTH             (INT_REQ_FLIT_W),
        .ENABLE            (INT_PIPE_REQ_W)
    ) u80_ipipe_req_w (
        .clk               (clk),
        .rst_n             (rst_n),
        .in_data           (ce_req_data),
        .in_valid          (ce_req_v),
        .out_credit_return (o_req_w_credit_ret),
        .in_credit_return  (ce_req_crd),
        .out_data          (o_req_w_flit),
        .out_valid         (o_req_w_valid)
    );

    //========================================================================
    // Reverse path: network -> int_pipe -> credit_ingress -> ext_pipe -> bca_slv
    //========================================================================
    lb_iniu_int_pipe #(
        .WIDTH             (INT_RSP_RD_FLIT_W),
        .ENABLE            (INT_PIPE_RSP_RD)
    ) u81_ipipe_rsp_rd (
        .clk               (clk),
        .rst_n             (rst_n),
        .in_data           (i_rsp_rd_flit),
        .in_valid          (i_rsp_rd_valid),
        .out_credit_return (cig_rsp_rd_crd),
        .in_credit_return  (i_rsp_rd_credit_ret),
        .out_data          (ip_rsp_rd_data),
        .out_valid         (ip_rsp_rd_v)
    );

    lb_iniu_int_pipe #(
        .WIDTH             (INT_RSP_WR_FLIT_W),
        .ENABLE            (INT_PIPE_RSP_WR)
    ) u82_ipipe_rsp_wr (
        .clk               (clk),
        .rst_n             (rst_n),
        .in_data           (i_rsp_wr_flit),
        .in_valid          (i_rsp_wr_valid),
        .out_credit_return (cig_rsp_wr_crd),
        .in_credit_return  (i_rsp_wr_credit_ret),
        .out_data          (ip_rsp_wr_data),
        .out_valid         (ip_rsp_wr_v)
    );

    lb_credit_ingress #(
        .WIDTH         (INT_RSP_RD_FLIT_W),
        .DEPTH         (INT_RSP_RD_CREDIT_FIFO)
    ) u90_cig_rsp_rd (
        .clk           (clk),
        .rst_n         (rst_n),
        .in_data       (ip_rsp_rd_data),
        .in_valid      (ip_rsp_rd_v),
        .out_ready     (ci_rsp_rd_r),
        .out_data      (ci_rsp_rd_data),
        .out_valid     (ci_rsp_rd_v),
        .credit_return (cig_rsp_rd_crd)
    );

    lb_credit_ingress #(
        .WIDTH         (INT_RSP_WR_FLIT_W),
        .DEPTH         (INT_RSP_WR_CREDIT_FIFO)
    ) u91_cig_rsp_wr (
        .clk           (clk),
        .rst_n         (rst_n),
        .in_data       (ip_rsp_wr_data),
        .in_valid      (ip_rsp_wr_v),
        .out_ready     (ci_rsp_wr_r),
        .out_data      (ci_rsp_wr_data),
        .out_valid     (ci_rsp_wr_v),
        .credit_return (cig_rsp_wr_crd)
    );

    //------------------------------------------------------------------------
    // RSP_RD reverse: feed-through. With phase-aligned width conversion the data
    // arrives already at MST width and at the correct phase; beats that lie
    // fully outside the valid range were dropped inside the gearbox by the range
    // criterion, so no byte-level repacking is needed here.
    // The external RSP_RD interface carries no byte_valid; if a byte mask is needed
    // internally it can be rebuilt from (addr_lane, total_bytes, beat index),
    // which is why the flit's total_bytes / addr_lane / src_id fields are carried
    // but not extracted here.
    //
    // The RSP_RD flit has TWO last bits and they mean different things:
    //   last       FRAGMENT level. The Switch burst-locks on this one (LAST_POS
    //              points at it), so a fragment is the fabric's non-interruptible
    //              unit -- which is exactly what makes read interleaving cheap.
    //              Under interleaving with SLV width >= W_frag every beat is its
    //              own fragment, so this bit is 1 on EVERY beat. It is an internal
    //              transport marker and must not leave the fabric.
    //   trans_last TRANSACTION level, asserted once per read. This is the one the
    //              Master IP needs: its rsp_rd_last has to mean "this burst is over",
    //              the same thing the Slave IP's rsp_rd_last means on the other side.
    // So the external rsp_rd_last is trans_last. It used to be `last`, which made a
    // 3-beat interleaved read look like three finished transactions to the IP.
    // (Without interleaving lb_tniu_int_core drives both bits from the Slave's own
    // rsp_rd_last, so the two are the same wire and this choice changes nothing.)
    //------------------------------------------------------------------------
    assign ci_rsp_rd_data_d = ci_rsp_rd_data[RSP_RDF_DATA_LSB +: EXT_DATA_WIDTH];
    assign rsp_rd_last      = ci_rsp_rd_data[RSP_RDF_TLAST_LSB];
    assign rsp_rd_resp      = ci_rsp_rd_data[RSP_RDF_RESP_LSB +: `LB_RESP_WIDTH];
    assign rsp_rd_id_i      = ci_rsp_rd_data[RSP_RDF_ID_LSB   +: INT_ID_WIDTH];
    assign rsp_rd_id_e      = rsp_rd_id_i[EXT_TXNID_WIDTH-1:0];
    assign ci_rsp_rd_ext    = { ci_rsp_rd_hdr, ci_rsp_rd_data_d };

    // A Master with no RSP_RD user pin takes no user member in its external payload
    // (EXT_RSP_RD_W drops the addend by itself), so the field is never extracted from
    // the flit either -- the bus may still carry one for the other Masters, which
    // is why the RSP_RDF_*_LSB ladder above is untouched. Keyed on EXT: INT is the
    // bus-wide maximum, so EXT > 0 guarantees the else arm's slices are in range.
    generate
    if (EXT_USER_WIDTH_RSP_RD == 0) begin : g_rsp_rd_nouser
        assign ci_rsp_rd_hdr = { rsp_rd_id_e, rsp_rd_resp, rsp_rd_last };
    end
    else begin : g_rsp_rd_user
        wire [INT_USER_WIDTH_RSP_RD-1:0] rsp_rd_user_i; // RSP_RD flit user field (in-network width)
        wire [EXT_USER_WIDTH_RSP_RD-1:0] rsp_rd_user_e; // RSP_RD user truncated to external width
        assign rsp_rd_user_i = ci_rsp_rd_data[RSP_RDF_USER_LSB +: INT_USER_WIDTH_RSP_RD];
        assign rsp_rd_user_e = rsp_rd_user_i[EXT_USER_WIDTH_RSP_RD-1:0];
        assign ci_rsp_rd_hdr = { rsp_rd_id_e, rsp_rd_resp, rsp_rd_user_e, rsp_rd_last };
    end
    endgenerate

    //------------------------------------------------------------------------
    // RSP_WR reverse: the flit is at in-network width (INT_ID / INT_USER), truncated
    // back to the external width. Same two shapes as RSP_RD above.
    //------------------------------------------------------------------------
    assign rsp_wr_resp   = ci_rsp_wr_data[RSP_WRF_RESP_LSB +: `LB_RESP_WIDTH];
    assign rsp_wr_id_i   = ci_rsp_wr_data[RSP_WRF_ID_LSB   +: INT_ID_WIDTH];
    assign rsp_wr_id_e   = rsp_wr_id_i[EXT_TXNID_WIDTH-1:0];

    generate
    if (EXT_USER_WIDTH_RSP_WR == 0) begin : g_rsp_wr_nouser
        assign ci_rsp_wr_ext = { rsp_wr_id_e, rsp_wr_resp };
    end
    else begin : g_rsp_wr_user
        wire [INT_USER_WIDTH_RSP_WR-1:0] rsp_wr_user_i; // RSP_WR flit user field (in-network width)
        wire [EXT_USER_WIDTH_RSP_WR-1:0] rsp_wr_user_e; // RSP_WR user truncated to external width
        assign rsp_wr_user_i = ci_rsp_wr_data[RSP_WRF_USER_LSB +: INT_USER_WIDTH_RSP_WR];
        assign rsp_wr_user_e = rsp_wr_user_i[EXT_USER_WIDTH_RSP_WR-1:0];
        assign ci_rsp_wr_ext = { rsp_wr_id_e, rsp_wr_resp, rsp_wr_user_e };
    end
    endgenerate

    //------------------------------------------------------------------------
    // reverse ext_pipe (valid-ready skid buffer per channel)
    //------------------------------------------------------------------------
    lb_iniu_pipe #(
        .WIDTH     (EXT_RSP_RD_W),
        .ENABLE    (EXT_PIPE_RSP_RD)
    ) u100_epipe_rsp_rd (
        .clk       (clk),
        .rst_n     (rst_n),
        .in_data   (ci_rsp_rd_ext),
        .in_valid  (ci_rsp_rd_v),
        .out_ready (rev_rsp_rd_r),
        .in_ready  (ci_rsp_rd_r),
        .out_data  (rev_rsp_rd_pk),
        .out_valid (rev_rsp_rd_v)
    );

    lb_iniu_pipe #(
        .WIDTH     (EXT_RSP_WR_W),
        .ENABLE    (EXT_PIPE_RSP_WR)
    ) u101_epipe_rsp_wr (
        .clk       (clk),
        .rst_n     (rst_n),
        .in_data   (ci_rsp_wr_ext),
        .in_valid  (ci_rsp_wr_v),
        .out_ready (rev_rsp_wr_r),
        .in_ready  (ci_rsp_wr_r),
        .out_data  (rev_rsp_wr_pk),
        .out_valid (rev_rsp_wr_v)
    );

    //------------------------------------------------------------------------
    // reverse RSP_RD/RSP_WR leave straight onto the valid-ready border
    //------------------------------------------------------------------------
    assign vr_rsp_rd_data  = rev_rsp_rd_pk;
    assign vr_rsp_rd_valid = rev_rsp_rd_v;
    assign rev_rsp_rd_r    = vr_rsp_rd_ready;
    assign vr_rsp_wr_data  = rev_rsp_wr_pk;
    assign vr_rsp_wr_valid = rev_rsp_wr_v;
    assign rev_rsp_wr_r    = vr_rsp_wr_ready;

    //------------------------------------------------------------------------
    // Simulation-only guard: every in-network user width must cover its external
    // one. The INIU side had no such check at all (only the TNIU did), yet it
    // violates the invariant in both directions: lb_iniu_id_decode zero-extends
    // CMD, and the RSP_RD / RSP_WR paths truncate back down. A width too small makes the
    // zero-extend a NEGATIVE repeat count and the truncation an out-of-range
    // part-select -- both of which iverilog accepts quietly enough to reach the
    // waveform as corrupt data rather than as an error.
    //------------------------------------------------------------------------
`ifndef LB_NO_ASSERT
    // synthesis translate_off
    initial begin
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
        // Same containment rule for the atomic-modifier segment (Adv 15): the
        // in-network MOD segment is the bus-wide maximum, so every pin fits.
        if (INT_MOD_W < EXT_MOD_W) begin
            $display("ERROR %m: INT_MOD_W=%0d < EXT_MOD_W=%0d",
                     INT_MOD_W, EXT_MOD_W);
        end
        // Same containment rule for the QoS segment: the in-network width is
        // the bus-wide maximum, so this Master's pin zero-extends into it.
        if (INT_QOS_W < EXT_QOS_W) begin
            $display("ERROR %m: INT_QOS_W=%0d < EXT_QOS_W=%0d",
                     INT_QOS_W, EXT_QOS_W);
        end
    end

    //------------------------------------------------------------------------
    // trans_last must imply last, on every RSP_RD flit that reaches this NIU.
    //
    // This is not a style rule, it is what keeps the IP's rsp_rd_last usable. The
    // external rsp_rd_last is trans_last, and lb_rsp_rd_unify_adapt restores the invariant
    // by ANDing trans_last with last -- a MASKING fix. So if some upstream block
    // ever asserts trans_last a beat early, that mask deletes it and the failure
    // mode is not "the IP sees rsp_rd_last twice" (recoverable, visible) but "the IP
    // never sees rsp_rd_last and hangs waiting for the burst to end". Catch the
    // violation at the source instead of debugging the hang.
    //
    // Checked here rather than inside the adapter because a bus can have no unify
    // node at all (bus_16x16 has none) -- this point is on every path.
    always @(posedge clk)
        if (rst_n && ci_rsp_rd_v && ci_rsp_rd_data[RSP_RDF_TLAST_LSB]
                  && !ci_rsp_rd_data[RSP_RDF_LAST_LSB]) begin
            // rsp_rd_last IS trans_last externally, so this beat is masked away and
            // the Master IP never sees the burst end.
            $display("ERROR %m: RSP_RD flit has trans_last without last");
        end

    //------------------------------------------------------------------------
    // A request must arrive on the channel that matches its opcode.
    //
    // This is the mistake the four-channel split makes possible, and nothing else
    // catches it. A write sent down REQ_R is accepted, routed, allocated a LID and
    // delivered to the Slave -- it simply arrives with no data, because REQ_R has
    // no WD half to carry any. A read sent down REQ_W wastes a full CMD+WD flit
    // and lands on the write allocate port, so the response comes back on RSP_WR
    // instead of RSP_RD and the Master waits forever for read data.
    //
    // Both are silent: correct widths, clean elaboration, no protocol violation
    // any downstream block can see. Checked at the point the opcode is first
    // visible on each arm.
    //------------------------------------------------------------------------
    always @(posedge clk) begin
        if (rst_n && ep_rq_r_v && dr_opcode[`LB_OPCODE_WR_BIT]) begin
            $display("ERROR %m: opcode %0h on REQ_R has the write bit set",
                     dr_opcode);
        end
        if (rst_n && ep_rq_w_v && !d_opcode[`LB_OPCODE_WR_BIT]) begin
            $display("ERROR %m: opcode %0h on REQ_W has the write bit clear",
                     d_opcode);
        end
    end
    // synthesis translate_on
`endif

endmodule
