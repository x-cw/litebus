//============================================================================
// Filename    : lb_tniu_ext_core.v
// Author      : litebus
// Description : TNIU external half, core (Slave side, single clk_s domain)
// Date        : 2026-08-06
// Revision    : 1.0 initial -- per-round changes are in doc/HISTORY.md
//----------------------------------------------------------------------------
// Everything the TNIU external half does EXCEPT crossing a clock domain:
//   - pass REQ_R (a bare CMD) through the adaptor to the Slave
//   - split REQ_W into its cmd / wd fields, and recombine the adaptor's cmd / wd
//     back into the Slave-facing REQ_W
//   - Adaptor (clk_s, internal <-> Slave feed-through)
//
// The border towards lb_tniu_int_bca is a plain valid-ready interface (vr_*). See
// lb_tniu_ext_bca.v for why the two forms are two MODULES rather than one module
// with a parameter or a `define.
//
// Unlike the INIU side, this core does NOT degenerate to pure wiring when
// EXT_PROTOCOL=0: the adaptor and the REQ_W cmd/wd split-and-recombine live here
// regardless.
//
// The opcode-driven write detect is GONE. It existed because one REQ channel
// carried both kinds and the WD half had to be suppressed on reads; now REQ_W is
// a write by construction, so `is_write` is the constant 1 and every conditional
// it gated collapses. The check that a REQ_W flit really does carry a write
// opcode lives in lb_tniu_int_core, where the flit arrives from the fabric.
//
// Parameter naming convention (CODING_STYLE 1.4): EXT_* are external (Slave-IP
// side) interface widths, INT_* are in-network quantities. No bca depth /
// synchronizer depth here -- this core has no bca.
//============================================================================
`include "lb_defines.vh"

module lb_tniu_ext_core #(
    parameter EXT_PROTOCOL         = 0,   // 0 = litebus (feed-through), 1 = AXI, 2 = APB
    parameter EXT_DATA_WIDTH       = 64,  // Slave-side data width
    parameter EXT_LEN_WIDTH        = 4,   // Slave-side len width (beat count - 1)
    parameter EXT_USER_WIDTH_CMD   = 8,   // CMD user sideband width
    parameter EXT_USER_WIDTH_RSP_RD    = 8,   // RSP_RD user sideband width
    parameter EXT_USER_WIDTH_RSP_WR    = 8,   // RSP_WR user sideband width
    parameter EXT_TXNID_WIDTH      = 3,   // Slave-side external transaction ID width
    parameter EXT_MOD_W            = 0,   // REQ_W atomic-modifier pin width, 0 = none (Adv 15)
    parameter EXT_QOS_W            = 0,   // REQ qos pin width, 0 = no pin (rides inside the CMD payload)
    parameter INT_ADDR_LOCAL_WIDTH = 32,  // in-network local (rebased) address width
    // ---- derived parameters; used in the port list, so they must stay in the
    // ---- parameter list (V2001 body localparams cannot size ports). Not to be
    // ---- overridden externally -- the defaults are the correct values.
    parameter EXT_STRB_WIDTH = EXT_DATA_WIDTH/8,
    // qos (if any) leads the CMD payload; it is opaque to this half throughout.
    parameter EXT_CMD_W  = EXT_QOS_W + `LB_OPCODE_WIDTH + INT_ADDR_LOCAL_WIDTH + EXT_LEN_WIDTH +
                           EXT_TXNID_WIDTH + EXT_USER_WIDTH_CMD,
    // {CMD, MOD} travel as one segment through the adaptor's cmd channel; at
    // EXT_MOD_W = 0 this collapses onto EXT_CMD_W and nothing below changes.
    parameter EXT_CMDM_W = EXT_CMD_W + EXT_MOD_W,
    parameter EXT_WD_W   = EXT_DATA_WIDTH + EXT_STRB_WIDTH + 1 + EXT_TXNID_WIDTH,
    // external REQ = {cmd field (high) | modifier (Adv 15, often absent) | wd field (low)}
    parameter EXT_REQ_W  = EXT_CMDM_W + EXT_WD_W,
    parameter EXT_RSP_RD_W   = EXT_TXNID_WIDTH + `LB_RESP_WIDTH + EXT_USER_WIDTH_RSP_RD + 1 + EXT_DATA_WIDTH,
    parameter EXT_RSP_WR_W   = EXT_TXNID_WIDTH + `LB_RESP_WIDTH + EXT_USER_WIDTH_RSP_WR,
    // user pin widths floored at 1: a V2001 port list is fixed at elaboration, so a
    // width-0 user cannot delete these ports (CODING_STYLE 4A.4). The generated top
    // omits the IP pin and ties the stub to zero. Declaring them
    // [EXT_USER_WIDTH_RSP_RD-1:0] at width 0 would read [-1:0] -- a LEGAL 2-bit
    // ascending range, i.e. silently wrong rather than an error.
    parameter EXT_USER_RSP_RD_PW = (EXT_USER_WIDTH_RSP_RD < 1) ? 1 : EXT_USER_WIDTH_RSP_RD,
    parameter EXT_USER_RSP_WR_PW = (EXT_USER_WIDTH_RSP_WR < 1) ? 1 : EXT_USER_WIDTH_RSP_WR
) (
    //--------- inputs: clock / reset (Slave clock domain) ---------
    input wire                             clk_s,              // Slave-side clock
    input wire                             srst_n,             // Slave-side async reset, active low
    //--------- inputs: external interface (Slave side, Valid-Ready) ---------
    input wire                             req_r_ready,        // REQ_R ready (backpressure from Slave)
    input wire                             req_w_ready,        // REQ_W ready (backpressure from Slave)
    input wire  [EXT_DATA_WIDTH-1:0]       rsp_rd_data,        // RSP_RD data from Slave
    input wire                             rsp_rd_last,        // RSP_RD last beat
    input wire  [`LB_RESP_WIDTH-1:0]       rsp_rd_resp,        // RSP_RD response code
    input wire  [EXT_TXNID_WIDTH-1:0]      rsp_rd_ext_txnid,   // RSP_RD Slave-side transaction ID
    input wire  [EXT_USER_RSP_RD_PW-1:0]       rsp_rd_user,    // RSP_RD user sideband (tied off at width 0)
    input wire                             rsp_rd_valid,       // RSP_RD valid
    input wire  [`LB_RESP_WIDTH-1:0]       rsp_wr_resp,        // RSP_WR response code
    input wire  [EXT_TXNID_WIDTH-1:0]      rsp_wr_ext_txnid,   // RSP_WR Slave-side transaction ID
    input wire  [EXT_USER_RSP_WR_PW-1:0]       rsp_wr_user,    // RSP_WR user sideband (tied off at width 0)
    input wire                             rsp_wr_valid,       // RSP_WR valid
    //--------- outputs: external interface (Slave side, Valid-Ready) ---------
    // Two request channels to the Slave: REQ_R is a bare CMD, REQ_W is
    // {cmd field | wd field} on one beat.
    output wire [EXT_CMD_W-1:0]            req_r_data,         // REQ_R packed payload (CMD only)
    output wire                            req_r_valid,        // REQ_R valid
    output wire [EXT_REQ_W-1:0]            req_w_data,         // REQ_W packed payload (CMD + WD)
    output wire                            req_w_valid,        // REQ_W valid
    output wire                            rsp_rd_ready,       // RSP_RD ready (backpressure to Slave)
    output wire                            rsp_wr_ready,       // RSP_WR ready (backpressure to Slave)
    //--------- inputs: direct valid-ready border (towards lb_tniu_int_bca) ---------
    input wire  [EXT_CMD_W-1:0]            vr_req_r_data,      // REQ_R packed payload from the internal half
    input wire                             vr_req_r_valid,     // REQ_R valid
    input wire  [EXT_REQ_W-1:0]            vr_req_w_data,      // REQ_W packed payload from the internal half
    input wire                             vr_req_w_valid,     // REQ_W valid
    input wire                             vr_rsp_rd_ready,    // RSP_RD  ready from the internal half
    input wire                             vr_rsp_wr_ready,    // RSP_WR  ready from the internal half
    //--------- outputs: direct valid-ready border (towards lb_tniu_int_bca) ---------
    output wire                            vr_req_r_ready,     // REQ_R ready to the internal half
    output wire                            vr_req_w_ready,     // REQ_W ready to the internal half
    output wire [EXT_RSP_RD_W-1:0]             vr_rsp_rd_data, // RSP_RD  packed payload to the internal half
    output wire                            vr_rsp_rd_valid,    // RSP_RD  valid
    output wire [EXT_RSP_WR_W-1:0]             vr_rsp_wr_data, // RSP_WR  packed payload to the internal half
    output wire                            vr_rsp_wr_valid     // RSP_WR  valid
);
    //------------------------------------------------------------------------
    // Declarations (all up front, one per line)
    //------------------------------------------------------------------------
    // REQ_W off the border, split into cmd/wd for the adaptor
    wire [EXT_CMDM_W-1:0]       sd_cmd_pk;         // CMD (+MOD) field of the REQ_W payload
    wire [EXT_WD_W-1:0]         sd_wd_pk;          // WD field of the REQ_W payload
    wire                        sd_cmd_r;          // CMD ready (backpressure) from the adaptor
    wire                        sd_wd_r;           // WD ready (backpressure) from the adaptor
    // adaptor external side (Slave-facing) cmd/wd, recombined into req_w_data
    wire [EXT_CMDM_W-1:0]       s_cmd_pk;          // Slave-side CMD (+MOD) packed payload
    wire                        s_cmd_v;           // Slave-side CMD valid
    wire                        s_cmd_r;           // Slave-side CMD ready (backpressure)
    wire [EXT_WD_W-1:0]         s_wd_pk;           // Slave-side WD packed payload
    wire                        s_wd_v;            // Slave-side WD valid
    wire                        s_wd_r;            // Slave-side WD ready (backpressure)
    // reverse payloads, packed here rather than in the instance port expression:
    // the user member is present or absent by elaboration and a port expression
    // cannot carry a generate
    wire [EXT_RSP_RD_W-1:0]         ext_rsp_rd_pk; // Slave-side RSP_RD packed payload
    wire [EXT_RSP_WR_W-1:0]         ext_rsp_wr_pk; // Slave-side RSP_WR packed payload

    //------------------------------------------------------------------------
    // split REQ_W: high = cmd field, low = wd field. Both halves are always real
    // on this channel, so the beat is consumed when both are accepted -- there is
    // no read case to suppress the WD half for any more.
    //------------------------------------------------------------------------
    assign sd_cmd_pk      = vr_req_w_data[EXT_REQ_W-1 -: EXT_CMDM_W];
    assign sd_wd_pk       = vr_req_w_data[0 +: EXT_WD_W];
    assign vr_req_w_ready = sd_cmd_r && sd_wd_r;

    //------------------------------------------------------------------------
    // Pack the reverse payloads coming off the Slave pins. A channel whose
    // EXT_USER_WIDTH_* is 0 has no user member here (EXT_RSP_RD_W / EXT_RSP_WR_W drop the
    // addend by themselves) and no pin on the top, so the port is the 1-bit floor,
    // tied to zero by the caller and ignored. Keyed on EXT alone: the in-network
    // width is the bus-wide maximum, so EXT = 0 is the only case in which this
    // payload can lack the field.
    //------------------------------------------------------------------------
    generate
    if (EXT_USER_WIDTH_RSP_RD == 0) begin : g_rsp_rd_nouser
        assign ext_rsp_rd_pk = {rsp_rd_ext_txnid, rsp_rd_resp, rsp_rd_last, rsp_rd_data};
    end
    else begin : g_rsp_rd_user
        assign ext_rsp_rd_pk = {rsp_rd_ext_txnid, rsp_rd_resp, rsp_rd_user, rsp_rd_last, rsp_rd_data};
    end
    endgenerate

    generate
    if (EXT_USER_WIDTH_RSP_WR == 0) begin : g_rsp_wr_nouser
        assign ext_rsp_wr_pk = {rsp_wr_ext_txnid, rsp_wr_resp};
    end
    else begin : g_rsp_wr_user
        assign ext_rsp_wr_pk = {rsp_wr_ext_txnid, rsp_wr_resp, rsp_wr_user};
    end
    endgenerate

    //------------------------------------------------------------------------
    // Adaptor (clk_s): internal (border side) <-> Slave fields
    //------------------------------------------------------------------------
    lb_tniu_adaptor #(
        .EXT_PROTOCOL   (EXT_PROTOCOL),
        .EXT_CMD_W      (EXT_CMD_W),
        .EXT_MOD_W      (EXT_MOD_W),
        .EXT_WD_W       (EXT_WD_W),
        .EXT_RSP_RD_W       (EXT_RSP_RD_W),
        .EXT_RSP_WR_W       (EXT_RSP_WR_W)
    ) u10_adaptor (
        .clk            (clk_s),
        .rst_n          (srst_n),
        // ---- inputs (internal side, network direction) ----
        .int_rq_r_data  (vr_req_r_data),
        .int_rq_r_valid (vr_req_r_valid),
        .int_cmd_data   (sd_cmd_pk),
        .int_cmd_valid  (vr_req_w_valid),
        .int_wd_data    (sd_wd_pk),
        .int_wd_valid   (vr_req_w_valid),
        .int_rsp_rd_ready   (vr_rsp_rd_ready),
        .int_rsp_wr_ready   (vr_rsp_wr_ready),
        // ---- inputs (external side, Slave direction) ----
        .ext_rq_r_ready (req_r_ready),
        .ext_cmd_ready  (s_cmd_r),
        .ext_wd_ready   (s_wd_r),
        .ext_rsp_rd_data    (ext_rsp_rd_pk),
        .ext_rsp_rd_valid   (rsp_rd_valid),
        .ext_rsp_wr_data    (ext_rsp_wr_pk),
        .ext_rsp_wr_valid   (rsp_wr_valid),
        // ---- outputs (internal side) ----
        .int_rq_r_ready (vr_req_r_ready),
        .int_cmd_ready  (sd_cmd_r),
        .int_wd_ready   (sd_wd_r),
        .int_rsp_rd_data    (vr_rsp_rd_data),
        .int_rsp_rd_valid   (vr_rsp_rd_valid),
        .int_rsp_wr_data    (vr_rsp_wr_data),
        .int_rsp_wr_valid   (vr_rsp_wr_valid),
        // ---- outputs (external side) ----
        .ext_rq_r_data  (req_r_data),
        .ext_rq_r_valid (req_r_valid),
        .ext_cmd_data   (s_cmd_pk),
        .ext_cmd_valid  (s_cmd_v),
        .ext_wd_data    (s_wd_pk),
        .ext_wd_valid   (s_wd_v),
        .ext_rsp_rd_ready   (rsp_rd_ready),
        .ext_rsp_wr_ready   (rsp_wr_ready)
    );

    //------------------------------------------------------------------------
    // adaptor cmd/wd recombined into the external REQ_W ({cmd | wd}). REQ_R needs
    // no recombination -- it leaves the adaptor already in its final form.
    //------------------------------------------------------------------------
    assign req_w_data  = { s_cmd_pk, s_wd_pk };
    assign req_w_valid = s_cmd_v && s_wd_v;
    assign s_cmd_r     = req_w_ready;
    assign s_wd_r      = req_w_ready;

endmodule
