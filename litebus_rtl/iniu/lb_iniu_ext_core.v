//============================================================================
// Filename    : lb_iniu_ext_core.v
// Author      : litebus
// Description : INIU external half, core (Master side, single clk_m domain)
// Date        : 2026-08-06
// Revision    : 1.0 initial -- per-round changes are in doc/HISTORY.md
//----------------------------------------------------------------------------
// Everything the INIU external half does EXCEPT crossing a clock domain:
//   - Adaptor (clk_m, LiteBus feed-through)
//   - unpack of the reverse RSP_RD / RSP_WR payloads into named Master-side fields
//
// The border towards lb_iniu_int_bca is a plain valid-ready interface (vr_*). Two
// wrappers exist around this core and they differ only in that border:
//   lb_iniu_ext_bca  (see lb_iniu_ext_bca.v) adds bca, exposes the async border
//   -- instantiated directly when both NIU halves share one clock
// The form is carried by the MODULE NAME, not by a parameter, because the two
// borders are different sets of ports and a V2001 port list is fixed at
// elaboration. Picking the form with a `define cannot work either: a define is
// global to the compilation unit, while one top can hold NIUs of both forms.
//
// The vr_ prefix is not decoration. This module's IP-side interface is ALSO
// valid-ready, so the border group has to be tellable from it by name.
//
// Parameter naming convention: EXT_* are external (IP-side) interface widths.
// No bca depth / synchronizer depth here -- this core has no bca.
//
// FOUR CHANNELS. The forward request is two channels, REQ_R (read requests) and
// REQ_W (write requests plus their write data), each with its own pins and its
// own handshake, so a read request is never queued behind write data on one
// port. Their payload WIDTHS are the two that already existed, because a read
// request is exactly the CMD half and a write request is CMD + WD:
//     REQ_R payload = EXT_CMD_W       (opcode, addr, len, txnid, user)
//     REQ_W payload = EXT_REQ_W       (the above, plus wdata / wstrb / wlast)
// No new width parameter is introduced for either; EXT_REQ_W keeps its exact
// former meaning and now names the REQ_W channel.
//============================================================================
`include "lb_defines.vh"

module lb_iniu_ext_core #(
    parameter EXT_PROTOCOL       = 0,   // 0 = litebus (feed-through), 1 = AXI, 2 = APB
    parameter EXT_ADDR_WIDTH     = 32,  // external global address width
    parameter EXT_DATA_WIDTH     = 64,  // external (MST-side) data width
    parameter EXT_LEN_WIDTH      = 4,   // external len width (beat count - 1)
    parameter EXT_TXNID_WIDTH    = 8,   // external transaction ID width
    parameter EXT_USER_WIDTH_CMD = 8,   // external CMD user sideband width
    parameter EXT_USER_WIDTH_RSP_RD  = 8,   // external RSP_RD user sideband width
    parameter EXT_USER_WIDTH_RSP_WR  = 8,   // external RSP_WR user sideband width
    parameter EXT_MOD_W          = 0,   // REQ_W atomic-modifier width, 0 = no MOD segment (Adv 15)
    parameter EXT_QOS_W          = 0,   // REQ qos pin width, 0 = none (rides inside the CMD payload)
    // ---- derived parameters; used in the port list, so they must stay in the
    // ---- parameter list (V2001 body localparams cannot size ports). Not to be
    // ---- overridden externally -- the defaults are the correct values.
    parameter EXT_CMD_W = EXT_QOS_W + `LB_OPCODE_WIDTH + EXT_ADDR_WIDTH + EXT_LEN_WIDTH +
                          EXT_TXNID_WIDTH + EXT_USER_WIDTH_CMD,
    parameter EXT_WD_W  = EXT_DATA_WIDTH + EXT_DATA_WIDTH/8 + 1 + EXT_TXNID_WIDTH,
    // external REQ = {cmd field (high) | modifier (Adv 15, often absent) | wd field (low)}
    parameter EXT_REQ_W = EXT_CMD_W + EXT_MOD_W + EXT_WD_W,
    parameter EXT_RSP_RD_W  = EXT_TXNID_WIDTH + `LB_RESP_WIDTH + EXT_USER_WIDTH_RSP_RD + 1 + EXT_DATA_WIDTH,
    parameter EXT_RSP_WR_W  = EXT_TXNID_WIDTH + `LB_RESP_WIDTH + EXT_USER_WIDTH_RSP_WR,
    // user pin widths floored at 1: a V2001 port list is fixed at elaboration, so a
    // width-0 user cannot delete these ports (CODING_STYLE 4A.4). The generated top
    // omits the IP pin instead and leaves the stub open. Declaring them
    // [EXT_USER_WIDTH_RSP_RD-1:0] at width 0 would read [-1:0] -- a LEGAL 2-bit
    // ascending range, i.e. silently wrong rather than an error.
    parameter EXT_USER_RSP_RD_PW = (EXT_USER_WIDTH_RSP_RD < 1) ? 1 : EXT_USER_WIDTH_RSP_RD,
    parameter EXT_USER_RSP_WR_PW = (EXT_USER_WIDTH_RSP_WR < 1) ? 1 : EXT_USER_WIDTH_RSP_WR
) (
    //--------- inputs: clock / reset (IP clock domain) ---------
    input wire                          clk_m,              // IP-side clock
    input wire                          mrst_n,             // IP-side async reset, active low
    //--------- inputs: external interface (Master side, Valid-Ready) ---------
    // Two forward channels: REQ_R carries the CMD half alone, REQ_W carries
    // CMD + WD on one beat. Independent handshakes, so neither waits on the other.
    input wire  [EXT_CMD_W-1:0]         req_r_data,         // REQ_R packed payload (CMD only)
    input wire                          req_r_valid,        // REQ_R valid
    input wire  [EXT_REQ_W-1:0]         req_w_data,         // REQ_W packed payload (CMD + WD)
    input wire                          req_w_valid,        // REQ_W valid
    input wire                          rsp_rd_ready,       // RSP_RD ready (backpressure from IP)
    input wire                          rsp_wr_ready,       // RSP_WR ready (backpressure from IP)
    //--------- outputs: external interface (Master side, Valid-Ready) ---------
    output wire                         req_r_ready,        // REQ_R ready (backpressure to IP)
    output wire                         req_w_ready,        // REQ_W ready (backpressure to IP)
    output wire [EXT_DATA_WIDTH-1:0]    rsp_rd_data,        // RSP_RD data
    output wire                         rsp_rd_last,        // RSP_RD last beat
    output wire [`LB_RESP_WIDTH-1:0]    rsp_rd_resp,        // RSP_RD response code
    output wire [EXT_TXNID_WIDTH-1:0]   rsp_rd_ext_txnid,   // RSP_RD external transaction ID
    output wire [EXT_USER_RSP_RD_PW-1:0]    rsp_rd_user,    // RSP_RD user sideband (zero stub at width 0)
    output wire                         rsp_rd_valid,       // RSP_RD valid
    output wire [`LB_RESP_WIDTH-1:0]    rsp_wr_resp,        // RSP_WR response code
    output wire [EXT_TXNID_WIDTH-1:0]   rsp_wr_ext_txnid,   // RSP_WR external transaction ID
    output wire [EXT_USER_RSP_WR_PW-1:0]    rsp_wr_user,    // RSP_WR user sideband (zero stub at width 0)
    output wire                         rsp_wr_valid,       // RSP_WR valid
    //--------- inputs: direct valid-ready border (towards lb_iniu_int_bca) ---------
    input wire                          vr_req_r_ready,     // REQ_R ready from the internal half
    input wire                          vr_req_w_ready,     // REQ_W ready from the internal half
    input wire  [EXT_RSP_RD_W-1:0]          vr_rsp_rd_data, // RSP_RD  packed payload from the internal half
    input wire                          vr_rsp_rd_valid,    // RSP_RD  valid
    input wire  [EXT_RSP_WR_W-1:0]          vr_rsp_wr_data, // RSP_WR  packed payload from the internal half
    input wire                          vr_rsp_wr_valid,    // RSP_WR  valid
    //--------- outputs: direct valid-ready border (towards lb_iniu_int_bca) ---------
    output wire [EXT_CMD_W-1:0]         vr_req_r_data,      // REQ_R packed payload to the internal half
    output wire                         vr_req_r_valid,     // REQ_R valid
    output wire [EXT_REQ_W-1:0]         vr_req_w_data,      // REQ_W packed payload to the internal half
    output wire                         vr_req_w_valid,     // REQ_W valid
    output wire                         vr_rsp_rd_ready,    // RSP_RD  ready to the internal half
    output wire                         vr_rsp_wr_ready     // RSP_WR  ready to the internal half
);
    //------------------------------------------------------------------------
    // Declarations (all up front, one per line)
    //------------------------------------------------------------------------
    wire [EXT_RSP_RD_W-1:0]  ext_rsp_rd_pk; // external side RSP_RD packed payload
    wire [EXT_RSP_WR_W-1:0]  ext_rsp_wr_pk; // external side RSP_WR packed payload

    //------------------------------------------------------------------------
    // Adaptor: litebus (EXT_PROTOCOL=0) feed-through.
    //   The IP interface already is the internal flit format, so all four
    //   channels pass whole: no conversion logic.
    //   REQ_R carries a CMD alone; REQ_W carries CMD + WD on one beat, so its two
    //   fields are naturally aligned under one handshake. The split into two
    //   channels is what removes the read-behind-write-data queueing that a single
    //   REQ port had.
    //   AXI/APB (EXT_PROTOCOL=1/2): TODO, instantiate the matching adaptor here to
    //   convert external protocol <-> internal REQ_R(cmd) / REQ_W({cmd|wd}) / RSP_RD /
    //   RSP_WR. AXI is a good fit for the split: AR maps to REQ_R and AW+W to REQ_W.
    //   That pending work is why this core is kept even when it degenerates to
    //   pure wiring: it marks where protocol adaptation belongs.
    //------------------------------------------------------------------------
    assign vr_req_r_data  = req_r_data;
    assign vr_req_r_valid = req_r_valid;
    assign req_r_ready    = vr_req_r_ready;
    assign vr_req_w_data  = req_w_data;
    assign vr_req_w_valid = req_w_valid;
    assign req_w_ready    = vr_req_w_ready;
    assign ext_rsp_rd_pk    = vr_rsp_rd_data;
    assign rsp_rd_valid     = vr_rsp_rd_valid;
    assign vr_rsp_rd_ready  = rsp_rd_ready;
    assign ext_rsp_wr_pk    = vr_rsp_wr_data;
    assign rsp_wr_valid     = vr_rsp_wr_valid;
    assign vr_rsp_wr_ready  = rsp_wr_ready;

    //------------------------------------------------------------------------
    // Unpack the reverse payloads back into named external fields.
    // A channel whose EXT_USER_WIDTH_* is 0 has no user member in the payload
    // (EXT_RSP_RD_W / EXT_RSP_WR_W drop the addend by themselves) and no pin on the top,
    // so the port here is the 1-bit floor and carries a constant zero. Keyed on EXT
    // alone: the in-network width is the bus-wide maximum, so EXT = 0 is the only
    // case in which this payload can lack the field.
    //------------------------------------------------------------------------
    generate
    if (EXT_USER_WIDTH_RSP_RD == 0) begin : g_rsp_rd_nouser
        assign {rsp_rd_ext_txnid, rsp_rd_resp, rsp_rd_last, rsp_rd_data} = ext_rsp_rd_pk;
        assign rsp_rd_user = {EXT_USER_RSP_RD_PW{1'b0}};
    end
    else begin : g_rsp_rd_user
        assign {rsp_rd_ext_txnid, rsp_rd_resp, rsp_rd_user, rsp_rd_last, rsp_rd_data} = ext_rsp_rd_pk;
    end
    endgenerate

    generate
    if (EXT_USER_WIDTH_RSP_WR == 0) begin : g_rsp_wr_nouser
        assign {rsp_wr_ext_txnid, rsp_wr_resp} = ext_rsp_wr_pk;
        assign rsp_wr_user = {EXT_USER_RSP_WR_PW{1'b0}};
    end
    else begin : g_rsp_wr_user
        assign {rsp_wr_ext_txnid, rsp_wr_resp, rsp_wr_user} = ext_rsp_wr_pk;
    end
    endgenerate

    //------------------------------------------------------------------------
    // Simulation-only guard: only EXT_PROTOCOL=0 (litebus) has an adaptor.
    //------------------------------------------------------------------------
`ifndef LB_NO_ASSERT
    // synthesis translate_off
    initial begin
        if (EXT_PROTOCOL != 0) begin
            $display("ERROR %m: EXT_PROTOCOL=%0d (AXI/APB) adaptor is not implemented yet",
                     EXT_PROTOCOL);
        end
    end
    // synthesis translate_on
`endif

endmodule
