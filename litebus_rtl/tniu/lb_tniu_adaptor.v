//============================================================================
// Filename    : lb_tniu_adaptor.v
// Author      : litebus
// Description : TNIU protocol adaptor (external Slave side, optional)
// Date        : 2026-08-06
// Revision    : 1.0 initial -- per-round changes are in doc/HISTORY.md
//----------------------------------------------------------------------------
// Symmetric to the INIU adaptor but reversed: TNIU faces the Slave, so the
// request channels are outputs to the Slave and RSP_RD/RSP_WR are inputs from the Slave.
//   - EXT_PROTOCOL = LITEBUS : feed-through (default).
//   - EXT_PROTOCOL = AXI/APB : Litebus<->AXI/APB conversion (TODO placeholder).
//   EXT_PROTOCOL : conversion type; EXT_{CMD,WD,RSP_RD,RSP_WR}_W are the per-channel
//                  widths, all on the external (Slave-facing) side, hence EXT_*
//                  per CODING_STYLE 1.4. Both port sides carry the same widths;
//                  int_*/ext_* here name the bca side vs the Slave side of this
//                  adaptor, not the fabric-vs-IP interface split.
//
// FIVE CHANNELS since REQ split into REQ_R and REQ_W:
//   rq_r    the read-request channel, a CMD payload on its own (EXT_CMD_W)
//   cmd     the CMD half of the write-request channel
//   wd      the WD half of the write-request channel
//   rsp_rd / rsp_wr the two response channels
// rq_r is a separate channel rather than a second user of `cmd` because the whole
// point of the split is that a read request never queues behind write data. Every
// channel here is an independent feed-through, so carrying one more costs one more
// assign triple and keeps protocol adaptation in one place for both request
// channels -- an AXI adaptor maps AR onto rq_r and AW+W onto cmd+wd.
//============================================================================
`include "lb_defines.vh"

module lb_tniu_adaptor #(
    parameter EXT_PROTOCOL = 0,              // 0 = LITEBUS, 1 = AXI, 2 = APB
    parameter EXT_CMD_W    = 64,             // CMD channel width
    // The atomic modifier (Adv 15) rides the cmd channel as its low bits -- it is
    // a command attribute (an AXI adaptor would map it near AWATOP), so the cmd
    // ports below run at EXT_CMD_W + EXT_MOD_W while rq_r stays a bare CMD.
    parameter EXT_MOD_W    = 0,              // REQ_W modifier width, 0 = absent
    parameter EXT_WD_W     = 64,             // WD channel width
    parameter EXT_RSP_RD_W     = 64,             // RSP_RD channel width
    parameter EXT_RSP_WR_W     = 64              // RSP_WR channel width
) (
    // ---- inputs (clock/reset) ----
    input wire                   clk,                   // clock
    input wire                   rst_n,                 // async reset, active low
    // ---- inputs (internal side, network direction) ----
    input wire  [EXT_CMD_W-1:0]  int_rq_r_data,         // internal REQ_R data (a CMD payload)
    input wire                   int_rq_r_valid,        // internal REQ_R valid
    input wire  [EXT_CMD_W+EXT_MOD_W-1:0] int_cmd_data, // internal CMD (+MOD) data (REQ_W half)
    input wire                   int_cmd_valid,         // internal CMD valid
    input wire  [EXT_WD_W-1:0]   int_wd_data,           // internal WD data (REQ_W half)
    input wire                   int_wd_valid,          // internal WD valid
    input wire                   int_rsp_rd_ready,      // internal RSP_RD ready
    input wire                   int_rsp_wr_ready,      // internal RSP_WR ready
    // ---- inputs (external side, Slave direction) ----
    input wire                   ext_rq_r_ready,        // external REQ_R ready
    input wire                   ext_cmd_ready,         // external CMD ready
    input wire                   ext_wd_ready,          // external WD ready
    input wire  [EXT_RSP_RD_W-1:0]   ext_rsp_rd_data,   // external RSP_RD data
    input wire                   ext_rsp_rd_valid,      // external RSP_RD valid
    input wire  [EXT_RSP_WR_W-1:0]   ext_rsp_wr_data,   // external RSP_WR data
    input wire                   ext_rsp_wr_valid,      // external RSP_WR valid
    // ---- outputs (internal side) ----
    output wire                  int_rq_r_ready,        // internal REQ_R ready
    output wire                  int_cmd_ready,         // internal CMD ready
    output wire                  int_wd_ready,          // internal WD ready
    output wire [EXT_RSP_RD_W-1:0]   int_rsp_rd_data,   // internal RSP_RD data
    output wire                  int_rsp_rd_valid,      // internal RSP_RD valid
    output wire [EXT_RSP_WR_W-1:0]   int_rsp_wr_data,   // internal RSP_WR data
    output wire                  int_rsp_wr_valid,      // internal RSP_WR valid
    // ---- outputs (external side) ----
    output wire [EXT_CMD_W-1:0]  ext_rq_r_data,         // external REQ_R data
    output wire                  ext_rq_r_valid,        // external REQ_R valid
    output wire [EXT_CMD_W+EXT_MOD_W-1:0] ext_cmd_data, // external CMD (+MOD) data
    output wire                  ext_cmd_valid,         // external CMD valid
    output wire [EXT_WD_W-1:0]   ext_wd_data,           // external WD data
    output wire                  ext_wd_valid,          // external WD valid
    output wire                  ext_rsp_rd_ready,      // external RSP_RD ready
    output wire                  ext_rsp_wr_ready       // external RSP_WR ready
);
    generate
    if (EXT_PROTOCOL == 0) begin : g_litebus_passthrough
        //--------------------------------------------------------------------
        // LITEBUS feed-through: internal <-> external per channel
        // (REQ_R / CMD / WD out to the Slave, RSP_RD / RSP_WR in from it)
        //--------------------------------------------------------------------
        // REQ_R (out to Slave)
        assign ext_rq_r_data  = int_rq_r_data;
        assign ext_rq_r_valid = int_rq_r_valid;
        assign int_rq_r_ready = ext_rq_r_ready;
        // CMD (out to Slave)
        assign ext_cmd_data  = int_cmd_data;
        assign ext_cmd_valid = int_cmd_valid;
        assign int_cmd_ready = ext_cmd_ready;
        // WD (out to Slave)
        assign ext_wd_data   = int_wd_data;
        assign ext_wd_valid  = int_wd_valid;
        assign int_wd_ready  = ext_wd_ready;
        // RSP_RD (in from Slave)
        assign int_rsp_rd_data   = ext_rsp_rd_data;
        assign int_rsp_rd_valid  = ext_rsp_rd_valid;
        assign ext_rsp_rd_ready  = int_rsp_rd_ready;
        // RSP_WR (in from Slave)
        assign int_rsp_wr_data   = ext_rsp_wr_data;
        assign int_rsp_wr_valid  = ext_rsp_wr_valid;
        assign ext_rsp_wr_ready  = int_rsp_wr_ready;
    end else begin : g_axi_apb
        //--------------------------------------------------------------------
        // AXI/APB conversion (TODO placeholder): tie off to safe defaults
        //--------------------------------------------------------------------
        assign ext_rq_r_data  = {EXT_CMD_W{1'b0}};
        assign ext_rq_r_valid = 1'b0;
        assign int_rq_r_ready = 1'b0;
        assign ext_cmd_data  = {(EXT_CMD_W + EXT_MOD_W){1'b0}};
        assign ext_cmd_valid = 1'b0;
        assign int_cmd_ready = 1'b0;
        assign ext_wd_data   = {EXT_WD_W{1'b0}};
        assign ext_wd_valid  = 1'b0;
        assign int_wd_ready  = 1'b0;
        assign int_rsp_rd_data   = {EXT_RSP_RD_W{1'b0}};
        assign int_rsp_rd_valid  = 1'b0;
        assign ext_rsp_rd_ready  = 1'b0;
        assign int_rsp_wr_data   = {EXT_RSP_WR_W{1'b0}};
        assign int_rsp_wr_valid  = 1'b0;
        assign ext_rsp_wr_ready  = 1'b0;
    end
    endgenerate
endmodule
