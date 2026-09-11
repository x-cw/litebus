//============================================================================
// Filename    : adapter_slv_axi4.v
// Description : LiteBus TNIU -> AXI4 master. No AWATOP, no AXI5_EN, no atomic.
//============================================================================
`include "adapter_defs.vh"
`default_nettype none
`timescale 1ns/1ps

module adapter_slv_axi4 #(
    parameter ADDR_W = 32,
    parameter DATA_W = 64,
    parameter LEN_W  = 8,
    parameter ID_W   = 8,
    parameter USER_CMD_PW = 1,
    parameter USER_RSP_PW = 1,
    parameter QOS_PW = 1,
    parameter MOD_PW = 1,
    parameter MAX_RD_OST = 256,
    parameter MAX_WR_OST = 256,
    parameter W_BYTES = DATA_W/8,
    parameter EXT_CMD_W = QOS_PW + 4 + ADDR_W + LEN_W + ID_W + USER_CMD_PW,
    parameter EXT_WD_W  = DATA_W + W_BYTES + 1 + ID_W,
    parameter EXT_REQ_W = EXT_CMD_W + MOD_PW + EXT_WD_W
`ifdef ADP_QCH_TO
    , parameter QCH_TO_W = 16
`endif
) (
    input  wire                   clk,
    input  wire                   rst_n,
    input  wire [EXT_CMD_W-1:0]   req_r_data,
    input  wire                   req_r_valid,
    output wire                   req_r_ready,
    input  wire [EXT_REQ_W-1:0]   req_w_data,
    input  wire                   req_w_valid,
    output wire                   req_w_ready,
    output wire [DATA_W-1:0]      rsp_rd_data,
    output wire                   rsp_rd_last,
    output wire [1:0]             rsp_rd_resp,
    output wire [ID_W-1:0]        rsp_rd_ext_txnid,
    output wire [USER_RSP_PW-1:0] rsp_rd_user,
    output wire                   rsp_rd_valid,
    input  wire                   rsp_rd_ready,
    output wire [1:0]             rsp_wr_resp,
    output wire [ID_W-1:0]        rsp_wr_ext_txnid,
    output wire [USER_RSP_PW-1:0] rsp_wr_user,
    output wire                   rsp_wr_valid,
    input  wire                   rsp_wr_ready,
    output wire [ID_W-1:0]        arid,
    output wire [ADDR_W-1:0]      araddr,
    output wire [LEN_W-1:0]       arlen,
    output wire [2:0]             arsize,
    output wire [1:0]             arburst,
    output wire                   arvalid,
    input  wire                   arready,
    input  wire [ID_W-1:0]        rid,
    input  wire [DATA_W-1:0]      rdata,
    input  wire [1:0]             rresp,
    input  wire                   rlast,
    input  wire                   rvalid,
    output wire                   rready,
    output wire [ID_W-1:0]        awid,
    output wire [ADDR_W-1:0]      awaddr,
    output wire [LEN_W-1:0]       awlen,
    output wire [2:0]             awsize,
    output wire [1:0]             awburst,
    output wire                   awvalid,
    input  wire                   awready,
    output wire [DATA_W-1:0]      wdata,
    output wire [W_BYTES-1:0]     wstrb,
    output wire                   wlast,
    output wire                   wvalid,
    input  wire                   wready,
    input  wire [ID_W-1:0]        bid,
    input  wire [1:0]             bresp,
    input  wire                   bvalid,
    output wire                   bready,
    input  wire                   qreqn,
    output wire                   qacceptn,
    output wire                   qdeny,
    output wire                   qactive,
    output wire                   lbus_pwrdn,
    output wire                   intercept
`ifdef ADP_QCH_TO
    , input  wire                 qch_to_en
    , input  wire [QCH_TO_W-1:0]  qch_to_lim
    , input  wire                 qch_to_mode
    , output wire                 irq_qch_to
`endif
);
    adapter_slv_axi_core #(
        .ADDR_W(ADDR_W), .DATA_W(DATA_W), .LEN_W(LEN_W), .ID_W(ID_W),
        .HAS_ATOMIC(0), .USER_CMD_PW(USER_CMD_PW), .USER_RSP_PW(USER_RSP_PW),
        .QOS_PW(QOS_PW), .MOD_PW(MOD_PW),
        .MAX_RD_OST(MAX_RD_OST), .MAX_WR_OST(MAX_WR_OST)
`ifdef ADP_QCH_TO
        , .QCH_TO_W(QCH_TO_W)
`endif
    ) u_core (
        .clk(clk), .rst_n(rst_n),
        .req_r_data(req_r_data), .req_r_valid(req_r_valid), .req_r_ready(req_r_ready),
        .req_w_data(req_w_data), .req_w_valid(req_w_valid), .req_w_ready(req_w_ready),
        .rsp_rd_data(rsp_rd_data), .rsp_rd_last(rsp_rd_last),
        .rsp_rd_resp(rsp_rd_resp), .rsp_rd_ext_txnid(rsp_rd_ext_txnid),
        .rsp_rd_user(rsp_rd_user), .rsp_rd_valid(rsp_rd_valid),
        .rsp_rd_ready(rsp_rd_ready),
        .rsp_wr_resp(rsp_wr_resp), .rsp_wr_ext_txnid(rsp_wr_ext_txnid),
        .rsp_wr_user(rsp_wr_user), .rsp_wr_valid(rsp_wr_valid),
        .rsp_wr_ready(rsp_wr_ready),
        .arid(arid), .araddr(araddr), .arlen(arlen), .arsize(arsize),
        .arburst(arburst), .arvalid(arvalid), .arready(arready),
        .rid(rid), .rdata(rdata), .rresp(rresp), .rlast(rlast),
        .rvalid(rvalid), .rready(rready),
        .awid(awid), .awaddr(awaddr), .awlen(awlen), .awsize(awsize),
        .awburst(awburst), .awvalid(awvalid), .awready(awready),
        .wdata(wdata), .wstrb(wstrb), .wlast(wlast), .wvalid(wvalid),
        .wready(wready),
        .bid(bid), .bresp(bresp), .bvalid(bvalid), .bready(bready),
        .o_awatop(),
        .qreqn(qreqn), .qacceptn(qacceptn), .qdeny(qdeny), .qactive(qactive),
        .lbus_pwrdn(lbus_pwrdn), .intercept(intercept)
`ifdef ADP_QCH_TO
        , .qch_to_en(qch_to_en), .qch_to_lim(qch_to_lim), .qch_to_mode(qch_to_mode),
        .irq_qch_to(irq_qch_to)
`endif
    );
endmodule
