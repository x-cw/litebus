//============================================================================
// Filename    : adapter_mst_axi5.v
// Description : Independent AXI5 slave -> LiteBus top (AWATOP + atomic).
//               Does not switch adapter_mst_axi4 via AXI5_EN.
//============================================================================
`include "adapter_defs.vh"
`default_nettype none
`timescale 1ns/1ps

module adapter_mst_axi5 #(
    parameter ADDR_W = 32,
    parameter DATA_W = 64,
    parameter LEN_W  = 8,
    parameter ID_W   = 8,
    parameter ATOMIC_FAIL_RESP = `AXI_RESP_EXOKAY,
    parameter QOS_PW = 1,
    parameter USER_CMD_PW = 1,
    parameter MOD_PW = 3,
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
    input  wire [ID_W-1:0]        awid,
    input  wire [ADDR_W-1:0]      awaddr,
    input  wire [LEN_W-1:0]       awlen,
    input  wire [2:0]             awsize,
    input  wire [1:0]             awburst,
    input  wire                   awvalid,
    output wire                   awready,
    input  wire [DATA_W-1:0]      wdata,
    input  wire [W_BYTES-1:0]     wstrb,
    input  wire                   wlast,
    input  wire                   wvalid,
    output wire                   wready,
    output wire [ID_W-1:0]        bid,
    output wire [1:0]             bresp,
    output wire                   bvalid,
    input  wire                   bready,
    input  wire [ID_W-1:0]        arid,
    input  wire [ADDR_W-1:0]      araddr,
    input  wire [LEN_W-1:0]       arlen,
    input  wire [2:0]             arsize,
    input  wire [1:0]             arburst,
    input  wire                   arvalid,
    output wire                   arready,
    output wire [ID_W-1:0]        rid,
    output wire [DATA_W-1:0]      rdata,
    output wire [1:0]             rresp,
    output wire                   rlast,
    output wire                   rvalid,
    input  wire                   rready,
    input  wire [4:0]             awatop,
    output wire [EXT_CMD_W-1:0]   req_r_data,
    output wire                   req_r_valid,
    input  wire                   req_r_ready,
    output wire [EXT_REQ_W-1:0]   req_w_data,
    output wire                   req_w_valid,
    input  wire                   req_w_ready,
    input  wire [DATA_W-1:0]      rsp_rd_data,
    input  wire                   rsp_rd_last,
    input  wire [1:0]             rsp_rd_resp,
    input  wire [ID_W-1:0]        rsp_rd_ext_txnid,
    input  wire                   rsp_rd_valid,
    output wire                   rsp_rd_ready,
    input  wire [1:0]             rsp_wr_resp,
    input  wire [ID_W-1:0]        rsp_wr_ext_txnid,
    input  wire                   rsp_wr_valid,
    output wire                   rsp_wr_ready,
    input  wire                   qreqn,
    input  wire                   reg_qdeny_en,
    input  wire                   reg_err_en,
    output wire                   qacceptn,
    output wire                   qdeny,
    output wire                   qactive,
    output wire                   lbus_pwrdn
`ifdef ADP_QCH_TO
    , input  wire                 qch_to_en
    , input  wire [QCH_TO_W-1:0]  qch_to_lim
    , input  wire                 qch_to_mode
    , output wire                 irq_qch_to
`endif
);
    adapter_mst_axi_core #(
        .ADDR_W(ADDR_W), .DATA_W(DATA_W), .LEN_W(LEN_W), .ID_W(ID_W),
        .HAS_ATOMIC(1), .ATOMIC_FAIL_RESP(ATOMIC_FAIL_RESP),
        .QOS_PW(QOS_PW), .USER_CMD_PW(USER_CMD_PW), .MOD_PW(MOD_PW),
        .MAX_RD_OST(MAX_RD_OST), .MAX_WR_OST(MAX_WR_OST)
`ifdef ADP_QCH_TO
        , .QCH_TO_W(QCH_TO_W)
`endif
    ) u_core (
        .clk(clk), .rst_n(rst_n),
        .awid(awid), .awaddr(awaddr), .awlen(awlen), .awsize(awsize),
        .awburst(awburst), .awvalid(awvalid), .awready(awready),
        .wdata(wdata), .wstrb(wstrb), .wlast(wlast), .wvalid(wvalid),
        .wready(wready),
        .bid(bid), .bresp(bresp), .bvalid(bvalid), .bready(bready),
        .arid(arid), .araddr(araddr), .arlen(arlen), .arsize(arsize),
        .arburst(arburst), .arvalid(arvalid), .arready(arready),
        .rid(rid), .rdata(rdata), .rresp(rresp), .rlast(rlast),
        .rvalid(rvalid), .rready(rready),
        .i_awatop(awatop),
        .req_r_data(req_r_data), .req_r_valid(req_r_valid), .req_r_ready(req_r_ready),
        .req_w_data(req_w_data), .req_w_valid(req_w_valid), .req_w_ready(req_w_ready),
        .rsp_rd_data(rsp_rd_data), .rsp_rd_last(rsp_rd_last), .rsp_rd_resp(rsp_rd_resp),
        .rsp_rd_ext_txnid(rsp_rd_ext_txnid), .rsp_rd_valid(rsp_rd_valid),
        .rsp_rd_ready(rsp_rd_ready),
        .rsp_wr_resp(rsp_wr_resp), .rsp_wr_ext_txnid(rsp_wr_ext_txnid),
        .rsp_wr_valid(rsp_wr_valid), .rsp_wr_ready(rsp_wr_ready),
        .qreqn(qreqn), .reg_qdeny_en(reg_qdeny_en), .reg_err_en(reg_err_en),
        .qacceptn(qacceptn), .qdeny(qdeny), .qactive(qactive),
        .lbus_pwrdn(lbus_pwrdn)
`ifdef ADP_QCH_TO
        , .qch_to_en(qch_to_en), .qch_to_lim(qch_to_lim), .qch_to_mode(qch_to_mode),
        .irq_qch_to(irq_qch_to)
`endif
    );
endmodule
