//============================================================================
// Filename    : adapter_mst_axi5.v
// Description : adapter_mst top: AXI5 slave -> LiteBus INIU-side interface.
//               Thin wrapper of adapter_mst_axi4 with ATOMIC_EN=1 and the
//               AWATOP port exposed.
//============================================================================
`include "adapter_ip_defs.vh"
`default_nettype none
`timescale 1ns/1ps

module adapter_mst_axi5 #(
    parameter ADDR_W  = 32,
    parameter DATA_W  = 64,
    parameter LEN_W   = 8,
    parameter ID_W    = 8,
    parameter NARROW_EN = 1,
    parameter SAME_ID_EN = 1,
    parameter SPLIT_EN   = 0,
    parameter PEND_TX   = 8,
    parameter PEND_WR   = 4,
    parameter R_SKID_DEPTH = 4,
    parameter LB_MAX_BURST_BYTES = 0,
    parameter AWATOP_W  = 5,
    parameter ATOMIC_FAIL_RESP = `AXI_RESP_EXOKAY,
    // derived
    parameter W_BYTES = DATA_W/8,
    parameter LANE_W  = `adp_clog2(W_BYTES),
    parameter IDX_W   = `adp_clog2(PEND_TX),
    parameter QOS_PW  = 1,
    parameter USER_CMD_PW = 1,
    parameter MOD_W    = 3,
    parameter MOD_PW   = (MOD_W < 1) ? 1 : MOD_W,
    parameter EXT_CMD_W = QOS_PW + 4 + ADDR_W + LEN_W + ID_W + USER_CMD_PW,
    parameter EXT_WD_W  = DATA_W + W_BYTES + 1 + ID_W,
    parameter EXT_REQ_W = EXT_CMD_W + MOD_PW + EXT_WD_W
) (
    input  wire                    clk,
    input  wire                    rst_n,
    input  wire [ID_W-1:0]         awid,
    input  wire [ADDR_W-1:0]       awaddr,
    input  wire [LEN_W-1:0]        awlen,
    input  wire [2:0]              awsize,
    input  wire [1:0]              awburst,
    input  wire                    awvalid,
    output wire                    awready,
    input  wire [DATA_W-1:0]       wdata,
    input  wire [W_BYTES-1:0]      wstrb,
    input  wire                    wlast,
    input  wire                    wvalid,
    output wire                    wready,
    output wire [ID_W-1:0]         bid,
    output wire [1:0]              bresp,
    output wire                    bvalid,
    input  wire                    bready,
    input  wire [ID_W-1:0]         arid,
    input  wire [ADDR_W-1:0]       araddr,
    input  wire [LEN_W-1:0]        arlen,
    input  wire [2:0]              arsize,
    input  wire [1:0]              arburst,
    input  wire                    arvalid,
    output wire                    arready,
    output wire [ID_W-1:0]         rid,
    output wire [DATA_W-1:0]       rdata,
    output wire [1:0]              rresp,
    output wire                    rlast,
    output wire                    rvalid,
    input  wire                    rready,
    input  wire [4:0]              awatop,
    output wire [EXT_CMD_W-1:0]    req_r_data,
    output wire                    req_r_valid,
    input  wire                    req_r_ready,
    output wire [EXT_REQ_W-1:0]    req_w_data,
    output wire                    req_w_valid,
    input  wire                    req_w_ready,
    input  wire [DATA_W-1:0]       rsp_rd_data,
    input  wire                    rsp_rd_last,
    input  wire [1:0]              rsp_rd_resp,
    input  wire [ID_W-1:0]         rsp_rd_ext_txnid,
    input  wire                    rsp_rd_valid,
    output wire                    rsp_rd_ready,
    input  wire [1:0]              rsp_wr_resp,
    input  wire [ID_W-1:0]         rsp_wr_ext_txnid,
    input  wire                    rsp_wr_valid,
    output wire                    rsp_wr_ready
);
    adapter_mst_axi4 #(
        .ADDR_W(ADDR_W), .DATA_W(DATA_W), .LEN_W(LEN_W), .ID_W(ID_W),
        .NARROW_EN(NARROW_EN), .SAME_ID_EN(SAME_ID_EN), .SPLIT_EN(SPLIT_EN),
        .ATOMIC_EN(1), .PEND_TX(PEND_TX), .PEND_WR(PEND_WR),
        .R_SKID_DEPTH(R_SKID_DEPTH), .LB_MAX_BURST_BYTES(LB_MAX_BURST_BYTES),
        .AWATOP_W(AWATOP_W), .ATOMIC_FAIL_RESP(ATOMIC_FAIL_RESP),
        .MOD_W(MOD_W), .USER_CMD_W(0), .QOS_W(0)
    ) u_mst (
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
        .awatop(awatop),
        .req_r_data(req_r_data), .req_r_valid(req_r_valid), .req_r_ready(req_r_ready),
        .req_w_data(req_w_data), .req_w_valid(req_w_valid), .req_w_ready(req_w_ready),
        .rsp_rd_data(rsp_rd_data), .rsp_rd_last(rsp_rd_last), .rsp_rd_resp(rsp_rd_resp),
        .rsp_rd_ext_txnid(rsp_rd_ext_txnid), .rsp_rd_valid(rsp_rd_valid),
        .rsp_rd_ready(rsp_rd_ready),
        .rsp_wr_resp(rsp_wr_resp), .rsp_wr_ext_txnid(rsp_wr_ext_txnid),
        .rsp_wr_valid(rsp_wr_valid), .rsp_wr_ready(rsp_wr_ready),
        .req_r_addition(), .req_w_addition(),
        .rsp_rd_addition({(LANE_W+1){1'b0}}),
        .qreqn(1'b1), .reg_qdeny_en(1'b0), .reg_err_en(1'b0),
        .qacceptn(), .qdeny(), .qactive(), .lbus_pwrdn()
    );

endmodule
