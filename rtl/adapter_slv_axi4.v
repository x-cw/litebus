//============================================================================
// Filename    : adapter_slv_axi4.v
// Description : adapter_slv top: LiteBus TNIU Slave IP (L0) -> AXI4 master.
//               SBS_EN=0 S1 direct; SBS_EN=1 S2 simple burst split.
//============================================================================
`include "adapter_ip_defs.vh"
`default_nettype none
`timescale 1ns/1ps

module adapter_slv_axi4 #(
    parameter ADDR_W  = 32,
    parameter DATA_W  = 64,
    parameter LEN_W   = 8,
    parameter ID_W    = 8,
    parameter SBS_EN  = 0,
    parameter SLV_MAX_LEN = 15,
    parameter USER_CMD_PW = 1,
    parameter USER_RSP_PW  = 1,
    parameter QOS_PW  = 1,
    parameter MOD_W   = 0,
    parameter QCH_EN  = 0,
    parameter ADDITION_EN = 0,
    parameter MOD_PW  = (MOD_W < 1) ? 1 : MOD_W,
    parameter W_BYTES = DATA_W/8,
    parameter LANE_W  = `adp_clog2(W_BYTES),
    parameter ADDITION_W = LANE_W + 1,
    parameter EXT_CMD_W = QOS_PW + 4 + ADDR_W + LEN_W + ID_W + USER_CMD_PW,
    parameter EXT_WD_W  = DATA_W + W_BYTES + 1 + ID_W,
    parameter EXT_REQ_W = EXT_CMD_W + MOD_PW + EXT_WD_W
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
    input  wire [ADDITION_W-1:0]  req_r_addition,
    input  wire [ADDITION_W-1:0]  req_w_addition,
    output wire [ADDITION_W-1:0]  rsp_rd_addition,
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
    input  wire                   reg_qdeny_en,
    output wire                   qacceptn,
    output wire                   qdeny,
    output wire                   qactive,
    output wire                   lbus_pwrdn,
    output wire                   intercept
);
    wire [EXT_CMD_W-1:0]   s_req_r_data;
    wire                   s_req_r_valid;
    wire                   s_req_r_ready;
    wire [EXT_REQ_W-1:0]   s_req_w_data;
    wire                   s_req_w_valid;
    wire                   s_req_w_ready;
    wire [DATA_W-1:0]      s_rsp_rd_data;
    wire                   s_rsp_rd_last;
    wire [1:0]             s_rsp_rd_resp;
    wire [ID_W-1:0]        s_rsp_rd_txnid;
    wire [USER_RSP_PW-1:0] s_rsp_rd_user;
    wire                   s_rsp_rd_valid;
    wire                   s_rsp_rd_ready;
    wire [1:0]             s_rsp_wr_resp;
    wire [ID_W-1:0]        s_rsp_wr_txnid;
    wire [USER_RSP_PW-1:0] s_rsp_wr_user;
    wire                   s_rsp_wr_valid;
    wire                   s_rsp_wr_ready;

    adapter_slv_prot #(
        .QCH_EN(QCH_EN), .ADDR_W(ADDR_W), .DATA_W(DATA_W), .LEN_W(LEN_W),
        .ID_W(ID_W), .USER_CMD_PW(USER_CMD_PW), .USER_RSP_PW(USER_RSP_PW),
        .QOS_PW(QOS_PW), .MOD_PW(MOD_PW)
    ) u_prot (
        .clk(clk), .rst_n(rst_n),
        .qreqn(qreqn), .qacceptn(qacceptn), .qdeny(qdeny), .qactive(qactive),
        .lbus_pwrdn(lbus_pwrdn), .intercept(intercept),
        .req_r_data(req_r_data), .req_r_valid(req_r_valid), .req_r_ready(req_r_ready),
        .req_w_data(req_w_data), .req_w_valid(req_w_valid), .req_w_ready(req_w_ready),
        .rsp_rd_data(rsp_rd_data), .rsp_rd_last(rsp_rd_last), .rsp_rd_resp(rsp_rd_resp),
        .rsp_rd_ext_txnid(rsp_rd_ext_txnid), .rsp_rd_user(rsp_rd_user),
        .rsp_rd_valid(rsp_rd_valid), .rsp_rd_ready(rsp_rd_ready),
        .rsp_wr_resp(rsp_wr_resp), .rsp_wr_ext_txnid(rsp_wr_ext_txnid),
        .rsp_wr_user(rsp_wr_user), .rsp_wr_valid(rsp_wr_valid),
        .rsp_wr_ready(rsp_wr_ready),
        .s_req_r_data(s_req_r_data), .s_req_r_valid(s_req_r_valid),
        .s_req_r_ready(s_req_r_ready),
        .s_req_w_data(s_req_w_data), .s_req_w_valid(s_req_w_valid),
        .s_req_w_ready(s_req_w_ready),
        .s_rsp_rd_data(s_rsp_rd_data), .s_rsp_rd_last(s_rsp_rd_last),
        .s_rsp_rd_resp(s_rsp_rd_resp), .s_rsp_rd_ext_txnid(s_rsp_rd_txnid),
        .s_rsp_rd_user(s_rsp_rd_user), .s_rsp_rd_valid(s_rsp_rd_valid),
        .s_rsp_rd_ready(s_rsp_rd_ready),
        .s_rsp_wr_resp(s_rsp_wr_resp), .s_rsp_wr_ext_txnid(s_rsp_wr_txnid),
        .s_rsp_wr_user(s_rsp_wr_user), .s_rsp_wr_valid(s_rsp_wr_valid),
        .s_rsp_wr_ready(s_rsp_wr_ready)
    );

    adapter_sbs #(
        .ADDR_W(ADDR_W), .DATA_W(DATA_W), .LEN_W(LEN_W), .ID_W(ID_W),
        .SBS_EN(SBS_EN), .SLV_MAX_LEN(SLV_MAX_LEN),
        .USER_CMD_PW(USER_CMD_PW), .USER_RSP_PW(USER_RSP_PW), .QOS_PW(QOS_PW),
        .MOD_W(MOD_W)
    ) u_sbs (
        .clk(clk), .rst_n(rst_n),
        .req_r_data(s_req_r_data), .req_r_valid(s_req_r_valid), .req_r_ready(s_req_r_ready),
        .req_w_data(s_req_w_data), .req_w_valid(s_req_w_valid), .req_w_ready(s_req_w_ready),
        .rsp_rd_data(s_rsp_rd_data), .rsp_rd_last(s_rsp_rd_last),
        .rsp_rd_resp(s_rsp_rd_resp), .rsp_rd_ext_txnid(s_rsp_rd_txnid),
        .rsp_rd_user(s_rsp_rd_user), .rsp_rd_valid(s_rsp_rd_valid),
        .rsp_rd_ready(s_rsp_rd_ready),
        .rsp_wr_resp(s_rsp_wr_resp), .rsp_wr_ext_txnid(s_rsp_wr_txnid),
        .rsp_wr_user(s_rsp_wr_user), .rsp_wr_valid(s_rsp_wr_valid),
        .rsp_wr_ready(s_rsp_wr_ready),
        .arid(arid), .araddr(araddr), .arlen(arlen), .arsize(arsize),
        .arburst(arburst), .arvalid(arvalid), .arready(arready),
        .rid(rid), .rdata(rdata), .rresp(rresp), .rlast(rlast),
        .rvalid(rvalid), .rready(rready),
        .awid(awid), .awaddr(awaddr), .awlen(awlen), .awsize(awsize),
        .awburst(awburst), .awvalid(awvalid), .awready(awready),
        .wdata(wdata), .wstrb(wstrb), .wlast(wlast), .wvalid(wvalid),
        .wready(wready),
        .bid(bid), .bresp(bresp), .bvalid(bvalid), .bready(bready),
        .awatop()
    );

    // Addition echo: latch on REQ, return on last RSP_RD (D21). Unused by
    // frozen LiteBus; ADDITION_EN=0 drives 0.
    reg [ADDITION_W-1:0] add_r, add_w;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            add_r <= {ADDITION_W{1'b0}};
            add_w <= {ADDITION_W{1'b0}};
        end else begin
            if (s_req_r_valid && s_req_r_ready)
                add_r <= req_r_addition;
            if (s_req_w_valid && s_req_w_ready)
                add_w <= req_w_addition;
        end
    end
    assign rsp_rd_addition = ADDITION_EN
        ? (s_rsp_rd_last ? add_r : {ADDITION_W{1'b0}})
        : {ADDITION_W{1'b0}};

endmodule
