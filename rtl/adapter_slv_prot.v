//============================================================================
// Filename    : adapter_slv_prot.v
// Description : adapter_slv Q-Channel wrapper (QCH_EN). slv has no qdeny /
//               backpressure: qreqn=0 -> qacceptn=0 immediately (busy=0).
//               While lbus_pwrdn (domain still clocked) new fabric requests
//               are FAILed locally and not forwarded to AXI/APB (§10.4).
//============================================================================
`include "adapter_ip_defs.vh"
`default_nettype none
`timescale 1ns/1ps

module adapter_slv_prot #(
    parameter QCH_EN = 0,
    parameter ADDR_W = 32,
    parameter DATA_W = 64,
    parameter LEN_W  = 8,
    parameter ID_W   = 8,
    parameter USER_CMD_PW = 1,
    parameter USER_RSP_PW = 1,
    parameter QOS_PW = 1,
    parameter MOD_PW = 1,
    parameter W_BYTES = DATA_W/8,
    parameter EXT_CMD_W = QOS_PW + 4 + ADDR_W + LEN_W + ID_W + USER_CMD_PW,
    parameter EXT_WD_W  = DATA_W + W_BYTES + 1 + ID_W,
    parameter EXT_REQ_W = EXT_CMD_W + MOD_PW + EXT_WD_W
) (
    input  wire                   clk,
    input  wire                   rst_n,
    input  wire                   qreqn,
    output wire                   qacceptn,
    output wire                   qdeny,
    output wire                   qactive,
    output wire                   lbus_pwrdn,
    output wire                   intercept,
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
    output wire [EXT_CMD_W-1:0]   s_req_r_data,
    output wire                   s_req_r_valid,
    input  wire                   s_req_r_ready,
    output wire [EXT_REQ_W-1:0]   s_req_w_data,
    output wire                   s_req_w_valid,
    input  wire                   s_req_w_ready,
    input  wire [DATA_W-1:0]      s_rsp_rd_data,
    input  wire                   s_rsp_rd_last,
    input  wire [1:0]             s_rsp_rd_resp,
    input  wire [ID_W-1:0]        s_rsp_rd_ext_txnid,
    input  wire [USER_RSP_PW-1:0] s_rsp_rd_user,
    input  wire                   s_rsp_rd_valid,
    output wire                   s_rsp_rd_ready,
    input  wire [1:0]             s_rsp_wr_resp,
    input  wire [ID_W-1:0]        s_rsp_wr_ext_txnid,
    input  wire [USER_RSP_PW-1:0] s_rsp_wr_user,
    input  wire                   s_rsp_wr_valid,
    output wire                   s_rsp_wr_ready
);
    generate
    if (QCH_EN) begin : g_qch
        adapter_qch #(.HAS_QDENY(0)) u_qch (
            .clk(clk), .rst_n(rst_n),
            .qreqn(qreqn), .reg_qdeny_en(1'b0), .reg_err_en(1'b0),
            .busy(1'b0),
            .qacceptn(qacceptn), .qdeny(qdeny), .qactive(qactive),
            .lbus_pwrdn(lbus_pwrdn), .o_quiesce(), .o_err_mode()
        );
        assign intercept = lbus_pwrdn;
        adapter_ip_fail #(
            .ADDR_W(ADDR_W), .DATA_W(DATA_W), .LEN_W(LEN_W), .ID_W(ID_W),
            .USER_CMD_PW(USER_CMD_PW), .USER_RSP_PW(USER_RSP_PW),
            .QOS_PW(QOS_PW), .MOD_PW(MOD_PW)
        ) u_fail (
            .clk(clk), .rst_n(rst_n),
            .intercept(lbus_pwrdn),
            .u_req_r_data(req_r_data), .u_req_r_valid(req_r_valid),
            .u_req_r_ready(req_r_ready),
            .u_req_w_data(req_w_data), .u_req_w_valid(req_w_valid),
            .u_req_w_ready(req_w_ready),
            .u_rsp_rd_data(rsp_rd_data), .u_rsp_rd_last(rsp_rd_last),
            .u_rsp_rd_resp(rsp_rd_resp), .u_rsp_rd_ext_txnid(rsp_rd_ext_txnid),
            .u_rsp_rd_user(rsp_rd_user), .u_rsp_rd_valid(rsp_rd_valid),
            .u_rsp_rd_ready(rsp_rd_ready),
            .u_rsp_wr_resp(rsp_wr_resp), .u_rsp_wr_ext_txnid(rsp_wr_ext_txnid),
            .u_rsp_wr_user(rsp_wr_user), .u_rsp_wr_valid(rsp_wr_valid),
            .u_rsp_wr_ready(rsp_wr_ready),
            .d_req_r_data(s_req_r_data), .d_req_r_valid(s_req_r_valid),
            .d_req_r_ready(s_req_r_ready),
            .d_req_w_data(s_req_w_data), .d_req_w_valid(s_req_w_valid),
            .d_req_w_ready(s_req_w_ready),
            .d_rsp_rd_data(s_rsp_rd_data), .d_rsp_rd_last(s_rsp_rd_last),
            .d_rsp_rd_resp(s_rsp_rd_resp), .d_rsp_rd_ext_txnid(s_rsp_rd_ext_txnid),
            .d_rsp_rd_user(s_rsp_rd_user), .d_rsp_rd_valid(s_rsp_rd_valid),
            .d_rsp_rd_ready(s_rsp_rd_ready),
            .d_rsp_wr_resp(s_rsp_wr_resp), .d_rsp_wr_ext_txnid(s_rsp_wr_ext_txnid),
            .d_rsp_wr_user(s_rsp_wr_user), .d_rsp_wr_valid(s_rsp_wr_valid),
            .d_rsp_wr_ready(s_rsp_wr_ready)
        );
    end else begin : g_noqch
        assign qacceptn   = 1'b1;
        assign qdeny      = 1'b0;
        assign qactive    = 1'b0;
        assign lbus_pwrdn = 1'b0;
        assign intercept  = 1'b0;
        assign s_req_r_data  = req_r_data;
        assign s_req_r_valid = req_r_valid;
        assign req_r_ready   = s_req_r_ready;
        assign s_req_w_data  = req_w_data;
        assign s_req_w_valid = req_w_valid;
        assign req_w_ready   = s_req_w_ready;
        assign rsp_rd_data      = s_rsp_rd_data;
        assign rsp_rd_last      = s_rsp_rd_last;
        assign rsp_rd_resp      = s_rsp_rd_resp;
        assign rsp_rd_ext_txnid = s_rsp_rd_ext_txnid;
        assign rsp_rd_user      = s_rsp_rd_user;
        assign rsp_rd_valid     = s_rsp_rd_valid;
        assign s_rsp_rd_ready   = rsp_rd_ready;
        assign rsp_wr_resp      = s_rsp_wr_resp;
        assign rsp_wr_ext_txnid = s_rsp_wr_ext_txnid;
        assign rsp_wr_user      = s_rsp_wr_user;
        assign rsp_wr_valid     = s_rsp_wr_valid;
        assign s_rsp_wr_ready   = rsp_wr_ready;
    end
    endgenerate

endmodule
