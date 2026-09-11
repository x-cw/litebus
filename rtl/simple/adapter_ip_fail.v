//============================================================================
// Filename    : adapter_ip_fail.v
// Description : Same-clock FAIL mux while the slv domain is still clocked.
//               RD / WR / atomic (RSP_RD then RSP_WR). CMD sliced past MOD_PW.
//============================================================================
`include "adapter_defs.vh"
`default_nettype none
`timescale 1ns/1ps

module adapter_ip_fail #(
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
    input  wire                   intercept,
    input  wire [EXT_CMD_W-1:0]   u_req_r_data,
    input  wire                   u_req_r_valid,
    output wire                   u_req_r_ready,
    input  wire [EXT_REQ_W-1:0]   u_req_w_data,
    input  wire                   u_req_w_valid,
    output wire                   u_req_w_ready,
    output wire [DATA_W-1:0]      u_rsp_rd_data,
    output wire                   u_rsp_rd_last,
    output wire [1:0]             u_rsp_rd_resp,
    output wire [ID_W-1:0]        u_rsp_rd_ext_txnid,
    output wire [USER_RSP_PW-1:0] u_rsp_rd_user,
    output wire                   u_rsp_rd_valid,
    input  wire                   u_rsp_rd_ready,
    output wire [1:0]             u_rsp_wr_resp,
    output wire [ID_W-1:0]        u_rsp_wr_ext_txnid,
    output wire [USER_RSP_PW-1:0] u_rsp_wr_user,
    output wire                   u_rsp_wr_valid,
    input  wire                   u_rsp_wr_ready,
    output wire [EXT_CMD_W-1:0]   d_req_r_data,
    output wire                   d_req_r_valid,
    input  wire                   d_req_r_ready,
    output wire [EXT_REQ_W-1:0]   d_req_w_data,
    output wire                   d_req_w_valid,
    input  wire                   d_req_w_ready,
    input  wire [DATA_W-1:0]      d_rsp_rd_data,
    input  wire                   d_rsp_rd_last,
    input  wire [1:0]             d_rsp_rd_resp,
    input  wire [ID_W-1:0]        d_rsp_rd_ext_txnid,
    input  wire [USER_RSP_PW-1:0] d_rsp_rd_user,
    input  wire                   d_rsp_rd_valid,
    output wire                   d_rsp_rd_ready,
    input  wire [1:0]             d_rsp_wr_resp,
    input  wire [ID_W-1:0]        d_rsp_wr_ext_txnid,
    input  wire [USER_RSP_PW-1:0] d_rsp_wr_user,
    input  wire                   d_rsp_wr_valid,
    output wire                   d_rsp_wr_ready
);
    localparam TXN_HI = EXT_CMD_W - QOS_PW - 4 - ADDR_W - LEN_W - 1;
    localparam OP_HI  = EXT_CMD_W - QOS_PW - 1;
    localparam I_IDLE = 2'd0;
    localparam I_RD   = 2'd1;
    localparam I_WR   = 2'd2;
    localparam I_AT   = 2'd3;

    reg [1:0]      ist;
    reg [ID_W-1:0] itxn;
    reg            ineed_r;

    wire on = intercept || (ist != I_IDLE);

    wire [EXT_CMD_W+MOD_PW-1:0] wr_pack = u_req_w_data[EXT_WD_W +: (EXT_CMD_W+MOD_PW)];
    wire [EXT_CMD_W-1:0]        wr_cmd  = wr_pack[MOD_PW +: EXT_CMD_W];
    wire [ID_W-1:0]             wr_txn  = wr_cmd[TXN_HI -: ID_W];
    wire [3:0]                  wr_op   = wr_cmd[OP_HI -: 4];
    wire                        wr_last = u_req_w_data[ID_W];
    wire [ID_W-1:0]             rd_txn  = u_req_r_data[TXN_HI -: ID_W];
    wire wr_need_r = (wr_op == `LB_OP_ATOMIC_LOAD) ||
                     (wr_op == `LB_OP_ATOMIC_SWAP) ||
                     (wr_op == `LB_OP_ATOMIC_COMPARE);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ist     <= I_IDLE;
            itxn    <= {ID_W{1'b0}};
            ineed_r <= 1'b0;
        end else if (on) begin
            case (ist)
            I_IDLE: begin
                if (intercept && u_req_r_valid) begin
                    ist  <= I_RD;
                    itxn <= rd_txn;
                end else if (intercept && u_req_w_valid && wr_last) begin
                    ist     <= wr_need_r ? I_AT : I_WR;
                    itxn    <= wr_txn;
                    ineed_r <= wr_need_r;
                end
            end
            I_RD: if (u_rsp_rd_ready) ist <= I_IDLE;
            I_WR: if (u_rsp_wr_ready) ist <= I_IDLE;
            I_AT: begin
                if (u_rsp_rd_ready && ineed_r)
                    ineed_r <= 1'b0;
                else if (!ineed_r && u_rsp_wr_ready)
                    ist <= I_IDLE;
            end
            default: ist <= I_IDLE;
            endcase
        end else
            ist <= I_IDLE;
    end

    assign d_req_r_data  = u_req_r_data;
    assign d_req_w_data  = u_req_w_data;
    assign d_req_r_valid = on ? 1'b0 : u_req_r_valid;
    assign d_req_w_valid = on ? 1'b0 : u_req_w_valid;
    assign u_req_r_ready = on ? (ist == I_IDLE) : d_req_r_ready;
    assign u_req_w_ready = on ? 1'b1 : d_req_w_ready;

    assign u_rsp_rd_data      = on ? {DATA_W{1'b0}} : d_rsp_rd_data;
    assign u_rsp_rd_last      = on ? 1'b1 : d_rsp_rd_last;
    assign u_rsp_rd_resp      = on ? `LB_RESP_FAIL : d_rsp_rd_resp;
    assign u_rsp_rd_ext_txnid = on ? itxn : d_rsp_rd_ext_txnid;
    assign u_rsp_rd_user      = on ? {USER_RSP_PW{1'b0}} : d_rsp_rd_user;
    assign u_rsp_rd_valid     = on ? ((ist == I_RD) || (ist == I_AT && ineed_r))
                                   : d_rsp_rd_valid;
    assign d_rsp_rd_ready     = on ? 1'b0 : u_rsp_rd_ready;

    assign u_rsp_wr_resp      = on ? `LB_RESP_FAIL : d_rsp_wr_resp;
    assign u_rsp_wr_ext_txnid = on ? itxn : d_rsp_wr_ext_txnid;
    assign u_rsp_wr_user      = on ? {USER_RSP_PW{1'b0}} : d_rsp_wr_user;
    assign u_rsp_wr_valid     = on ? ((ist == I_WR) || (ist == I_AT && !ineed_r))
                                   : d_rsp_wr_valid;
    assign d_rsp_wr_ready     = on ? 1'b0 : u_rsp_wr_ready;

endmodule
