//============================================================================
// Filename    : adapter_slv_apb.v
// Description : LiteBus TNIU Slave IP (L0) -> APB master. len must be 0.
//============================================================================
`include "adapter_ip_defs.vh"
`default_nettype none
`timescale 1ns/1ps

module adapter_slv_apb #(
    parameter ADDR_W = 32,
    parameter DATA_W = 32,
    parameter LEN_W  = 8,
    parameter QCH_EN = 0,
    parameter W_BYTES = DATA_W/8,
    parameter ID_W    = 1,
    parameter QOS_PW  = 1,
    parameter USER_CMD_PW = 1,
    parameter USER_RSP_PW  = 1,
    parameter MOD_PW  = 1,
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
    output wire                   psel,
    output wire                   penable,
    output wire                   pwrite,
    output wire [ADDR_W-1:0]      paddr,
    output wire [DATA_W-1:0]      pwdata,
    output wire [W_BYTES-1:0]     pstrb,
    input  wire [DATA_W-1:0]      prdata,
    input  wire                   pready,
    input  wire                   pslverr,
    input  wire                   qreqn,
    output wire                   qacceptn,
    output wire                   qdeny,
    output wire                   qactive,
    output wire                   lbus_pwrdn,
    output wire                   intercept
);
    localparam OP_HI   = EXT_CMD_W - QOS_PW - 1;
    localparam ADDR_HI = EXT_CMD_W - QOS_PW - 4 - 1;
    localparam LEN_HI  = EXT_CMD_W - QOS_PW - 4 - ADDR_W - 1;
    localparam TXN_HI  = EXT_CMD_W - QOS_PW - 4 - ADDR_W - LEN_W - 1;

    wire [EXT_CMD_W-1:0] s_req_r_data;
    wire                 s_req_r_valid;
    wire                 s_req_r_ready;
    wire [EXT_REQ_W-1:0] s_req_w_data;
    wire                 s_req_w_valid;
    wire                 s_req_w_ready;
    wire [DATA_W-1:0]    s_rsp_rd_data;
    wire                 s_rsp_rd_last;
    wire [1:0]           s_rsp_rd_resp;
    wire [ID_W-1:0]      s_rsp_rd_txnid;
    wire [USER_RSP_PW-1:0] s_rsp_rd_user;
    wire                 s_rsp_rd_valid;
    wire                 s_rsp_rd_ready;
    wire [1:0]           s_rsp_wr_resp;
    wire [ID_W-1:0]      s_rsp_wr_txnid;
    wire [USER_RSP_PW-1:0] s_rsp_wr_user;
    wire                 s_rsp_wr_valid;
    wire                 s_rsp_wr_ready;

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

    wire [EXT_CMD_W-1:0] wr_cmd = s_req_w_data[EXT_WD_W +: EXT_CMD_W];
    wire [EXT_WD_W-1:0]  wr_wd  = s_req_w_data[0 +: EXT_WD_W];

    wire [ADDR_W-1:0] rr_addr  = s_req_r_data[ADDR_HI -: ADDR_W];
    wire [LEN_W-1:0]  rr_len   = s_req_r_data[LEN_HI -: LEN_W];
    wire [ID_W-1:0]   rr_txnid = s_req_r_data[TXN_HI -: ID_W];
    wire [ADDR_W-1:0] cmd_addr = wr_cmd[ADDR_HI -: ADDR_W];
    wire [LEN_W-1:0]  cmd_len  = wr_cmd[LEN_HI -: LEN_W];
    wire [ID_W-1:0]   cmd_txnid= wr_cmd[TXN_HI -: ID_W];
    wire [DATA_W-1:0] wd_data  = wr_wd[W_BYTES + 1 + ID_W +: DATA_W];
    wire [W_BYTES-1:0] wd_strb = wr_wd[ID_W + 1 +: W_BYTES];

    localparam S_IDLE   = 3'd0;
    localparam S_SETUP  = 3'd1;
    localparam S_ACCESS = 3'd2;

    reg [2:0] state;
    reg       is_write;
    reg [ADDR_W-1:0] addr_q;
    reg [DATA_W-1:0] wdata_q;
    reg [W_BYTES-1:0] wstrb_q;
    reg [ID_W-1:0]  txnid_q;
    reg [1:0]       resp_q;

    assign s_req_r_ready = (state == S_IDLE) && !s_req_w_valid;
    assign s_req_w_ready = (state == S_IDLE);

    assign psel    = (state == S_SETUP) || (state == S_ACCESS);
    assign penable = (state == S_ACCESS);
    assign pwrite  = is_write;
    assign paddr   = addr_q;
    assign pwdata  = wdata_q;
    assign pstrb   = wstrb_q;

    assign s_rsp_rd_data       = prdata;
    assign s_rsp_rd_last       = 1'b1;
    assign s_rsp_rd_resp       = resp_q;
    assign s_rsp_rd_txnid      = txnid_q;
    assign s_rsp_rd_user       = {USER_RSP_PW{1'b0}};
    assign s_rsp_rd_valid      = (state == S_ACCESS) && !is_write && pready;
    assign s_rsp_wr_resp       = resp_q;
    assign s_rsp_wr_txnid      = txnid_q;
    assign s_rsp_wr_user       = {USER_RSP_PW{1'b0}};
    assign s_rsp_wr_valid      = (state == S_ACCESS) && is_write && pready;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state    <= S_IDLE;
            is_write <= 1'b0;
            addr_q   <= {ADDR_W{1'b0}};
            wdata_q  <= {DATA_W{1'b0}};
            wstrb_q  <= {W_BYTES{1'b0}};
            txnid_q  <= {ID_W{1'b0}};
            resp_q   <= `LB_RESP_OK;
        end else begin
            case (state)
            S_IDLE: begin
                if (s_req_w_valid) begin
                    is_write <= 1'b1;
                    addr_q   <= cmd_addr;
                    txnid_q  <= cmd_txnid;
                    wdata_q  <= wd_data;
                    wstrb_q  <= wd_strb;
                    state    <= S_SETUP;
                end else if (s_req_r_valid) begin
                    is_write <= 1'b0;
                    addr_q   <= rr_addr;
                    txnid_q  <= rr_txnid;
                    state    <= S_SETUP;
                end
            end
            S_SETUP: begin
                state <= S_ACCESS;
            end
            S_ACCESS: begin
                if (pready) begin
                    resp_q <= pslverr ? `LB_RESP_FAIL : `LB_RESP_OK;
                    state  <= S_IDLE;
                end
            end
            default: state <= S_IDLE;
            endcase
        end
    end

`ifndef LB_NO_ASSERT
    always @(posedge clk) begin
        if (s_req_r_valid && (rr_len != 0))
            $display("[%0t] ERROR %m: APB slv requires len=0, got %0d", $time, rr_len);
        if (s_req_w_valid && (cmd_len != 0))
            $display("[%0t] ERROR %m: APB slv requires len=0, got %0d", $time, cmd_len);
    end
`endif

endmodule
