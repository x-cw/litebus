//============================================================================
// Filename    : adapter_slv_apb.v
// Description : LiteBus TNIU -> APB4 master. len must be 0. CMD sliced past
//               MOD_PW. QCH + ip_fail; ACCESS holds until LiteBus RSP ready.
//============================================================================
`include "adapter_defs.vh"
`default_nettype none
`timescale 1ns/1ps

module adapter_slv_apb #(
    parameter ADDR_W = 32,
    parameter DATA_W = 32,
    parameter LEN_W  = 8,
    parameter W_BYTES = DATA_W/8,
    parameter ID_W    = 1,
    parameter QOS_PW  = 1,
    parameter USER_CMD_PW = 1,
    parameter USER_RSP_PW = 1,
    parameter MOD_PW  = 1,
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
`ifdef ADP_QCH_TO
    , input  wire                 qch_to_en
    , input  wire [QCH_TO_W-1:0]  qch_to_lim
    , input  wire                 qch_to_mode
    , output wire                 irq_qch_to
`endif
);
    localparam OP_HI   = EXT_CMD_W - QOS_PW - 1;
    localparam ADDR_HI = EXT_CMD_W - QOS_PW - 4 - 1;
    localparam LEN_HI  = EXT_CMD_W - QOS_PW - 4 - ADDR_W - 1;
    localparam TXN_HI  = EXT_CMD_W - QOS_PW - 4 - ADDR_W - LEN_W - 1;
    localparam S_IDLE   = 2'd0;
    localparam S_SETUP  = 2'd1;
    localparam S_ACCESS = 2'd2;
    localparam S_RSP    = 2'd3;

    reg [1:0]         state;
    wire                   q_u0, q_u1;
`ifdef ADP_QCH_TO
    wire q_to_err;
    adapter_qch #(.HAS_QDENY(0), .QCH_TO_W(QCH_TO_W)) u_qch (
        .clk(clk), .rst_n(rst_n),
        .qreqn(qreqn), .reg_qdeny_en(1'b0), .reg_err_en(1'b0),
        .busy(state != S_IDLE),
        .qacceptn(qacceptn), .qdeny(qdeny), .qactive(qactive),
        .lbus_pwrdn(lbus_pwrdn), .o_quiesce(q_u0), .o_err_mode(q_u1),
        .qch_to_en(qch_to_en), .qch_to_lim(qch_to_lim), .qch_to_mode(qch_to_mode),
        .irq_qch_to(irq_qch_to), .o_to_err(q_to_err)
    );
`else
    wire q_to_err = 1'b0;
    adapter_qch #(.HAS_QDENY(0)) u_qch (
        .clk(clk), .rst_n(rst_n),
        .qreqn(qreqn), .reg_qdeny_en(1'b0), .reg_err_en(1'b0),
        .busy(state != S_IDLE),
        .qacceptn(qacceptn), .qdeny(qdeny), .qactive(qactive),
        .lbus_pwrdn(lbus_pwrdn), .o_quiesce(q_u0), .o_err_mode(q_u1)
    );
`endif
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
        .d_rsp_rd_resp(s_rsp_rd_resp), .d_rsp_rd_ext_txnid(s_rsp_rd_txnid),
        .d_rsp_rd_user(s_rsp_rd_user), .d_rsp_rd_valid(s_rsp_rd_valid),
        .d_rsp_rd_ready(s_rsp_rd_ready),
        .d_rsp_wr_resp(s_rsp_wr_resp), .d_rsp_wr_ext_txnid(s_rsp_wr_txnid),
        .d_rsp_wr_user(s_rsp_wr_user), .d_rsp_wr_valid(s_rsp_wr_valid),
        .d_rsp_wr_ready(s_rsp_wr_ready)
    );

    wire [EXT_CMD_W+MOD_PW-1:0] wr_pack = s_req_w_data[EXT_WD_W +: (EXT_CMD_W+MOD_PW)];
    wire [EXT_CMD_W-1:0]        wr_cmd  = wr_pack[MOD_PW +: EXT_CMD_W];
    wire [EXT_WD_W-1:0]         wr_wd   = s_req_w_data[0 +: EXT_WD_W];

    wire [ADDR_W-1:0] rr_addr   = s_req_r_data[ADDR_HI -: ADDR_W];
    wire [ID_W-1:0]   rr_txnid  = s_req_r_data[TXN_HI -: ID_W];
    wire [ADDR_W-1:0] cmd_addr  = wr_cmd[ADDR_HI -: ADDR_W];
    wire [ID_W-1:0]   cmd_txnid = wr_cmd[TXN_HI -: ID_W];
    wire [DATA_W-1:0] wd_data   = wr_wd[W_BYTES + 1 + ID_W +: DATA_W];
    wire [W_BYTES-1:0] wd_strb  = wr_wd[ID_W + 1 +: W_BYTES];

    reg               is_write;
    reg [ADDR_W-1:0]  addr_q;
    reg [DATA_W-1:0]  wdata_q;
    reg [W_BYTES-1:0] wstrb_q;
    reg [ID_W-1:0]    txnid_q;
    reg [DATA_W-1:0]  rdata_q;
    reg [1:0]         resp_q;

    assign s_req_r_ready = (state == S_IDLE) && !s_req_w_valid && !q_u0 && !lbus_pwrdn;
    assign s_req_w_ready = (state == S_IDLE) && !q_u0 && !lbus_pwrdn;

    assign psel    = (state == S_SETUP) || (state == S_ACCESS);
    assign penable = (state == S_ACCESS);
    assign pwrite  = is_write;
    assign paddr   = addr_q;
    assign pwdata  = wdata_q;
    assign pstrb   = wstrb_q;

    assign s_rsp_rd_data   = rdata_q;
    assign s_rsp_rd_last   = 1'b1;
    assign s_rsp_rd_resp   = resp_q;
    assign s_rsp_rd_txnid  = txnid_q;
    assign s_rsp_rd_user   = {USER_RSP_PW{1'b0}};
    assign s_rsp_rd_valid  = (state == S_RSP) && !is_write;
    assign s_rsp_wr_resp   = resp_q;
    assign s_rsp_wr_txnid  = txnid_q;
    assign s_rsp_wr_user   = {USER_RSP_PW{1'b0}};
    assign s_rsp_wr_valid  = (state == S_RSP) && is_write;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state    <= S_IDLE;
            is_write <= 1'b0;
            addr_q   <= {ADDR_W{1'b0}};
            wdata_q  <= {DATA_W{1'b0}};
            wstrb_q  <= {W_BYTES{1'b0}};
            txnid_q  <= {ID_W{1'b0}};
            rdata_q  <= {DATA_W{1'b0}};
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
                if (q_to_err) begin
                    rdata_q <= {DATA_W{1'b0}};
                    resp_q  <= `LB_RESP_FAIL;
                    state   <= S_RSP;
                end else
                    state <= S_ACCESS;
            end
            S_ACCESS: begin
                if (q_to_err) begin
                    rdata_q <= {DATA_W{1'b0}};
                    resp_q  <= `LB_RESP_FAIL;
                    state   <= S_RSP;
                end else if (pready) begin
                    rdata_q <= prdata;
                    resp_q  <= pslverr ? `LB_RESP_FAIL : `LB_RESP_OK;
                    state   <= S_RSP;
                end
            end
            S_RSP: begin
                if (is_write && s_rsp_wr_ready)
                    state <= S_IDLE;
                else if (!is_write && s_rsp_rd_ready)
                    state <= S_IDLE;
            end
            default: state <= S_IDLE;
            endcase
        end
    end

endmodule
