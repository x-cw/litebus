//============================================================================
// Filename    : adapter_mst_apb.v
// Description : APB4 slave -> LiteBus INIU. Single-beat (len=0). Capture on
//               SETUP; PREADY only in ACCESS after LiteBus RSP. QCH always on.
//============================================================================
`include "adapter_defs.vh"
`default_nettype none
`timescale 1ns/1ps

module adapter_mst_apb #(
    parameter ADDR_W = 32,
    parameter DATA_W = 32,
    parameter LEN_W  = 8,
    parameter PSTRB_EN = 1,
    parameter W_BYTES = DATA_W/8,
    parameter ID_W    = 1,
    parameter QOS_PW  = 1,
    parameter USER_CMD_PW = 1,
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
    input  wire                   psel,
    input  wire                   penable,
    input  wire                   pwrite,
    input  wire [ADDR_W-1:0]      paddr,
    input  wire [DATA_W-1:0]      pwdata,
    input  wire [W_BYTES-1:0]     pstrb,
    output wire [DATA_W-1:0]      prdata,
    output wire                   pready,
    output wire                   pslverr,
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
    localparam S_IDLE = 2'd0;
    localparam S_REQ  = 2'd1;
    localparam S_WAIT = 2'd2;
    localparam S_HOLD = 2'd3;

    reg [1:0]            state;
    reg                  is_write;
    reg [ADDR_W-1:0]     addr_q;
    reg [DATA_W-1:0]     wdata_q;
    reg [W_BYTES-1:0]    wstrb_q;
    reg [DATA_W-1:0]     rdata_q;
    reg [1:0]            resp_q;

    wire [EXT_CMD_W-1:0] cmd_rd = {
        {QOS_PW{1'b0}}, `LB_OP_RD, addr_q, {LEN_W{1'b0}},
        {ID_W{1'b0}}, {USER_CMD_PW{1'b0}}
    };
    wire [EXT_CMD_W-1:0] cmd_wr = {
        {QOS_PW{1'b0}}, `LB_OP_WR, addr_q, {LEN_W{1'b0}},
        {ID_W{1'b0}}, {USER_CMD_PW{1'b0}}
    };
    wire [W_BYTES-1:0] wstrb_eff = PSTRB_EN ? wstrb_q : {W_BYTES{1'b1}};
    wire [EXT_WD_W-1:0] wd = {wdata_q, wstrb_eff, 1'b1, {ID_W{1'b0}}};

    assign req_r_data  = cmd_rd;
    assign req_r_valid = (state == S_REQ) && !is_write;
    assign req_w_data  = {cmd_wr, {MOD_PW{1'b0}}, wd};
    assign req_w_valid = (state == S_REQ) && is_write;

    reg pend_lbus;
    assign rsp_rd_ready = ((state == S_WAIT) && !is_write) ||
                          (pend_lbus && !is_write);
    assign rsp_wr_ready = ((state == S_WAIT) && is_write) ||
                          (pend_lbus && is_write);
    wire rd_hit = (state == S_WAIT) && !is_write && rsp_rd_valid;
    wire wr_hit = (state == S_WAIT) && is_write && rsp_wr_valid;

    assign pready  = (state == S_HOLD) && psel && penable;
    assign prdata  = rdata_q;
    assign pslverr = (state == S_HOLD) && (resp_q != `LB_RESP_OK);

    wire busy = (state != S_IDLE);
    wire q_quiesce;
    wire q_err_mode;
`ifdef ADP_QCH_TO
    wire q_to_err;
    adapter_qch #(.HAS_QDENY(1), .QCH_TO_W(QCH_TO_W)) u_qch (
        .clk(clk), .rst_n(rst_n),
        .qreqn(qreqn), .reg_qdeny_en(reg_qdeny_en), .reg_err_en(reg_err_en),
        .busy(busy),
        .qacceptn(qacceptn), .qdeny(qdeny), .qactive(qactive),
        .lbus_pwrdn(lbus_pwrdn), .o_quiesce(q_quiesce), .o_err_mode(q_err_mode),
        .qch_to_en(qch_to_en), .qch_to_lim(qch_to_lim), .qch_to_mode(qch_to_mode),
        .irq_qch_to(irq_qch_to), .o_to_err(q_to_err)
    );
`else
    wire q_to_err = 1'b0;
    adapter_qch #(.HAS_QDENY(1)) u_qch (
        .clk(clk), .rst_n(rst_n),
        .qreqn(qreqn), .reg_qdeny_en(reg_qdeny_en), .reg_err_en(reg_err_en),
        .busy(busy),
        .qacceptn(qacceptn), .qdeny(qdeny), .qactive(qactive),
        .lbus_pwrdn(lbus_pwrdn), .o_quiesce(q_quiesce), .o_err_mode(q_err_mode)
    );
`endif

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state    <= S_IDLE;
            is_write <= 1'b0;
            addr_q   <= {ADDR_W{1'b0}};
            wdata_q  <= {DATA_W{1'b0}};
            wstrb_q  <= {W_BYTES{1'b0}};
            rdata_q  <= {DATA_W{1'b0}};
            resp_q   <= 2'b00;
            pend_lbus<= 1'b0;
        end else begin
            if (pend_lbus && ((is_write && rsp_wr_valid) ||
                              (!is_write && rsp_rd_valid)))
                pend_lbus <= 1'b0;
            case (state)
            S_IDLE: begin
                if (q_err_mode && psel && !penable) begin
                    addr_q   <= paddr;
                    is_write <= pwrite;
                    wdata_q  <= pwdata;
                    wstrb_q  <= pstrb;
                    rdata_q  <= {DATA_W{1'b0}};
                    resp_q   <= `LB_RESP_FAIL;
                    state    <= S_HOLD;
                end else if (!q_quiesce && !lbus_pwrdn && psel && !penable) begin
                    addr_q   <= paddr;
                    is_write <= pwrite;
                    wdata_q  <= pwdata;
                    wstrb_q  <= pstrb;
                    state    <= S_REQ;
                end
            end
            S_REQ: begin
                if (!is_write && req_r_ready)
                    state <= S_WAIT;
                else if (is_write && req_w_ready)
                    state <= S_WAIT;
            end
            S_WAIT: begin
                if (q_to_err) begin
                    rdata_q   <= {DATA_W{1'b0}};
                    resp_q    <= `LB_RESP_FAIL;
                    pend_lbus <= !(rd_hit || wr_hit);
                    state     <= S_HOLD;
                end else if (rd_hit) begin
                    rdata_q <= rsp_rd_data;
                    resp_q  <= rsp_rd_resp;
                    state   <= S_HOLD;
                end else if (wr_hit) begin
                    resp_q <= rsp_wr_resp;
                    state  <= S_HOLD;
                end
            end
            S_HOLD: begin
                if (!psel)
                    state <= S_IDLE;
            end
            default: state <= S_IDLE;
            endcase
        end
    end

endmodule
