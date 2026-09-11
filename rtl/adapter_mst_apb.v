//============================================================================
// Filename    : adapter_mst_apb.v
// Description : APB4 slave -> LiteBus INIU IP (L0). Issue LiteBus as soon as
//               SETUP is captured; PREADY only in ACCESS after RSP.
//============================================================================
`include "adapter_ip_defs.vh"
`default_nettype none
`timescale 1ns/1ps

module adapter_mst_apb #(
    parameter ADDR_W = 32,
    parameter DATA_W = 32,
    parameter LEN_W  = 8,
    parameter PSTRB_EN = 1,
    parameter QCH_EN = 0,
    parameter W_BYTES = DATA_W/8,
    parameter ID_W    = 1,
    parameter QOS_PW  = 1,
    parameter USER_CMD_PW = 1,
    parameter MOD_PW  = 1,
    parameter EXT_CMD_W = QOS_PW + 4 + ADDR_W + LEN_W + ID_W + USER_CMD_PW,
    parameter EXT_WD_W  = DATA_W + W_BYTES + 1 + ID_W,
    parameter EXT_REQ_W = EXT_CMD_W + MOD_PW + EXT_WD_W
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
);
    localparam S_IDLE = 2'd0;
    localparam S_REQ  = 2'd1;
    localparam S_WAIT = 2'd2;
    localparam S_HOLD = 2'd3;

    reg [1:0] state;
    reg       is_write;
    reg [ADDR_W-1:0] addr_q;
    reg [DATA_W-1:0] wdata_q;
    reg [W_BYTES-1:0] wstrb_q;
    reg [DATA_W-1:0] rdata_q;
    reg [1:0]   resp_q;
    reg         have_rsp;

    wire [EXT_CMD_W-1:0] cmd_rd = { {QOS_PW{1'b0}}, `LB_OP_RD, addr_q, {(LEN_W){1'b0}}, {ID_W{1'b0}}, {USER_CMD_PW{1'b0}} };
    wire [EXT_CMD_W-1:0] cmd_wr = { {QOS_PW{1'b0}}, `LB_OP_WR, addr_q, {(LEN_W){1'b0}}, {ID_W{1'b0}}, {USER_CMD_PW{1'b0}} };
    wire [W_BYTES-1:0] wstrb_eff = PSTRB_EN ? wstrb_q : {W_BYTES{1'b1}};
    wire [EXT_WD_W-1:0] wd = {wdata_q, wstrb_eff, 1'b1, {ID_W{1'b0}}};
    wire [EXT_REQ_W-1:0] reqw = {cmd_wr, {MOD_PW{1'b0}}, wd};

    // Pulse LiteBus request for one handshake; wait for RSP in S_WAIT so a
    // multi-cycle slave does not see req_valid held and enqueue duplicates.
    assign req_r_data  = cmd_rd;
    assign req_r_valid = (state == S_REQ) && !is_write;
    assign req_w_data  = reqw;
    assign req_w_valid = (state == S_REQ) && is_write;

    assign rsp_rd_ready = (state == S_WAIT) && !is_write;
    assign rsp_wr_ready = (state == S_WAIT) && is_write;
    wire rd_hit = (state == S_WAIT) && !is_write && rsp_rd_valid;
    wire wr_hit = (state == S_WAIT) && is_write && rsp_wr_valid;

    assign pready  = (state == S_HOLD) && psel && penable;
    assign prdata  = rdata_q;
    assign pslverr = (state == S_HOLD) && (resp_q != `LB_RESP_OK);

    wire busy = (state != S_IDLE);
    wire q_quiesce;
    wire q_err_mode;
    generate
    if (QCH_EN) begin : g_qch
        adapter_qch #(.HAS_QDENY(1)) u_qch (
            .clk(clk), .rst_n(rst_n),
            .qreqn(qreqn), .reg_qdeny_en(reg_qdeny_en), .reg_err_en(reg_err_en),
            .busy(busy),
            .qacceptn(qacceptn), .qdeny(qdeny), .qactive(qactive),
            .lbus_pwrdn(lbus_pwrdn), .o_quiesce(q_quiesce), .o_err_mode(q_err_mode)
        );
    end else begin : g_noqch
        assign q_quiesce  = 1'b0;
        assign q_err_mode = 1'b0;
        assign qacceptn   = 1'b1;
        assign qdeny      = 1'b0;
        assign qactive    = 1'b0;
        assign lbus_pwrdn = 1'b0;
    end
    endgenerate

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state    <= S_IDLE;
            is_write <= 1'b0;
            addr_q   <= {ADDR_W{1'b0}};
            wdata_q  <= {DATA_W{1'b0}};
            wstrb_q  <= {W_BYTES{1'b0}};
            rdata_q  <= {DATA_W{1'b0}};
            resp_q   <= 2'b00;
            have_rsp <= 1'b0;
        end else begin
            case (state)
            S_IDLE: begin
                have_rsp <= 1'b0;
                // Capture only SETUP (PSEL && !PENABLE). ACCESS leftover after
                // S_HOLD would otherwise re-issue the same transfer.
                if (q_err_mode && psel && !penable) begin
                    addr_q   <= paddr;
                    is_write <= pwrite;
                    wdata_q  <= pwdata;
                    wstrb_q  <= pstrb;
                    rdata_q  <= {DATA_W{1'b0}};
                    resp_q   <= `LB_RESP_FAIL;
                    have_rsp <= 1'b1;
                    state    <= S_HOLD;
                end else if (!q_quiesce && psel && !penable) begin
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
                if (rd_hit) begin
                    rdata_q  <= rsp_rd_data;
                    resp_q   <= rsp_rd_resp;
                    have_rsp <= 1'b1;
                    state    <= S_HOLD;
                end else if (wr_hit) begin
                    resp_q   <= rsp_wr_resp;
                    have_rsp <= 1'b1;
                    state    <= S_HOLD;
                end
            end
            S_HOLD: begin
                // Stay until the master drops PSEL so ACCESS leftover cannot
                // look like a new SETUP, and PREADY remains high for ACCESS.
                if (!psel)
                    state <= S_IDLE;
            end
            default: state <= S_IDLE;
            endcase
        end
    end

endmodule
