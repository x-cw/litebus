//============================================================================
// Filename    : adapter_slv_axi_core.v
// Description : Shared LiteBus -> AXI master datapath. HAS_ATOMIC=0 is AXI4
//               (no AWATOP). HAS_ATOMIC=1 drives o_awatop from opcode/mod.
//               Wrappers in adapter_slv_axi4/axi5.v expose public pins.
//============================================================================
`include "adapter_defs.vh"
`default_nettype none
`timescale 1ns/1ps

module adapter_slv_axi_core #(
    parameter ADDR_W = 32,
    parameter DATA_W = 64,
    parameter LEN_W  = 8,
    parameter ID_W   = 8,
    parameter HAS_ATOMIC = 0,
    parameter USER_CMD_PW = 1,
    parameter USER_RSP_PW = 1,
    parameter QOS_PW = 1,
    parameter MOD_PW = HAS_ATOMIC ? 3 : 1,
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
    output wire [4:0]             o_awatop,
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
    localparam LANE_W  = `adp_clog2(W_BYTES);
    localparam RD_PTR_W = (`adp_clog2(MAX_RD_OST) < 1) ? 1 : `adp_clog2(MAX_RD_OST);
    localparam WR_PTR_W = (`adp_clog2(MAX_WR_OST) < 1) ? 1 : `adp_clog2(MAX_WR_OST);
    localparam WR_CNT_W = `adp_clog2(MAX_WR_OST) + 1;
    localparam RD_CNT_W = `adp_clog2(MAX_RD_OST) + 1;
    localparam [RD_PTR_W-1:0] RD_PTR_LAST = MAX_RD_OST - 1;
    localparam [WR_PTR_W-1:0] WR_PTR_LAST = MAX_WR_OST - 1;

    function [ADDR_W-1:0] align_dn;
        input [ADDR_W-1:0] a;
        begin
            align_dn = a & ~{{(ADDR_W-LANE_W){1'b0}}, {LANE_W{1'b1}}};
        end
    endfunction

    function [1:0] map_resp;
        input [1:0] r;
        begin
            if ((r == `AXI_RESP_OKAY) || (r == `AXI_RESP_EXOKAY))
                map_resp = `LB_RESP_OK;
            else
                map_resp = `LB_RESP_FAIL;
        end
    endfunction

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

    wire q_quiesce;
    wire q_err_unused;

    wire [EXT_CMD_W+MOD_PW-1:0] ext_cmd_mod = s_req_w_data[EXT_WD_W +: (EXT_CMD_W+MOD_PW)];
    wire [EXT_CMD_W-1:0]        ext_cmd     = ext_cmd_mod[MOD_PW +: EXT_CMD_W];
    wire [EXT_WD_W-1:0]         ext_wd      = s_req_w_data[0 +: EXT_WD_W];

    wire [3:0]        cmd_op    = ext_cmd[OP_HI -: 4];
    wire [LEN_W-1:0]  cmd_len   = ext_cmd[LEN_HI -: LEN_W];
    wire [ID_W-1:0]   cmd_txnid = ext_cmd[TXN_HI -: ID_W];
    wire [ADDR_W-1:0] cmd_addr  = ext_cmd[ADDR_HI -: ADDR_W];
    wire              lb_wlast  = ext_wd[ID_W];
    wire [MOD_PW-1:0] cmd_mod   = ext_cmd_mod[0 +: MOD_PW];
    wire cmd_need_r = HAS_ATOMIC &&
                      ((cmd_op == `LB_OP_ATOMIC_LOAD) ||
                       (cmd_op == `LB_OP_ATOMIC_SWAP) ||
                       (cmd_op == `LB_OP_ATOMIC_COMPARE));

    wire [2:0] mod3 = (MOD_PW >= 3) ? cmd_mod[2:0] : {3{1'b0}};
    assign o_awatop = (HAS_ATOMIC && cmd_op[3]) ? {mod3, cmd_op[1:0]} : 5'b0;

    reg wr_first;
    reg wr_active;
    reg wr_wait_r;
    reg [ID_W-1:0] wr_txnid;
    reg [WR_CNT_W-1:0] wr_ost;
    reg [RD_CNT_W-1:0] rd_ost;
    reg [RD_CNT_W-1:0] rd_axi;
    reg [RD_CNT_W-1:0] rf_n;
    reg [RD_PTR_W-1:0] rf_w;
    reg [RD_PTR_W-1:0] rf_r;
    reg [ID_W-1:0]     rf_id   [0:MAX_RD_OST-1];
    reg [ADDR_W-1:0]   rf_addr [0:MAX_RD_OST-1];
    reg [LEN_W-1:0]    rf_len  [0:MAX_RD_OST-1];

`ifdef ADP_QCH_TO
    reg [ID_W-1:0]     wr_txn_q [0:MAX_WR_OST-1];
    reg [WR_PTR_W-1:0] wr_tq_w;
    reg [WR_PTR_W-1:0] wr_tq_r;
    reg [WR_CNT_W-1:0] wr_axi_pend;
    reg [ID_W-1:0]     rd_txn_q [0:MAX_RD_OST-1];
    reg [RD_PTR_W-1:0] rd_tq_w;
    reg [RD_PTR_W-1:0] rd_tq_r;
    wire q_to_err;
`else
    wire q_to_err = 1'b0;
`endif

    wire slv_busy = (rd_ost != {RD_CNT_W{1'b0}}) ||
                    (wr_ost != {WR_CNT_W{1'b0}}) || wr_active;

`ifdef ADP_QCH_TO
    adapter_qch #(.HAS_QDENY(0), .QCH_TO_W(QCH_TO_W)) u_qch (
        .clk(clk), .rst_n(rst_n),
        .qreqn(qreqn), .reg_qdeny_en(1'b0), .reg_err_en(1'b0),
        .busy(slv_busy),
        .qacceptn(qacceptn), .qdeny(qdeny), .qactive(qactive),
        .lbus_pwrdn(lbus_pwrdn), .o_quiesce(q_quiesce), .o_err_mode(q_err_unused),
        .qch_to_en(qch_to_en), .qch_to_lim(qch_to_lim), .qch_to_mode(qch_to_mode),
        .irq_qch_to(irq_qch_to), .o_to_err(q_to_err)
    );
`else
    adapter_qch #(.HAS_QDENY(0)) u_qch (
        .clk(clk), .rst_n(rst_n),
        .qreqn(qreqn), .reg_qdeny_en(1'b0), .reg_err_en(1'b0),
        .busy(slv_busy),
        .qacceptn(qacceptn), .qdeny(qdeny), .qactive(qactive),
        .lbus_pwrdn(lbus_pwrdn), .o_quiesce(q_quiesce), .o_err_mode(q_err_unused)
    );
`endif
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

    wire stall_new = q_quiesce || lbus_pwrdn || q_to_err;
    wire wr_ost_full = (wr_ost == MAX_WR_OST[WR_CNT_W-1:0]);
    wire rd_ost_full = (rd_ost == MAX_RD_OST[RD_CNT_W-1:0]);

    assign awid    = cmd_txnid;
    assign awaddr  = align_dn(cmd_addr);
    assign awlen   = cmd_len;
    assign awsize  = `adp_clog2(W_BYTES) & 3'h7;
    assign awburst = `AXI_BURST_INCR;
    assign awvalid = s_req_w_valid && wr_first && !wr_active && !wr_ost_full &&
                     !wr_wait_r && !stall_new && !(cmd_need_r && (rd_ost != {RD_CNT_W{1'b0}}));

    wire w_can = wr_active || (awvalid && awready);
    assign wdata  = ext_wd[W_BYTES + 1 + ID_W +: DATA_W];
    assign wstrb  = ext_wd[ID_W + 1 +: W_BYTES];
    assign wlast  = lb_wlast;
    assign wvalid = s_req_w_valid && w_can;
    assign s_req_w_ready = wr_active ? wready :
                           (stall_new || wr_ost_full) ? 1'b0 :
                           (awready && wready && !(cmd_need_r && (rd_ost != {RD_CNT_W{1'b0}})) &&
                            !wr_wait_r);

    wire loc_wr = q_to_err && (wr_ost != {WR_CNT_W{1'b0}});
    wire loc_rd = q_to_err && (rd_ost != {RD_CNT_W{1'b0}});
    wire loc_at = q_to_err && wr_wait_r;

`ifdef ADP_QCH_TO
    assign bready         = q_to_err ? (wr_axi_pend != {WR_CNT_W{1'b0}}) :
                            ((wr_ost != {WR_CNT_W{1'b0}}) && s_rsp_wr_ready);
    assign s_rsp_wr_valid = loc_wr ? 1'b1 : ((wr_ost != {WR_CNT_W{1'b0}}) && bvalid);
    assign s_rsp_wr_txnid = loc_wr ? wr_txn_q[wr_tq_r] : bid;
    assign s_rsp_wr_resp  = loc_wr ? `LB_RESP_FAIL : map_resp(bresp);
`else
    assign bready         = (wr_ost != {WR_CNT_W{1'b0}}) && s_rsp_wr_ready;
    assign s_rsp_wr_valid = (wr_ost != {WR_CNT_W{1'b0}}) && bvalid;
    assign s_rsp_wr_txnid = bid;
    assign s_rsp_wr_resp  = map_resp(bresp);
`endif
    assign s_rsp_wr_user  = {USER_RSP_PW{1'b0}};

    wire aw_hs = awvalid && awready;
    wire w_hs  = wvalid && wready && lb_wlast;
    wire b_hs  = bvalid && bready && !q_to_err;
    wire loc_wr_hs = loc_wr && s_rsp_wr_ready;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wr_first  <= 1'b1;
            wr_active <= 1'b0;
            wr_wait_r <= 1'b0;
            wr_txnid  <= {ID_W{1'b0}};
            wr_ost    <= {WR_CNT_W{1'b0}};
`ifdef ADP_QCH_TO
            wr_tq_w      <= {WR_PTR_W{1'b0}};
            wr_tq_r      <= {WR_PTR_W{1'b0}};
            wr_axi_pend  <= {WR_CNT_W{1'b0}};
`endif
        end else begin
            if (aw_hs) begin
                wr_active <= 1'b1;
                wr_txnid  <= cmd_txnid;
`ifdef ADP_QCH_TO
                wr_txn_q[wr_tq_w] <= cmd_txnid;
                wr_tq_w <= (wr_tq_w == WR_PTR_LAST) ? {WR_PTR_W{1'b0}} :
                           (wr_tq_w + 1'b1);
`endif
            end
            if (w_hs) begin
                wr_active <= 1'b0;
                wr_wait_r <= cmd_need_r;
            end
            if (s_req_w_valid && s_req_w_ready)
                wr_first <= lb_wlast;
            if (wr_wait_r && rvalid && rready && rlast && !q_to_err)
                wr_wait_r <= 1'b0;
            if (loc_at && s_rsp_rd_ready)
                wr_wait_r <= 1'b0;
            wr_ost <= wr_ost + (aw_hs ? 1'b1 : 1'b0) -
                      ((b_hs || loc_wr_hs) ? 1'b1 : 1'b0);
`ifdef ADP_QCH_TO
            if (b_hs || loc_wr_hs)
                wr_tq_r <= (wr_tq_r == WR_PTR_LAST) ? {WR_PTR_W{1'b0}} :
                           (wr_tq_r + 1'b1);
            wr_axi_pend <= wr_axi_pend + (aw_hs ? 1'b1 : 1'b0) -
                           ((bvalid && bready) ? 1'b1 : 1'b0);
`endif
        end
    end

    wire [LEN_W-1:0]  rr_len   = s_req_r_data[LEN_HI -: LEN_W];
    wire [ID_W-1:0]   rr_txnid = s_req_r_data[TXN_HI -: ID_W];
    wire [ADDR_W-1:0] rr_addr  = s_req_r_data[ADDR_HI -: ADDR_W];

    assign arid    = rf_id[rf_r];
    assign araddr  = align_dn(rf_addr[rf_r]);
    assign arlen   = rf_len[rf_r];
    assign arsize  = `adp_clog2(W_BYTES) & 3'h7;
    assign arburst = `AXI_BURST_INCR;
    assign arvalid = (rf_n != {RD_CNT_W{1'b0}}) && !wr_wait_r && !q_to_err;
    assign s_req_r_ready = !stall_new && !rd_ost_full && !wr_wait_r;

    wire rr_hs = s_req_r_valid && s_req_r_ready;
    wire ar_hs = arvalid && arready;
    wire rd_r_hs = rvalid && rready && rlast && (rd_axi != {RD_CNT_W{1'b0}}) &&
                   !wr_wait_r && !q_to_err;
    wire loc_rd_hs = loc_rd && s_rsp_rd_ready && !wr_wait_r;

    wire r_from_rd = rvalid && (rd_axi != {RD_CNT_W{1'b0}}) && !wr_wait_r && !q_to_err;
    wire r_from_at = rvalid && wr_wait_r && !q_to_err;
`ifdef ADP_QCH_TO
    assign s_rsp_rd_data  = (loc_rd || loc_at) ? {DATA_W{1'b0}} : rdata;
    assign s_rsp_rd_last  = (loc_rd || loc_at || wr_wait_r) ? 1'b1 : rlast;
    assign s_rsp_rd_resp  = (loc_rd || loc_at) ? `LB_RESP_FAIL : map_resp(rresp);
    assign s_rsp_rd_txnid = loc_at ? wr_txnid :
                            (loc_rd ? rd_txn_q[rd_tq_r] :
                             (wr_wait_r ? wr_txnid : rid));
    assign s_rsp_rd_valid = loc_rd || loc_at || r_from_rd || r_from_at;
    assign rready         = q_to_err ? rvalid :
                            ((rd_axi != {RD_CNT_W{1'b0}} && s_rsp_rd_ready) ||
                             (wr_wait_r && s_rsp_rd_ready));
`else
    assign s_rsp_rd_data  = rdata;
    assign s_rsp_rd_last  = wr_wait_r ? 1'b1 : rlast;
    assign s_rsp_rd_resp  = map_resp(rresp);
    assign s_rsp_rd_txnid = wr_wait_r ? wr_txnid : rid;
    assign s_rsp_rd_valid = r_from_rd || r_from_at;
    assign rready         = (rd_axi != {RD_CNT_W{1'b0}} && s_rsp_rd_ready) ||
                            (wr_wait_r && s_rsp_rd_ready);
`endif
    assign s_rsp_rd_user  = {USER_RSP_PW{1'b0}};

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rf_n   <= {RD_CNT_W{1'b0}};
            rf_w   <= {RD_PTR_W{1'b0}};
            rf_r   <= {RD_PTR_W{1'b0}};
            rd_ost <= {RD_CNT_W{1'b0}};
            rd_axi <= {RD_CNT_W{1'b0}};
`ifdef ADP_QCH_TO
            rd_tq_w <= {RD_PTR_W{1'b0}};
            rd_tq_r <= {RD_PTR_W{1'b0}};
`endif
        end else begin
            if (rr_hs) begin
                rf_id[rf_w]   <= rr_txnid;
                rf_addr[rf_w] <= rr_addr;
                rf_len[rf_w]  <= rr_len;
                rf_w <= (rf_w == RD_PTR_LAST) ? {RD_PTR_W{1'b0}} : (rf_w + 1'b1);
`ifdef ADP_QCH_TO
                rd_txn_q[rd_tq_w] <= rr_txnid;
                rd_tq_w <= (rd_tq_w == RD_PTR_LAST) ? {RD_PTR_W{1'b0}} :
                           (rd_tq_w + 1'b1);
`endif
            end
            if (ar_hs)
                rf_r <= (rf_r == RD_PTR_LAST) ? {RD_PTR_W{1'b0}} : (rf_r + 1'b1);
            rf_n   <= rf_n   + (rr_hs ? 1'b1 : 1'b0) - (ar_hs ? 1'b1 : 1'b0);
            rd_ost <= rd_ost + (rr_hs ? 1'b1 : 1'b0) -
                      ((rd_r_hs || loc_rd_hs) ? 1'b1 : 1'b0);
            rd_axi <= rd_axi + (ar_hs ? 1'b1 : 1'b0) -
                      ((rd_r_hs || (q_to_err && rvalid && rready && rlast &&
                        (rd_axi != {RD_CNT_W{1'b0}}) && !wr_wait_r)) ? 1'b1 : 1'b0);
`ifdef ADP_QCH_TO
            if (rd_r_hs || loc_rd_hs)
                rd_tq_r <= (rd_tq_r == RD_PTR_LAST) ? {RD_PTR_W{1'b0}} :
                           (rd_tq_r + 1'b1);
`endif
        end
    end

endmodule
