//============================================================================
// Filename    : adapter_mst_axi_core.v
// Description : Shared AXI master-side datapath. HAS_ATOMIC=0 is AXI4;
//               HAS_ATOMIC=1 is AXI5 (AWATOP). Not an axi4 top — wrappers
//               in adapter_mst_axi4/axi5.v expose the public pins.
//               txnid = per-channel slot (read and write both start at 0).
//============================================================================
`include "adapter_defs.vh"
`default_nettype none
`timescale 1ns/1ps

module adapter_mst_axi_core #(
    parameter ADDR_W = 32,
    parameter DATA_W = 64,
    parameter LEN_W  = 8,
    parameter ID_W   = 8,
    parameter HAS_ATOMIC = 0,
    parameter ATOMIC_FAIL_RESP = `AXI_RESP_EXOKAY,
    parameter QOS_PW = 1,
    parameter USER_CMD_PW = 1,
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
    input  wire [4:0]             i_awatop,
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
    localparam RD_PTR_W = (`adp_clog2(MAX_RD_OST) < 1) ? 1 : `adp_clog2(MAX_RD_OST);
    localparam WR_PTR_W = (`adp_clog2(MAX_WR_OST) < 1) ? 1 : `adp_clog2(MAX_WR_OST);
    localparam RD_CNT_W = `adp_clog2(MAX_RD_OST) + 1;
    localparam WR_CNT_W = `adp_clog2(MAX_WR_OST) + 1;
    localparam [RD_PTR_W-1:0] RD_PTR_LAST = MAX_RD_OST - 1;
    localparam [WR_PTR_W-1:0] WR_PTR_LAST = MAX_WR_OST - 1;

    function [1:0] map_resp;
        input [1:0] r;
        begin
            if (r == `LB_RESP_OK)
                map_resp = `AXI_RESP_OKAY;
            else if (HAS_ATOMIC && (r == `LB_RESP_ATOMIC_FAIL))
                map_resp = ATOMIC_FAIL_RESP[1:0];
            else
                map_resp = `AXI_RESP_SLVERR;
        end
    endfunction

    wire [3:0]        at_opcode;
    wire              at_need_r;
    wire [MOD_PW-1:0] at_mod;

    generate
    if (HAS_ATOMIC) begin : g_at
        adapter_atomic #(.MOD_PW(MOD_PW)) u_at (
            .i_awatop(i_awatop),
            .o_opcode(at_opcode),
            .o_need_r(at_need_r),
            .o_mod(at_mod)
        );
    end else begin : g_noat
        assign at_opcode = `LB_OP_WR;
        assign at_need_r = 1'b0;
        assign at_mod    = {MOD_PW{1'b0}};
    end
    endgenerate

    wire aw_is_atomic   = HAS_ATOMIC && (|i_awatop);
    wire at_live_need_r = aw_is_atomic && at_need_r;

    reg [MAX_RD_OST-1:0] rd_v;
    reg [ID_W-1:0]       rd_axi_id [0:MAX_RD_OST-1];
    reg [ADDR_W-1:0]     rd_addr   [0:MAX_RD_OST-1];
    reg [LEN_W-1:0]      rd_len    [0:MAX_RD_OST-1];
    reg [RD_PTR_W-1:0]   rd_iss_q  [0:MAX_RD_OST-1];
    reg [RD_PTR_W-1:0]   rd_iss_w;
    reg [RD_PTR_W-1:0]   rd_iss_r;
    reg [RD_CNT_W-1:0]   rd_iss_n;
    reg [RD_CNT_W-1:0]   rd_ost;

    reg [MAX_WR_OST-1:0] wr_v;
    reg [MAX_WR_OST-1:0] wr_w_done;
    reg [MAX_WR_OST-1:0] wr_need_r;
    reg [MAX_WR_OST-1:0] wr_b_got;
    reg [ID_W-1:0]       wr_axi_id [0:MAX_WR_OST-1];
    reg [ADDR_W-1:0]     wr_addr   [0:MAX_WR_OST-1];
    reg [LEN_W-1:0]      wr_len    [0:MAX_WR_OST-1];
    reg [3:0]            wr_op     [0:MAX_WR_OST-1];
    reg [MOD_PW-1:0]     wr_mod    [0:MAX_WR_OST-1];
    reg [WR_PTR_W-1:0]   wr_ord    [0:MAX_WR_OST-1];
    reg [WR_PTR_W-1:0]   wr_ord_w;
    reg [WR_PTR_W-1:0]   wr_ord_r;
    reg [WR_CNT_W-1:0]   wr_ord_n;
    reg [WR_CNT_W-1:0]   wr_ost;

    reg              tr_r_vld;
    reg [ID_W-1:0]   tr_rid;
    reg              tr_aw;
    reg              tr_drain;
    reg              tr_b_vld;
    reg [ID_W-1:0]   tr_bid;
    reg              tr_need_r;

    integer i;
    integer j;

    reg                  rd_has_free;
    reg [RD_PTR_W-1:0]   rd_free;
    reg                  wr_has_free;
    reg [WR_PTR_W-1:0]   wr_free;
    reg                  rd_id_hit;
    reg                  wr_id_hit;

    always @* begin
        rd_has_free = 1'b0;
        rd_free     = {RD_PTR_W{1'b0}};
        rd_id_hit   = 1'b0;
        for (i = 0; i < MAX_RD_OST; i = i + 1) begin
            if (rd_v[i] && (rd_axi_id[i] == arid))
                rd_id_hit = 1'b1;
            if (!rd_v[i] && !rd_has_free) begin
                rd_has_free = 1'b1;
                rd_free     = i[RD_PTR_W-1:0];
            end
        end
        wr_has_free = 1'b0;
        wr_free     = {WR_PTR_W{1'b0}};
        wr_id_hit   = 1'b0;
        for (j = 0; j < MAX_WR_OST; j = j + 1) begin
            if (wr_v[j] && (wr_axi_id[j] == awid))
                wr_id_hit = 1'b1;
            if (!wr_v[j] && !wr_has_free) begin
                wr_has_free = 1'b1;
                wr_free     = j[WR_PTR_W-1:0];
            end
        end
    end

    wire tr_busy = tr_r_vld || tr_aw || tr_drain || tr_b_vld;

    wire q_quiesce;
    wire q_err_mode;
`ifdef ADP_QCH_TO
    wire q_to_err;
    reg [MAX_RD_OST-1:0] rd_axi_done;
    reg [MAX_RD_OST-1:0] rd_lbus_done;
    reg [MAX_WR_OST-1:0] wr_axi_done;
    reg [MAX_WR_OST-1:0] wr_lbus_done;
    wire rd_axi_busy = |(rd_v & ~rd_axi_done);
    wire wr_axi_busy = |(wr_v & ~wr_axi_done);
    wire mst_busy = rd_axi_busy || wr_axi_busy || tr_busy;

    adapter_qch #(.HAS_QDENY(1), .QCH_TO_W(QCH_TO_W)) u_qch (
        .clk(clk), .rst_n(rst_n),
        .qreqn(qreqn), .reg_qdeny_en(reg_qdeny_en), .reg_err_en(reg_err_en),
        .busy(mst_busy),
        .qacceptn(qacceptn), .qdeny(qdeny), .qactive(qactive),
        .lbus_pwrdn(lbus_pwrdn), .o_quiesce(q_quiesce), .o_err_mode(q_err_mode),
        .qch_to_en(qch_to_en), .qch_to_lim(qch_to_lim), .qch_to_mode(qch_to_mode),
        .irq_qch_to(irq_qch_to), .o_to_err(q_to_err)
    );

    reg                to_r_v;
    reg [RD_PTR_W-1:0] to_r_i;
    reg                to_b_v;
    reg [WR_PTR_W-1:0] to_b_i;
    integer ti;
    integer tj;
    always @* begin
        to_r_v = 1'b0;
        to_r_i = {RD_PTR_W{1'b0}};
        for (ti = 0; ti < MAX_RD_OST; ti = ti + 1) begin
            if (rd_v[ti] && !rd_axi_done[ti] && !to_r_v) begin
                to_r_v = 1'b1;
                to_r_i = ti[RD_PTR_W-1:0];
            end
        end
        to_b_v = 1'b0;
        to_b_i = {WR_PTR_W{1'b0}};
        for (tj = 0; tj < MAX_WR_OST; tj = tj + 1) begin
            if (wr_v[tj] && !wr_axi_done[tj] && !to_b_v) begin
                to_b_v = 1'b1;
                to_b_i = tj[WR_PTR_W-1:0];
            end
        end
    end
    wire loc_r = q_to_err && to_r_v;
    wire loc_b = q_to_err && to_b_v;
`else
    wire q_to_err = 1'b0;
    wire loc_r    = 1'b0;
    wire loc_b    = 1'b0;
    wire [RD_PTR_W-1:0] to_r_i = {RD_PTR_W{1'b0}};
    wire [WR_PTR_W-1:0] to_b_i = {WR_PTR_W{1'b0}};
    wire mst_busy = (rd_ost != {RD_CNT_W{1'b0}}) ||
                    (wr_ost != {WR_CNT_W{1'b0}}) || tr_busy;

    adapter_qch #(.HAS_QDENY(1)) u_qch (
        .clk(clk), .rst_n(rst_n),
        .qreqn(qreqn), .reg_qdeny_en(reg_qdeny_en), .reg_err_en(reg_err_en),
        .busy(mst_busy),
        .qacceptn(qacceptn), .qdeny(qdeny), .qactive(qactive),
        .lbus_pwrdn(lbus_pwrdn), .o_quiesce(q_quiesce), .o_err_mode(q_err_mode)
    );
`endif

    assign arready = q_err_mode ? (!tr_r_vld) :
                     (rd_has_free && !rd_id_hit && !q_quiesce && !lbus_pwrdn &&
                      !q_to_err);
    assign awready = q_err_mode ? (!tr_aw && !tr_drain && !tr_b_vld &&
                                   (wr_ord_n == {WR_CNT_W{1'b0}})) :
                     (wr_has_free && !wr_id_hit && !q_quiesce && !lbus_pwrdn &&
                      !q_to_err &&
                      !(at_live_need_r && (rd_ost != {RD_CNT_W{1'b0}})));

    wire [WR_PTR_W-1:0] wr_wslot = (wr_ord_n != {WR_CNT_W{1'b0}}) ?
                                   wr_ord[wr_ord_r] : wr_free;
    wire wr_can_w = (wr_ord_n != {WR_CNT_W{1'b0}}) ||
                    (awvalid && awready && !q_err_mode && !q_to_err);

    assign wready = q_to_err ? ((wr_ord_n != {WR_CNT_W{1'b0}}) || tr_aw || tr_drain) :
                    ((wr_ord_n != {WR_CNT_W{1'b0}}) ? req_w_ready :
                     (q_err_mode ? (tr_aw || tr_drain) : (wr_can_w && req_w_ready)));

    wire [ADDR_W-1:0] wr_addr_now = (wr_ord_n != {WR_CNT_W{1'b0}}) ?
                                    wr_addr[wr_ord[wr_ord_r]] : awaddr;
    wire [LEN_W-1:0]  wr_len_now  = (wr_ord_n != {WR_CNT_W{1'b0}}) ?
                                    wr_len[wr_ord[wr_ord_r]] : awlen;
    wire [3:0]        wr_op_now   = (wr_ord_n != {WR_CNT_W{1'b0}}) ?
                                    wr_op[wr_ord[wr_ord_r]] :
                                    (aw_is_atomic ? at_opcode : `LB_OP_WR);
    wire [MOD_PW-1:0] wr_mod_now  = (wr_ord_n != {WR_CNT_W{1'b0}}) ?
                                    wr_mod[wr_ord[wr_ord_r]] :
                                    (aw_is_atomic ? at_mod : {MOD_PW{1'b0}});
    wire [ID_W-1:0]   wr_txn_now;
    assign wr_txn_now = wr_wslot;

    wire [EXT_CMD_W-1:0] cmd_wr = {
        {QOS_PW{1'b0}}, wr_op_now, wr_addr_now, wr_len_now, wr_txn_now,
        {USER_CMD_PW{1'b0}}
    };
    wire [EXT_WD_W-1:0] wd_wr = {wdata, wstrb, wlast, wr_txn_now};

    assign req_w_data  = {cmd_wr, wr_mod_now, wd_wr};
    assign req_w_valid = wr_can_w && wvalid && !q_to_err &&
                         (!q_err_mode || (wr_ord_n != {WR_CNT_W{1'b0}}));

    wire [WR_PTR_W-1:0] wr_b_slot = rsp_wr_ext_txnid[WR_PTR_W-1:0];
    wire wr_in_range = rsp_wr_valid && (rsp_wr_ext_txnid < MAX_WR_OST) &&
                       wr_v[wr_b_slot];
`ifdef ADP_QCH_TO
    wire wr_axi_done_hit = wr_in_range && wr_axi_done[wr_b_slot];
`else
    wire wr_axi_done_hit = 1'b0;
`endif
    wire wr_b_ok = !q_to_err && wr_in_range && wr_w_done[wr_b_slot] &&
                   !wr_axi_done_hit;

    assign bvalid       = tr_b_vld || loc_b || wr_b_ok;
    assign bid          = tr_b_vld ? tr_bid :
                          (loc_b ? wr_axi_id[to_b_i] : wr_axi_id[wr_b_slot]);
    assign bresp        = (tr_b_vld || loc_b) ? `AXI_RESP_SLVERR :
                          map_resp(rsp_wr_resp);
    assign rsp_wr_ready = (q_to_err || wr_axi_done_hit) ? wr_in_range :
                          (wr_b_ok && bready && !tr_b_vld);

    wire [RD_PTR_W-1:0] rd_iss_slot = rd_iss_q[rd_iss_r];
    wire [ID_W-1:0]     rd_txn_now;
    assign rd_txn_now = rd_iss_slot;
    wire [EXT_CMD_W-1:0] cmd_rd = {
        {QOS_PW{1'b0}}, `LB_OP_RD, rd_addr[rd_iss_slot], rd_len[rd_iss_slot],
        rd_txn_now, {USER_CMD_PW{1'b0}}
    };
    assign req_r_data  = cmd_rd;
    assign req_r_valid = (rd_iss_n != {RD_CNT_W{1'b0}}) && !q_to_err;

    wire [WR_PTR_W-1:0] wr_r_slot = rsp_rd_ext_txnid[WR_PTR_W-1:0];
    wire [RD_PTR_W-1:0] rd_rsp_slot = rsp_rd_ext_txnid[RD_PTR_W-1:0];
`ifdef ADP_QCH_TO
    wire rd_axi_done_hit = rsp_rd_valid && (rsp_rd_ext_txnid < MAX_RD_OST) &&
                           rd_v[rd_rsp_slot] && rd_axi_done[rd_rsp_slot];
    wire wr_r_done_hit = HAS_ATOMIC && rsp_rd_valid &&
                         (rsp_rd_ext_txnid < MAX_WR_OST) &&
                         wr_v[wr_r_slot] && wr_axi_done[wr_r_slot];
`else
    wire rd_axi_done_hit = 1'b0;
    wire wr_r_done_hit   = 1'b0;
`endif
    wire wr_r_ok = HAS_ATOMIC && !q_to_err && rsp_rd_valid &&
                   (rsp_rd_ext_txnid < MAX_WR_OST) &&
                   wr_v[wr_r_slot] && wr_need_r[wr_r_slot] && !wr_r_done_hit;

    wire rd_rsp_ok = !q_to_err && !wr_r_ok && rsp_rd_valid &&
                     (rsp_rd_ext_txnid < MAX_RD_OST) &&
                     rd_v[rd_rsp_slot] && !rd_axi_done_hit;

    assign rvalid       = tr_r_vld || loc_r || rd_rsp_ok || wr_r_ok;
    assign rid          = tr_r_vld ? tr_rid :
                          (loc_r ? rd_axi_id[to_r_i] :
                           (rd_rsp_ok ? rd_axi_id[rd_rsp_slot] :
                            wr_axi_id[wr_r_slot]));
    assign rdata        = (tr_r_vld || loc_r) ? {DATA_W{1'b0}} : rsp_rd_data;
    assign rresp        = (tr_r_vld || loc_r) ? `AXI_RESP_SLVERR :
                          map_resp(rsp_rd_resp);
    assign rlast        = (tr_r_vld || loc_r || wr_r_ok) ? 1'b1 : rsp_rd_last;
    assign rsp_rd_ready = (q_to_err || rd_axi_done_hit || wr_r_done_hit) ?
                          rsp_rd_valid :
                          (!tr_r_vld && rready && (rd_rsp_ok || wr_r_ok));

    wire aw_hs = awvalid && awready && !q_err_mode && !q_to_err;
    wire w_hs  = req_w_valid && req_w_ready && wlast;
    wire w_drain_last = q_to_err && wvalid && wready && wlast &&
                        (wr_ord_n != {WR_CNT_W{1'b0}});
    wire ar_hs = arvalid && arready && !q_err_mode && !q_to_err;
    wire rr_hs = req_r_valid && req_r_ready;
    wire b_hs  = wr_b_ok && rsp_wr_ready;
    wire r_hs  = rd_rsp_ok && rsp_rd_ready && rsp_rd_last;
    wire at_r_hs = wr_r_ok && rsp_rd_ready;
    wire loc_r_hs = loc_r && rready && !tr_r_vld;
    wire loc_b_hs = loc_b && bready && !tr_b_vld;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rd_v     <= {MAX_RD_OST{1'b0}};
            rd_iss_w <= {RD_PTR_W{1'b0}};
            rd_iss_r <= {RD_PTR_W{1'b0}};
            rd_iss_n <= {RD_CNT_W{1'b0}};
            rd_ost   <= {RD_CNT_W{1'b0}};
            wr_v     <= {MAX_WR_OST{1'b0}};
            wr_w_done<= {MAX_WR_OST{1'b0}};
            wr_need_r<= {MAX_WR_OST{1'b0}};
            wr_b_got <= {MAX_WR_OST{1'b0}};
            wr_ord_w <= {WR_PTR_W{1'b0}};
            wr_ord_r <= {WR_PTR_W{1'b0}};
            wr_ord_n <= {WR_CNT_W{1'b0}};
            wr_ost   <= {WR_CNT_W{1'b0}};
            tr_r_vld <= 1'b0;
            tr_rid   <= {ID_W{1'b0}};
            tr_aw    <= 1'b0;
            tr_drain <= 1'b0;
            tr_b_vld <= 1'b0;
            tr_bid   <= {ID_W{1'b0}};
            tr_need_r<= 1'b0;
`ifdef ADP_QCH_TO
            rd_axi_done  <= {MAX_RD_OST{1'b0}};
            rd_lbus_done <= {MAX_RD_OST{1'b0}};
            wr_axi_done  <= {MAX_WR_OST{1'b0}};
            wr_lbus_done <= {MAX_WR_OST{1'b0}};
`endif
        end else begin
            if (ar_hs) begin
                rd_v[rd_free]      <= 1'b1;
                rd_axi_id[rd_free] <= arid;
                rd_addr[rd_free]   <= araddr;
                rd_len[rd_free]    <= arlen;
                rd_iss_q[rd_iss_w] <= rd_free;
                rd_iss_w <= (rd_iss_w == RD_PTR_LAST) ?
                            {RD_PTR_W{1'b0}} : (rd_iss_w + 1'b1);
`ifdef ADP_QCH_TO
                rd_axi_done[rd_free]  <= 1'b0;
                rd_lbus_done[rd_free] <= 1'b0;
`endif
            end
            if (rr_hs)
                rd_iss_r <= (rd_iss_r == RD_PTR_LAST) ?
                            {RD_PTR_W{1'b0}} : (rd_iss_r + 1'b1);
            if (r_hs)
                rd_v[rd_rsp_slot] <= 1'b0;
`ifdef ADP_QCH_TO
            if (loc_r_hs)
                rd_axi_done[to_r_i] <= 1'b1;
            if ((q_to_err || rd_axi_done_hit) && rsp_rd_valid && rsp_rd_ready &&
                (rsp_rd_ext_txnid < MAX_RD_OST) &&
                rd_v[rsp_rd_ext_txnid[RD_PTR_W-1:0]] && !wr_r_ok) begin
                rd_lbus_done[rsp_rd_ext_txnid[RD_PTR_W-1:0]] <= 1'b1;
                if (rd_axi_done[rsp_rd_ext_txnid[RD_PTR_W-1:0]] || loc_r_hs &&
                    (to_r_i == rsp_rd_ext_txnid[RD_PTR_W-1:0]))
                    rd_v[rsp_rd_ext_txnid[RD_PTR_W-1:0]] <= 1'b0;
            end
            if (loc_r_hs && rd_lbus_done[to_r_i])
                rd_v[to_r_i] <= 1'b0;
`endif
            rd_iss_n <= rd_iss_n + (ar_hs ? 1'b1 : 1'b0) - (rr_hs ? 1'b1 : 1'b0);
            rd_ost   <= rd_ost   + (ar_hs ? 1'b1 : 1'b0) -
                        ((r_hs || loc_r_hs) ? 1'b1 : 1'b0);

            if (aw_hs) begin
                wr_v[wr_free]      <= 1'b1;
                wr_w_done[wr_free] <= 1'b0;
                wr_need_r[wr_free] <= at_live_need_r;
                wr_b_got[wr_free]  <= 1'b0;
                wr_axi_id[wr_free] <= awid;
                wr_addr[wr_free]   <= awaddr;
                wr_len[wr_free]    <= awlen;
                wr_op[wr_free]     <= wr_op_now;
                wr_mod[wr_free]    <= wr_mod_now;
                wr_ord[wr_ord_w]   <= wr_free;
                wr_ord_w <= (wr_ord_w == WR_PTR_LAST) ?
                            {WR_PTR_W{1'b0}} : (wr_ord_w + 1'b1);
`ifdef ADP_QCH_TO
                wr_axi_done[wr_free]  <= 1'b0;
                wr_lbus_done[wr_free] <= 1'b0;
`endif
            end
            if (w_hs || w_drain_last) begin
                wr_w_done[wr_wslot] <= 1'b1;
                wr_ord_r <= (wr_ord_r == WR_PTR_LAST) ?
                            {WR_PTR_W{1'b0}} : (wr_ord_r + 1'b1);
`ifdef ADP_QCH_TO
                if (w_drain_last)
                    wr_lbus_done[wr_wslot] <= 1'b1;
`endif
            end
            wr_ord_n <= wr_ord_n + (aw_hs ? 1'b1 : 1'b0) -
                        ((w_hs || w_drain_last) ? 1'b1 : 1'b0);

            if (b_hs)
                wr_b_got[wr_b_slot] <= 1'b1;
            if (at_r_hs)
                wr_need_r[wr_r_slot] <= 1'b0;

            if (b_hs && (!wr_need_r[wr_b_slot] ||
                         (at_r_hs && (wr_r_slot == wr_b_slot)))) begin
                wr_v[wr_b_slot] <= 1'b0;
                wr_ost <= wr_ost + (aw_hs ? 1'b1 : 1'b0) - 1'b1;
            end else if (at_r_hs && (wr_b_got[wr_r_slot] ||
                         (b_hs && (wr_b_slot == wr_r_slot)))) begin
                wr_v[wr_r_slot] <= 1'b0;
                wr_ost <= wr_ost + (aw_hs ? 1'b1 : 1'b0) - 1'b1;
            end else
                wr_ost <= wr_ost + (aw_hs ? 1'b1 : 1'b0) -
                          (loc_b_hs ? 1'b1 : 1'b0);

`ifdef ADP_QCH_TO
            if (loc_b_hs)
                wr_axi_done[to_b_i] <= 1'b1;
            if ((q_to_err || wr_axi_done_hit) && rsp_wr_valid && rsp_wr_ready &&
                (rsp_wr_ext_txnid < MAX_WR_OST) &&
                wr_v[rsp_wr_ext_txnid[WR_PTR_W-1:0]]) begin
                wr_lbus_done[rsp_wr_ext_txnid[WR_PTR_W-1:0]] <= 1'b1;
                if (wr_axi_done[rsp_wr_ext_txnid[WR_PTR_W-1:0]] ||
                    (loc_b_hs && (to_b_i == rsp_wr_ext_txnid[WR_PTR_W-1:0])))
                    wr_v[rsp_wr_ext_txnid[WR_PTR_W-1:0]] <= 1'b0;
            end
            if (loc_b_hs && wr_lbus_done[to_b_i])
                wr_v[to_b_i] <= 1'b0;
`endif

            if (tr_r_vld && rready)
                tr_r_vld <= 1'b0;
            if (q_err_mode && arvalid && arready) begin
                tr_r_vld <= 1'b1;
                tr_rid   <= arid;
            end
            if (tr_b_vld && bready)
                tr_b_vld <= 1'b0;
            if ((tr_aw || tr_drain) && wvalid && wready) begin
                tr_aw <= 1'b0;
                if (wlast) begin
                    tr_drain  <= 1'b0;
                    tr_b_vld  <= 1'b1;
                    if (tr_need_r) begin
                        tr_r_vld <= 1'b1;
                        tr_rid   <= tr_bid;
                    end
                    tr_need_r <= 1'b0;
                end else
                    tr_drain <= 1'b1;
            end
            if (q_err_mode && awvalid && awready) begin
                tr_bid    <= awid;
                tr_aw     <= 1'b1;
                tr_need_r <= at_live_need_r;
            end
        end
    end

endmodule
