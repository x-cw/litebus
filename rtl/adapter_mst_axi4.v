//============================================================================
// Filename    : adapter_mst_axi4.v
// Description : adapter_mst top: AXI4 slave <-> LiteBus INIU-side interface.
//               Selectively instantiates feature modules:
//                 adapter_narrow_pack / adapter_narrow_split (NARROW_EN)
//                 adapter_rob                                (always; it also
//                    enforces sub-transaction order when SPLIT_EN)
//                 adapter_burst_split                        (SPLIT_EN geometry)
//                 adapter_atomic                             (ATOMIC_EN)
//               Permanent logic: in-flight table, admission (same-ID gate in
//               mode A), AW queue, direct datapaths, response mapping, R
//               output skid.
//               Downstream txnid = int_id (table entry index) in all modes;
//               SAME_ID_EN=0 additionally rejects same-ID admission.
//               Reads always issue via the rob issuable path (1-cycle REQ_R
//               latency even in mode A -- acceptable, keeps one unified path).
//============================================================================
`include "adapter_ip_defs.vh"
`default_nettype none
`timescale 1ns/1ps

module adapter_mst_axi4 #(
    parameter ADDR_W  = 32,
    parameter DATA_W  = 64,
    parameter LEN_W   = 8,
    parameter ID_W    = 8,
    parameter USER_CMD_W     = 0,
    parameter USER_RSP_RD_W  = 0,
    parameter USER_RSP_WR_W  = 0,
    parameter QOS_W   = 0,
    parameter MOD_W   = 0,
    parameter NARROW_EN      = 1,
    parameter SAME_ID_EN     = 1,
    parameter SPLIT_EN       = 0,
    parameter ATOMIC_EN      = 0,
    parameter QCH_EN         = 0,
    parameter ADDITION_EN    = 0,
    parameter ROB_DEPTH      = 1,
    parameter PEND_TX        = 8,
    parameter PEND_WR        = 4,
    parameter R_SKID_DEPTH   = 4,
    parameter LB_MAX_BURST_BYTES = 0,
    parameter AWATOP_W       = 5,
    parameter ATOMIC_FAIL_RESP = `AXI_RESP_EXOKAY,
    // ---- derived (port dependent) ----
    parameter W_BYTES   = DATA_W/8,
    parameter LANE_W    = `adp_clog2(W_BYTES),
    parameter ADDITION_W = LANE_W + 1,
    parameter SZB       = LANE_W + 1,
    parameter IDX_W     = `adp_clog2(PEND_TX),
    parameter SEQ_W     = 8,
    parameter QOS_PW    = (QOS_W < 1) ? 1 : QOS_W,
    parameter USER_CMD_PW = (USER_CMD_W < 1) ? 1 : USER_CMD_W,
    parameter MOD_PW    = (MOD_W < 1) ? 1 : MOD_W,
    parameter AWATOP_PW = (AWATOP_W < 1) ? 1 : AWATOP_W,
    parameter EXT_CMD_W = QOS_PW + 4 + ADDR_W + LEN_W + ID_W + USER_CMD_PW,
    parameter EXT_WD_W  = DATA_W + W_BYTES + 1 + ID_W,
    parameter EXT_REQ_W = EXT_CMD_W + MOD_PW + EXT_WD_W,
    parameter J         = (LB_MAX_BURST_BYTES > 0) ? (LB_MAX_BURST_BYTES / W_BYTES) : ((1 << LEN_W) * W_BYTES),
    parameter MAX_SUB   = (SPLIT_EN && (LB_MAX_BURST_BYTES > 0)) ?
                          ((((1 << LEN_W) + 1) * W_BYTES + LB_MAX_BURST_BYTES - 1) / LB_MAX_BURST_BYTES) : 1,
    parameter SUB_IDX_W = (`adp_clog2(MAX_SUB) < 1) ? 1 : `adp_clog2(MAX_SUB),
    parameter RSK_W        = ID_W + DATA_W + 2 + 1,
    parameter ROB_DATA_W   = RSK_W,
    parameter ROB_MEM_TYPE = 0          // 0=REG, 1=RAM; forwarded to adapter_rob
) (
    input  wire                    clk,
    input  wire                    rst_n,
    // ---------------- AXI4 slave ----------------
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
    input  wire [AWATOP_PW-1:0]    awatop,
    // ---------------- LiteBus INIU-side ----------------
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
    output wire                    rsp_wr_ready,
    output wire [ADDITION_W-1:0]   req_r_addition,
    output wire [ADDITION_W-1:0]   req_w_addition,
    input  wire [ADDITION_W-1:0]   rsp_rd_addition,
    input  wire                    qreqn,
    input  wire                    reg_qdeny_en,
    input  wire                    reg_err_en,
    output wire                    qacceptn,
    output wire                    qdeny,
    output wire                    qactive,
    output wire                    lbus_pwrdn
);
    //=======================================================================
    // in-flight table
    //=======================================================================
    reg [PEND_TX-1:0]                e_valid;
    reg [PEND_TX*ID_W-1:0]           e_ext_id;
    reg [PEND_TX-1:0]                e_is_wr;
    reg [PEND_TX-1:0]                e_is_atomic;
    reg [PEND_TX-1:0]                e_need_r;
    reg [PEND_TX*ADDR_W-1:0]         e_addr;
    reg [PEND_TX*SZB-1:0]            e_size;
    reg [PEND_TX*LANE_W-1:0]         e_lane;
    reg [PEND_TX*LEN_W-1:0]          e_len;
    reg [PEND_TX*(LEN_W+1)-1:0]      e_len_p;
    reg [PEND_TX*4-1:0]              e_opcode;
    reg [PEND_TX*MOD_PW-1:0]         e_mod;
    reg [PEND_TX*QOS_PW-1:0]         e_qos;
    reg [PEND_TX*USER_CMD_PW-1:0]    e_user;
    reg [PEND_TX-1:0]                e_issued;
    reg [PEND_TX-1:0]                e_b_vld;
    reg [PEND_TX*2-1:0]              e_b_q;
    reg [PEND_TX-1:0]                e_b_pres;
    reg [PEND_TX-1:0]                e_b_err;     // any sub B failed (aggregated at head)
    reg [PEND_TX*IDX_W-1:0]          e_head;      // transaction head entry index
    reg [PEND_TX*(SUB_IDX_W+1)-1:0]  e_subn;      // number of sub-transactions
    reg [PEND_TX*(SUB_IDX_W+1)-1:0]  e_b_cnt;     // Bs received (counted at head)
    reg [PEND_TX-1:0]                e_ar_vld;
    reg [PEND_TX*DATA_W-1:0]         e_ar_q;
    reg [PEND_TX*2-1:0]              e_ar_resp;
    reg [PEND_TX-1:0]                e_r_pres;
    reg [PEND_TX*(LEN_W+1)-1:0]      e_bl;
    reg [PEND_TX*(LEN_W+1)-1:0]      e_axi_total;
    reg [PEND_TX-1:0]                e_rd_first;
    reg [PEND_TX-1:0]                e_last_sub;
    reg [PEND_TX*DATA_W-1:0]         e_rd_acc;
    reg [PEND_TX*W_BYTES-1:0]        e_rd_vld;
    reg [PEND_TX*(LANE_W+1)-1:0]     e_rd_cnt;
    reg [PEND_TX*(LANE_W+1)-1:0]     e_vb_first;
    reg [PEND_TX*(LANE_W+1)-1:0]     e_vb_last;
    reg [PEND_TX*ADDITION_W-1:0]     e_addition;
    reg [PEND_TX*2-1:0]              e_burst;
    reg [PEND_TX*SEQ_W-1:0]          e_seq;
    reg [SEQ_W-1:0]                  seq_ctr;

    //=======================================================================
    // AW queue (one slot per write transaction: {n_subs, head_idx})
    //=======================================================================
    localparam Q_W    = IDX_W + (SUB_IDX_W + 1);
    localparam QPTR_W = `adp_clog2(PEND_WR) + 1;
    reg [PEND_WR*Q_W-1:0] awq_mem;
    reg [QPTR_W-1:0]      awq_wr;
    reg [QPTR_W-1:0]      awq_rd;

    reg  awq_full_r;
    always @* begin
        awq_full_r = 1'b0;
        if (awq_wr == PEND_WR)
            awq_full_r = (awq_rd == 0);
        else
            awq_full_r = (awq_wr + 1'b1 == awq_rd);
    end
    wire awq_empty = (awq_wr == awq_rd);

    // QCH err_mode truncate (no table, no LiteBus)
    reg              tr_r_vld;
    reg [ID_W-1:0]   tr_rid;
    reg              tr_aw;      // AW accepted, waiting for W
    reg              tr_drain;   // draining W to wlast
    reg              tr_b_vld;
    reg [ID_W-1:0]   tr_bid;
    wire tr_busy = tr_r_vld || tr_aw || tr_drain || tr_b_vld;

    wire mst_busy = (|e_valid) || !awq_empty || tr_busy;
    wire q_quiesce;
    wire q_err_mode;
    generate
    if (QCH_EN) begin : g_qch
        adapter_qch #(.HAS_QDENY(1)) u_qch (
            .clk(clk), .rst_n(rst_n),
            .qreqn(qreqn), .reg_qdeny_en(reg_qdeny_en), .reg_err_en(reg_err_en),
            .busy(mst_busy),
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

    wire [Q_W-1:0]      awq_head = awq_mem[awq_rd[QPTR_W-2:0]*Q_W +: Q_W];
    wire [IDX_W-1:0]    awq_head_idx = awq_head[IDX_W-1:0];
    wire [SUB_IDX_W:0]  awq_head_n   = awq_head[Q_W-1 -: (SUB_IDX_W+1)];

    reg [SUB_IDX_W-1:0] cur_sub;
    reg [LEN_W:0]       cur_beat;

    //=======================================================================
    // admission geometry
    //=======================================================================
    wire [LANE_W-1:0] aw_lane = awaddr[LANE_W-1:0];
    wire [LANE_W-1:0] ar_lane = araddr[LANE_W-1:0];
    reg [SZB-1:0] aw_sz;
    reg [SZB-1:0] ar_sz;
    always @* begin
        aw_sz = (8'b1 << awsize);
        ar_sz = (8'b1 << arsize);
    end

    reg [LEN_W+SZB:0] aw_bytes;
    reg [LEN_W+SZB:0] aw_beats;
    reg [SUB_IDX_W:0] aw_N;
    reg [LEN_W+SZB:0] ar_bytes;
    reg [LEN_W+SZB:0] ar_beats;
    reg [SUB_IDX_W:0] ar_N;
    always @* begin
        aw_bytes = ({1'b0, awlen} + 1) * aw_sz;
        aw_beats = (aw_lane + aw_bytes + (W_BYTES - 1)) / W_BYTES;
        if (SPLIT_EN) aw_N = (aw_beats + J - 1) / J; else aw_N = 1;
        ar_bytes = ({1'b0, arlen} + 1) * ar_sz;
        ar_beats = (ar_lane + ar_bytes + (W_BYTES - 1)) / W_BYTES;
        if (SPLIT_EN) ar_N = (ar_beats + J - 1) / J; else ar_N = 1;
    end

    //=======================================================================
    // admission
    //=======================================================================
    integer fc;
    reg [IDX_W:0] free_cnt;
    always @* begin
        free_cnt = 0;
        for (fc = 0; fc < PEND_TX; fc = fc + 1)
            if (!e_valid[fc]) free_cnt = free_cnt + 1;
    end

    wire [PEND_TX-1:0] aw_id_hit_v;
    wire [PEND_TX-1:0] ar_id_hit_v;
    genvar gid;
    generate
    for (gid = 0; gid < PEND_TX; gid = gid + 1) begin : g_idhit
        assign aw_id_hit_v[gid] = e_valid[gid] && (e_ext_id[gid*ID_W +: ID_W] == awid);
        assign ar_id_hit_v[gid] = e_valid[gid] && (e_ext_id[gid*ID_W +: ID_W] == arid);
    end
    endgenerate
    wire aw_id_hit = (SAME_ID_EN == 0) && (|aw_id_hit_v);
    wire ar_id_hit = (SAME_ID_EN == 0) && (|ar_id_hit_v);

    wire ar_accept = (free_cnt >= (ar_N + (awvalid ? aw_N : 0))) && !ar_id_hit && !q_quiesce && !q_err_mode;
    wire aw_accept = (free_cnt >= (aw_N + (arvalid ? ar_N : 0))) && !aw_id_hit && !awq_full_r && !q_quiesce && !q_err_mode;
    assign awready = q_err_mode ? (!tr_aw && !tr_drain && !tr_b_vld) : aw_accept;
    assign arready = q_err_mode ? (!tr_r_vld) : ar_accept;
    wire ar_hsk = arvalid && ar_accept;
    wire aw_hsk = awvalid && aw_accept;

    //=======================================================================
    // allocation chain
    //=======================================================================
    wire [PEND_TX-1:0] ar_alloc;
    wire [PEND_TX-1:0] aw_alloc;
    wire [SUB_IDX_W-1:0] ar_alloc_cnt [0:PEND_TX-1];
    wire [SUB_IDX_W-1:0] aw_alloc_cnt [0:PEND_TX-1];
    wire [SUB_IDX_W:0]   ar_cnt_nxt [0:PEND_TX-1];
    wire [SUB_IDX_W:0]   aw_cnt_nxt [0:PEND_TX-1];

    genvar ga;
    generate
    for (ga = 0; ga < PEND_TX; ga = ga + 1) begin : g_alloc
        assign ar_cnt_nxt[ga] = (ga == 0) ? 0 : (ar_alloc[ga-1] ? (ar_cnt_nxt[ga-1] + 1) : ar_cnt_nxt[ga-1]);
        assign aw_cnt_nxt[ga] = (ga == 0) ? 0 : (aw_alloc[ga-1] ? (aw_cnt_nxt[ga-1] + 1) : aw_cnt_nxt[ga-1]);
        assign ar_alloc_cnt[ga] = ar_cnt_nxt[ga][SUB_IDX_W-1:0];
        assign aw_alloc_cnt[ga] = aw_cnt_nxt[ga][SUB_IDX_W-1:0];
        assign ar_alloc[ga] = ar_hsk && !e_valid[ga] && (ar_cnt_nxt[ga] < ar_N);
        assign aw_alloc[ga] = aw_hsk && !e_valid[ga] && (aw_cnt_nxt[ga] < aw_N);
    end
    endgenerate

    // first allocated index (head)
    reg [IDX_W-1:0] ar_head_idx_r;
    reg [IDX_W-1:0] aw_head_idx_r;
    integer hf;
    always @* begin
        ar_head_idx_r = 0;
        for (hf = 0; hf < PEND_TX; hf = hf + 1)
            if (ar_alloc[hf]) begin ar_head_idx_r = hf[IDX_W-1:0]; hf = PEND_TX; end
        aw_head_idx_r = 0;
        for (hf = 0; hf < PEND_TX; hf = hf + 1)
            if (aw_alloc[hf]) begin aw_head_idx_r = hf[IDX_W-1:0]; hf = PEND_TX; end
    end
    wire [IDX_W-1:0] ar_head_idx = ar_head_idx_r;
    wire [IDX_W-1:0] aw_head_idx = aw_head_idx_r;

    // per-entry allocation geometry (burst_split per table entry)
    wire [ADDR_W-1:0] g_aw_addr_k [0:PEND_TX-1];
    wire [LEN_W:0]    g_aw_len_p  [0:PEND_TX-1];
    wire [LANE_W:0]   g_aw_vbf    [0:PEND_TX-1];
    wire [LANE_W:0]   g_aw_vbl    [0:PEND_TX-1];
    wire [LEN_W:0]    g_aw_axi_beats [0:PEND_TX-1];
    wire [LANE_W-1:0] g_aw_lane_k [0:PEND_TX-1];
    wire [ADDR_W-1:0] g_ar_addr_k [0:PEND_TX-1];
    wire [LEN_W:0]    g_ar_len_p  [0:PEND_TX-1];
    wire [LANE_W:0]   g_ar_vbf    [0:PEND_TX-1];
    wire [LANE_W:0]   g_ar_vbl    [0:PEND_TX-1];
    wire [LEN_W:0]    g_ar_axi_beats [0:PEND_TX-1];
    wire [LANE_W-1:0] g_ar_lane_k [0:PEND_TX-1];
    wire              g_ar_last      [0:PEND_TX-1];
    wire              g_aw_last      [0:PEND_TX-1];
    wire [LANE_W:0]   g_aw_add       [0:PEND_TX-1];
    wire [LANE_W:0]   g_ar_add       [0:PEND_TX-1];
    genvar gg;
    generate
    for (gg = 0; gg < PEND_TX; gg = gg + 1) begin : g_geom
        adapter_burst_split #(
            .ADDR_W(ADDR_W), .LEN_W(LEN_W), .W_BYTES(W_BYTES), .J(J), .MAX_SUB(MAX_SUB)
        ) u_bs_aw (
            .i_addr(awaddr), .i_lane(aw_lane), .i_len(awlen), .i_size(aw_sz),
            .i_sub(aw_alloc_cnt[gg]),
            .o_addr_k(g_aw_addr_k[gg]), .o_len_p_k(g_aw_len_p[gg]),
            .o_lane_k(g_aw_lane_k[gg]),
            .o_vb_first(g_aw_vbf[gg]), .o_vb_last(g_aw_vbl[gg]),
            .o_axi_beats(g_aw_axi_beats[gg]), .o_last_k(g_aw_last[gg]),
            .o_addition_k(g_aw_add[gg]), .o_N()
        );
        adapter_burst_split #(
            .ADDR_W(ADDR_W), .LEN_W(LEN_W), .W_BYTES(W_BYTES), .J(J), .MAX_SUB(MAX_SUB)
        ) u_bs_ar (
            .i_addr(araddr), .i_lane(ar_lane), .i_len(arlen), .i_size(ar_sz),
            .i_sub(ar_alloc_cnt[gg]),
            .o_addr_k(g_ar_addr_k[gg]), .o_len_p_k(g_ar_len_p[gg]),
            .o_lane_k(g_ar_lane_k[gg]),
            .o_vb_first(g_ar_vbf[gg]), .o_vb_last(g_ar_vbl[gg]),
            .o_axi_beats(g_ar_axi_beats[gg]), .o_last_k(g_ar_last[gg]),
            .o_addition_k(g_ar_add[gg]), .o_N()
        );
    end
    endgenerate

    //=======================================================================
    // atomic decode (AXI5)
    //=======================================================================
    wire [3:0] at_opcode;
    wire       at_need_r;
    wire [MOD_PW-1:0] at_mod;
    generate
    if (ATOMIC_EN) begin : g_at
        adapter_atomic #(.AWATOP_W(AWATOP_PW), .MOD_W(MOD_W))
        u_atomic (.i_awatop(awatop), .o_opcode(at_opcode), .o_need_r(at_need_r), .o_mod(at_mod));
    end else begin : g_noat
        assign at_opcode = `LB_OP_WR;
        assign at_need_r = 1'b0;
        assign at_mod    = {MOD_PW{1'b0}};
    end
    endgenerate
    wire       aw_is_atomic = ATOMIC_EN && (|awatop);
    wire [3:0] aw_op        = aw_is_atomic ? at_opcode : `LB_OP_WR;
    wire       aw_need_r    = aw_is_atomic ? at_need_r : 1'b0;
    wire [MOD_PW-1:0] aw_mod = aw_is_atomic ? at_mod : {MOD_PW{1'b0}};

    //=======================================================================
    // rob (always instantiated: sub-transaction order needs it with SPLIT_EN)
    //=======================================================================
    wire [PEND_TX-1:0] rob_issuable;
    wire [PEND_TX-1:0] rob_pres_b;
    wire [PEND_TX-1:0] rob_pres_ar;
    wire [IDX_W-1:0]   rob_grant_b;
    wire               rob_grant_b_vld;
    wire [IDX_W-1:0]   rob_grant_ar;
    wire               rob_grant_ar_vld;
    wire               b_taken;
    wire               ar_taken;
    wire               rob_wr_en;
    wire               rob_wr_ready;
    wire [IDX_W-1:0]   rob_wr_idx;
    wire [ROB_DATA_W-1:0] rob_wr_data;
    wire [ROB_DATA_W-1:0] rob_rd_data;
    wire               rob_rd_valid;
    wire               rob_wr_b_en;
    wire [IDX_W-1:0]   rob_wr_b_idx;
    wire [ROB_DATA_W-1:0] rob_wr_b_data;
    wire [ROB_DATA_W-1:0] rob_b_data;

    generate
    if (SAME_ID_EN) begin : g_rob
        adapter_rob #(
            .PEND_TX(PEND_TX),
            .ID_W(ID_W),
            .SEQ_W(SEQ_W),
            .ROB_DEPTH(ROB_DEPTH),
            .ROB_DATA_W(ROB_DATA_W),
            .MEM_TYPE(ROB_MEM_TYPE)
        ) u_rob (
            .clk(clk), .rst_n(rst_n),
            .i_valid(e_valid), .i_ext_id(e_ext_id), .i_seq(e_seq),
            .i_issued(e_issued), .i_is_wr(e_is_wr), .i_b_vld(e_b_vld), .i_ar_vld(e_ar_vld),
            .o_issuable(rob_issuable), .o_presentable_b(rob_pres_b),
            .o_presentable_ar(rob_pres_ar),
            .o_grant_b(rob_grant_b), .o_grant_b_vld(rob_grant_b_vld),
            .o_grant_ar(rob_grant_ar), .o_grant_ar_vld(rob_grant_ar_vld),
            .i_b_taken(b_taken), .i_ar_taken(ar_taken),
            .i_wr_en(rob_wr_en), .i_wr_idx(rob_wr_idx), .i_wr_data(rob_wr_data),
            .o_wr_ready(rob_wr_ready),
            .o_rd_data(rob_rd_data), .o_rd_valid(rob_rd_valid),
            .i_wr_b_en(rob_wr_b_en), .i_wr_b_idx(rob_wr_b_idx),
            .i_wr_b_data(rob_wr_b_data), .o_b_data(rob_b_data)
        );
    end else begin : g_norob
        assign rob_issuable     = e_valid & ~e_issued;
        assign rob_pres_b       = e_valid & e_b_vld;
        assign rob_pres_ar      = e_valid & e_ar_vld;
        assign rob_grant_b_vld  = |rob_pres_b;
        assign rob_grant_ar_vld = |rob_pres_ar;
        assign rob_wr_ready     = 1'b0;
        assign rob_rd_valid     = 1'b0;
        assign rob_rd_data      = {ROB_DATA_W{1'b0}};
        assign rob_b_data       = {ROB_DATA_W{1'b0}};
        reg [IDX_W-1:0] gnt_b;
        reg [IDX_W-1:0] gnt_ar;
        integer gb;
        always @* begin
            gnt_b  = {IDX_W{1'b0}};
            gnt_ar = {IDX_W{1'b0}};
            for (gb = 0; gb < PEND_TX; gb = gb + 1) begin
                if (rob_pres_b[gb])  gnt_b  = gb[IDX_W-1:0];
                if (rob_pres_ar[gb]) gnt_ar = gb[IDX_W-1:0];
            end
        end
        assign rob_grant_b  = gnt_b;
        assign rob_grant_ar = gnt_ar;
    end
    endgenerate

    //=======================================================================
    // REQ_R send path (unified: rob-issuable reads)
    //=======================================================================
    wire [PEND_TX-1:0] rdy_send;
    genvar gs;
    generate
    for (gs = 0; gs < PEND_TX; gs = gs + 1) begin : g_rdy
        assign rdy_send[gs] = e_valid[gs] && !e_issued[gs] && !e_is_wr[gs] && rob_issuable[gs];
    end
    endgenerate
    wire rd_send_any = |rdy_send;
    reg [IDX_W-1:0] rd_grant;
    integer rg;
    always @* begin
        rd_grant = 0;
        for (rg = 0; rg < PEND_TX; rg = rg + 1)
            if (rdy_send[rg]) begin rd_grant = rg[IDX_W-1:0]; rg = PEND_TX; end
    end

    reg [EXT_CMD_W-1:0] cmd_entry;
    wire [LEN_W:0] rd_len_p = e_len_p[rd_grant*(LEN_W+1) +: (LEN_W+1)];
    always @* begin
        cmd_entry = { e_qos[rd_grant*QOS_PW +: QOS_PW], `LB_OP_RD,
                      e_addr[rd_grant*ADDR_W +: ADDR_W],
                      rd_len_p[LEN_W-1:0],
                      {{(ID_W-IDX_W){1'b0}}, rd_grant},
                      e_user[rd_grant*USER_CMD_PW +: USER_CMD_PW] };
    end

    assign req_r_valid = rd_send_any;
    assign req_r_data  = cmd_entry;
    assign req_r_addition = ADDITION_EN ? e_addition[rd_grant*ADDITION_W +: ADDITION_W]
                                        : {ADDITION_W{1'b0}};
    wire ar_rd_fire = req_r_valid && req_r_ready;

    //=======================================================================
    // W path
    //=======================================================================
    wire [ADDR_W-1:0] bs_w_addr_k;
    wire [LEN_W:0]    bs_w_len_p;
    wire [LEN_W:0]    bs_w_axi_beats;
    wire [LANE_W:0]   bs_w_add;
    adapter_burst_split #(
        .ADDR_W(ADDR_W), .LEN_W(LEN_W), .W_BYTES(W_BYTES), .J(J), .MAX_SUB(MAX_SUB)
    ) u_bs_w (
        .i_addr(e_addr[awq_head_idx*ADDR_W +: ADDR_W]),
        .i_lane(e_lane[awq_head_idx*LANE_W +: LANE_W]),
        .i_len(e_len[awq_head_idx*LEN_W +: LEN_W]),
        .i_size(e_size[awq_head_idx*SZB +: SZB]),
        .i_sub(cur_sub),
        .o_addr_k(bs_w_addr_k), .o_len_p_k(bs_w_len_p), .o_axi_beats(bs_w_axi_beats),
        .o_addition_k(bs_w_add), .o_last_k(), .o_N(), .o_lane_k(), .o_vb_first(), .o_vb_last()
    );

    wire [LEN_W:0]   cur_beats_k = bs_w_len_p + 1'b1;
    wire [IDX_W-1:0] cur_int_id  = (awq_head_idx + cur_sub) & (PEND_TX - 1);
    wire [LANE_W-1:0] cur_lane = (cur_sub == 0) ? e_lane[awq_head_idx*LANE_W +: LANE_W]
                                                : {LANE_W{1'b0}};
    wire sub_first_beat = (cur_beat == 0);
    wire w_in_last_sub  = (cur_beat == (bs_w_axi_beats - 1'b1));
    wire cur_wrap = (e_burst[awq_head_idx*2 +: 2] == `AXI_BURST_WRAP);
    wire [ADDR_W-1:0] wr_bytes = ({1'b0, e_len[awq_head_idx*LEN_W +: LEN_W]} + 1'b1)
                                 * e_size[awq_head_idx*SZB +: SZB];
    wire [ADDR_W-1:0] wr_mask = wr_bytes - 1'b1;
    wire [ADDR_W-1:0] wr_base = e_addr[awq_head_idx*ADDR_W +: ADDR_W] & ~wr_mask;
    wire [ADDR_W-1:0] wr_ak   = wr_base
        | ((e_addr[awq_head_idx*ADDR_W +: ADDR_W]
            + cur_beat * e_size[awq_head_idx*SZB +: SZB]) & wr_mask);
    wire [ADDR_W-1:0] wr_aligned = e_addr[awq_head_idx*ADDR_W +: ADDR_W]
                                 - {{(ADDR_W-LANE_W){1'b0}}, e_lane[awq_head_idx*LANE_W +: LANE_W]};
    wire [7:0] wr_pos = wr_ak - wr_aligned;

    assign req_w_addition = ADDITION_EN ? bs_w_add : {ADDITION_W{1'b0}};

    reg [EXT_CMD_W-1:0] cmd_w;
    always @* begin
        cmd_w = { e_qos[awq_head_idx*QOS_PW +: QOS_PW],
                  e_opcode[awq_head_idx*4 +: 4],
                  bs_w_addr_k,
                  bs_w_len_p[LEN_W-1:0],
                  {{(ID_W-IDX_W){1'b0}}, cur_int_id},
                  e_user[awq_head_idx*USER_CMD_PW +: USER_CMD_PW] };
    end

    wire w_sub_done;
    wire w_hsk;

    // packer (NARROW_EN=1)
    wire [EXT_REQ_W-1:0] pk_data;
    wire                 pk_valid;
    wire                 pk_ready;
    wire                 pk_done;
    wire                 pk_i_ready;
    wire                 wready_n;

    generate
    if (NARROW_EN) begin : g_w_narrow
        adapter_narrow_pack #(
            .DATA_W(DATA_W), .ID_W(ID_W), .CMD_W(EXT_CMD_W), .MOD_W(MOD_W)
        ) u_pack (
            .clk(clk), .rst_n(rst_n),
            .i_data(wdata), .i_strb(wstrb), .i_last(w_in_last_sub),
            .i_valid(wvalid && !awq_empty), .o_ready(pk_i_ready),
            .i_lane(cur_lane),
            .i_size(e_size[awq_head_idx*SZB +: SZB]),
            .i_start(sub_first_beat),
            .i_wrap(cur_wrap), .i_pos(wr_pos),
            .i_cmd(cmd_w), .i_mod(e_mod[awq_head_idx*MOD_PW +: MOD_PW]),
            .i_txnid(cur_int_id),
            .o_data(pk_data), .o_valid(pk_valid), .i_ready(pk_ready),
            .o_txn_done(pk_done)
        );
        assign req_w_data  = pk_data;
        assign req_w_valid = pk_valid;
        assign pk_ready    = req_w_ready;
        assign wready_n    = pk_i_ready && !awq_empty;
        assign w_hsk       = wvalid && wready_n;
        assign w_sub_done  = pk_done;
    end else begin : g_w_direct
        assign req_w_data  = {cmd_w, e_mod[awq_head_idx*MOD_PW +: MOD_PW],
                              wdata, wstrb, w_in_last_sub,
                              {{(ID_W-IDX_W){1'b0}}, cur_int_id}};
        assign req_w_valid = wvalid && !awq_empty;
        assign wready_n    = req_w_ready && !awq_empty;
        assign w_hsk       = wvalid && wready_n;
        assign w_sub_done  = w_hsk && w_in_last_sub;
    end
    endgenerate
    assign wready = (!awq_empty) ? wready_n : (q_err_mode && (tr_aw || tr_drain));

    //=======================================================================
    // RSP_WR -> B (unified via rob)
    //=======================================================================
    wire [IDX_W-1:0] wr_idx = rsp_wr_ext_txnid[IDX_W-1:0];
    assign rsp_wr_ready = 1'b1;
    wire [IDX_W-1:0] wr_head = e_head[wr_idx*IDX_W +: IDX_W];
    wire b_last_sub = (e_b_cnt[wr_head*(SUB_IDX_W+1) +: (SUB_IDX_W+1)] ==
                       e_subn[wr_head*(SUB_IDX_W+1) +: (SUB_IDX_W+1)] - 1'b1);
    assign rob_wr_b_en  = SAME_ID_EN && rsp_wr_valid && b_last_sub;
    assign rob_wr_b_idx = wr_head;
    wire wr_b_fail = (e_b_err[wr_head] === 1'b1) ||
                     (rsp_wr_resp === `LB_RESP_FAIL) ||
                     (rsp_wr_resp === `LB_RESP_ATOMIC_FAIL);
    assign rob_wr_b_data = {e_ext_id[wr_head*ID_W +: ID_W], {DATA_W{1'b0}},
                            wr_b_fail ? `AXI_RESP_SLVERR : `AXI_RESP_OKAY, 1'b1};

    function [1:0] map_resp;
        input [1:0] r;
        begin
            map_resp = (r == `LB_RESP_OK)  ? `AXI_RESP_OKAY  :
                       (r == `LB_RESP_FAIL) ? `AXI_RESP_SLVERR : ATOMIC_FAIL_RESP;
        end
    endfunction

    // B presentation gate: atomic B waits for its R (R-then-B order)
    wire [PEND_TX-1:0] pres_b_eff;
    genvar gb;
    generate
    for (gb = 0; gb < PEND_TX; gb = gb + 1) begin : g_pb
        assign pres_b_eff[gb] = rob_pres_b[gb] && (!e_is_atomic[gb] || !e_need_r[gb] || e_r_pres[gb]);
    end
    endgenerate
    wire b_valid_w = |pres_b_eff;
    // fixed-priority grant among pres_b_eff (rob grant may hit a gated entry)
    reg [IDX_W-1:0] b_idx_r;
    integer bb;
    always @* begin
        b_idx_r = 0;
        for (bb = 0; bb < PEND_TX; bb = bb + 1)
            if (pres_b_eff[bb]) begin b_idx_r = bb[IDX_W-1:0]; bb = PEND_TX; end
    end
    wire [IDX_W-1:0] b_idx = b_idx_r;
    wire b_from_tr = tr_b_vld && !b_valid_w;
    // packed beat: {ext_id, data, resp[1:0], last} — constant LSB fields
    assign bid    = b_from_tr ? tr_bid :
                    (SAME_ID_EN ? rob_b_data[(3+DATA_W) +: ID_W]
                                : e_ext_id[b_idx*ID_W +: ID_W]);
    assign bresp  = b_from_tr ? `AXI_RESP_SLVERR :
                    (SAME_ID_EN ? rob_b_data[1 +: 2]
                                : (e_b_err[b_idx] ? `AXI_RESP_SLVERR : `AXI_RESP_OKAY));
    assign bvalid = b_valid_w || tr_b_vld;
    assign b_taken = b_valid_w && bready;

    //=======================================================================
    // RSP_RD -> R
    //=======================================================================
    wire [IDX_W-1:0] rd_idx = rsp_rd_ext_txnid[IDX_W-1:0];
    wire rd_is_atomic = e_is_atomic[rd_idx];
    wire rd_wrap      = (e_burst[rd_idx*2 +: 2] == `AXI_BURST_WRAP);
    wire [ADDR_W-1:0] rd_wrap_bytes =
        ({1'b0, e_len[rd_idx*LEN_W +: LEN_W]} + 1'b1) * e_size[rd_idx*SZB +: SZB];
    wire [ADDR_W-1:0] rd_addr_w = e_addr[rd_idx*ADDR_W +: ADDR_W];
    wire [7:0] rd_start_off = rd_addr_w[7:0] & (rd_wrap_bytes[7:0] - 8'b1);
    wire rd_direct    = !rd_wrap && (e_size[rd_idx*SZB +: SZB] == W_BYTES) &&
                        (e_lane[rd_idx*LANE_W +: LANE_W] == 0);

    wire sp_ready;
    wire sp_rvalid;
    wire [DATA_W-1:0] sp_rdata;
    wire sp_rlast;
    wire [1:0] sp_rresp;
    wire sp_entry_done;
    wire sp_done_txn;
    wire [DATA_W-1:0] sp_acc_data;
    wire [W_BYTES-1:0] sp_acc_vld;
    wire [LANE_W:0] sp_acc_cnt;
    wire [LEN_W+1:0] sp_bl;

    wire rd_is_first = e_rd_first[rd_idx];
    wire [LANE_W:0] rd_last_vb = ADDITION_EN
        ? (W_BYTES[LANE_W:0] - rsp_rd_addition)
        : e_vb_last[rd_idx*(LANE_W+1) +: (LANE_W+1)];
    wire [LANE_W:0] rd_vbytes =
        rd_is_first ? e_vb_first[rd_idx*(LANE_W+1) +: (LANE_W+1)]
                    : (rsp_rd_last ? rd_last_vb
                                   : {1'b0, W_BYTES[LANE_W:0]});

    generate
    if (NARROW_EN) begin : g_sp
        adapter_narrow_split #(.DATA_W(DATA_W), .LEN_W(LEN_W), .ID_W(ID_W))
        u_split (
            .clk(clk), .rst_n(rst_n),
            .i_data(rsp_rd_data), .i_last(rsp_rd_last), .i_resp(rsp_rd_resp),
            .i_valid(rsp_rd_valid && !rd_is_atomic && !rd_direct && sp_ready),
            .o_ready(sp_ready),
            .i_acc_data(e_rd_acc[rd_idx*DATA_W +: DATA_W]),
            .i_acc_vld(e_rd_vld[rd_idx*W_BYTES +: W_BYTES]),
            .i_acc_cnt(e_rd_cnt[rd_idx*(LANE_W+1) +: (LANE_W+1)]),
            .i_beats_left(e_bl[rd_idx*(LEN_W+1) +: (LEN_W+1)]),
            .i_lane(e_lane[rd_idx*LANE_W +: LANE_W]),
            .i_size(e_size[rd_idx*SZB +: SZB]),
            .i_vbytes(rd_vbytes),
            .i_first(rd_is_first),
            .i_emit_cnt(e_axi_total[rd_idx*(LEN_W+1) +: (LEN_W+1)] -
                        e_bl[rd_idx*(LEN_W+1) +: (LEN_W+1)]),
            .i_wrap(rd_wrap),
            .i_start_off(rd_start_off),
            .i_axi_len(e_len[rd_idx*LEN_W +: LEN_W]),
            .o_acc_data(sp_acc_data), .o_acc_vld(sp_acc_vld), .o_acc_cnt(sp_acc_cnt),
            .o_beats_left(sp_bl), .o_entry_done(sp_entry_done), .o_done_txn(sp_done_txn),
            .o_rdata(sp_rdata), .o_rlast(sp_rlast), .o_rresp(sp_rresp),
            .o_rvalid(sp_rvalid),
            .i_rready(SAME_ID_EN ? (rob_wr_ready &&
                                    !(rsp_rd_valid && !rd_is_atomic && rd_direct))
                                 : (skid_room && !rd_push))
        );
    end else begin : g_sp_no
        assign sp_ready      = 1'b0;
        assign sp_rvalid     = 1'b0;
        assign sp_rdata      = {DATA_W{1'b0}};
        assign sp_rlast      = 1'b0;
        assign sp_rresp      = 2'b00;
        assign sp_entry_done = 1'b0;
        assign sp_done_txn   = 1'b0;
        assign sp_acc_data   = {DATA_W{1'b0}};
        assign sp_acc_vld    = {W_BYTES{1'b0}};
        assign sp_acc_cnt    = {(LANE_W+1){1'b0}};
        assign sp_bl         = {(LEN_W+2){1'b0}};
    end
    endgenerate

    // atomic R presentation (R before B)
    wire [PEND_TX-1:0] pres_ar = rob_pres_ar;
    wire ar_grant_vld = |pres_ar;
    reg [IDX_W-1:0] ar_grant;
    integer aa;
    always @* begin
        ar_grant = 0;
        for (aa = 0; aa < PEND_TX; aa = aa + 1)
            if (pres_ar[aa]) begin ar_grant = aa[IDX_W-1:0]; aa = PEND_TX; end
    end

    //=======================================================================
    // R output skid
    //=======================================================================
    localparam RSKP_W = `adp_clog2(R_SKID_DEPTH) + 1;
    reg [R_SKID_DEPTH*RSK_W-1:0] rsk_mem;
    reg [RSKP_W-1:0] rsk_cnt;
    reg [RSKP_W-2:0] rsk_wp;
    reg [RSKP_W-2:0] rsk_rp;

    wire skid_room      = (rsk_cnt < R_SKID_DEPTH);
    wire skid_not_empty = (rsk_cnt > 0);
    wire r_room         = SAME_ID_EN ? rob_wr_ready : skid_room;

    wire rd_push = (!SAME_ID_EN) && rsp_rd_valid && rsp_rd_ready && !rd_is_atomic &&
                   rd_direct && skid_room;
    wire sp_push = (!SAME_ID_EN) && sp_rvalid && skid_room && !rd_push;
    wire at_push = (!SAME_ID_EN) && ar_grant_vld && skid_room && !(rd_push || sp_push);

    wire rob_push_d = SAME_ID_EN && rsp_rd_valid && rsp_rd_ready && !rd_is_atomic && rd_direct;
    wire rob_push_s = SAME_ID_EN && sp_rvalid && rob_wr_ready && !rob_push_d;
    wire rob_push_a = SAME_ID_EN && rsp_rd_valid && rsp_rd_ready && rd_is_atomic &&
                      !rob_push_d && !rob_push_s;
    assign rob_wr_en  = rob_push_d || rob_push_s || rob_push_a;
    assign rob_wr_idx = rd_idx;
    // ROB last bit = last beat of THIS table entry (LiteBus/sub last).
    // AXI rlast is reconstructed at presentation with e_last_sub so a
    // non-final split sub can be freed without asserting AXI rlast.
    assign rob_wr_data = rob_push_d ?
        {e_ext_id[rd_idx*ID_W +: ID_W], rsp_rd_data, map_resp(rsp_rd_resp),
         rsp_rd_last} :
        rob_push_s ?
        {e_ext_id[rd_idx*ID_W +: ID_W], sp_rdata, map_resp(sp_rresp),
         sp_rlast} :
        {e_ext_id[rd_idx*ID_W +: ID_W], rsp_rd_data, map_resp(rsp_rd_resp), 1'b1};

    wire [RSK_W-1:0] push_dat =
        rd_push ? {e_ext_id[rd_idx*ID_W +: ID_W], rsp_rd_data, map_resp(rsp_rd_resp),
                   rsp_rd_last && e_last_sub[rd_idx]} :
        sp_push ? {e_ext_id[rd_idx*ID_W +: ID_W], sp_rdata, map_resp(sp_rresp),
                   sp_rlast && e_last_sub[rd_idx]} :
                  {e_ext_id[ar_grant*ID_W +: ID_W], e_ar_q[ar_grant*DATA_W +: DATA_W],
                   map_resp(e_ar_resp[ar_grant*2 +: 2]), 1'b1};
    wire any_push = rd_push || sp_push || at_push;

    assign rsp_rd_ready =
        rd_is_atomic ? (SAME_ID_EN ? rob_wr_ready : 1'b1) :
        rd_direct    ? r_room :
        NARROW_EN    ? sp_ready : 1'b0;

    wire [RSK_W-1:0] rsk_head = rsk_mem[rsk_rp*RSK_W +: RSK_W];
    wire [RSK_W-1:0] r_beat = SAME_ID_EN ? rob_rd_data : rsk_head;
    wire r_pend = SAME_ID_EN ? rob_rd_valid : skid_not_empty;
    wire r_from_tr = tr_r_vld && !r_pend;
    assign rid    = r_from_tr ? tr_rid : r_beat[(3+DATA_W) +: ID_W];
    assign rdata  = r_from_tr ? {DATA_W{1'b0}} : r_beat[3 +: DATA_W];
    assign rresp  = r_from_tr ? `AXI_RESP_SLVERR : r_beat[1 +: 2];
    assign rlast  = r_from_tr ? 1'b1 :
                    (SAME_ID_EN ? (r_beat[0] && (e_last_sub[rob_grant_ar] ||
                                                 e_is_atomic[rob_grant_ar]))
                                : r_beat[0]);
    assign rvalid = r_pend || tr_r_vld;
    wire rsk_pop  = skid_not_empty && rready;

    assign ar_taken = SAME_ID_EN ? (rob_rd_valid && rready && !r_from_tr) : at_push;

    //=======================================================================
    // table free / presentation events
    //=======================================================================
    wire [PEND_TX-1:0] b_pres_v;
    wire [PEND_TX-1:0] b_free_v;
    wire [PEND_TX-1:0] rd_free_v;
    wire [PEND_TX-1:0] sp_free_v;
    wire [PEND_TX-1:0] at_rpres_v;
    wire [IDX_W-1:0]   r_done_idx = SAME_ID_EN ? rob_grant_ar : rd_idx;
    genvar gf;
    generate
    for (gf = 0; gf < PEND_TX; gf = gf + 1) begin : g_free
        assign b_pres_v[gf]   = b_taken && (b_idx == gf) && e_valid[gf];
        assign at_rpres_v[gf] = ar_taken && e_is_atomic[gf] &&
                                (SAME_ID_EN ? (rob_grant_ar == gf)
                                            : (ar_grant == gf));
        // free the whole transaction group when the head's B is presented
        assign b_free_v[gf]   = b_taken && (e_head[gf*IDX_W +: IDX_W] == b_idx) &&
                                e_valid[gf] &&
                                (!e_is_atomic[gf] || !e_need_r[gf] ||
                                 at_rpres_v[gf] || e_r_pres[gf]);
        assign rd_free_v[gf]  = SAME_ID_EN ?
                                (ar_taken && r_beat[0] && (r_done_idx == gf) &&
                                 !e_is_wr[gf] && !e_is_atomic[gf]) :
                                (rd_push && (rd_idx == gf) && rsp_rd_last);
        assign sp_free_v[gf]  = SAME_ID_EN ? 1'b0 : (sp_done_txn && (rd_idx == gf));
    end
    endgenerate

    //=======================================================================
    // sequential updates
    //=======================================================================
    integer k;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            e_valid  <= {PEND_TX{1'b0}};
            e_issued <= {PEND_TX{1'b0}};
            e_b_vld  <= {PEND_TX{1'b0}};
            e_b_err  <= {PEND_TX{1'b0}};
            e_last_sub <= {PEND_TX{1'b0}};
            e_b_pres <= {PEND_TX{1'b0}};
            e_ar_vld <= {PEND_TX{1'b0}};
            e_r_pres <= {PEND_TX{1'b0}};
            seq_ctr  <= {SEQ_W{1'b0}};
            awq_wr   <= {QPTR_W{1'b0}};
            awq_rd   <= {QPTR_W{1'b0}};
            cur_sub  <= {SUB_IDX_W{1'b0}};
            cur_beat <= {(LEN_W+1){1'b0}};
            rsk_cnt  <= {RSKP_W{1'b0}};
            rsk_wp   <= {(RSKP_W-1){1'b0}};
            rsk_rp   <= {(RSKP_W-1){1'b0}};
            tr_r_vld <= 1'b0;
            tr_aw    <= 1'b0;
            tr_drain <= 1'b0;
            tr_b_vld <= 1'b0;
            tr_rid   <= {ID_W{1'b0}};
            tr_bid   <= {ID_W{1'b0}};
        end else begin
            //----------- frees & presentation flags
            for (k = 0; k < PEND_TX; k = k + 1) begin
                if (b_free_v[k] || rd_free_v[k] || sp_free_v[k])
                    e_valid[k] <= 1'b0;
                if (b_pres_v[k]) begin
                    e_b_pres[k] <= 1'b1;
                    e_b_vld[k]  <= 1'b0;
                end
                if (at_rpres_v[k]) begin
                    e_r_pres[k] <= 1'b1;
                    e_ar_vld[k] <= 1'b0;
                end
            end
            //----------- allocations
            if (ar_hsk) begin
                for (k = 0; k < PEND_TX; k = k + 1) begin
                    if (ar_alloc[k]) begin
                        e_valid[k]     <= 1'b1;
                        e_ext_id[k*ID_W +: ID_W] <= arid;
                        e_is_wr[k]     <= 1'b0;
                        e_is_atomic[k] <= 1'b0;
                        e_need_r[k]    <= 1'b0;
                        e_addr[k*ADDR_W +: ADDR_W] <= g_ar_addr_k[k];
                        e_size[k*SZB +: SZB] <= ar_sz;
                        e_lane[k*LANE_W +: LANE_W] <= g_ar_lane_k[k];
                        e_len[k*LEN_W +: LEN_W] <= arlen;
                        e_len_p[k*(LEN_W+1) +: (LEN_W+1)] <= g_ar_len_p[k];
                        e_opcode[k*4 +: 4] <= `LB_OP_RD;
                        e_mod[k*MOD_PW +: MOD_PW] <= {MOD_PW{1'b0}};
                        e_qos[k*QOS_PW +: QOS_PW] <= {QOS_PW{1'b0}};
                        e_user[k*USER_CMD_PW +: USER_CMD_PW] <= {USER_CMD_PW{1'b0}};
                        e_issued[k]    <= 1'b0;
                        e_b_vld[k]     <= 1'b0;
                        e_b_pres[k]    <= 1'b0;
                        e_ar_vld[k]    <= 1'b0;
                        e_r_pres[k]    <= 1'b0;
                        e_bl[k*(LEN_W+1) +: (LEN_W+1)] <= g_ar_axi_beats[k];
                        e_axi_total[k*(LEN_W+1) +: (LEN_W+1)] <= g_ar_axi_beats[k];
                        e_rd_first[k]    <= 1'b1;
                        e_last_sub[k]    <= g_ar_last[k];
                        e_rd_acc[k*DATA_W +: DATA_W] <= {DATA_W{1'b0}};
                        e_rd_vld[k*W_BYTES +: W_BYTES] <= {W_BYTES{1'b0}};
                        e_rd_cnt[k*(LANE_W+1) +: (LANE_W+1)] <= {(LANE_W+1){1'b0}};
                        e_vb_first[k*(LANE_W+1) +: (LANE_W+1)] <= g_ar_vbf[k];
                        e_vb_last[k*(LANE_W+1) +: (LANE_W+1)] <= g_ar_vbl[k];
                        e_addition[k*ADDITION_W +: ADDITION_W] <= g_ar_add[k];
                        e_burst[k*2 +: 2] <= arburst;
                        e_seq[k*SEQ_W +: SEQ_W] <= seq_ctr + ar_alloc_cnt[k];
                    end
                end
                seq_ctr <= seq_ctr + ar_N;
            end
            if (aw_hsk) begin
                for (k = 0; k < PEND_TX; k = k + 1) begin
                    if (aw_alloc[k]) begin
                        e_valid[k]     <= 1'b1;
                        e_ext_id[k*ID_W +: ID_W] <= awid;
                        e_is_wr[k]     <= 1'b1;
                        e_is_atomic[k] <= aw_is_atomic;
                        e_need_r[k]    <= aw_need_r;
                        e_addr[k*ADDR_W +: ADDR_W] <= g_aw_addr_k[k];
                        e_size[k*SZB +: SZB] <= aw_sz;
                        e_lane[k*LANE_W +: LANE_W] <= g_aw_lane_k[k];
                        e_len[k*LEN_W +: LEN_W] <= awlen;
                        e_len_p[k*(LEN_W+1) +: (LEN_W+1)] <= g_aw_len_p[k];
                        e_opcode[k*4 +: 4] <= aw_op;
                        e_mod[k*MOD_PW +: MOD_PW] <= aw_mod;
                        e_qos[k*QOS_PW +: QOS_PW] <= {QOS_PW{1'b0}};
                        e_user[k*USER_CMD_PW +: USER_CMD_PW] <= {USER_CMD_PW{1'b0}};
                        e_issued[k]    <= 1'b1;
                        e_b_vld[k]     <= 1'b0;
                        e_b_pres[k]    <= 1'b0;
                        e_ar_vld[k]    <= 1'b0;
                        e_r_pres[k]    <= 1'b0;
                        e_bl[k*(LEN_W+1) +: (LEN_W+1)] <= {(LEN_W+1){1'b0}};
                        e_rd_acc[k*DATA_W +: DATA_W] <= {DATA_W{1'b0}};
                        e_rd_vld[k*W_BYTES +: W_BYTES] <= {W_BYTES{1'b0}};
                        e_rd_cnt[k*(LANE_W+1) +: (LANE_W+1)] <= {(LANE_W+1){1'b0}};
                        e_vb_first[k*(LANE_W+1) +: (LANE_W+1)] <= g_aw_vbf[k];
                        e_vb_last[k*(LANE_W+1) +: (LANE_W+1)] <= g_aw_vbl[k];
                        e_addition[k*ADDITION_W +: ADDITION_W] <= g_aw_add[k];
                        e_burst[k*2 +: 2] <= awburst;
                        e_seq[k*SEQ_W +: SEQ_W] <= seq_ctr + aw_alloc_cnt[k];
                        e_head[k*IDX_W +: IDX_W] <= aw_head_idx;
                        e_subn[k*(SUB_IDX_W+1) +: (SUB_IDX_W+1)] <= aw_N;
                        e_b_cnt[k*(SUB_IDX_W+1) +: (SUB_IDX_W+1)] <= {(SUB_IDX_W+1){1'b0}};
                        e_b_err[k] <= 1'b0;
                        e_last_sub[k] <= g_aw_last[k];
                    end
                end
                seq_ctr <= seq_ctr + aw_N;
                awq_mem[awq_wr[QPTR_W-2:0]*Q_W +: Q_W] <= {aw_N, aw_head_idx};
                awq_wr <= (awq_wr == PEND_WR) ? {QPTR_W{1'b0}} : (awq_wr + 1'b1);
            end
            //----------- REQ_R fire
            if (ar_rd_fire)
                e_issued[rd_grant] <= 1'b1;
            //----------- W progress
            if (w_sub_done) begin
                if (cur_sub == (awq_head_n - 1)) begin
                    awq_rd  <= (awq_rd == PEND_WR) ? {QPTR_W{1'b0}} : (awq_rd + 1'b1);
                    cur_sub <= {SUB_IDX_W{1'b0}};
                end else begin
                    cur_sub <= cur_sub + 1'b1;
                end
                // a new beat accepted in the same cycle belongs to the next sub
                cur_beat <= w_hsk ? 1'b1 : {(LEN_W+1){1'b0}};
            end else if (w_hsk) begin
                cur_beat <= cur_beat + 1'b1;
            end
            //----------- RSP_WR receive (aggregate subs' Bs at the head entry)
            if (rsp_wr_valid) begin
                e_b_q[wr_idx*2 +: 2] <= rsp_wr_resp;
                if (rsp_wr_resp != `LB_RESP_OK)
                    e_b_err[e_head[wr_idx*IDX_W +: IDX_W]] <= 1'b1;
                if (e_b_cnt[e_head[wr_idx*IDX_W +: IDX_W]*(SUB_IDX_W+1) +: (SUB_IDX_W+1)] ==
                    e_subn[e_head[wr_idx*IDX_W +: IDX_W]*(SUB_IDX_W+1) +: (SUB_IDX_W+1)] - 1'b1)
                    e_b_vld[e_head[wr_idx*IDX_W +: IDX_W]] <= 1'b1;
                e_b_cnt[e_head[wr_idx*IDX_W +: IDX_W]*(SUB_IDX_W+1) +: (SUB_IDX_W+1)] <=
                    e_b_cnt[e_head[wr_idx*IDX_W +: IDX_W]*(SUB_IDX_W+1) +: (SUB_IDX_W+1)] + 1'b1;
            end
            //----------- RSP_RD atomic receive
            if (rsp_rd_valid && rsp_rd_ready && rd_is_atomic) begin
                e_ar_q[rd_idx*DATA_W +: DATA_W] <= rsp_rd_data;
                e_ar_resp[rd_idx*2 +: 2]        <= rsp_rd_resp;
                e_ar_vld[rd_idx]                <= 1'b1;
            end
            //----------- splitter entry writeback
            if (sp_entry_done) begin
                e_rd_acc[rd_idx*DATA_W +: DATA_W]         <= sp_acc_data;
                e_rd_vld[rd_idx*W_BYTES +: W_BYTES]       <= sp_acc_vld;
                e_rd_cnt[rd_idx*(LANE_W+1) +: (LANE_W+1)] <= sp_acc_cnt;
                e_bl[rd_idx*(LEN_W+1) +: (LEN_W+1)]       <= sp_bl;
            end
            // first-beat flag clears when the splitter accepts a beat
            if (rsp_rd_valid && rsp_rd_ready && !rd_is_atomic && !rd_direct)
                e_rd_first[rd_idx] <= 1'b0;
            //----------- skid push/pop
            if (any_push && rsk_pop) begin
                rsk_mem[rsk_wp*RSK_W +: RSK_W] <= push_dat;
                rsk_wp <= rsk_wp + 1'b1;
                rsk_rp <= rsk_rp + 1'b1;
            end else if (any_push) begin
                rsk_mem[rsk_wp*RSK_W +: RSK_W] <= push_dat;
                rsk_wp <= rsk_wp + 1'b1;
                rsk_cnt <= rsk_cnt + 1'b1;
            end else if (rsk_pop) begin
                rsk_rp  <= rsk_rp + 1'b1;
                rsk_cnt <= rsk_cnt - 1'b1;
            end
            //----------- QCH err_mode truncate
            if (tr_r_vld && rready && !r_pend)
                tr_r_vld <= 1'b0;
            if (q_err_mode && arvalid && arready) begin
                tr_r_vld <= 1'b1;
                tr_rid   <= arid;
            end
            if (tr_b_vld && bready && !b_valid_w)
                tr_b_vld <= 1'b0;
            if ((tr_aw || tr_drain) && wvalid && wready && awq_empty) begin
                tr_aw <= 1'b0;
                if (wlast) begin
                    tr_drain <= 1'b0;
                    tr_b_vld <= 1'b1;
                end else
                    tr_drain <= 1'b1;
            end
            if (q_err_mode && awvalid && awready) begin
                tr_bid <= awid;
                if (wvalid && wlast) begin
                    tr_b_vld <= 1'b1;
                    tr_aw    <= 1'b0;
                    tr_drain <= 1'b0;
                end else begin
                    tr_aw <= 1'b1;
                end
            end
        end
    end

`ifndef LB_NO_ASSERT
    // synthesis translate_off
    always @(posedge clk) begin
        if (awvalid && (awsize > LANE_W)) begin
            $display("[%0t] ERROR %m: awsize=%0d exceeds W_BYTES", $time, awsize);
        end
        if (arvalid && (arsize > LANE_W)) begin
            $display("[%0t] ERROR %m: arsize=%0d exceeds W_BYTES", $time, arsize);
        end
        if (awvalid && (awburst == `AXI_BURST_FIXED)) begin
            $display("[%0t] ERROR %m: FIXED burst not supported", $time);
        end
        if (arvalid && (arburst == `AXI_BURST_FIXED)) begin
            $display("[%0t] ERROR %m: FIXED burst not supported", $time);
        end
        if (awvalid && (awburst == `AXI_BURST_WRAP) && !NARROW_EN) begin
            $display("[%0t] ERROR %m: WRAP requires NARROW_EN", $time);
        end
        if (arvalid && (arburst == `AXI_BURST_WRAP) && !NARROW_EN) begin
            $display("[%0t] ERROR %m: WRAP requires NARROW_EN", $time);
        end
        if (rsp_wr_valid && !e_valid[wr_idx]) begin
            $display("[%0t] ERROR %m: RSP_WR txnid=%0d no entry", $time, rsp_wr_ext_txnid);
        end
        if (rsp_rd_valid && !e_valid[rd_idx]) begin
            $display("[%0t] ERROR %m: RSP_RD txnid=%0d no entry", $time, rsp_rd_ext_txnid);
        end
        if (SPLIT_EN && awvalid && (aw_N > MAX_SUB)) begin
            $display("[%0t] ERROR %m: aw_N=%0d exceeds MAX_SUB=%0d", $time, aw_N, MAX_SUB);
        end
        if (SPLIT_EN && awvalid && (awburst == `AXI_BURST_WRAP)) begin
            $display("[%0t] ERROR %m: WRAP+SPLIT not supported", $time);
        end
        if (SPLIT_EN && arvalid && (arburst == `AXI_BURST_WRAP)) begin
            $display("[%0t] ERROR %m: WRAP+SPLIT not supported", $time);
        end
        if (SPLIT_EN && !SAME_ID_EN) begin
            $display("[%0t] ERROR %m: SPLIT_EN requires SAME_ID_EN", $time);
        end
    end
    // synthesis translate_on
`endif

endmodule
