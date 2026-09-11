//============================================================================
// Filename    : adapter_sbs.v
// Description : [feature] slv-side core: LiteBus TNIU interface -> AMBA master
//               port. SBS_EN=0 : direct translation (version S1).
//               SBS_EN=1 : adds Simple burst split (version S2): incoming
//               transactions with len > SLV_MAX_LEN are split into
//               sub-transactions (same txnid; slave must support same-ID).
//               Writes are serialized at the AW/W channel (AXI4 forbids W
//               interleave); a pending split B-aggregation coexists with the
//               next write stream and B responses are routed by bid.
//               One split-read active at a time; non-split reads pass
//               through concurrently (AR channel priority: passthrough).
//============================================================================
`include "adapter_ip_defs.vh"
`default_nettype none
`timescale 1ns/1ps

module adapter_sbs #(
    parameter ADDR_W  = 32,
    parameter DATA_W  = 64,
    parameter LEN_W   = 8,
    parameter ID_W    = 8,
    parameter MOD_W   = 0,
    parameter ATOMIC_EN = 0,
    parameter SBS_EN  = 0,
    parameter SLV_MAX_LEN = 15,
    parameter USER_CMD_PW = 1,
    parameter USER_RSP_PW  = 1,
    parameter QOS_PW  = 1,
    parameter MOD_PW  = (MOD_W < 1) ? 1 : MOD_W,
    parameter W_BYTES = DATA_W/8,
    parameter EXT_CMD_W = QOS_PW + 4 + ADDR_W + LEN_W + ID_W + USER_CMD_PW,
    parameter EXT_WD_W  = DATA_W + W_BYTES + 1 + ID_W,
    parameter EXT_REQ_W = EXT_CMD_W + MOD_PW + EXT_WD_W,
    parameter MAX_SUB = (SBS_EN ? ((1 << LEN_W) / (SLV_MAX_LEN + 1)) + 2 : 1),
    parameter SUBW    = `adp_clog2(MAX_SUB) + 1
) (
    input  wire                   clk,
    input  wire                   rst_n,
    // ---------------- LiteBus TNIU Slave IP (L0, INIU mirror) ----------------
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
    // ---------------- AMBA master port ----------------
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
    output wire [4:0]             awatop
);
    //-----------------------------------------------------------------
    // CMD field slicing (CMD = {qos, opcode, addr, len, txnid, user})
    //-----------------------------------------------------------------
    localparam OP_HI   = EXT_CMD_W - QOS_PW - 1;
    localparam ADDR_HI = EXT_CMD_W - QOS_PW - 4 - 1;
    localparam LEN_HI  = EXT_CMD_W - QOS_PW - 4 - ADDR_W - 1;
    localparam TXN_HI  = EXT_CMD_W - QOS_PW - 4 - ADDR_W - LEN_W - 1;
    localparam LANE_W  = `adp_clog2(W_BYTES);

    // align an address down to W_BYTES (TNIU delivers phase-aligned WD,
    // so the AMBA side must use the ALIGNED base address)
    function [ADDR_W-1:0] align_dn;
        input [ADDR_W-1:0] a;
        begin
            align_dn = a & ~{{(ADDR_W-LANE_W){1'b0}}, {LANE_W{1'b1}}};
        end
    endfunction

    function [1:0] map_resp;
        input [1:0] r;
        begin
            map_resp = ((r == `AXI_RESP_OKAY) || (r == `AXI_RESP_EXOKAY)) ? `LB_RESP_OK : `LB_RESP_FAIL;
        end
    endfunction

    // L0 combined REQ_W / REQ_R -> internal CMD+WD / rq_r
    wire [EXT_CMD_W-1:0]           ext_rq_r_data = req_r_data;
    wire                           ext_rq_r_valid = req_r_valid;
    wire                           ext_rq_r_ready;
    assign req_r_ready = ext_rq_r_ready;

    wire [EXT_CMD_W+MOD_PW-1:0]    ext_cmd_mod  = req_w_data[EXT_WD_W +: (EXT_CMD_W+MOD_PW)];
    wire [EXT_CMD_W-1:0]           ext_cmd_data = ext_cmd_mod[MOD_PW +: EXT_CMD_W];
    wire [EXT_WD_W-1:0]            ext_wd_data  = req_w_data[0 +: EXT_WD_W];
    reg                            wr_first;
    wire                           ext_cmd_valid = req_w_valid && wr_first;
    wire                           ext_wd_valid  = req_w_valid;
    wire                           ext_cmd_ready;
    wire                           ext_wd_ready;
    assign req_w_ready = wr_first ? (ext_cmd_ready && ext_wd_ready) : ext_wd_ready;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            wr_first <= 1'b1;
        else if (req_w_valid && req_w_ready)
            wr_first <= ext_wd_data[ID_W];
    end

    //-----------------------------------------------------------------
    // effective write CMD (live or held)
    //-----------------------------------------------------------------
    wire [3:0]      cmd_opcode = ext_cmd_data[OP_HI -: 4];
    wire [LEN_W-1:0] cmd_len   = ext_cmd_data[LEN_HI -: LEN_W];
    wire [ID_W-1:0]  cmd_txnid = ext_cmd_data[TXN_HI -: ID_W];
    wire [ADDR_W-1:0] cmd_addr = ext_cmd_data[ADDR_HI -: ADDR_W];

    wire [3:0]      held_opcode = held_cmd[MOD_PW + OP_HI -: 4];
    wire [LEN_W-1:0] held_len   = held_cmd[MOD_PW + LEN_HI -: LEN_W];
    wire [ID_W-1:0]  held_txnid = held_cmd[MOD_PW + TXN_HI -: ID_W];
    wire [ADDR_W-1:0] held_addr = held_cmd[MOD_PW + ADDR_HI -: ADDR_W];

    reg [EXT_CMD_W+MOD_PW-1:0] held_cmd;
    reg held_v;

    wire use_held     = held_v;
    wire [3:0]      e_opcode = use_held ? held_opcode : cmd_opcode;
    wire [LEN_W-1:0] e_len    = use_held ? held_len   : cmd_len;
    wire [ID_W-1:0]  e_txnid  = use_held ? held_txnid : cmd_txnid;
    wire [ADDR_W-1:0] e_addr  = use_held ? held_addr  : cmd_addr;
    wire [MOD_PW-1:0] e_mod   = use_held ? held_cmd[0 +: MOD_PW] : ext_cmd_mod[0 +: MOD_PW];

    wire do_split_wr = SBS_EN && (e_len > SLV_MAX_LEN);

    //-----------------------------------------------------------------
    // write stream FSM (AW + W per write; sub AW re-issue at boundaries)
    //-----------------------------------------------------------------
    reg aw_issued;             // current write stream active (W not finished)
    reg split_wr;              // current stream is a split write
    reg [ADDR_W-1:0] sp_wr_base;
    reg [LEN_W:0]    sp_wr_left;      // beats remaining in the whole split write
    reg [SUBW-1:0]   sp_wr_sub_done;  // sub-transactions whose AW issued
    reg [LEN_W:0]    wd_cnt;
    reg [LEN_W:0]    wd_need_r;       // beats in the current stream/sub (registered at AW)
    reg [ID_W-1:0]   cur_txnid;
    reg [3:0]        cur_opcode;
    reg [MOD_PW-1:0] cur_mod;
    reg [ID_W-1:0]   sp_wr_txnid;

    // sub geometry for the AW about to be issued
    reg [LEN_W-1:0] aw_len_r;
    reg [ADDR_W-1:0] aw_addr_r;
    always @* begin
        if (split_wr) begin
            if (sp_wr_left > (SLV_MAX_LEN + 1))
                aw_len_r = SLV_MAX_LEN;
            else
                aw_len_r = sp_wr_left - 1'b1;
            aw_addr_r = sp_wr_base + sp_wr_sub_done * ((SLV_MAX_LEN + 1) * W_BYTES);
        end else if (do_split_wr) begin
            // first sub: must advertise the sub length, not the parent len
            aw_len_r  = ((e_len + 1'b1) > (SLV_MAX_LEN + 1)) ? SLV_MAX_LEN : e_len;
            aw_addr_r = align_dn(e_addr);
        end else begin
            aw_len_r  = e_len;
            aw_addr_r = align_dn(e_addr);
        end
    end

    // AWATOP for atomic opcodes (C/D/E/F -> {mod, op}).
    // assign (not always @*) so ATOMIC_EN=0 is not an empty sensitivity list.
    generate
    if (ATOMIC_EN) begin : g_awatop
        assign awatop = e_opcode[3] ?
                        { ((MOD_PW >= 3) ? e_mod[2:0] : 3'b000), e_opcode[1:0] } : 5'b0;
    end else begin : g_no_awatop
        assign awatop = 5'b0;
    end
    endgenerate

    assign awid    = split_wr ? sp_wr_txnid : e_txnid;
    assign awaddr  = aw_addr_r;
    assign awlen   = aw_len_r;
    assign awsize  = `adp_clog2(W_BYTES) & 3'h7;
    assign awburst = `AXI_BURST_INCR;
    assign awvalid = !aw_issued && (split_wr || ext_cmd_valid || held_v);

    assign ext_cmd_ready = (aw_issued || split_wr) ? !held_v : awready;

    // WD: allow same-cycle AW+W (combined REQ_W first beat)
    wire w_can = aw_issued || (awvalid && awready);
    wire lb_wlast = ext_wd_data[ID_W];
    // AXI WLAST is the last beat of the *current* AW burst. For a split write
    // that is the sub boundary, not the parent LiteBus last bit.
    wire [LEN_W:0] wr_beat_idx = (awvalid && !aw_issued) ? {(LEN_W+1){1'b0}} : wd_cnt;
    wire [LEN_W:0] wr_need_now = (awvalid && !aw_issued) ?
                                 ({1'b0, aw_len_r} + 1'b1) : wd_need_r;
    wire axi_wlast = split_wr ? (wr_beat_idx == (wr_need_now - 1'b1)) : lb_wlast;
    assign wdata  = ext_wd_data[W_BYTES + 1 + ID_W +: DATA_W];
    assign wstrb  = ext_wd_data[ID_W + 1 +: W_BYTES];
    assign wlast  = axi_wlast;
    assign wvalid = ext_wd_valid && w_can;
    assign ext_wd_ready = wready && w_can;
    wire w_hsk = wvalid && wready;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            aw_issued     <= 1'b0;
            split_wr      <= 1'b0;
            sp_wr_base    <= {ADDR_W{1'b0}};
            sp_wr_left    <= {(LEN_W+1){1'b0}};
            sp_wr_sub_done<= {SUBW{1'b0}};
            wd_cnt        <= {(LEN_W+1){1'b0}};
            wd_need_r     <= {(LEN_W+1){1'b0}};
            held_v        <= 1'b0;
            held_cmd      <= {(EXT_CMD_W+MOD_PW){1'b0}};
            sp_wr_txnid   <= {ID_W{1'b0}};
        end else begin
            // AW issue
            if (awvalid && awready) begin
                aw_issued <= 1'b1;
                if (split_wr) begin
                    wd_need_r <= (sp_wr_left > (SLV_MAX_LEN+1)) ? (SLV_MAX_LEN+1) : sp_wr_left;
                    sp_wr_sub_done <= sp_wr_sub_done + 1'b1;
                end else if (do_split_wr) begin
                    split_wr    <= 1'b1;
                    sp_wr_base  <= align_dn(e_addr);
                    sp_wr_left  <= e_len + 1'b1;
                    sp_wr_sub_done <= 1'b1;
                    sp_wr_txnid <= e_txnid;
                    wd_need_r   <= (e_len + 1'b1 > (SLV_MAX_LEN+1)) ? (SLV_MAX_LEN+1) : (e_len + 1'b1);
                end else begin
                    split_wr  <= 1'b0;
                    wd_need_r <= e_len + 1'b1;
                end
                cur_txnid  <= e_txnid;
                cur_opcode <= e_opcode;
                cur_mod    <= e_mod;
                if (use_held) held_v <= 1'b0;
            end else if (ext_cmd_valid && (aw_issued || split_wr) && !held_v) begin
                held_v    <= 1'b1;
                held_cmd  <= ext_cmd_mod;
            end
            // W progress (lb_wlast = parent last; axi_wlast = current AW last)
            if (w_hsk) begin
                if (awvalid && awready)
                    wd_cnt <= {{LEN_W{1'b0}}, 1'b1};
                else
                    wd_cnt <= wd_cnt + 1'b1;
                if (split_wr && axi_wlast) begin
                    aw_issued <= 1'b0;
                    sp_wr_left <= sp_wr_left - wr_need_now;
                    if (lb_wlast)
                        split_wr <= 1'b0;
                end else if (lb_wlast) begin
                    aw_issued <= 1'b0;
                end
            end else if (awvalid && awready) begin
                wd_cnt <= {(LEN_W+1){1'b0}};
            end
        end
    end

    //-----------------------------------------------------------------
    // B responses: route by bid (split aggregation vs passthrough)
    //-----------------------------------------------------------------
    reg sp_b_active;
    reg [SUBW-1:0] sp_b_need;
    reg [SUBW-1:0] sp_b_got;
    reg sp_b_err;
    reg [ID_W-1:0] sp_b_txnid;
    reg sp_rsp_vld;
    reg [1:0] sp_rsp_resp;

    wire b_is_split = sp_b_active && (bid == sp_b_txnid);
    wire b_pass     = bvalid && !b_is_split;

    assign rsp_wr_valid = sp_rsp_vld || b_pass;
    assign rsp_wr_ext_txnid = sp_rsp_vld ? sp_b_txnid : bid;
    assign rsp_wr_resp      = sp_rsp_vld ? sp_rsp_resp : map_resp(bresp);
    assign rsp_wr_user      = {USER_RSP_PW{1'b0}};
    assign bready = b_is_split ? 1'b1 : (rsp_wr_ready && !sp_rsp_vld);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sp_b_active <= 1'b0;
            sp_b_need   <= {SUBW{1'b0}};
            sp_b_got    <= {SUBW{1'b0}};
            sp_b_err    <= 1'b0;
            sp_b_txnid  <= {ID_W{1'b0}};
            sp_rsp_vld  <= 1'b0;
            sp_rsp_resp <= `LB_RESP_OK;
        end else begin
            // capture B for every split sub from the first AW, so early B
            // cannot leak as a parent LiteBus write response
            if (awvalid && awready && do_split_wr && !split_wr) begin
                sp_b_active <= 1'b1;
                sp_b_need   <= {{(SUBW-1){1'b0}}, 1'b1};
                sp_b_got    <= {SUBW{1'b0}};
                sp_b_err    <= 1'b0;
                sp_b_txnid  <= e_txnid;
            end else if (awvalid && awready && split_wr) begin
                sp_b_need <= sp_b_need + 1'b1;
            end
            if (bvalid && bready && b_is_split) begin
                sp_b_got <= sp_b_got + 1'b1;
                if ((bresp != `AXI_RESP_OKAY) && (bresp != `AXI_RESP_EXOKAY))
                    sp_b_err <= 1'b1;
            end
            if (sp_b_active && !sp_rsp_vld && !split_wr && !aw_issued &&
                (sp_b_got == sp_b_need) && !(bvalid && bready && b_is_split)) begin
                sp_rsp_vld  <= 1'b1;
                sp_rsp_resp <= sp_b_err ? `LB_RESP_FAIL : `LB_RESP_OK;
            end
            if (sp_rsp_vld && rsp_wr_ready) begin
                sp_rsp_vld  <= 1'b0;
                sp_b_active <= 1'b0;
            end
        end
    end

    //-----------------------------------------------------------------
    // read channel: passthrough + split (SBS_EN)
    //-----------------------------------------------------------------
    wire [3:0]      rr_opcode = ext_rq_r_data[OP_HI -: 4];
    wire [LEN_W-1:0] rr_len   = ext_rq_r_data[LEN_HI -: LEN_W];
    wire [ID_W-1:0]  rr_txnid = ext_rq_r_data[TXN_HI -: ID_W];
    wire [ADDR_W-1:0] rr_addr = ext_rq_r_data[ADDR_HI -: ADDR_W];

    wire rr_split = SBS_EN && (rr_len > SLV_MAX_LEN);

    reg  sp_rd_active;
    reg  sp_rd_ar_pend;          // a sub AR is pending issue
    reg [ADDR_W-1:0] sp_rd_base;
    reg [LEN_W:0]    sp_rd_left;
    reg [SUBW-1:0]   sp_rd_sub_done;   // ARs issued so far
    reg [SUBW-1:0]   sp_rd_sub_total;
    reg [ID_W-1:0]   sp_rd_txnid;
    reg [LEN_W-1:0]  sp_rd_cur_len;

    wire sp_rd_start = rr_split && ext_rq_r_valid && !sp_rd_active && arready;
    wire [LEN_W:0] rd_left_eff = sp_rd_start ? ({1'b0, rr_len} + 1'b1) : sp_rd_left;
    wire [LEN_W-1:0] sp_rd_sub_len =
        (rd_left_eff > (SLV_MAX_LEN + 1)) ? SLV_MAX_LEN : (rd_left_eff[LEN_W-1:0] - 1'b1);
    wire [ADDR_W-1:0] sp_rd_sub_addr =
        sp_rd_base + sp_rd_sub_done * ((SLV_MAX_LEN + 1) * W_BYTES);

    // AR arbitration: passthrough read first, then split sub AR
    wire ar_pass   = ext_rq_r_valid && !rr_split;
    wire ar_split  = sp_rd_ar_pend || sp_rd_start;
    wire ar_use_split = ar_split && !ar_pass;

    assign arvalid = ar_pass || ar_split;
    assign arid    = ar_use_split ? (sp_rd_start ? rr_txnid : sp_rd_txnid) : rr_txnid;
    assign araddr  = ar_use_split ? (sp_rd_start ? align_dn(rr_addr) : sp_rd_sub_addr)
                                  : align_dn(rr_addr);
    assign arlen   = ar_use_split ? sp_rd_sub_len : rr_len;
    assign arsize  = `adp_clog2(W_BYTES) & 3'h7;
    assign arburst = `AXI_BURST_INCR;
    assign ext_rq_r_ready = arready && !sp_rd_active && (rr_split ? ar_split : 1'b1);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sp_rd_active    <= 1'b0;
            sp_rd_ar_pend   <= 1'b0;
            sp_rd_left      <= {(LEN_W+1){1'b0}};
            sp_rd_sub_done  <= {SUBW{1'b0}};
            sp_rd_sub_total <= {SUBW{1'b0}};
            sp_rd_cur_len   <= {LEN_W{1'b0}};
            sp_rd_txnid     <= {ID_W{1'b0}};
            sp_rd_base      <= {ADDR_W{1'b0}};
        end else begin
            // activation
            if (sp_rd_start) begin
                sp_rd_active    <= 1'b1;
                sp_rd_ar_pend   <= 1'b0;   // first AR handshakes this cycle
                sp_rd_base      <= align_dn(rr_addr);
                sp_rd_left      <= rr_len + 1'b1;
                sp_rd_sub_done  <= 1'b1;
                sp_rd_sub_total <= (rr_len + 1'b1 + SLV_MAX_LEN) / (SLV_MAX_LEN + 1);
                sp_rd_txnid     <= rr_txnid;
                sp_rd_cur_len   <= sp_rd_sub_len;
            end
            // sub AR handshake
            if (sp_rd_active && ar_use_split && arvalid && arready) begin
                sp_rd_ar_pend  <= 1'b0;
                sp_rd_sub_done <= sp_rd_sub_done + 1'b1;
                sp_rd_cur_len  <= sp_rd_sub_len;
            end
            // sub R completion
            if (sp_rd_active && rvalid && rready && rlast) begin
                sp_rd_left <= sp_rd_left - sp_rd_cur_len - 1'b1;
                if (sp_rd_sub_done == sp_rd_sub_total)
                    sp_rd_active <= 1'b0;
                else
                    sp_rd_ar_pend <= 1'b1;
            end
        end
    end

    // R passthrough; last override for split reads
    wire sp_rd_last_sub = (sp_rd_sub_done == sp_rd_sub_total);
    assign rsp_rd_ext_txnid = rid;
    assign rsp_rd_resp      = map_resp(rresp);
    assign rsp_rd_user      = {USER_RSP_PW{1'b0}};
    assign rsp_rd_last      = sp_rd_active ? (rlast && sp_rd_last_sub) : rlast;
    assign rsp_rd_data      = rdata;
    assign rsp_rd_valid     = rvalid;
    assign rready           = rsp_rd_ready;

endmodule
