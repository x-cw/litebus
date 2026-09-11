//============================================================================
// Filename    : adapter_slv_model.v
// Description : LiteBus slave model (INIU-side mirror) for standalone
//               adapter verification. Byte-accurate memory; consumes
//               REQ_R / REQ_W, returns RSP_RD (len+1 beats) / RSP_WR.
//               Transactions with addr in [ERR_ADDR, ERR_ADDR+8) return
//               LB_RESP_FAIL. MAX_BEATS asserts the downstream burst limit.
//               Writes arrive as sequential REQ_W streams (no interleave).
//============================================================================
`include "adapter_ip_defs.vh"
`default_nettype none
`timescale 1ns/1ps

module adapter_slv_model #(
    parameter ADDR_W = 32,
    parameter DATA_W = 64,
    parameter LEN_W  = 8,
    parameter ID_W   = 8,
    parameter QOS_PW = 1,
    parameter USER_CMD_PW = 1,
    parameter MOD_PW = 1,
    parameter MEM_BYTES = 16384,
    parameter ERR_ADDR  = 32'h4000_0000,
    parameter MAX_BEATS = 1024,
    parameter EXT_CMD_W = QOS_PW + 4 + ADDR_W + LEN_W + ID_W + USER_CMD_PW,
    parameter EXT_WD_W  = DATA_W + DATA_W/8 + 1 + ID_W,
    parameter EXT_REQ_W = EXT_CMD_W + MOD_PW + EXT_WD_W
) (
    input  wire                    clk,
    input  wire                    rst_n,
    // ---------------- INIU-side (mirror) ----------------
    input  wire [EXT_CMD_W-1:0]   req_r_data,
    input  wire                    req_r_valid,
    output wire                    req_r_ready,
    input  wire [EXT_REQ_W-1:0]   req_w_data,
    input  wire                    req_w_valid,
    output wire                    req_w_ready,
    output reg  [DATA_W-1:0]       rsp_rd_data,
    output reg                     rsp_rd_last,
    output reg  [1:0]              rsp_rd_resp,
    output reg  [ID_W-1:0]         rsp_rd_ext_txnid,
    output reg                     rsp_rd_valid,
    input  wire                    rsp_rd_ready,
    output wire [1:0]              rsp_wr_resp,
    output wire [ID_W-1:0]         rsp_wr_ext_txnid,
    output wire                    rsp_wr_valid,
    input  wire                    rsp_wr_ready
);
    localparam W_BYTES = DATA_W/8;
    localparam LANE_W  = `adp_clog2(W_BYTES);
    localparam OP_HI   = EXT_CMD_W - QOS_PW - 1;
    localparam ADDR_HI = EXT_CMD_W - QOS_PW - 4 - 1;
    localparam LEN_HI  = EXT_CMD_W - QOS_PW - 4 - ADDR_W - 1;
    localparam TXN_HI  = EXT_CMD_W - QOS_PW - 4 - ADDR_W - LEN_W - 1;
    localparam WD_DHI  = EXT_WD_W - 1;
    localparam WD_SHI  = ID_W + W_BYTES;
    localparam WD_LHI  = ID_W;

    reg [7:0] mem [0:MEM_BYTES-1];

    //------------------ read task FIFO (depth 4, in-order response) ----------
    reg [ADDR_W-1:0]  rd_q_addr [0:3];
    reg [LEN_W:0]     rd_q_beats [0:3];
    reg [ID_W-1:0]    rd_q_txnid [0:3];
    reg [DATA_W-1:0]  rd_q_old [0:3];
    reg               rd_q_use_old [0:3];
    reg [1:0]         rd_q_cnt;
    reg [1:0]         rd_q_rd;

    reg               rd_active;
    reg [LEN_W:0]     rd_beats_left;
    reg [LEN_W:0]     rd_beat;      // current beat index
    reg [ADDR_W-1:0]  rd_addr;
    reg [ID_W-1:0]    rd_txnid;
    reg               rd_err;
    reg [DATA_W-1:0]  rd_old;
    reg               rd_use_old;

    assign req_r_ready = (rd_q_cnt < 4);

    //------------------ write FSM (sequential REQ_W streams) ---------------
    reg [LEN_W:0]     wr_cnt;
    reg [LEN_W:0]     wr_total;
    reg [ADDR_W-1:0]  wr_addr;
    reg [LANE_W-1:0]  wr_lane;
    reg [ID_W-1:0]    wr_txnid;
    reg               wr_err;
    reg               wr_active;
    reg [3:0]         wr_opcode;
    reg               wr_need_r;    // atomic LOAD/SWAP/COMPARE
    reg [DATA_W-1:0]  wr_old;
    reg               wr_ar_pend;   // atomic R accepted, B pending
    reg               wr_b_pend;    // emit RSP_WR after atomic R

    // write-response queue (multi outstanding B)
    reg [1:0]         bq_resp [0:3];
    reg [ID_W-1:0]    bq_txn  [0:3];
    reg [1:0]         bq_w;
    reg [1:0]         bq_r;
    reg [2:0]         bq_n;

    assign req_w_ready = 1'b1;
    assign rsp_wr_valid      = (bq_n != 3'd0);
    assign rsp_wr_resp       = bq_resp[bq_r];
    assign rsp_wr_ext_txnid  = bq_txn[bq_r];
    wire bq_push = wr_b_pend ||
                   (req_w_valid && w_wd_last &&
                    ((!wr_active && !at_need_r) || (wr_active && !wr_need_r)));
    wire bq_pop  = rsp_wr_valid && rsp_wr_ready;

    wire [EXT_CMD_W-1:0] w_cmd        = req_w_data[EXT_REQ_W-1 -: EXT_CMD_W];
    wire [ADDR_W-1:0]    w_cmd_addr   = w_cmd[ADDR_HI -: ADDR_W];
    wire [LEN_W-1:0]     w_cmd_len    = w_cmd[LEN_HI -: LEN_W];
    wire [ID_W-1:0]      w_cmd_txnid  = w_cmd[TXN_HI -: ID_W];
    wire [3:0]           w_cmd_opcode = w_cmd[OP_HI -: 4];
    wire [DATA_W-1:0] w_wd_data   = req_w_data[WD_DHI -: DATA_W];
    wire [W_BYTES-1:0] w_wd_strb  = req_w_data[WD_SHI -: W_BYTES];
    wire              w_wd_last   = req_w_data[WD_LHI];

    wire err_region    = (w_cmd_addr >= ERR_ADDR) && (w_cmd_addr < (ERR_ADDR + 8));
    wire rd_err_region = (rd_q_addr[rd_q_rd] >= ERR_ADDR) && (rd_q_addr[rd_q_rd] < (ERR_ADDR + 8));

    integer bj;
    integer be;
    reg [ADDR_W-1:0] byte_addr;

    genvar gb_old;
    wire [DATA_W-1:0] w_old_snap;
    generate
    for (gb_old = 0; gb_old < W_BYTES; gb_old = gb_old + 1) begin : g_old
        assign w_old_snap[gb_old*8 +: 8] = mem[w_cmd_addr + gb_old];
    end
    endgenerate
    wire at_need_r  = w_cmd_opcode[3] && (w_cmd_opcode[3:2] == 2'b11) &&
                      (w_cmd_opcode != `LB_OP_ATOMIC_STORE);
    // LOAD never writes; COMPARE writes only when the compare data matches
    wire at_no_wr   = (w_cmd_opcode == `LB_OP_ATOMIC_LOAD) ||
                      ((w_cmd_opcode == `LB_OP_ATOMIC_COMPARE) && (w_wd_data != w_old_snap));

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rd_q_cnt    <= 2'b0;
            rd_q_rd     <= 2'b0;
            rd_active   <= 1'b0;
            rd_beat     <= {(LEN_W+1){1'b0}};
            rsp_rd_valid<= 1'b0;
            rsp_rd_resp <= `LB_RESP_OK;
            wr_active   <= 1'b0;
            wr_err      <= 1'b0;
            wr_addr     <= {ADDR_W{1'b0}};
            wr_ar_pend  <= 1'b0;
            wr_b_pend   <= 1'b0;
            bq_w        <= 2'b0;
            bq_r        <= 2'b0;
            bq_n        <= 3'b0;
        end else begin
            //---------- write path
            if (req_w_valid && !wr_active) begin
                wr_addr   <= w_cmd_addr;
                wr_txnid  <= w_cmd_txnid;
                wr_total  <= w_cmd_len + 1'b1;
                wr_lane   <= w_cmd_addr[LANE_W-1:0];
                wr_err    <= err_region;
                wr_cnt    <= 1'b1;
                wr_active <= 1'b1;
                wr_opcode <= w_cmd_opcode;
                wr_need_r <= at_need_r;
                wr_old    <= w_old_snap;
                for (bj = 0; bj < W_BYTES; bj = bj + 1) begin
                    byte_addr = w_cmd_addr + bj - {1'b0, w_cmd_addr[LANE_W-1:0]};
                    if (!at_no_wr && (bj >= w_cmd_addr[LANE_W-1:0]) &&
                        w_wd_strb[bj] && (byte_addr < MEM_BYTES))
                        mem[byte_addr] <= w_wd_data[bj*8 +: 8];
                end
                // single-beat transactions respond on the first (and last) beat
                if (w_wd_last) begin
                    wr_active <= 1'b0;
                    if (at_need_r) begin
                        rd_q_addr[rd_q_cnt]   <= w_cmd_addr;
                        rd_q_beats[rd_q_cnt]  <= 1'b1;
                        rd_q_txnid[rd_q_cnt]  <= w_cmd_txnid;
                        rd_q_use_old[rd_q_cnt]<= 1'b1;
                        rd_q_old[rd_q_cnt]    <= w_old_snap;
                        rd_q_cnt <= rd_q_cnt + 1'b1;
                        wr_ar_pend <= 1'b1;
                    end else begin
                        bq_resp[bq_w] <= err_region ? `LB_RESP_FAIL : `LB_RESP_OK;
                        bq_txn[bq_w]  <= w_cmd_txnid;
                        bq_w <= bq_w + 1'b1;
                    end
                end
            end else if (req_w_valid && wr_active) begin
                for (bj = 0; bj < W_BYTES; bj = bj + 1) begin
                    byte_addr = wr_addr + wr_cnt*W_BYTES + bj - {1'b0, wr_lane};
                    if (w_wd_strb[bj] && (byte_addr < MEM_BYTES))
                        mem[byte_addr] <= w_wd_data[bj*8 +: 8];
                end
                wr_cnt <= wr_cnt + 1'b1;
                if (w_wd_last) begin
                    wr_active <= 1'b0;
                    if (wr_need_r) begin
                        // atomic LOAD/SWAP/COMPARE: R first (old value), then B
                        rd_q_addr[rd_q_cnt]   <= wr_addr;
                        rd_q_beats[rd_q_cnt]  <= 1'b1;
                        rd_q_txnid[rd_q_cnt]  <= wr_txnid;
                        rd_q_use_old[rd_q_cnt]<= 1'b1;
                        rd_q_old[rd_q_cnt]    <= wr_old;
                        rd_q_cnt <= rd_q_cnt + 1'b1;
                        wr_ar_pend <= 1'b1;
                    end else begin
                        bq_resp[bq_w] <= wr_err ? `LB_RESP_FAIL : `LB_RESP_OK;
                        bq_txn[bq_w]  <= wr_txnid;
                        bq_w <= bq_w + 1'b1;
                    end
                end
            end
            if (wr_ar_pend && rsp_rd_valid && rsp_rd_ready && rd_use_old) begin
                wr_ar_pend <= 1'b0;
                wr_b_pend  <= 1'b1;
            end
            if (wr_b_pend) begin
                wr_b_pend     <= 1'b0;
                bq_resp[bq_w] <= wr_err ? `LB_RESP_FAIL : `LB_RESP_OK;
                bq_txn[bq_w]  <= wr_txnid;
                bq_w <= bq_w + 1'b1;
            end
            if (bq_pop)
                bq_r <= bq_r + 1'b1;
            bq_n <= bq_n + (bq_push ? 1'b1 : 1'b0) - (bq_pop ? 1'b1 : 1'b0);

            //---------- read task queue
            if (req_r_valid && req_r_ready && (rd_q_cnt < 4)) begin
                rd_q_addr[rd_q_cnt]  <= req_r_data[ADDR_HI -: ADDR_W];
                rd_q_beats[rd_q_cnt] <= req_r_data[LEN_HI -: LEN_W] + 1'b1;
                rd_q_txnid[rd_q_cnt] <= req_r_data[TXN_HI -: ID_W];
                rd_q_use_old[rd_q_cnt] <= 1'b0;
                rd_q_cnt <= rd_q_cnt + 1'b1;
            end

            //---------- read drain
            if (rsp_rd_valid && rsp_rd_ready)
                rsp_rd_valid <= 1'b0;

            if (!rd_active && (rd_q_cnt != rd_q_rd)) begin
                rd_addr       <= rd_q_addr[rd_q_rd];
                rd_beats_left <= rd_q_beats[rd_q_rd];
                rd_txnid      <= rd_q_txnid[rd_q_rd];
                rd_use_old    <= rd_q_use_old[rd_q_rd];
                rd_old        <= rd_q_old[rd_q_rd];
                rd_err        <= rd_err_region;
                rd_beat       <= {(LEN_W+1){1'b0}};
                rd_q_rd       <= rd_q_rd + 1'b1;
                rd_active     <= 1'b1;
            end else if (rd_active && !(rsp_rd_valid && !rsp_rd_ready)) begin
                if (rd_use_old)
                    rsp_rd_data <= rd_old;
                else begin
                    for (bj = 0; bj < W_BYTES; bj = bj + 1) begin
                        byte_addr = rd_addr + rd_beat*W_BYTES + bj - {1'b0, rd_addr[LANE_W-1:0]};
                        if (byte_addr < MEM_BYTES)
                            rsp_rd_data[bj*8 +: 8] <= mem[byte_addr];
                    end
                end
                rsp_rd_ext_txnid <= rd_txnid;
                rsp_rd_resp      <= rd_err ? `LB_RESP_FAIL : `LB_RESP_OK;
                rsp_rd_last      <= (rd_beats_left == 1);
                rsp_rd_valid     <= 1'b1;
                if (rd_beats_left == 1)
                    rd_active <= 1'b0;
                else begin
                    rd_beats_left <= rd_beats_left - 1'b1;
                    rd_beat       <= rd_beat + 1'b1;
                end
            end
        end
    end

    //------------------ memory init (deterministic pattern) ---------------
    initial begin : mem_init
        for (be = 0; be < MEM_BYTES; be = be + 1)
            mem[be] = be[7:0] ^ 8'hA5;
    end

`ifndef LB_NO_ASSERT
    // synthesis translate_off
    always @(posedge clk) begin
        if (req_w_valid && (w_cmd_len + 1'b1 > MAX_BEATS)) begin
            $display("[%0t] ERROR %m: write len+1=%0d exceeds MAX_BEATS=%0d",
                     $time, w_cmd_len + 1'b1, MAX_BEATS);
        end
        if (req_r_valid && (req_r_data[LEN_HI -: LEN_W] + 1'b1 > MAX_BEATS)) begin
            $display("[%0t] ERROR %m: read len+1=%0d exceeds MAX_BEATS=%0d",
                     $time, req_r_data[LEN_HI -: LEN_W] + 1'b1, MAX_BEATS);
        end
    end
    // synthesis translate_on
`endif

endmodule
