//============================================================================
// Filename    : adapter_narrow_split.v
// Description : [feature] mst-side read splitter (NARROW_EN).
//               Per-beat function: accepts one RSP_RD beat plus the entry's
//               carry state, drains the assembled byte stream into AXI R
//               beats (size bytes each, placed at lanes [lane, lane+size)),
//               writes the carry back on o_entry_done and pulses o_done_txn
//               when the transaction is complete. Over-cover tail bytes are
//               dropped. One active drain at a time (input serialization).
//============================================================================
`include "adapter_ip_defs.vh"
`default_nettype none
`timescale 1ns/1ps

module adapter_narrow_split #(
    parameter DATA_W = 64,
    parameter LEN_W  = 8,
    parameter ID_W   = 8,
    // derived
    parameter W_BYTES = DATA_W/8,
    parameter LANE_W  = `adp_clog2(W_BYTES),
    parameter HOLD_W  = 2*DATA_W,
    parameter HCNT_W  = `adp_clog2(2*W_BYTES) + 1
) (
    input  wire                clk,
    input  wire                rst_n,
    // ---- RSP_RD beat (top has decoded the entry by txnid) ----
    input  wire [DATA_W-1:0]   i_data,
    input  wire                i_last,
    input  wire [1:0]          i_resp,
    input  wire                i_valid,
    output wire                o_ready,
    // ---- entry context ----
    input  wire [DATA_W-1:0]   i_acc_data,
    input  wire [W_BYTES-1:0]  i_acc_vld,
    input  wire [LANE_W:0]     i_acc_cnt,
    input  wire [LEN_W+1:0]    i_beats_left,
    input  wire [LANE_W-1:0]   i_lane,
    input  wire [LANE_W:0]     i_size,
    input  wire [LANE_W:0]     i_vbytes,
    input  wire                i_first,
    input  wire [LEN_W:0]      i_emit_cnt,  // AXI beats emitted so far (from entry)
    input  wire                i_wrap,
    input  wire [7:0]          i_start_off, // A mod wrap_bytes
    input  wire [LEN_W:0]      i_axi_len,   // AXI len (beats-1)
    // ---- entry update (valid at o_entry_done pulse) ----
    output wire [DATA_W-1:0]   o_acc_data,
    output wire [W_BYTES-1:0]  o_acc_vld,
    output wire [LANE_W:0]     o_acc_cnt,
    output wire [LEN_W+1:0]    o_beats_left,
    output wire                o_entry_done,
    output wire                o_done_txn,
    // ---- R beat out ----
    output reg  [DATA_W-1:0]   o_rdata,
    output reg                 o_rlast,
    output reg  [1:0]          o_rresp,
    output reg                 o_rvalid,
    input  wire                i_rready
);
    localparam HB = 2*W_BYTES;   // hold buffer depth in bytes

    localparam S_IDLE  = 1'b0;
    localparam S_DRAIN = 1'b1;

    reg state;
    reg [HOLD_W-1:0] hold_data;
    reg [HCNT_W-1:0] hold_vld;
    reg [LEN_W+1:0]  bl_left;
    reg [1:0]        resp_q;
    reg [LANE_W-1:0] lane_q;
    reg [LANE_W:0]   size_q;
    reg [LEN_W:0]    emit_cnt;   // AXI beats emitted for this transaction

    wire drain_emit = !i_wrap && (state == S_DRAIN) && (bl_left > 0) && (hold_vld >= {1'b0, size_q});
    wire drain_end  = !i_wrap && (state == S_DRAIN) && !drain_emit;

    // WRAP collect/emit: RSP beats stored as aligned W-beats, AXI replay by wrap pos
    localparam MAX_LB = 16;
    localparam LBWlg = 4;
    reg [DATA_W-1:0] wbuf [0:MAX_LB-1];
    reg [LBWlg-1:0]  wcnt;
    reg [LEN_W:0]    wemit_k;
    reg              wemit;
    reg [LANE_W:0]   wsize_q;
    reg [LANE_W-1:0] wlane_q;
    reg [7:0]        woff_q;
    reg [LEN_W:0]    wlen_q;
    reg [1:0]        wresp_q;
    wire [15:0] wrap_bytes = ({8'b0, wlen_q} + 1) * {8'b0, wsize_q};
    wire [15:0] w_idx16 = ({8'b0, woff_q} + wemit_k * {8'b0, wsize_q});
    wire [15:0] w_off16 = (wrap_bytes == 0) ? 16'b0 : (w_idx16 % wrap_bytes);
    wire [15:0] w_p16   = w_off16 + {8'b0, wlane_q} - {8'b0, woff_q};
    // p_k = (start_off+k*size)%wrap - start_off + lane ; lane==i_lane at accept
    wire [LANE_W-1:0] w_place = w_p16[LANE_W-1:0];
    wire [LBWlg-1:0]  w_slot  = w_p16[LANE_W +: LBWlg];
    wire wrap_emit = i_wrap && wemit && (wemit_k <= wlen_q);

    assign o_ready = i_wrap ? (!wemit) : (state == S_IDLE);

    //--------------------- accept merge (carry ++ beat) ---------------------
    wire [HOLD_W-1:0] merged;
    genvar p;
    generate
    for (p = 0; p < HB; p = p + 1) begin : g_merge
        wire in_carry = (p < i_acc_cnt);
        wire in_beat  = (p >= i_acc_cnt) && (p < (i_acc_cnt + i_vbytes));
        wire [7:0] cb = i_acc_data[((p < W_BYTES) ? p : 0)*8 +: 8];
        // first beat: valid bytes at lanes [lane, lane+vbytes) with wrap
        wire [7:0] bb = i_data[(in_beat ? (i_first ? ((i_lane + (p - i_acc_cnt)) & (W_BYTES-1))
                                                  : (p - i_acc_cnt)) : 0)*8 +: 8];
        assign merged[p*8 +: 8] = in_carry ? cb : (in_beat ? bb : 8'h0);
    end
    endgenerate

    //--------------------- R beat presentation ---------------------
    // beat k's byte j goes to AXI lane (lane_q + k*size_q + j) mod W
    integer sh;
    integer jj;
    always @* begin
        o_rdata  = {DATA_W{1'b0}};
        o_rlast  = 1'b0;
        o_rresp  = resp_q;
        o_rvalid = 1'b0;
        if (drain_emit) begin
            for (sh = 0; sh < W_BYTES; sh = sh + 1) begin
                jj = sh - ((lane_q + emit_cnt * size_q) & (W_BYTES-1));
                if (jj < 0) jj = jj + W_BYTES;
                if (jj < size_q)
                    o_rdata[sh*8 +: 8] = hold_data[jj*8 +: 8];
            end
            o_rlast  = (bl_left == 1);
            o_rvalid = 1'b1;
        end else if (wrap_emit) begin
            for (sh = 0; sh < W_BYTES; sh = sh + 1) begin
                if ((sh >= w_place) && (sh < (w_place + wsize_q)))
                    o_rdata[sh*8 +: 8] = wbuf[w_slot][sh*8 +: 8];
            end
            o_rlast  = (wemit_k == wlen_q);
            o_rresp  = wresp_q;
            o_rvalid = 1'b1;
        end
    end

    //--------------------- entry write-back values ---------------------
    wire [W_BYTES-1:0] up_vld_mask;
    generate
    for (p = 0; p < W_BYTES; p = p + 1) begin : g_upvld
        assign up_vld_mask[p] = (p < hold_vld);
    end
    endgenerate

    assign o_acc_data   = hold_data[0 +: DATA_W];
    assign o_acc_vld    = up_vld_mask;
    assign o_acc_cnt    = hold_vld[LANE_W:0];
    assign o_beats_left = bl_left;
    assign o_entry_done = i_wrap ? (wrap_emit && i_rready && (wemit_k == wlen_q)) : drain_end;
    assign o_done_txn   = i_wrap ? (wrap_emit && i_rready && (wemit_k == wlen_q))
                                 : (drain_end && (bl_left == 0));

    //--------------------- state machine ---------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state     <= S_IDLE;
            hold_data <= {HOLD_W{1'b0}};
            hold_vld  <= {HCNT_W{1'b0}};
            bl_left   <= {(LEN_W+2){1'b0}};
            resp_q    <= 2'b00;
            lane_q    <= {LANE_W{1'b0}};
            size_q    <= {(LANE_W+1){1'b0}};
            emit_cnt  <= {(LEN_W+1){1'b0}};
            wcnt      <= {LBWlg{1'b0}};
            wemit     <= 1'b0;
            wemit_k   <= {(LEN_W+1){1'b0}};
            wlane_q   <= {LANE_W{1'b0}};
        end else begin
            if (i_wrap) begin
                if (!wemit) begin
                    if (i_valid) begin
                        wbuf[i_first ? {LBWlg{1'b0}} : wcnt] <= i_data;
                        wresp_q    <= i_resp;
                        wsize_q    <= i_size;
                        wlane_q    <= i_lane;
                        woff_q     <= i_start_off;
                        wlen_q     <= i_axi_len;
                        if (i_last) begin
                            wemit   <= 1'b1;
                            wemit_k <= {(LEN_W+1){1'b0}};
                        end else if (i_first)
                            wcnt <= 1'b1;
                        else
                            wcnt <= wcnt + 1'b1;
                    end
                end else if (wrap_emit && i_rready) begin
                    if (wemit_k == wlen_q) begin
                        wemit <= 1'b0;
                        wcnt  <= {LBWlg{1'b0}};
                    end else
                        wemit_k <= wemit_k + 1'b1;
                end
            end else
            case (state)
            S_IDLE: begin
                if (i_valid) begin
                    hold_data <= merged;
                    hold_vld  <= {1'b0, i_acc_cnt} + {1'b0, i_vbytes};
                    bl_left   <= i_beats_left;
                    resp_q    <= i_resp;
                    lane_q    <= i_lane;
                    size_q    <= i_size;
                    emit_cnt  <= i_emit_cnt;
                    state     <= S_DRAIN;
                end
            end
            S_DRAIN: begin
                if (drain_emit) begin
                    if (i_rready) begin
                        hold_data <= hold_data >> (size_q*8);
                        hold_vld  <= hold_vld - {1'b0, size_q};
                        bl_left   <= bl_left - 1;
                        emit_cnt  <= emit_cnt + 1'b1;
                    end
                end else begin
                    state <= S_IDLE;
                end
            end
            default: state <= S_IDLE;
            endcase
        end
    end

`ifndef LB_NO_ASSERT
    // synthesis translate_off
    always @(posedge clk) begin
        if (i_valid && ((i_vbytes == 0) || (i_vbytes > W_BYTES))) begin
            $display("[%0t] ERROR %m: i_vbytes=%0d out of range", $time, i_vbytes);
        end
        if (i_valid && (i_acc_cnt + i_vbytes > HB)) begin
            $display("[%0t] ERROR %m: hold overflow acc=%0d vbytes=%0d", $time, i_acc_cnt, i_vbytes);
        end
    end
    // synthesis translate_on
`endif

endmodule
