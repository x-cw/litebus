//============================================================================
// Filename    : adapter_narrow_pack.v
// Description : [feature] mst-side write byte-stream packer (NARROW_EN)
//               AXI beats (size bytes at byte-lane [lane, lane+size)) are
//               repacked into aligned W_BYTES-wide REQ_W beats.
//               The len'+1 over-cover tail bytes carry strb=0 (masked writes).
//               i_last marks the final beat OF THE CURRENT SUB-TRANSACTION;
//               o_txn_done pulses when the sub's final REQ_W beat is presented.
//============================================================================
`include "adapter_ip_defs.vh"
`default_nettype none
`timescale 1ns/1ps

module adapter_narrow_pack #(
    parameter DATA_W = 64,
    parameter ID_W   = 8,
    parameter CMD_W  = 52,
    parameter MOD_W  = 0,
    // derived (port-list dependent, must stay in the parameter list)
    parameter W_BYTES   = DATA_W/8,
    parameter LANE_W    = `adp_clog2(W_BYTES),
    parameter MOD_PW    = (MOD_W < 1) ? 1 : MOD_W,
    parameter EXT_REQ_W = CMD_W + MOD_PW + (DATA_W + W_BYTES + 1 + ID_W),
    parameter POS_W     = 8,
    parameter MAX_SLOT  = 16
) (
    input  wire                    clk,
    input  wire                    rst_n,
    // ---- AXI beat stream ----
    input  wire [DATA_W-1:0]       i_data,
    input  wire [W_BYTES-1:0]      i_strb,
    input  wire                    i_last,
    input  wire                    i_valid,
    output wire                    o_ready,
    // ---- transaction config ----
    input  wire [LANE_W-1:0]       i_lane,   // byte offset of beat data within the W window
    input  wire [LANE_W:0]         i_size,   // bytes per AXI beat (1..W_BYTES)
    input  wire                    i_start,  // first beat of the current sub-transaction
    input  wire                    i_wrap,   // WRAP: merge by i_pos, emit after last
    input  wire [POS_W-1:0]        i_pos,    // stream position of beat's first byte
    // ---- cmd sideband (sampled on emitted beat) ----
    input  wire [CMD_W-1:0]        i_cmd,
    input  wire [MOD_PW-1:0]       i_mod,
    input  wire [ID_W-1:0]         i_txnid,
    // ---- REQ_W beat stream ----
    output wire [EXT_REQ_W-1:0]    o_data,
    output wire                    o_valid,
    input  wire                    i_ready,
    // ---- status ----
    output wire                    o_txn_done  // final REQ_W beat of the sub presented
);
    localparam SZB  = LANE_W + 1;
    localparam WD_W = DATA_W + W_BYTES + 1 + ID_W;

    //------------------------- accumulator state -------------------------
    reg [DATA_W-1:0]    acc_data;
    reg [W_BYTES-1:0]   acc_strb;
    reg [LANE_W:0]      acc_cnt;   // valid bytes in acc, 0..W_BYTES
    reg                 last_seen; // all input beats consumed, tail may remain
    reg [EXT_REQ_W-1:0] out_q;
    reg                 out_v;
    reg                 out_final;

    //------------------------- WRAP slot buffer -------------------------
    localparam SLOT_W = (`adp_clog2(MAX_SLOT) < 1) ? 1 : `adp_clog2(MAX_SLOT);
    reg [DATA_W-1:0]  wslot_d [0:MAX_SLOT-1];
    reg [W_BYTES-1:0] wslot_s [0:MAX_SLOT-1];
    reg [SLOT_W-1:0]  wslot_hi;
    reg [SLOT_W-1:0]  wemit_i;
    reg               w_emit;

    wire [SLOT_W-1:0] w_sidx  = i_pos >> LANE_W;
    wire [LANE_W-1:0] w_place = i_pos[LANE_W-1:0];

    //------------------------- handshake control -------------------------
    wire incr_ready = !out_v || i_ready;
    wire wrap_ready = !w_emit && incr_ready;
    assign o_ready = i_wrap ? wrap_ready : incr_ready;
    wire in_hsk      = i_valid && o_ready;
    wire in_hsk_incr = in_hsk && !i_wrap;
    wire in_hsk_wrap = in_hsk &&  i_wrap;

    wire [LANE_W:0] cnt_eff = i_start ? {1'b0, i_lane} : acc_cnt;
    wire [LANE_W:0] cnt_sum = cnt_eff + {1'b0, i_size};
    wire do_emit  = in_hsk_incr && (cnt_sum >= {1'b0, W_BYTES[LANE_W:0]});
    wire crossing = cnt_sum > {1'b0, W_BYTES[LANE_W:0]};
    wire tail_emit = !in_hsk && last_seen && (acc_cnt > 0) && !out_v;

    //------------------------- byte-wise datapath -------------------------
    genvar b;
    wire [DATA_W-1:0]   em_data;
    wire [W_BYTES-1:0]  em_strb;
    wire [DATA_W-1:0]   na_data;
    wire [W_BYTES-1:0]  na_strb;
    wire [LANE_W:0]     na_cnt;

    wire acc_keep = !do_emit && !i_start;

    generate
    for (b = 0; b < W_BYTES; b = b + 1) begin : g_byte
        // byte at stream position p lives at AXI data lane p mod W,
        // so aligned position b is always extracted from lane b.
        wire [7:0] ib  = i_data[b*8 +: 8];
        wire       isb = i_strb[b];
        wire [7:0] xb  = i_data[b*8 +: 8];
        wire       xsb = i_strb[b];

        // emitted beat byte b: acc below cnt_eff, input beat above
        assign em_data[b*8 +: 8] = (b < cnt_eff) ? acc_data[b*8 +: 8] : ib;
        assign em_strb[b]         = (b < cnt_eff) ? acc_strb[b] : isb;

        // new accumulator byte b
        assign na_data[b*8 +: 8] =
            (acc_keep && (b < cnt_eff))              ? acc_data[b*8 +: 8] :
            (!do_emit && (b >= cnt_eff) && (b < cnt_sum)) ? ib :
            (do_emit && crossing && (b < (cnt_sum - W_BYTES))) ? xb :
            8'h0;
        assign na_strb[b] =
            (acc_keep && (b < cnt_eff))              ? acc_strb[b] :
            (!do_emit && (b >= cnt_eff) && (b < cnt_sum)) ? isb :
            (do_emit && crossing && (b < (cnt_sum - W_BYTES))) ? xsb :
            1'b0;
    end
    endgenerate

    assign na_cnt = do_emit ? (crossing ? (cnt_sum - W_BYTES) : {(LANE_W+1){1'b0}})
                            : cnt_sum;

    //------------------------- output register -------------------------
    wire [DATA_W-1:0]  od_sel = tail_emit ? acc_data : em_data;
    wire [W_BYTES-1:0] os_sel = tail_emit ? acc_strb : em_strb;
    wire               ow_last= tail_emit ? 1'b1 : (i_last && !crossing);
    wire [WD_W-1:0]    wd     = {od_sel, os_sel, ow_last, i_txnid};
    wire [EXT_REQ_W-1:0] out_next_incr = {i_cmd, i_mod, wd};

    wire w_last_slot = (wemit_i == wslot_hi);
    wire [WD_W-1:0] wd_wrap = {wslot_d[wemit_i], wslot_s[wemit_i], w_last_slot, i_txnid};
    wire [EXT_REQ_W-1:0] out_next_wrap = {i_cmd, i_mod, wd_wrap};
    wire [EXT_REQ_W-1:0] out_next = w_emit ? out_next_wrap : out_next_incr;

    wire ld_out_incr = (do_emit || tail_emit) && !(out_v && !i_ready);
    wire ld_out_wrap = w_emit && !(out_v && !i_ready);
    wire ld_out   = i_wrap ? ld_out_wrap : ld_out_incr;
    wire set_last = in_hsk_incr && i_last && (crossing || !do_emit);
    wire clr_last = out_v && i_ready && out_final;

    assign o_data  = out_q;
    assign o_valid = out_v;
    assign o_txn_done = out_v && i_ready && out_final;

    integer bwi;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            acc_data  <= {DATA_W{1'b0}};
            acc_strb  <= {W_BYTES{1'b0}};
            acc_cnt   <= {(LANE_W+1){1'b0}};
            last_seen <= 1'b0;
            out_q     <= {EXT_REQ_W{1'b0}};
            out_v     <= 1'b0;
            out_final <= 1'b0;
            w_emit    <= 1'b0;
            wslot_hi  <= {SLOT_W{1'b0}};
            wemit_i   <= {SLOT_W{1'b0}};
        end else begin
            if (in_hsk_incr) begin
                acc_data <= na_data;
                acc_strb <= na_strb;
                acc_cnt  <= na_cnt;
            end else if (tail_emit) begin
                acc_data <= {DATA_W{1'b0}};
                acc_strb <= {W_BYTES{1'b0}};
                acc_cnt  <= {(LANE_W+1){1'b0}};
            end
            if (set_last)      last_seen <= 1'b1;
            else if (clr_last) last_seen <= 1'b0;
            if (i_wrap && i_start && in_hsk_wrap) begin
                for (bwi = 0; bwi < MAX_SLOT; bwi = bwi + 1) begin
                    wslot_d[bwi] <= {DATA_W{1'b0}};
                    wslot_s[bwi] <= {W_BYTES{1'b0}};
                end
                wslot_hi <= {SLOT_W{1'b0}};
            end
            if (in_hsk_wrap) begin
                for (bwi = 0; bwi < W_BYTES; bwi = bwi + 1) begin
                    if ((bwi >= w_place) && (bwi < (w_place + i_size))) begin
                        wslot_d[w_sidx][bwi*8 +: 8] <= i_data[bwi*8 +: 8];
                        wslot_s[w_sidx][bwi]        <= i_strb[bwi];
                    end
                end
                if (w_sidx > wslot_hi)
                    wslot_hi <= w_sidx;
                if (i_last) begin
                    w_emit  <= 1'b1;
                    wemit_i <= {SLOT_W{1'b0}};
                end
            end
            if (ld_out) begin
                out_q     <= out_next;
                out_v     <= 1'b1;
                out_final <= w_emit ? w_last_slot :
                             (tail_emit || (in_hsk_incr && i_last && do_emit && !crossing));
                if (w_emit) begin
                    if (w_last_slot)
                        w_emit <= 1'b0;
                    else
                        wemit_i <= wemit_i + 1'b1;
                end
            end else if (out_v && i_ready) begin
                out_v     <= 1'b0;
                out_final <= 1'b0;
            end
        end
    end

`ifndef LB_NO_ASSERT
    // synthesis translate_off
    always @(posedge clk) begin
        if (in_hsk && (i_size == 0) || (i_size > W_BYTES)) begin
            $display("[%0t] ERROR %m: i_size=%0d out of range (W_BYTES=%0d)", $time, i_size, W_BYTES);
        end
        if (in_hsk_incr && (cnt_eff + i_size > 2*W_BYTES)) begin
            $display("[%0t] ERROR %m: beat crosses two boundaries", $time);
        end
    end
    // synthesis translate_on
`endif

endmodule
