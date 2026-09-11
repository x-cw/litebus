//============================================================================
// Filename    : adapter_burst_split.v
// Description : [feature] mst-side burst split geometry (SPLIT_EN).
//               Pure combinational: given an AXI transaction (addr, lane,
//               size, len) and a sub index, produce the sub-transaction
//               geometry: addr_k, len'_k, is-last, total sub count N, and the
//               valid-byte counts of the sub's first/last RSP_RD beats.
//               Sub 0 keeps the original (unaligned) address; sub k>=1 starts
//               at the aligned beat boundary k*J*W_BYTES after the base.
//============================================================================
`include "adapter_ip_defs.vh"
`default_nettype none
`timescale 1ns/1ps

module adapter_burst_split #(
    parameter ADDR_W    = 32,
    parameter LEN_W     = 8,
    parameter W_BYTES   = 8,     // bytes per aligned beat
    parameter J         = 8,     // max aligned beats per sub (LB_MAX_BURST_BYTES/W_BYTES)
    parameter MAX_SUB   = 16,
    parameter LANE_W    = `adp_clog2(W_BYTES),
    parameter SUB_IDX_W = (`adp_clog2(MAX_SUB) < 1) ? 1 : `adp_clog2(MAX_SUB),
    parameter VBC_W     = LANE_W + 1
) (
    input  wire [ADDR_W-1:0]    i_addr,
    input  wire [LANE_W-1:0]    i_lane,
    input  wire [LEN_W-1:0]     i_len,
    input  wire [LANE_W:0]      i_size,   // bytes per AXI beat
    input  wire [SUB_IDX_W-1:0] i_sub,
    output reg  [ADDR_W-1:0]    o_addr_k,
    output reg  [LEN_W:0]       o_len_p_k,
    output reg                  o_last_k,
    output reg  [SUB_IDX_W-1:0] o_N,
    output reg  [LANE_W-1:0]    o_lane_k,     // first aligned beat lane of sub
    output reg  [VBC_W-1:0]     o_vb_first,  // valid bytes of sub's first RSP_RD beat
    output reg  [VBC_W-1:0]     o_vb_last,   // valid bytes of sub's last RSP_RD beat
    output reg  [LEN_W:0]       o_axi_beats, // sub's AXI beat count = bytes_k / size
    output reg  [VBC_W-1:0]     o_addition_k // tail invalid bytes (over_cover of this sub)
);
    reg [ADDR_W+LEN_W+4:0] bytes_total;
    reg [ADDR_W+LEN_W+4:0] beats;
    reg [SUB_IDX_W:0]      N;
    reg [ADDR_W+LEN_W+4:0] start_beat;
    reg [ADDR_W+LEN_W+4:0] beats_k;
    reg [ADDR_W+LEN_W+4:0] lane_k;
    reg [ADDR_W+LEN_W+4:0] bytes_k;
    reg [ADDR_W+LEN_W+4:0] over_k;

    always @* begin
        bytes_total = ({1'b0, i_len} + 1) * i_size;
        beats       = ({1'b0, i_lane} + bytes_total + (W_BYTES - 1)) / W_BYTES;
        N           = (beats + J - 1) / J;
        start_beat  = i_sub * J;
        if (start_beat < beats) begin
            beats_k = beats - start_beat;
            if (beats_k > J) beats_k = J;
        end else begin
            beats_k = 0;
        end
        lane_k  = (i_sub == 0) ? i_lane : 0;
        o_lane_k = lane_k[LANE_W-1:0];
        // sub's AXI bytes = intersection of [lane, lane+bytes_total)
        //                   with [start_beat*W, start_beat*W + beats_k*W)
        // start_off = max(lane, start_beat*W)
        if (start_beat * W_BYTES > {1'b0, i_lane})
            bytes_k = ({1'b0, i_lane} + bytes_total) - (start_beat * W_BYTES);
        else
            bytes_k = bytes_total;
        if (bytes_k > (beats_k * W_BYTES - lane_k)) bytes_k = beats_k * W_BYTES - lane_k;
        over_k  = beats_k * W_BYTES - lane_k - bytes_k;

        o_len_p_k = beats_k - 1'b1;
        o_axi_beats = bytes_k / i_size;
        o_last_k  = (i_sub == (N - 1));
        if (i_sub == 0)
            o_addr_k = i_addr;
        else
            o_addr_k = i_addr - {{(ADDR_W-LANE_W){1'b0}}, i_lane} + start_beat * W_BYTES;

        // first-beat valid bytes
        if (bytes_k < (W_BYTES - lane_k))
            o_vb_first = bytes_k;
        else
            o_vb_first = W_BYTES - lane_k;
        // last-beat valid bytes
        if (beats_k <= 1)
            o_vb_last = bytes_k;
        else
            o_vb_last = W_BYTES - over_k;

        o_addition_k = over_k[VBC_W-1:0];
        o_N = N[SUB_IDX_W-1:0];
    end

endmodule
