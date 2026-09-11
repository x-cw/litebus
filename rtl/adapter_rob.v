//============================================================================
// Filename    : adapter_rob.v
// Description : [feature] mst-side same-ID reorder (SAME_ID_EN) + ROB memory.
//               Query/arbitration interface is unchanged:
//                 o_issuable[i]     : read entry i may send REQ_R
//                 o_presentable_b   : write B presentable in group order
//                 o_presentable_ar  : R (atomic or buffered read) presentable
//               Grants are round-robin over the presentable set.
//               Data interface: write by txnid (int_id); read the group-head
//               beat. Internal adapter_mem holds {ext_id, data, resp, last}
//               so ID restore and same-ID R interleave reorder both come
//               from memory (MEM_TYPE selects REG vs RAM).
//============================================================================
`include "adapter_ip_defs.vh"
`default_nettype none
`timescale 1ns/1ps

module adapter_rob #(
    parameter PEND_TX    = 8,
    parameter ID_W       = 8,
    parameter SEQ_W      = 8,
    parameter ROB_DEPTH  = 1,
    parameter ROB_DATA_W = 1,
    parameter MEM_TYPE   = 0
) (
    input  wire                     clk,
    input  wire                     rst_n,
    // ---- table view (arrays of per-entry fields) ----
    input  wire [PEND_TX-1:0]       i_valid,
    input  wire [PEND_TX*ID_W-1:0]  i_ext_id,
    input  wire [PEND_TX*SEQ_W-1:0] i_seq,
    input  wire [PEND_TX-1:0]       i_issued,
    input  wire [PEND_TX-1:0]       i_is_wr,
    input  wire [PEND_TX-1:0]       i_b_vld,
    input  wire [PEND_TX-1:0]       i_ar_vld,
    // ---- policy outputs ----
    output wire [PEND_TX-1:0]       o_issuable,
    output wire [PEND_TX-1:0]       o_presentable_b,
    output wire [PEND_TX-1:0]       o_presentable_ar,
    output wire [`adp_clog2(PEND_TX)-1:0] o_grant_b,
    output wire                     o_grant_b_vld,
    output wire [`adp_clog2(PEND_TX)-1:0] o_grant_ar,
    output wire                     o_grant_ar_vld,
    input  wire                     i_b_taken,
    input  wire                     i_ar_taken,
    // ---- response data in (by txnid); ordered beat out of group head ----
    input  wire                     i_wr_en,
    input  wire [`adp_clog2(PEND_TX)-1:0] i_wr_idx,
    input  wire [ROB_DATA_W-1:0]    i_wr_data,
    output wire                     o_wr_ready,
    output wire [ROB_DATA_W-1:0]    o_rd_data,
    output wire                     o_rd_valid,
    input  wire                     i_wr_b_en,
    input  wire [`adp_clog2(PEND_TX)-1:0] i_wr_b_idx,
    input  wire [ROB_DATA_W-1:0]    i_wr_b_data,
    output wire [ROB_DATA_W-1:0]    o_b_data
);
    localparam IDX_W     = `adp_clog2(PEND_TX);
    localparam PTR_W     = (ROB_DEPTH <= 1) ? 1 : `adp_clog2(ROB_DEPTH);
    localparam OCC_W     = (ROB_DEPTH < 2) ? 1 : `adp_clog2(ROB_DEPTH + 1);
    localparam MEM_DEPTH = PEND_TX * ROB_DEPTH;
    localparam MEM_AW    = (`adp_clog2(MEM_DEPTH) < 1) ? 1 : `adp_clog2(MEM_DEPTH);
    localparam B_AW      = (`adp_clog2(PEND_TX) < 1) ? 1 : `adp_clog2(PEND_TX);

    wire [OCC_W-1:0] depth_w = ROB_DEPTH[OCC_W-1:0];

    //--------------------- group ordering ---------------------
    wire [PEND_TX-1:0] older_block;
    wire [PEND_TX-1:0] r_has;
    wire [PEND_TX-1:0] full;
    genvar i, j;
    generate
    for (i = 0; i < PEND_TX; i = i + 1) begin : g_ent
        wire [PEND_TX-1:0] older_hit;
        wire [PEND_TX-1:0] older_uniss;
        wire [PEND_TX-1:0] same_iss_rd;
        for (j = 0; j < PEND_TX; j = j + 1) begin : g_peer
            assign older_hit[j] = i_valid[j]
                                && (i_ext_id[j*ID_W +: ID_W] == i_ext_id[i*ID_W +: ID_W])
                                && (i_seq[j*SEQ_W +: SEQ_W] < i_seq[i*SEQ_W +: SEQ_W]);
            assign older_uniss[j] = older_hit[j] && !i_issued[j];
            assign same_iss_rd[j] = i_valid[j] && i_issued[j] && !i_is_wr[j]
                                && (i_ext_id[j*ID_W +: ID_W] == i_ext_id[i*ID_W +: ID_W]);
        end
        assign older_block[i] = |older_hit;
        assign o_issuable[i]  = i_valid[i] && !i_issued[i]
                              && ((ROB_DEPTH <= 1) ? !older_block[i]
                                  : (!(|older_uniss) && (n_ones(same_iss_rd) < ROB_DEPTH)));
        assign o_presentable_b[i]  = i_valid[i] && i_b_vld[i] && !older_block[i];
        assign o_presentable_ar[i] = i_valid[i] && !older_block[i]
                                   && (i_ar_vld[i] || r_has[i]);
    end
    endgenerate

    function integer n_ones;
        input [PEND_TX-1:0] v;
        integer k;
        begin
            n_ones = 0;
            for (k = 0; k < PEND_TX; k = k + 1)
                if (v[k]) n_ones = n_ones + 1;
        end
    endfunction

    //--------------------- round-robin grant ---------------------
    function [IDX_W-1:0] rr_grant;
        input [PEND_TX-1:0] present;
        input [IDX_W-1:0]   rr_ptr;
        integer k;
        reg [PEND_TX-1:0] found;
        begin
            found = {PEND_TX{1'b0}};
            for (k = 0; k < PEND_TX; k = k + 1) begin
                if (present[(rr_ptr + k) % PEND_TX] && !(|found)) begin
                    found = {PEND_TX{1'b0}};
                    found[(rr_ptr + k) % PEND_TX] = 1'b1;
                end
            end
            rr_grant = 0;
            for (k = 0; k < PEND_TX; k = k + 1) begin
                if (found[k]) rr_grant = k[IDX_W-1:0];
            end
        end
    endfunction

    wire [IDX_W-1:0] grant_b  = rr_grant(o_presentable_b,  rr_b);
    wire [IDX_W-1:0] grant_ar = rr_grant(o_presentable_ar, rr_ar);

    assign o_grant_b     = grant_b;
    assign o_grant_b_vld = |o_presentable_b;
    assign o_grant_ar    = grant_ar;
    assign o_grant_ar_vld= |o_presentable_ar;

    reg [IDX_W-1:0] rr_b;
    reg [IDX_W-1:0] rr_ar;
    reg [PEND_TX*PTR_W-1:0] wr_ptr;
    reg [PEND_TX*PTR_W-1:0] rd_ptr;
    reg [PEND_TX*OCC_W-1:0] occ;

    generate
    for (i = 0; i < PEND_TX; i = i + 1) begin : g_occ
        assign r_has[i] = |occ[i*OCC_W +: OCC_W];
        assign full[i]  = (occ[i*OCC_W +: OCC_W] == depth_w);
    end
    endgenerate

    wire pop_fire = i_ar_taken && o_rd_valid;
    assign o_wr_ready = i_valid[i_wr_idx] &&
                        (!full[i_wr_idx] || (pop_fire && (grant_ar == i_wr_idx)));
    wire wr_fire = i_wr_en && o_wr_ready;
    wire wr_pop_same = wr_fire && pop_fire && (i_wr_idx == grant_ar);

    assign o_rd_valid = o_grant_ar_vld && r_has[grant_ar];

    integer k;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rr_b   <= {IDX_W{1'b0}};
            rr_ar  <= {IDX_W{1'b0}};
            wr_ptr <= {PEND_TX*PTR_W{1'b0}};
            rd_ptr <= {PEND_TX*PTR_W{1'b0}};
            occ    <= {PEND_TX*OCC_W{1'b0}};
        end else begin
            if (i_b_taken)
                rr_b <= grant_b + 1'b1;
            if (i_ar_taken)
                rr_ar <= grant_ar + 1'b1;
            for (k = 0; k < PEND_TX; k = k + 1) begin
                if (!i_valid[k]) begin
                    wr_ptr[k*PTR_W +: PTR_W] <= {PTR_W{1'b0}};
                    rd_ptr[k*PTR_W +: PTR_W] <= {PTR_W{1'b0}};
                    occ[k*OCC_W +: OCC_W]    <= {OCC_W{1'b0}};
                end
            end
            if (wr_pop_same) begin
                wr_ptr[i_wr_idx*PTR_W +: PTR_W] <=
                    (ROB_DEPTH <= 1) ? {PTR_W{1'b0}} :
                    ((wr_ptr[i_wr_idx*PTR_W +: PTR_W] == (ROB_DEPTH - 1)) ?
                        {PTR_W{1'b0}} : (wr_ptr[i_wr_idx*PTR_W +: PTR_W] + 1'b1));
                rd_ptr[grant_ar*PTR_W +: PTR_W] <=
                    (ROB_DEPTH <= 1) ? {PTR_W{1'b0}} :
                    ((rd_ptr[grant_ar*PTR_W +: PTR_W] == (ROB_DEPTH - 1)) ?
                        {PTR_W{1'b0}} : (rd_ptr[grant_ar*PTR_W +: PTR_W] + 1'b1));
            end else begin
                if (wr_fire) begin
                    wr_ptr[i_wr_idx*PTR_W +: PTR_W] <=
                        (ROB_DEPTH <= 1) ? {PTR_W{1'b0}} :
                        ((wr_ptr[i_wr_idx*PTR_W +: PTR_W] == (ROB_DEPTH - 1)) ?
                            {PTR_W{1'b0}} : (wr_ptr[i_wr_idx*PTR_W +: PTR_W] + 1'b1));
                    occ[i_wr_idx*OCC_W +: OCC_W] <= occ[i_wr_idx*OCC_W +: OCC_W] + 1'b1;
                end
                if (pop_fire) begin
                    rd_ptr[grant_ar*PTR_W +: PTR_W] <=
                        (ROB_DEPTH <= 1) ? {PTR_W{1'b0}} :
                        ((rd_ptr[grant_ar*PTR_W +: PTR_W] == (ROB_DEPTH - 1)) ?
                            {PTR_W{1'b0}} : (rd_ptr[grant_ar*PTR_W +: PTR_W] + 1'b1));
                    occ[grant_ar*OCC_W +: OCC_W] <= occ[grant_ar*OCC_W +: OCC_W] - 1'b1;
                end
            end
        end
    end

    //--------------------- R beat memory (write by txnid, read group head) ---
    wire [MEM_AW-1:0] mem_waddr;
    wire [MEM_AW-1:0] mem_raddr;
    generate
    if (ROB_DEPTH <= 1) begin : g_addr1
        assign mem_waddr = i_wr_idx;
        assign mem_raddr = grant_ar;
    end else begin : g_addrn
        assign mem_waddr = (i_wr_idx * ROB_DEPTH) + wr_ptr[i_wr_idx*PTR_W +: PTR_W];
        assign mem_raddr = (grant_ar * ROB_DEPTH) + rd_ptr[grant_ar*PTR_W +: PTR_W];
    end
    endgenerate

    adapter_mem #(
        .DATA_W(ROB_DATA_W),
        .DEPTH(MEM_DEPTH),
        .ADDR_W(MEM_AW),
        .MEM_TYPE(MEM_TYPE)
    ) u_mem_r (
        .clk(clk),
        .rst_n(rst_n),
        .i_we(wr_fire),
        .i_waddr(mem_waddr),
        .i_wdata(i_wr_data),
        .i_raddr(mem_raddr),
        .o_rdata(o_rd_data)
    );

    adapter_mem #(
        .DATA_W(ROB_DATA_W),
        .DEPTH(PEND_TX),
        .ADDR_W(B_AW),
        .MEM_TYPE(MEM_TYPE)
    ) u_mem_b (
        .clk(clk),
        .rst_n(rst_n),
        .i_we(i_wr_b_en),
        .i_waddr(i_wr_b_idx),
        .i_wdata(i_wr_b_data),
        .i_raddr(grant_b),
        .o_rdata(o_b_data)
    );

endmodule
