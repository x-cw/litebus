//============================================================================
// Filename    : axi_slave_mem.v
// Description : Simple AXI4 slave byte-memory model (for adapter_slv TBs).
//               Byte-accurate memory; B responses are queued so a new AW can
//               overlap an outstanding B (needed by SBS split writes).
//============================================================================
`default_nettype none
`timescale 1ns/1ps

module axi_slave_mem #(
    parameter ADDR_W = 32,
    parameter DATA_W = 64,
    parameter LEN_W  = 8,
    parameter ID_W   = 8,
    parameter MEM_BYTES = 16384,
    parameter ERR_ADDR = 32'h4000_0000
) (
    input  wire               clk,
    input  wire               rst_n,
    input  wire [ID_W-1:0]    awid,
    input  wire [ADDR_W-1:0]  awaddr,
    input  wire [LEN_W-1:0]   awlen,
    input  wire [2:0]         awsize,
    input  wire [1:0]         awburst,
    input  wire               awvalid,
    output wire               awready,
    input  wire [DATA_W-1:0]  wdata,
    input  wire [DATA_W/8-1:0] wstrb,
    input  wire               wlast,
    input  wire               wvalid,
    output wire               wready,
    output wire [ID_W-1:0]    bid,
    output wire [1:0]         bresp,
    output wire               bvalid,
    input  wire               bready,
    input  wire [ID_W-1:0]    arid,
    input  wire [ADDR_W-1:0]  araddr,
    input  wire [LEN_W-1:0]   arlen,
    input  wire [2:0]         arsize,
    input  wire [1:0]         arburst,
    input  wire               arvalid,
    output wire               arready,
    output reg  [ID_W-1:0]    rid,
    output reg  [DATA_W-1:0]  rdata,
    output reg  [1:0]         rresp,
    output reg                rlast,
    output reg                rvalid,
    input  wire               rready
);
    localparam W_BYTES = DATA_W/8;
    localparam BQ_N    = 8;

    reg [7:0] mem [0:MEM_BYTES-1];
    integer mi;

    reg             wr_active;
    reg [ID_W-1:0]  wr_id;
    reg [ADDR_W-1:0] wr_addr;
    reg [2:0]       wr_size;
    reg             wr_err;

    reg             rd_active;
    reg [ID_W-1:0]  rd_id;
    reg [ADDR_W-1:0] rd_addr;
    reg [LEN_W:0]   rd_left;
    reg [2:0]       rd_size;
    reg             rd_err;

    reg [ID_W-1:0]  bq_id  [0:BQ_N-1];
    reg             bq_err [0:BQ_N-1];
    reg [3:0]       bq_wptr;
    reg [3:0]       bq_rptr;
    reg [3:0]       bq_cnt;

    integer bj;
    reg [ADDR_W-1:0] byte_base;
    reg [ADDR_W-1:0] byte_addr;
    reg [ADDR_W-1:0] wr_cur;
    reg [ADDR_W-1:0] rd_cur;
    reg [2:0]        wr_sz;

    assign awready = !wr_active;
    assign wready  = wr_active || (awvalid && awready);
    assign arready = !rd_active;

    assign bvalid = (bq_cnt != 4'd0);
    assign bid    = bq_id[bq_rptr[2:0]];
    assign bresp  = bq_err[bq_rptr[2:0]] ? 2'b10 : 2'b00;

    wire err_w = (awaddr >= ERR_ADDR) && (awaddr < ERR_ADDR + 8);
    wire err_r = (araddr >= ERR_ADDR) && (araddr < ERR_ADDR + 8);
    wire push_b = wvalid && wready && wlast;
    wire pop_b  = bvalid && bready;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wr_active <= 1'b0;
            rd_active <= 1'b0;
            rvalid    <= 1'b0;
            wr_addr   <= {ADDR_W{1'b0}};
            rd_addr   <= {ADDR_W{1'b0}};
            wr_size   <= 3'd0;
            rd_size   <= 3'd0;
            wr_id     <= {ID_W{1'b0}};
            rd_id     <= {ID_W{1'b0}};
            wr_err    <= 1'b0;
            rd_err    <= 1'b0;
            rd_left   <= {(LEN_W+1){1'b0}};
            rid       <= {ID_W{1'b0}};
            rdata     <= {DATA_W{1'b0}};
            rresp     <= 2'b00;
            rlast     <= 1'b0;
            bq_wptr   <= 4'd0;
            bq_rptr   <= 4'd0;
            bq_cnt    <= 4'd0;
        end else begin
            //---- write AW / W (single wr_addr update to avoid NBA collision)
            if (wvalid && wready) begin
                wr_cur = wr_active ? wr_addr : awaddr;
                wr_sz  = wr_active ? wr_size : awsize;
                byte_base = wr_cur - (wr_cur % W_BYTES);
                for (bj = 0; bj < W_BYTES; bj = bj + 1) begin
                    byte_addr = byte_base + bj;
                    if (wstrb[bj] && (byte_addr < MEM_BYTES))
                        mem[byte_addr] <= wdata[bj*8 +: 8];
                end
                wr_addr <= wr_cur + ({{(ADDR_W-8){1'b0}}, 8'd1} << wr_sz);
                if (wlast)
                    wr_active <= 1'b0;
                else if (awvalid && awready) begin
                    wr_id   <= awid;
                    wr_size <= awsize;
                    wr_err  <= err_w;
                    wr_active <= 1'b1;
                end
            end else if (awvalid && awready) begin
                wr_id     <= awid;
                wr_addr   <= awaddr;
                wr_size   <= awsize;
                wr_err    <= err_w;
                wr_active <= 1'b1;
            end

            //---- B queue
            if (push_b && (bq_cnt != BQ_N[3:0])) begin
                bq_id[bq_wptr[2:0]]  <= wr_active ? wr_id : awid;
                bq_err[bq_wptr[2:0]] <= wr_active ? wr_err : err_w;
                bq_wptr <= (bq_wptr == (BQ_N-1)) ? 4'd0 : (bq_wptr + 1'b1);
            end
            if (pop_b)
                bq_rptr <= (bq_rptr == (BQ_N-1)) ? 4'd0 : (bq_rptr + 1'b1);
            if (push_b && !pop_b && (bq_cnt != BQ_N[3:0]))
                bq_cnt <= bq_cnt + 1'b1;
            else if (!push_b && pop_b && (bq_cnt != 4'd0))
                bq_cnt <= bq_cnt - 1'b1;

            //---- read
            if (arvalid && arready) begin
                rd_id     <= arid;
                rd_addr   <= araddr;
                rd_left   <= arlen + 1'b1;
                rd_size   <= arsize;
                rd_err    <= err_r;
                rd_active <= 1'b1;
            end

            if (rvalid && rready) begin
                if (rlast) begin
                    rvalid    <= 1'b0;
                    rd_active <= 1'b0;
                end else begin
                    rd_cur    = rd_addr + ({{(ADDR_W-8){1'b0}}, 8'd1} << rd_size);
                    rd_addr   <= rd_cur;
                    rd_left   <= rd_left - 1'b1;
                byte_base = rd_cur - (rd_cur % W_BYTES);
                    for (bj = 0; bj < W_BYTES; bj = bj + 1) begin
                        byte_addr = byte_base + bj;
                        if (byte_addr < MEM_BYTES)
                            rdata[bj*8 +: 8] <= mem[byte_addr];
                        else
                            rdata[bj*8 +: 8] <= 8'h00;
                    end
                    rid    <= rd_id;
                    rresp  <= rd_err ? 2'b10 : 2'b00;
                    rlast  <= (rd_left == 2);
                    rvalid <= 1'b1;
                end
            end else if (!rvalid && rd_active) begin
                byte_base = rd_addr - (rd_addr % W_BYTES);
                for (bj = 0; bj < W_BYTES; bj = bj + 1) begin
                    byte_addr = byte_base + bj;
                    if (byte_addr < MEM_BYTES)
                        rdata[bj*8 +: 8] <= mem[byte_addr];
                    else
                        rdata[bj*8 +: 8] <= 8'h00;
                end
                rid    <= rd_id;
                rresp  <= rd_err ? 2'b10 : 2'b00;
                rlast  <= (rd_left == 1);
                rvalid <= 1'b1;
            end
        end
    end

    initial begin : mem_init
        for (mi = 0; mi < MEM_BYTES; mi = mi + 1)
            mem[mi] = mi[7:0] ^ 8'hA5;
    end

endmodule
