//============================================================================
// Filename    : adapter_mem.v
// Description : 1R1W memory wrapper. Internal logic is only array write +
//               read; MEM_TYPE selects the storage implementation:
//                 0 = REG  flop array, async reset of contents, combo read
//                 1 = RAM  inferred 1R1W SRAM, no data reset, combo read
//               Both modes share the same cycle timing (sync write, async
//               read) so REG can be swapped for RAM without changing the
//               parent. Replace the g_ram block with a foundry/FPGA SRAM
//               wrapper if a compiled memory is required; keep this port list.
//============================================================================
`include "adapter_ip_defs.vh"
`default_nettype none
`timescale 1ns/1ps

module adapter_mem #(
    parameter DATA_W   = 32,
    parameter DEPTH    = 16,
    parameter ADDR_W   = 4,
    parameter MEM_TYPE = 0          // 0=REG, 1=RAM
) (
    input  wire                 clk,
    input  wire                 rst_n,
    input  wire                 i_we,
    input  wire [ADDR_W-1:0]    i_waddr,
    input  wire [DATA_W-1:0]    i_wdata,
    input  wire [ADDR_W-1:0]    i_raddr,
    output wire [DATA_W-1:0]    o_rdata
);
    wire [ADDR_W-1:0] waddr = (DEPTH == 1) ? {ADDR_W{1'b0}} : i_waddr;
    wire [ADDR_W-1:0] raddr = (DEPTH == 1) ? {ADDR_W{1'b0}} : i_raddr;

    generate
    if (MEM_TYPE == 0) begin : g_reg
        reg [DATA_W-1:0] mem [0:DEPTH-1];
        integer n;
        always @(posedge clk or negedge rst_n) begin
            if (!rst_n) begin
                for (n = 0; n < DEPTH; n = n + 1)
                    mem[n] <= {DATA_W{1'b0}};
            end else if (i_we) begin
                mem[waddr] <= i_wdata;
            end
        end
        assign o_rdata = mem[raddr];
    end else begin : g_ram
        // Inferred 1R1W SRAM (async-read / write-first on the next cycle).
        // Drop in a vendor SRAM here; ports stay {we, waddr, wdata, raddr, rdata}.
        reg [DATA_W-1:0] mem [0:DEPTH-1];
        always @(posedge clk) begin
            if (i_we)
                mem[waddr] <= i_wdata;
        end
        assign o_rdata = mem[raddr];
    end
    endgenerate

endmodule
