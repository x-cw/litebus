//============================================================================
// Filename    : adapter_atomic.v
// Description : [feature] mst-side AXI5 atomic decode (ATOMIC_EN).
//               AWATOP[1:0] -> LiteBus atomic opcode; AWATOP[4:2] -> modifier.
//               o_need_r = 1 for LOAD/SWAP/COMPARE (B + R responses).
//============================================================================
`include "adapter_ip_defs.vh"
`default_nettype none
`timescale 1ns/1ps

module adapter_atomic #(
    parameter AWATOP_W = 5,
    parameter MOD_W    = 0,
    parameter MOD_PW   = (MOD_W < 1) ? 1 : MOD_W
) (
    input  wire [AWATOP_W-1:0] i_awatop,
    output reg  [3:0]          o_opcode,
    output reg                 o_need_r,
    output reg  [MOD_PW-1:0]   o_mod
);
    always @* begin
        case (i_awatop[1:0])
            2'b00: begin o_opcode = `LB_OP_ATOMIC_STORE;   o_need_r = 1'b0; end
            2'b01: begin o_opcode = `LB_OP_ATOMIC_LOAD;    o_need_r = 1'b1; end
            2'b10: begin o_opcode = `LB_OP_ATOMIC_SWAP;    o_need_r = 1'b1; end
            default: begin o_opcode = `LB_OP_ATOMIC_COMPARE; o_need_r = 1'b1; end
        endcase
        o_mod = {MOD_PW{1'b0}};
        if (MOD_PW >= 3)
            o_mod[2:0] = i_awatop[4:2];
        else if (MOD_PW == 2)
            o_mod[1:0] = i_awatop[3:2];
        else if (MOD_PW == 1)
            o_mod[0] = i_awatop[2];
    end

endmodule
