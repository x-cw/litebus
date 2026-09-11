//============================================================================
// Filename    : adapter_ip_defs.vh
// Description : AXI/APB <-> LiteBus adapter shared definitions
//               Constant values mirror litebus_rtl/lb_defines.vh (authority).
//               They are re-declared here so standalone verification does not
//               need litebus_rtl on the include path. Guarded by `ifndef so a
//               real litebus_rtl include takes precedence later.
//============================================================================
`ifndef ADAPTER_IP_DEFS_VH
`define ADAPTER_IP_DEFS_VH

// ---------------- opcode / response constants (mirror lb_defines.vh) --------
`ifndef LB_OPCODE_WIDTH
`define LB_OPCODE_WIDTH 4
`endif
`ifndef LB_OPCODE_WR_BIT
`define LB_OPCODE_WR_BIT 3
`endif
`ifndef LB_RESP_WIDTH
`define LB_RESP_WIDTH 2
`endif
`ifndef LB_RESP_OK
`define LB_RESP_OK 2'b00
`endif
`ifndef LB_RESP_FAIL
`define LB_RESP_FAIL 2'b01
`endif
`ifndef LB_RESP_ATOMIC_FAIL
`define LB_RESP_ATOMIC_FAIL 2'b10
`endif
`ifndef LB_OP_WR
`define LB_OP_WR 4'h8
`endif
`ifndef LB_OP_RD
`define LB_OP_RD 4'h1
`endif
`ifndef LB_OP_ATOMIC_STORE
`define LB_OP_ATOMIC_STORE 4'hC
`endif
`ifndef LB_OP_ATOMIC_LOAD
`define LB_OP_ATOMIC_LOAD 4'hD
`endif
`ifndef LB_OP_ATOMIC_SWAP
`define LB_OP_ATOMIC_SWAP 4'hE
`endif
`ifndef LB_OP_ATOMIC_COMPARE
`define LB_OP_ATOMIC_COMPARE 4'hF
`endif

// AXI response codes
`define AXI_RESP_OKAY   2'b00
`define AXI_RESP_EXOKAY 2'b01
`define AXI_RESP_SLVERR 2'b10
`define AXI_RESP_DECERR 2'b11

// AXI burst types
`define AXI_BURST_FIXED 2'b00
`define AXI_BURST_INCR  2'b01
`define AXI_BURST_WRAP  2'b10

// ceil(log2(v)), v >= 1. Macro so Icarus (no file-scope functions) and
// Verilator can both use it in parameter/port widths: `adp_clog2(N)
`ifndef adp_clog2
`define adp_clog2(v) ( \
    ((v) <= 1) ? 0 : ((v) <= 2) ? 1 : ((v) <= 4) ? 2 : ((v) <= 8) ? 3 : \
    ((v) <= 16) ? 4 : ((v) <= 32) ? 5 : ((v) <= 64) ? 6 : ((v) <= 128) ? 7 : \
    ((v) <= 256) ? 8 : ((v) <= 512) ? 9 : ((v) <= 1024) ? 10 : ((v) <= 2048) ? 11 : \
    ((v) <= 4096) ? 12 : ((v) <= 8192) ? 13 : ((v) <= 16384) ? 14 : ((v) <= 32768) ? 15 : 16)
`endif

`endif // ADAPTER_IP_DEFS_VH
