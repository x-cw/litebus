//------------------------------------------------------------------------
// axi_litebus_adaptor.v - AXI4 full master transaction -> LiteBus IP interface
//------------------------------------------------------------------------
// Converts AXI4 (full) transactions from an AXI master into the LiteBus
// Master-side (IP) valid-ready interface exposed by lb_iniu_ext_core:
//
//     REQ_R  (CMD only) : AR            -> req_r_data {opcode, addr, len, txnid}
//     REQ_W  (CMD + WD) : AW/W per beat -> req_w_data {CMD, WD}
//     RSP_RD             : rsp_rd_*     -> R
//     RSP_WR             : rsp_wr_*     -> B
//
// Read channel is a pure combinational pass-through (naturally supports
// multiple outstanding reads, since no state is kept).  Write channel is a
// three-state FSM that serializes write transactions: one AW is accepted,
// then len+1 W beats are streamed (CMD repeated with the SAME starting
// address every beat -- the fabric tracks the burst position), then the B
// response is returned.  Reads and writes run fully in parallel.
//
// Supported subset (constraints, see README):
//   - INCR bursts only, data width == beat size (8 bytes at the default),
//     addresses aligned to the beat size
//   - AW must precede W (W arriving first is simply held: wready stays 0)
//   - Write transactions serialized; no write interleaving
//   - awsize/awburst/awlock/awcache/awprot/awqos are ignored
//   - user/qos sidebands are tied to zero (matches the target bus instance)
//
// LiteBus flit packing (MSB..LSB), for the target bus config
//   EXT_USER_WIDTH_* = 0, EXT_QOS_W = 0, EXT_MOD_W = 0:
//     CMD  = {opcode[3:0], addr[31:0], len[7:0], txnid[7:0]}
//     WD   = {data[63:0], strb[7:0], last, txnid[7:0]}
//     REQ_W = {CMD[51:0], WD[80:0]}
//   opcodes: LB_OP_RD = 4'h1 (write bit clear), LB_OP_WR = 4'h8 (write bit set)
//   resp:    LB_RESP_OK = 00, LB_RESP_FAIL = 01 -> AXI OKAY / SLVERR
//------------------------------------------------------------------------
`default_nettype none
`timescale 1ns/1ps

module axi_litebus_adaptor #(
    parameter AXI_ADDR_W = 32,
    parameter AXI_DATA_W = 64,
    parameter AXI_LEN_W  = 8,
    parameter AXI_ID_W   = 8
)(
    input  wire                    clk,
    input  wire                    rst_n,

    // ---------------- AXI4 full slave port ----------------
    input  wire [AXI_ID_W-1:0]     s_axi_awid,
    input  wire [AXI_ADDR_W-1:0]   s_axi_awaddr,
    input  wire [AXI_LEN_W-1:0]    s_axi_awlen,
    input  wire [2:0]              s_axi_awsize,
    input  wire [1:0]              s_axi_awburst,
    input  wire                    s_axi_awlock,
    input  wire [3:0]              s_axi_awcache,
    input  wire [2:0]              s_axi_awprot,
    input  wire [3:0]              s_axi_awqos,
    input  wire                    s_axi_awvalid,
    output reg                     s_axi_awready,
    input  wire [AXI_DATA_W-1:0]   s_axi_wdata,
    input  wire [AXI_DATA_W/8-1:0] s_axi_wstrb,
    input  wire                    s_axi_wlast,
    input  wire                    s_axi_wvalid,
    output reg                     s_axi_wready,
    output reg  [AXI_ID_W-1:0]     s_axi_bid,
    output reg  [1:0]              s_axi_bresp,
    output reg                     s_axi_bvalid,
    input  wire                    s_axi_bready,
    input  wire [AXI_ID_W-1:0]     s_axi_arid,
    input  wire [AXI_ADDR_W-1:0]   s_axi_araddr,
    input  wire [AXI_LEN_W-1:0]    s_axi_arlen,
    input  wire [2:0]              s_axi_arsize,
    input  wire [1:0]              s_axi_arburst,
    input  wire                    s_axi_arlock,
    input  wire [3:0]              s_axi_arcache,
    input  wire [2:0]              s_axi_arprot,
    input  wire [3:0]              s_axi_arqos,
    input  wire                    s_axi_arvalid,
    output wire                    s_axi_arready,
    output wire [AXI_ID_W-1:0]     s_axi_rid,
    output wire [AXI_DATA_W-1:0]   s_axi_rdata,
    output wire [1:0]              s_axi_rresp,
    output wire                    s_axi_rlast,
    output wire                    s_axi_rvalid,
    input  wire                    s_axi_rready,

    // ---------------- LiteBus Master-side (IP) interface ----------------
    output wire [EXT_CMD_W-1:0]    req_r_data,
    output wire                    req_r_valid,
    input  wire                    req_r_ready,
    output reg  [EXT_REQ_W-1:0]    req_w_data,
    output reg                     req_w_valid,
    input  wire                    req_w_ready,
    input  wire [AXI_DATA_W-1:0]   rsp_rd_data,
    input  wire                    rsp_rd_last,
    input  wire [1:0]              rsp_rd_resp,
    input  wire [AXI_ID_W-1:0]     rsp_rd_ext_txnid,
    input  wire                    rsp_rd_valid,
    output wire                    rsp_rd_user,        // 1-bit stub (user=0 bus)
    output wire                    rsp_rd_ready,
    input  wire [1:0]              rsp_wr_resp,
    input  wire [AXI_ID_W-1:0]     rsp_wr_ext_txnid,
    input  wire                    rsp_wr_valid,
    output wire                    rsp_wr_user,        // 1-bit stub (user=0 bus)
    output reg                     rsp_wr_ready
);
    //------------------------------------------------------------------------
    // Derived widths (parameter list sized from the AXI side).
    //------------------------------------------------------------------------
    localparam LB_STRB_W = AXI_DATA_W / 8;                              // 8
    localparam EXT_CMD_W = 4 + AXI_ADDR_W + AXI_LEN_W + AXI_ID_W;       // 52
    localparam EXT_WD_W  = AXI_DATA_W + LB_STRB_W + 1 + AXI_ID_W;       // 81
    localparam EXT_REQ_W = EXT_CMD_W + EXT_WD_W;                        // 133

    //------------------------------------------------------------------------
    // Opcode / response constants (lb_defines.vh).
    //------------------------------------------------------------------------
    localparam LB_OP_WR   = 4'h8;   // write opcode, bit[3] set
    localparam LB_OP_RD   = 4'h1;   // read  opcode, bit[3] clear
    localparam LB_RESP_OK = 2'b00;

    //------------------------------------------------------------------------
    // Write FSM states.
    //------------------------------------------------------------------------
    localparam S_IDLE  = 2'd0;    // accept AW, capture address/len/id
    localparam S_WDATA = 2'd1;    // stream len+1 W beats as REQ_W
    localparam S_WRESP = 2'd2;    // wait for RSP_WR, return B

    reg [1:0]              state;
    reg [1:0]              state_nxt;
    reg [AXI_ADDR_W-1:0]   awaddr_reg;
    reg [AXI_ADDR_W-1:0]   awaddr_nxt;
    reg [AXI_LEN_W-1:0]    awlen_reg;
    reg [AXI_LEN_W-1:0]    awlen_nxt;
    reg [AXI_ID_W-1:0]     awid_reg;
    reg [AXI_ID_W-1:0]     awid_nxt;
    reg [AXI_LEN_W:0]      beat_cnt;         // beats accepted so far
    reg [AXI_LEN_W:0]      beat_cnt_nxt;

    wire [EXT_CMD_W-1:0]   cmd_wr;
    wire [EXT_CMD_W-1:0]   cmd_rd;
    wire [EXT_WD_W-1:0]    wd;
    wire [EXT_REQ_W-1:0]   req_w_pk;

    //------------------------------------------------------------------------
    // Response code mapping: LB OK=00 / FAIL=01 / ATOMIC_FAIL=10
    //                          -> AXI OKAY=00 / SLVERR=10 (all non-OK -> SLVERR)
    //------------------------------------------------------------------------
    function [1:0] resp_map;
        input [1:0] lb_resp;
        begin
            resp_map = (lb_resp == LB_RESP_OK) ? 2'b00 : 2'b10;
        end
    endfunction

    //------------------------------------------------------------------------
    // LiteBus CMD / WD packing (see header comment for bit layout).
    // CMD of a write is repeated verbatim on every REQ_W beat: the fabric
    // tracks the write burst position internally, so the address is never
    // incremented here.
    //------------------------------------------------------------------------
    assign cmd_wr = {LB_OP_WR, awaddr_reg, awlen_reg, awid_reg};
    assign cmd_rd = {LB_OP_RD, s_axi_araddr, s_axi_arlen, s_axi_arid};
    // AXI W carries no ID: the WD txnid is the AW-captured awid.
    assign wd      = {s_axi_wdata, s_axi_wstrb, s_axi_wlast, awid_reg};
    assign req_w_pk = {cmd_wr, wd};

    //------------------------------------------------------------------------
    // Read path: pure combinational pass-through (AR -> REQ_R, RSP_RD -> R).
    // No state, so any number of outstanding reads are naturally supported.
    //------------------------------------------------------------------------
    assign req_r_valid   = s_axi_arvalid;
    assign req_r_data    = cmd_rd;
    assign s_axi_arready = req_r_ready;
    assign s_axi_rvalid  = rsp_rd_valid;
    assign s_axi_rdata   = rsp_rd_data;
    assign s_axi_rlast   = rsp_rd_last;
    assign s_axi_rresp   = resp_map(rsp_rd_resp);
    assign s_axi_rid     = rsp_rd_ext_txnid;
    assign rsp_rd_ready  = s_axi_rready;

    //------------------------------------------------------------------------
    // Write FSM (combinational): IDLE -> WDATA -> WRESP -> IDLE.
    //------------------------------------------------------------------------
    always @* begin
        state_nxt    = state;
        awaddr_nxt   = awaddr_reg;
        awlen_nxt    = awlen_reg;
        awid_nxt     = awid_reg;
        beat_cnt_nxt = beat_cnt;

        s_axi_awready = 1'b0;
        s_axi_wready  = 1'b0;
        req_w_valid   = 1'b0;
        req_w_data    = {EXT_REQ_W{1'b0}};
        s_axi_bvalid  = 1'b0;
        s_axi_bresp   = 2'b00;
        s_axi_bid     = {AXI_ID_W{1'b0}};
        rsp_wr_ready  = 1'b0;

        case (state)
            S_IDLE: begin
                // Accept AW immediately. W cannot be accepted before AW
                // (wready stays 0), which the constraint requires.
                if (s_axi_awvalid) begin
                    s_axi_awready = 1'b1;
                    awaddr_nxt    = s_axi_awaddr;
                    awlen_nxt     = s_axi_awlen;
                    awid_nxt      = s_axi_awid;
                    beat_cnt_nxt  = {(AXI_LEN_W+1){1'b0}};
                    state_nxt     = S_WDATA;
                end
            end

            S_WDATA: begin
                // One REQ_W flit per beat: CMD repeated + WD of this beat.
                s_axi_wready = req_w_ready;
                req_w_valid  = s_axi_wvalid;
                req_w_data   = req_w_pk;
                if (s_axi_wvalid && req_w_ready) begin
                    if (beat_cnt == awlen_reg) begin
                        state_nxt = S_WRESP;         // last beat of the burst
                    end else begin
                        beat_cnt_nxt = beat_cnt + 1'b1;
                    end
                end
            end

            S_WRESP: begin
                // RSP_WR -> B. The B ID is the RSP_WR txnid, which the fabric
                // returned as the AW id.
                s_axi_bvalid = rsp_wr_valid;
                s_axi_bresp  = resp_map(rsp_wr_resp);
                s_axi_bid    = rsp_wr_ext_txnid;
                rsp_wr_ready = s_axi_bready;
                if (rsp_wr_valid && s_axi_bready) begin
                    state_nxt = S_IDLE;
                end
            end

            default: begin
                state_nxt = S_IDLE;
            end
        endcase
    end

    //------------------------------------------------------------------------
    // Write FSM (sequential).  Async assert / sync release reset convention.
    //------------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state       <= S_IDLE;
            awaddr_reg  <= {AXI_ADDR_W{1'b0}};
            awlen_reg   <= {AXI_LEN_W{1'b0}};
            awid_reg    <= {AXI_ID_W{1'b0}};
            beat_cnt    <= {(AXI_LEN_W+1){1'b0}};
        end else begin
            state       <= state_nxt;
            awaddr_reg  <= awaddr_nxt;
            awlen_reg   <= awlen_nxt;
            awid_reg    <= awid_nxt;
            beat_cnt    <= beat_cnt_nxt;
        end
    end

    //------------------------------------------------------------------------
    // user sidebands: the target bus instance has EXT_USER_WIDTH_* = 0, so
    // these pins are 1-bit stubs driven low (never left floating).
    //------------------------------------------------------------------------
    assign rsp_rd_user = 1'b0;
    assign rsp_wr_user = 1'b0;

endmodule
