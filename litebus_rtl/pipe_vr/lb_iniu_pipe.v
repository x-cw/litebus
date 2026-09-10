//============================================================================
// Filename    : lb_iniu_pipe.v
// Author      : litebus
// Description : ext_pipe: valid-ready skid buffer for external interface
// Date        : 2026-08-06
// Revision    : 1.0 initial -- per-round changes are in doc/HISTORY.md
//----------------------------------------------------------------------------
// Generic register-slice for the external (valid-ready) interface (SPEC stage
// 5 Pipe, EXT_PIPE). Being valid-ready, it is a full-throughput skid buffer:
// inserts one register stage to break long combinational paths without losing
// bandwidth (a skid register holds one beat under downstream backpressure).
// For the internal credit interface use lb_iniu_int_pipe (no ready, pipelines
// credit signals).
//   WIDTH  : channel flit width, passed per channel by the parent.
//   ENABLE : insert a stage or not (EXT_PIPE_{ch} / INT_PIPE_{ch}); ENABLE=0
//            bypasses to pure combinational feed-through (zero area/latency).
// One instance per channel CMD/WD/RSP_RD/RSP_WR, only WIDTH/ENABLE differ.
//============================================================================
`include "lb_defines.vh"

module lb_iniu_pipe #(
    parameter WIDTH  = 32,                   // flit width
    parameter ENABLE = 1                     // 1 = insert one skid stage; 0 = feed-through
) (
    // ---- inputs ----
    input wire               clk,       // clock
    input wire               rst_n,     // async reset (sync release), active low
    input wire  [WIDTH-1:0]  in_data,   // upstream (producer) data
    input wire               in_valid,  // upstream valid
    input wire               out_ready, // downstream (consumer) ready
    // ---- outputs ----
    output wire              in_ready,  // upstream ready
    output wire [WIDTH-1:0]  out_data,  // downstream data
    output wire              out_valid  // downstream valid
);
    generate
    if (ENABLE == 0) begin : g_bypass
        //--------------------------------------------------------------------
        // No stage: pure combinational feed-through
        //--------------------------------------------------------------------
        assign out_data  = in_data;
        assign out_valid = in_valid;
        assign in_ready  = out_ready;
    end else begin : g_skid
        //--------------------------------------------------------------------
        // One skid buffer stage: pipelines without throughput loss.
        //   - Normally data is registered one beat then forwarded.
        //   - Under downstream backpressure (out_ready=0), the skid register
        //     holds one upstream beat so upstream can still be ready that
        //     cycle, avoiding bubbles.
        //--------------------------------------------------------------------
        reg  [WIDTH-1:0] data_q;  // main register data
        reg              valid_q; // main register valid
        reg  [WIDTH-1:0] skid_q;  // skid register data (holds under backpressure)
        reg              skid_v;  // skid register valid

        assign in_ready  = ~skid_v;  // upstream accepted while skid is free
        assign out_valid = valid_q;
        assign out_data  = data_q;

        always @(posedge clk or negedge rst_n) begin
            if (!rst_n) begin
                valid_q <= 1'b0;
                skid_v  <= 1'b0;
                data_q  <= {WIDTH{1'b0}};
                skid_q  <= {WIDTH{1'b0}};
            end else begin
                // output handshake: main register cleared when consumed
                if (out_valid && out_ready) begin
                    if (skid_v) begin
                        data_q  <= skid_q;   // refill main from skid
                        valid_q <= 1'b1;
                        skid_v  <= 1'b0;
                    end else begin
                        valid_q <= 1'b0;
                    end
                end
                // input handshake: load when upstream provides data
                if (in_valid && in_ready) begin
                    if (!valid_q || (out_valid && out_ready)) begin
                        data_q  <= in_data;  // main empty (or consumed this cycle) -> main
                        valid_q <= 1'b1;
                    end else begin
                        skid_q  <= in_data;  // main busy and backpressured -> skid
                        skid_v  <= 1'b1;
                    end
                end
            end
        end
    end
    endgenerate
endmodule
