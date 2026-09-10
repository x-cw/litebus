//============================================================================
// Filename    : lb_iniu_int_pipe.v
// Author      : litebus
// Description : internal credit-channel pipeline (INT_PIPE)
// Date        : 2026-08-06
// Revision    : 1.0 initial -- per-round changes are in doc/HISTORY.md
//----------------------------------------------------------------------------
// Unlike ext_pipe (valid-ready skid buffer), the internal interface uses credit
// flow control, with signals:
//   forward : payload + valid  (no ready -- credit guarantees peer has space)
//   reverse : credit_return    (one pulse per peer pop)
// So int_pipe is a simple pipeline register: it registers forward (payload,
// valid) and reverse credit_return by STAGES stages, purely adding pipeline
// depth; no skid, no credit generation/consumption.
//   WIDTH  : payload (flit) width.
//   ENABLE : pipeline or not (INT_PIPE_{ch}).
//   STAGES : number of pipeline stages (default 1). ENABLE=0 bypasses.
//============================================================================
`include "lb_defines.vh"

module lb_iniu_int_pipe #(
    parameter WIDTH  = 32,                   // payload (flit) width
    parameter ENABLE = 1,                    // 1 = pipeline; 0 = bypass
    parameter STAGES = 1                     // number of pipeline stages
) (
    // ---- inputs ----
    input wire               clk,               // clock
    input wire               rst_n,             // async reset, active low
    input wire  [WIDTH-1:0]  in_data,           // forward payload
    input wire               in_valid,          // forward valid
    input wire               out_credit_return, // reverse credit from downstream
    // ---- outputs ----
    output wire              in_credit_return,  // reverse credit to upstream
    output wire [WIDTH-1:0]  out_data,          // forward payload out
    output wire              out_valid          // forward valid out
);
    generate
    if (ENABLE == 0 || STAGES == 0) begin : g_bypass
        //--------------------------------------------------------------------
        // No pipeline: feed-through
        //--------------------------------------------------------------------
        assign out_data         = in_data;
        assign out_valid        = in_valid;
        assign in_credit_return = out_credit_return;
    end else begin : g_reg
        //--------------------------------------------------------------------
        // Forward: payload+valid by STAGES; reverse: credit_return by STAGES.
        // No handshake / no skid -- credit guarantees no loss (peer has space).
        //--------------------------------------------------------------------
        // ---- declarations (all up front, one per line) ----
        genvar           i;                // stage index
        wire [WIDTH-1:0] fdata [0:STAGES]; // forward data per stage
        wire             fval  [0:STAGES]; // forward valid per stage
        wire             crd   [0:STAGES]; // reverse credit per stage

        // ---- forward pipeline ----
        assign fdata[0] = in_data;
        assign fval[0]  = in_valid;
        for (i = 0; i < STAGES; i = i + 1) begin : g_fwd
            reg [WIDTH-1:0] d_q; // stage data register
            reg             v_q; // stage valid register
            always @(posedge clk or negedge rst_n) begin
                if (!rst_n) begin
                    d_q <= {WIDTH{1'b0}};
                    v_q <= 1'b0;
                end else begin
                    d_q <= fdata[i];
                    v_q <= fval[i];
                end
            end
            assign fdata[i+1] = d_q;
            assign fval[i+1]  = v_q;
        end
        assign out_data  = fdata[STAGES];
        assign out_valid = fval[STAGES];

        // ---- reverse credit pipeline ----
        assign crd[0] = out_credit_return;
        for (i = 0; i < STAGES; i = i + 1) begin : g_bwd
            reg c_q;                         // stage credit register
            always @(posedge clk or negedge rst_n) begin
                if (!rst_n) begin
                    c_q <= 1'b0;
                end
                else begin
                    c_q <= crd[i];
                end
            end
            assign crd[i+1] = c_q;
        end
        assign in_credit_return = crd[STAGES];
    end
    endgenerate
endmodule
