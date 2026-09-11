//============================================================================
// Filename    : adapter_qch.v
// Description : [feature] Q-Channel controller (QCH_EN). ARM-style qreqn /
//               qacceptn / qdeny plus qactive and lbus_pwrdn (L1).
//============================================================================
`include "adapter_ip_defs.vh"
`default_nettype none
`timescale 1ns/1ps

module adapter_qch #(
    parameter HAS_QDENY = 1
) (
    input  wire clk,
    input  wire rst_n,
    input  wire qreqn,
    input  wire reg_qdeny_en,
    input  wire reg_err_en,
    input  wire busy,
    output reg  qacceptn,
    output reg  qdeny,
    output wire qactive,
    output wire lbus_pwrdn,
    output wire o_quiesce,
    output wire o_err_mode
);
    localparam ST_UP      = 2'd0;
    localparam ST_DENY    = 2'd1;
    localparam ST_QUIESCE = 2'd2;
    localparam ST_DOWN    = 2'd3;

    reg [1:0] st;

    assign qactive    = busy || (st == ST_QUIESCE);
    assign lbus_pwrdn = (st == ST_DOWN);
    assign o_quiesce  = (st == ST_QUIESCE) && !reg_err_en;
    assign o_err_mode = (st == ST_QUIESCE) &&  reg_err_en;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st       <= ST_UP;
            qacceptn <= 1'b1;
            qdeny    <= 1'b0;
        end else begin
            case (st)
            ST_UP: begin
                qacceptn <= 1'b1;
                qdeny    <= 1'b0;
                if (!qreqn) begin
                    if (!busy) begin
                        st       <= ST_DOWN;
                        qacceptn <= 1'b0;
                    end else if (HAS_QDENY && reg_qdeny_en) begin
                        st    <= ST_DENY;
                        qdeny <= 1'b1;
                    end else begin
                        st <= ST_QUIESCE;
                    end
                end
            end
            ST_DENY: begin
                qdeny <= 1'b1;
                if (qreqn) begin
                    st    <= ST_UP;
                    qdeny <= 1'b0;
                end
            end
            ST_QUIESCE: begin
                if (!busy) begin
                    st       <= ST_DOWN;
                    qacceptn <= 1'b0;
                end
                if (qreqn) begin
                    st       <= ST_UP;
                    qacceptn <= 1'b1;
                end
            end
            ST_DOWN: begin
                qacceptn <= 1'b0;
                qdeny    <= 1'b0;
                if (qreqn) begin
                    st       <= ST_UP;
                    qacceptn <= 1'b1;
                end
            end
            default: st <= ST_UP;
            endcase
        end
    end

endmodule
