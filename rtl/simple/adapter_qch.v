//============================================================================
// Filename    : adapter_qch.v
// Description : Q-Channel (ARM-style) for the simple adapter. Same state
//               machine as rtl/adapter_qch.v: UP / DENY / QUIESCE / DOWN.
//============================================================================
`include "adapter_defs.vh"
`default_nettype none
`timescale 1ns/1ps

module adapter_qch #(
    parameter HAS_QDENY = 1
`ifdef ADP_QCH_TO
    , parameter QCH_TO_W = 16
`endif
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
`ifdef ADP_QCH_TO
    , input  wire                   qch_to_en
    , input  wire [QCH_TO_W-1:0]   qch_to_lim
    , input  wire                   qch_to_mode
    , output reg                    irq_qch_to
    , output wire                   o_to_err
`endif
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

`ifdef ADP_QCH_TO
    // Timeout lives only in QUIESCE. It does not change the 4-state graph:
    // WAIT keeps waiting for busy=0; ERR is reported so the parent can drop busy.
    // lim=0: first cycle already in QUIESCE is a timeout (next beat after entry).
    reg [QCH_TO_W-1:0] to_cnt;
    reg                to_mode_q;

    assign o_to_err = irq_qch_to && to_mode_q;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            to_cnt     <= {QCH_TO_W{1'b0}};
            irq_qch_to <= 1'b0;
            to_mode_q  <= 1'b0;
        end else if ((st == ST_DOWN) || qreqn) begin
            to_cnt     <= {QCH_TO_W{1'b0}};
            irq_qch_to <= 1'b0;
        end else if (st == ST_QUIESCE) begin
            if (qch_to_en && !irq_qch_to) begin
                if (to_cnt == qch_to_lim) begin
                    irq_qch_to <= 1'b1;
                    to_mode_q  <= qch_to_mode;
                end else
                    to_cnt <= to_cnt + 1'b1;
            end
        end else
            to_cnt <= {QCH_TO_W{1'b0}};
    end
`endif

endmodule
