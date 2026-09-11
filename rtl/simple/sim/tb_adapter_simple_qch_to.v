//============================================================================
// Filename    : tb_adapter_simple_qch_to.v
// Description : QCH timeout (设计方案 7.4.4). Compile with -DADP_QCH_TO and
//               only adapter_qch.v. mst/slv expose timeout ports under the
//               same macro; this TB stays qch-only.
//============================================================================
`include "adapter_defs.vh"
`default_nettype none
`timescale 1ns/1ps

module tb_adapter_simple_qch_to;
    parameter QCH_TO_W = 8;

    reg clk, rst_n;
    integer err_cnt, to, k;

    reg qreqn, busy, qdeny_en, err_en;
    reg qch_to_en, qch_to_mode;
    reg [QCH_TO_W-1:0] qch_to_lim;
    wire qacceptn, qdeny, qactive, lbus_pwrdn, quiesce, err_mode;
    wire irq_qch_to, o_to_err;

    adapter_qch #(.HAS_QDENY(1), .QCH_TO_W(QCH_TO_W)) u_qch (
        .clk(clk), .rst_n(rst_n),
        .qreqn(qreqn), .reg_qdeny_en(qdeny_en), .reg_err_en(err_en),
        .busy(busy),
        .qacceptn(qacceptn), .qdeny(qdeny), .qactive(qactive),
        .lbus_pwrdn(lbus_pwrdn), .o_quiesce(quiesce), .o_err_mode(err_mode),
        .qch_to_en(qch_to_en), .qch_to_lim(qch_to_lim), .qch_to_mode(qch_to_mode),
        .irq_qch_to(irq_qch_to), .o_to_err(o_to_err)
    );

    initial clk = 0;
    always #5 clk = ~clk;

    task enter_quiesce;
        begin
            busy = 1; qdeny_en = 0; err_en = 0; qreqn = 0;
            @(posedge clk); @(negedge clk);
            if (!quiesce) begin
                $display("FAIL not QUIESCE"); err_cnt = err_cnt + 1;
            end
        end
    endtask

    initial begin
        err_cnt = 0;
        rst_n = 0; qreqn = 1; busy = 0; qdeny_en = 0; err_en = 0;
        qch_to_en = 0; qch_to_lim = 0; qch_to_mode = 0;
        #20 rst_n = 1;
        @(negedge clk);

        $display("--- Q-01 to_en=0 never irq ---");
        enter_quiesce;
        qch_to_en = 0; qch_to_lim = 0; qch_to_mode = 0;
        repeat (12) @(posedge clk);
        @(negedge clk);
        if (irq_qch_to) begin $display("FAIL irq with to_en=0"); err_cnt = err_cnt + 1; end
        if (lbus_pwrdn) begin $display("FAIL down while busy"); err_cnt = err_cnt + 1; end
        qreqn = 1; busy = 0;
        @(posedge clk); @(negedge clk);

        $display("--- Q-02 lim=0 WAIT: irq next QUIESCE beat, no force DOWN ---");
        qch_to_en = 1; qch_to_lim = 0; qch_to_mode = 0;
        enter_quiesce;
        @(posedge clk); @(negedge clk);
        if (!irq_qch_to) begin $display("FAIL lim=0 no irq"); err_cnt = err_cnt + 1; end
        if (o_to_err) begin $display("FAIL WAIT o_to_err"); err_cnt = err_cnt + 1; end
        if (lbus_pwrdn || !qacceptn) begin
            $display("FAIL WAIT forced DOWN"); err_cnt = err_cnt + 1;
        end
        if (!quiesce) begin $display("FAIL WAIT left QUIESCE"); err_cnt = err_cnt + 1; end
        busy = 0;
        @(posedge clk); @(posedge clk); @(negedge clk);
        if (qacceptn || !lbus_pwrdn) begin $display("FAIL no DOWN after ost"); err_cnt = err_cnt + 1; end
        if (irq_qch_to) begin $display("FAIL irq sticky after DOWN"); err_cnt = err_cnt + 1; end
        qreqn = 1;
        @(posedge clk); @(negedge clk);

        $display("--- Q-03 lim=3 ERR: irq + o_to_err, WAIT until busy=0 ---");
        qch_to_en = 1; qch_to_lim = 8'd3; qch_to_mode = 1;
        enter_quiesce;
        for (k = 0; k < 8; k = k + 1) begin
            if (irq_qch_to) k = 99;
            else @(posedge clk);
        end
        @(negedge clk);
        if (!irq_qch_to) begin $display("FAIL lim=3 no irq"); err_cnt = err_cnt + 1; end
        if (!o_to_err) begin $display("FAIL ERR o_to_err"); err_cnt = err_cnt + 1; end
        if (lbus_pwrdn) begin $display("FAIL ERR auto DOWN"); err_cnt = err_cnt + 1; end
        repeat (4) @(posedge clk);
        if (!irq_qch_to) begin $display("FAIL irq not sticky"); err_cnt = err_cnt + 1; end
        busy = 0;
        @(posedge clk); @(posedge clk); @(negedge clk);
        if (!lbus_pwrdn) begin $display("FAIL ERR then DOWN"); err_cnt = err_cnt + 1; end
        if (irq_qch_to) begin $display("FAIL irq after ERR DOWN"); err_cnt = err_cnt + 1; end
        qreqn = 1;
        @(posedge clk); @(negedge clk);

        $display("--- Q-04 qreqn=1 clears irq in QUIESCE ---");
        qch_to_en = 1; qch_to_lim = 0; qch_to_mode = 0;
        enter_quiesce;
        @(posedge clk); @(negedge clk);
        if (!irq_qch_to) begin $display("FAIL Q-04 no irq"); err_cnt = err_cnt + 1; end
        qreqn = 1;
        @(posedge clk); @(negedge clk);
        if (irq_qch_to) begin $display("FAIL irq after qreqn=1"); err_cnt = err_cnt + 1; end
        if (!qacceptn) begin $display("FAIL not back UP"); err_cnt = err_cnt + 1; end
        busy = 0;
        @(posedge clk); @(negedge clk);

        $display("--- Q-05 abort before lim ---");
        qch_to_en = 1; qch_to_lim = 8'd20; qch_to_mode = 1;
        enter_quiesce;
        repeat (3) @(posedge clk);
        @(negedge clk);
        if (irq_qch_to) begin $display("FAIL early irq"); err_cnt = err_cnt + 1; end
        qreqn = 1; busy = 0;
        @(posedge clk); @(negedge clk);
        if (irq_qch_to) begin $display("FAIL irq after abort"); err_cnt = err_cnt + 1; end

        if (err_cnt == 0) $display("=== ALL TESTS PASSED ===");
        else $display("=== %0d TEST(S) FAILED ===", err_cnt);
        $finish;
    end

    initial begin
        #200000;
        $display("TIMEOUT tb_adapter_simple_qch_to");
        $finish;
    end
endmodule
