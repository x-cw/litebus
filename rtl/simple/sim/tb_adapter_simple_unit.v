//============================================================================
// Filename    : tb_adapter_simple_unit.v
// Description : Unit tests: qch, atomic, err_rsp, ip_fail.
//============================================================================
`include "adapter_defs.vh"
`default_nettype none
`timescale 1ns/1ps

module tb_adapter_simple_unit;
    parameter ADDR_W = 32;
    parameter DATA_W = 64;
    parameter LEN_W  = 8;
    parameter ID_W   = 8;
    parameter USER_CMD_PW = 1;
    parameter USER_RSP_PW = 1;
    parameter QOS_PW = 1;
    parameter MOD_PW = 1;
    parameter EXT_CMD_W = QOS_PW + 4 + ADDR_W + LEN_W + ID_W + USER_CMD_PW;
    parameter EXT_WD_W  = DATA_W + DATA_W/8 + 1 + ID_W;
    parameter EXT_REQ_W = EXT_CMD_W + MOD_PW + EXT_WD_W;

    reg clk, rst_n;
    integer err_cnt;
    integer to;

    // -------- QCH --------
    reg qreqn, busy, qdeny_en, err_en;
    wire qacceptn, qdeny, qactive, lbus_pwrdn, quiesce, err_mode;

    adapter_qch #(.HAS_QDENY(1)) u_qch (
        .clk(clk), .rst_n(rst_n),
        .qreqn(qreqn), .reg_qdeny_en(qdeny_en), .reg_err_en(err_en),
        .busy(busy),
        .qacceptn(qacceptn), .qdeny(qdeny), .qactive(qactive),
        .lbus_pwrdn(lbus_pwrdn), .o_quiesce(quiesce), .o_err_mode(err_mode)
    );

    // -------- atomic --------
    reg  [4:0] at_i;
    wire [3:0] at_op;
    wire       at_need;
    wire [2:0] at_mod;

    adapter_atomic #(.MOD_PW(3)) u_at (
        .i_awatop(at_i), .o_opcode(at_op), .o_need_r(at_need), .o_mod(at_mod)
    );

    // -------- err_rsp --------
    reg intercept;
    wire intercept_on;
    reg [EXT_CMD_W-1:0] u_req_r_data;
    reg u_req_r_valid;
    wire u_req_r_ready;
    reg [EXT_REQ_W-1:0] u_req_w_data;
    reg u_req_w_valid;
    wire u_req_w_ready;
    wire [DATA_W-1:0] u_rsp_rd_data;
    wire u_rsp_rd_last;
    wire [1:0] u_rsp_rd_resp;
    wire [ID_W-1:0] u_rsp_rd_txnid;
    wire [USER_RSP_PW-1:0] u_rsp_rd_user;
    wire u_rsp_rd_valid;
    reg u_rsp_rd_ready;
    wire [1:0] u_rsp_wr_resp;
    wire [ID_W-1:0] u_rsp_wr_txnid;
    wire [USER_RSP_PW-1:0] u_rsp_wr_user;
    wire u_rsp_wr_valid;
    reg u_rsp_wr_ready;
    wire [EXT_CMD_W-1:0] d_req_r_data;
    wire d_req_r_valid;
    reg d_req_r_ready;
    wire [EXT_REQ_W-1:0] d_req_w_data;
    wire d_req_w_valid;

    adapter_err_rsp u_err (
        .clk(clk), .rst_n(rst_n),
        .intercept(intercept), .intercept_on(intercept_on),
        .u_req_r_data(u_req_r_data), .u_req_r_valid(u_req_r_valid),
        .u_req_r_ready(u_req_r_ready),
        .u_req_w_data(u_req_w_data), .u_req_w_valid(u_req_w_valid),
        .u_req_w_ready(u_req_w_ready),
        .u_rsp_rd_data(u_rsp_rd_data), .u_rsp_rd_last(u_rsp_rd_last),
        .u_rsp_rd_resp(u_rsp_rd_resp), .u_rsp_rd_ext_txnid(u_rsp_rd_txnid),
        .u_rsp_rd_user(u_rsp_rd_user), .u_rsp_rd_valid(u_rsp_rd_valid),
        .u_rsp_rd_ready(u_rsp_rd_ready),
        .u_rsp_wr_resp(u_rsp_wr_resp), .u_rsp_wr_ext_txnid(u_rsp_wr_txnid),
        .u_rsp_wr_user(u_rsp_wr_user), .u_rsp_wr_valid(u_rsp_wr_valid),
        .u_rsp_wr_ready(u_rsp_wr_ready),
        .d_req_r_data(d_req_r_data), .d_req_r_valid(d_req_r_valid),
        .d_req_r_ready(d_req_r_ready),
        .d_req_w_data(d_req_w_data), .d_req_w_valid(d_req_w_valid),
        .d_req_w_ready(1'b1),
        .d_rsp_rd_data({DATA_W{1'b0}}), .d_rsp_rd_last(1'b1),
        .d_rsp_rd_resp(2'b00), .d_rsp_rd_ext_txnid({ID_W{1'b0}}),
        .d_rsp_rd_user({USER_RSP_PW{1'b0}}), .d_rsp_rd_valid(1'b0),
        .d_rsp_rd_ready(),
        .d_rsp_wr_resp(2'b00), .d_rsp_wr_ext_txnid({ID_W{1'b0}}),
        .d_rsp_wr_user({USER_RSP_PW{1'b0}}), .d_rsp_wr_valid(1'b0),
        .d_rsp_wr_ready()
    );

    // -------- ip_fail --------
    wire f_u_r_ready, f_u_w_ready;
    wire [1:0] f_rd_resp;
    wire f_rd_valid, f_rd_last;
    wire [ID_W-1:0] f_rd_txn;
    wire [1:0] f_wr_resp;
    wire f_wr_valid;
    wire f_d_r_valid, f_d_w_valid;
    reg  f_intercept;
    reg  f_u_r_valid;
    reg [EXT_CMD_W-1:0] f_u_r_data;
    reg f_u_w_valid;
    reg [EXT_REQ_W-1:0] f_u_w_data;
    reg f_rd_ready, f_wr_ready;

    adapter_ip_fail u_fail (
        .clk(clk), .rst_n(rst_n), .intercept(f_intercept),
        .u_req_r_data(f_u_r_data), .u_req_r_valid(f_u_r_valid),
        .u_req_r_ready(f_u_r_ready),
        .u_req_w_data(f_u_w_data), .u_req_w_valid(f_u_w_valid),
        .u_req_w_ready(f_u_w_ready),
        .u_rsp_rd_data(), .u_rsp_rd_last(f_rd_last), .u_rsp_rd_resp(f_rd_resp),
        .u_rsp_rd_ext_txnid(f_rd_txn), .u_rsp_rd_user(),
        .u_rsp_rd_valid(f_rd_valid), .u_rsp_rd_ready(f_rd_ready),
        .u_rsp_wr_resp(f_wr_resp), .u_rsp_wr_ext_txnid(), .u_rsp_wr_user(),
        .u_rsp_wr_valid(f_wr_valid), .u_rsp_wr_ready(f_wr_ready),
        .d_req_r_data(), .d_req_r_valid(f_d_r_valid), .d_req_r_ready(1'b1),
        .d_req_w_data(), .d_req_w_valid(f_d_w_valid), .d_req_w_ready(1'b1),
        .d_rsp_rd_data({DATA_W{1'b0}}), .d_rsp_rd_last(1'b1),
        .d_rsp_rd_resp(2'b00), .d_rsp_rd_ext_txnid(8'h0),
        .d_rsp_rd_user(1'b0), .d_rsp_rd_valid(1'b0), .d_rsp_rd_ready(),
        .d_rsp_wr_resp(2'b00), .d_rsp_wr_ext_txnid(8'h0),
        .d_rsp_wr_user(1'b0), .d_rsp_wr_valid(1'b0), .d_rsp_wr_ready()
    );

    function [EXT_CMD_W-1:0] mk_cmd;
        input [3:0] op;
        input [31:0] addr;
        input [7:0] txn;
        begin
            mk_cmd = { {QOS_PW{1'b0}}, op, addr, 8'd0, txn, {USER_CMD_PW{1'b0}} };
        end
    endfunction

    function [EXT_REQ_W-1:0] mk_req_w;
        input [3:0] op;
        input [31:0] addr;
        input [7:0] txn;
        begin
            mk_req_w = { mk_cmd(op, addr, txn), {MOD_PW{1'b0}},
                         {DATA_W{1'b0}}, {(DATA_W/8){1'b1}}, 1'b1, txn };
        end
    endfunction

    initial clk = 0;
    always #5 clk = ~clk;

    initial begin
        err_cnt = 0;
        rst_n = 0; qreqn = 1; busy = 0; qdeny_en = 0; err_en = 0;
        at_i = 0; intercept = 0;
        u_req_r_valid = 0; u_req_w_valid = 0;
        u_rsp_rd_ready = 1; u_rsp_wr_ready = 1; d_req_r_ready = 1;
        u_req_r_data = 0; u_req_w_data = 0;
        f_intercept = 0; f_u_r_valid = 0; f_u_r_data = 0;
        f_u_w_valid = 0; f_u_w_data = 0;
        f_rd_ready = 1; f_wr_ready = 1;
        #20 rst_n = 1;
        @(negedge clk);

        $display("--- qch ---");
        if (!qacceptn) begin $display("FAIL qch reset up"); err_cnt = err_cnt + 1; end
        if (qactive) begin $display("FAIL qch idle qactive"); err_cnt = err_cnt + 1; end
        qreqn = 0;
        @(posedge clk); @(negedge clk);
        if (qacceptn || !lbus_pwrdn) begin $display("FAIL qch idle down"); err_cnt = err_cnt + 1; end
        qreqn = 1;
        @(posedge clk); @(negedge clk);
        busy = 1;
        @(negedge clk);
        if (!qactive) begin $display("FAIL qch busy qactive"); err_cnt = err_cnt + 1; end
        qdeny_en = 1; qreqn = 0;
        @(posedge clk); @(negedge clk);
        if (!qdeny || !qacceptn) begin $display("FAIL qch deny"); err_cnt = err_cnt + 1; end
        qreqn = 1;
        @(posedge clk); @(negedge clk);
        qdeny_en = 0; busy = 1; qreqn = 0;
        @(posedge clk); @(negedge clk);
        if (!quiesce) begin $display("FAIL qch quiesce"); err_cnt = err_cnt + 1; end
        if (!qactive) begin $display("FAIL qch quiesce qactive"); err_cnt = err_cnt + 1; end
        busy = 0;
        @(posedge clk); @(negedge clk);
        if (qacceptn || !lbus_pwrdn) begin $display("FAIL qch quiesce down"); err_cnt = err_cnt + 1; end
        qreqn = 1;
        @(posedge clk); @(negedge clk);
        err_en = 1; busy = 1; qreqn = 0;
        @(posedge clk); @(negedge clk);
        if (!err_mode) begin $display("FAIL qch err_mode"); err_cnt = err_cnt + 1; end
        busy = 0; qreqn = 1; err_en = 0;
        @(posedge clk); @(negedge clk);

        $display("--- atomic ---");
        at_i = 5'b00000; #1;
        if (at_op !== `LB_OP_ATOMIC_STORE || at_need !== 1'b0) begin
            $display("FAIL atomic store"); err_cnt = err_cnt + 1;
        end
        at_i = 5'b00001; #1;
        if (at_op !== `LB_OP_ATOMIC_LOAD || at_need !== 1'b1) begin
            $display("FAIL atomic load"); err_cnt = err_cnt + 1;
        end
        at_i = 5'b01010; #1;
        if (at_op !== `LB_OP_ATOMIC_SWAP || at_mod !== 3'b010) begin
            $display("FAIL atomic swap mod %b", at_mod); err_cnt = err_cnt + 1;
        end
        at_i = 5'b11111; #1;
        if (at_op !== `LB_OP_ATOMIC_COMPARE || at_need !== 1'b1) begin
            $display("FAIL atomic cmp"); err_cnt = err_cnt + 1;
        end

        $display("--- err_rsp ---");
        u_req_r_valid = 1;
        @(negedge clk);
        if (!d_req_r_valid) begin $display("FAIL err passthrough"); err_cnt = err_cnt + 1; end
        u_req_r_valid = 0;
        intercept = 1;
        repeat (4) @(posedge clk);
        @(negedge clk);
        if (!intercept_on) begin $display("FAIL err latch"); err_cnt = err_cnt + 1; end
        u_req_r_data = mk_cmd(`LB_OP_RD, 32'h10, 8'hAB);
        u_req_r_valid = 1;
        @(posedge clk);
        while (!u_req_r_ready) @(posedge clk);
        @(negedge clk);
        u_req_r_valid = 0;
        intercept = 0;
        if (!intercept_on) begin $display("FAIL intercept_on sticky"); err_cnt = err_cnt + 1; end
        to = 0;
        while (!u_rsp_rd_valid && to < 80) begin
            @(posedge clk);
            to = to + 1;
        end
        if (!u_rsp_rd_valid) begin
            $display("TIMEOUT err rsp");
            err_cnt = err_cnt + 1;
        end else if (u_rsp_rd_resp !== `LB_RESP_FAIL || u_rsp_rd_txnid !== 8'hAB) begin
            $display("FAIL err resp %h id %h", u_rsp_rd_resp, u_rsp_rd_txnid);
            err_cnt = err_cnt + 1;
        end
        @(negedge clk);
        repeat (4) @(posedge clk);
        if (intercept_on) begin $display("FAIL intercept_on after rsp"); err_cnt = err_cnt + 1; end

        $display("--- err_rsp write ---");
        intercept = 1;
        repeat (4) @(posedge clk);
        @(negedge clk);
        if (!intercept_on) begin $display("FAIL err latch wr"); err_cnt = err_cnt + 1; end
        u_req_w_data = mk_req_w(`LB_OP_WR, 32'h30, 8'hCD);
        u_req_w_valid = 1;
        @(posedge clk);
        while (!u_req_w_ready) @(posedge clk);
        @(negedge clk);
        u_req_w_valid = 0;
        to = 0;
        while (!u_rsp_wr_valid && to < 80) begin
            @(posedge clk);
            to = to + 1;
        end
        if (!u_rsp_wr_valid) begin
            $display("TIMEOUT err wr rsp"); err_cnt = err_cnt + 1;
        end else if (u_rsp_wr_resp !== `LB_RESP_FAIL || u_rsp_wr_txnid !== 8'hCD) begin
            $display("FAIL err wr resp %h id %h", u_rsp_wr_resp, u_rsp_wr_txnid);
            err_cnt = err_cnt + 1;
        end
        if (d_req_w_valid) begin $display("FAIL err wr leaked downstream"); err_cnt = err_cnt + 1; end
        @(negedge clk);
        intercept = 0;
        repeat (4) @(posedge clk);

        $display("--- ip_fail ---");
        f_u_r_data = mk_cmd(`LB_OP_RD, 32'h20, 8'h11);
        f_u_r_valid = 1;
        @(negedge clk);
        if (!f_d_r_valid) begin $display("FAIL fail passthrough"); err_cnt = err_cnt + 1; end
        f_u_r_valid = 0;
        f_intercept = 1;
        @(negedge clk);
        f_u_r_valid = 1;
        @(posedge clk);
        while (!f_u_r_ready) @(posedge clk);
        @(negedge clk);
        f_u_r_valid = 0;
        to = 0;
        while (!f_rd_valid && to < 80) begin
            @(posedge clk);
            to = to + 1;
        end
        if (!f_rd_valid) begin
            $display("TIMEOUT ip_fail");
            err_cnt = err_cnt + 1;
        end else if (f_rd_resp !== `LB_RESP_FAIL || f_rd_txn !== 8'h11 || !f_rd_last) begin
            $display("FAIL ip_fail resp");
            err_cnt = err_cnt + 1;
        end
        @(negedge clk);
        f_intercept = 0;
        repeat (2) @(posedge clk);

        $display("--- ip_fail write ---");
        f_intercept = 1;
        @(negedge clk);
        f_u_w_data = mk_req_w(`LB_OP_WR, 32'h28, 8'h22);
        f_u_w_valid = 1;
        @(posedge clk);
        while (!f_u_w_ready) @(posedge clk);
        @(negedge clk);
        f_u_w_valid = 0;
        to = 0;
        while (!f_wr_valid && to < 80) begin
            @(posedge clk);
            to = to + 1;
        end
        if (!f_wr_valid) begin
            $display("TIMEOUT ip_fail wr"); err_cnt = err_cnt + 1;
        end else if (f_wr_resp !== `LB_RESP_FAIL) begin
            $display("FAIL ip_fail wr resp %h", f_wr_resp); err_cnt = err_cnt + 1;
        end
        if (f_d_w_valid) begin $display("FAIL ip_fail wr leaked"); err_cnt = err_cnt + 1; end

        if (err_cnt == 0) $display("=== ALL TESTS PASSED ===");
        else $display("=== %0d TEST(S) FAILED ===", err_cnt);
        $finish;
    end

    initial begin
        #200000;
        $display("TIMEOUT tb_adapter_simple_unit");
        $finish;
    end
endmodule
