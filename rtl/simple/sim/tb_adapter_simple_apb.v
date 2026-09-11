//============================================================================
// Filename    : tb_adapter_simple_apb.v
// Description : simple mst_apb vs slv_model; slv_apb vs APB memory.
//               SETUP capture, ERR_ADDR PSLVERR, slv DOWN FAIL.
//============================================================================
`include "adapter_defs.vh"
`default_nettype none
`timescale 1ns/1ps

module tb_adapter_simple_apb;
    parameter ADDR_W = 32;
    parameter DATA_W = 32;
    parameter LEN_W  = 8;
    parameter ID_W   = 1;
    parameter QOS_PW = 1;
    parameter USER_CMD_PW = 1;
    parameter USER_RSP_PW = 1;
    parameter MOD_PW = 1;
    parameter EXT_CMD_W = QOS_PW + 4 + ADDR_W + LEN_W + ID_W + USER_CMD_PW;
    parameter EXT_WD_W  = DATA_W + DATA_W/8 + 1 + ID_W;
    parameter EXT_REQ_W = EXT_CMD_W + MOD_PW + EXT_WD_W;

    reg clk, rst_n;
    integer err_cnt, to;
    integer slv_psel_seen;

    reg psel, penable, pwrite;
    reg [ADDR_W-1:0] paddr;
    reg [DATA_W-1:0] pwdata;
    reg [3:0] pstrb;
    wire [DATA_W-1:0] prdata;
    wire pready, pslverr;
    wire qacceptn, lbus_pwrdn;
    reg qreqn, err_en;

    wire [EXT_CMD_W-1:0] req_r_data;
    wire req_r_valid, req_r_ready, req_w_valid, req_w_ready;
    wire [EXT_REQ_W-1:0] req_w_data;
    wire [DATA_W-1:0] rsp_rd_data;
    wire rsp_rd_last, rsp_rd_valid, rsp_rd_ready, rsp_wr_valid, rsp_wr_ready;
    wire [1:0] rsp_rd_resp, rsp_wr_resp;
    wire [ID_W-1:0] rsp_rd_txn, rsp_wr_txn;

    adapter_mst_apb u_mst (
        .clk(clk), .rst_n(rst_n),
        .psel(psel), .penable(penable), .pwrite(pwrite),
        .paddr(paddr), .pwdata(pwdata), .pstrb(pstrb),
        .prdata(prdata), .pready(pready), .pslverr(pslverr),
        .req_r_data(req_r_data), .req_r_valid(req_r_valid), .req_r_ready(req_r_ready),
        .req_w_data(req_w_data), .req_w_valid(req_w_valid), .req_w_ready(req_w_ready),
        .rsp_rd_data(rsp_rd_data), .rsp_rd_last(rsp_rd_last), .rsp_rd_resp(rsp_rd_resp),
        .rsp_rd_ext_txnid(rsp_rd_txn), .rsp_rd_valid(rsp_rd_valid),
        .rsp_rd_ready(rsp_rd_ready),
        .rsp_wr_resp(rsp_wr_resp), .rsp_wr_ext_txnid(rsp_wr_txn),
        .rsp_wr_valid(rsp_wr_valid), .rsp_wr_ready(rsp_wr_ready),
        .qreqn(qreqn), .reg_qdeny_en(1'b0), .reg_err_en(err_en),
        .qacceptn(qacceptn), .qdeny(), .qactive(), .lbus_pwrdn(lbus_pwrdn)
    );

    adapter_slv_model #(
        .ADDR_W(ADDR_W), .DATA_W(DATA_W), .LEN_W(LEN_W), .ID_W(ID_W),
        .MEM_BYTES(65536), .ERR_ADDR(32'h4000_0000), .MAX_BEATS(8)
    ) u_model (
        .clk(clk), .rst_n(rst_n),
        .req_r_data(req_r_data), .req_r_valid(req_r_valid), .req_r_ready(req_r_ready),
        .req_w_data(req_w_data), .req_w_valid(req_w_valid), .req_w_ready(req_w_ready),
        .rsp_rd_data(rsp_rd_data), .rsp_rd_last(rsp_rd_last), .rsp_rd_resp(rsp_rd_resp),
        .rsp_rd_ext_txnid(rsp_rd_txn), .rsp_rd_valid(rsp_rd_valid),
        .rsp_rd_ready(rsp_rd_ready),
        .rsp_wr_resp(rsp_wr_resp), .rsp_wr_ext_txnid(rsp_wr_txn),
        .rsp_wr_valid(rsp_wr_valid), .rsp_wr_ready(rsp_wr_ready)
    );

    task apb_write;
        input [31:0] addr;
        input [31:0] data;
        begin
            @(negedge clk);
            psel = 1; penable = 0; pwrite = 1; paddr = addr; pwdata = data; pstrb = 4'hF;
            @(negedge clk);
            penable = 1;
            to = 0;
            while (!pready && to < 500) begin @(negedge clk); to = to + 1; end
            if (!pready) begin $display("TIMEOUT APB wr"); err_cnt = err_cnt + 1; end
            @(negedge clk);
            psel = 0; penable = 0;
        end
    endtask

    task apb_read_check;
        input [31:0] addr;
        input [31:0] exp;
        begin
            @(negedge clk);
            psel = 1; penable = 0; pwrite = 0; paddr = addr;
            @(negedge clk);
            penable = 1;
            to = 0;
            while (!pready && to < 500) begin @(negedge clk); to = to + 1; end
            if (prdata !== exp) begin
                $display("FAIL PRDATA %h exp %h", prdata, exp); err_cnt = err_cnt + 1;
            end
            if (pslverr) begin $display("FAIL unexpected PSLVERR @%h", addr); err_cnt = err_cnt + 1; end
            @(negedge clk);
            psel = 0; penable = 0;
        end
    endtask

    // slv APB
    reg [EXT_CMD_W-1:0] s_req_r;
    reg s_req_r_v;
    wire s_req_r_r;
    reg [EXT_REQ_W-1:0] s_req_w;
    reg s_req_w_v;
    wire s_req_w_r;
    wire [DATA_W-1:0] s_rd_data;
    wire s_rd_last, s_rd_v, s_wr_v;
    wire [1:0] s_rd_resp, s_wr_resp;
    wire s_psel, s_pen, s_pwrite;
    wire [ADDR_W-1:0] s_paddr;
    wire [DATA_W-1:0] s_pwdata, s_prdata;
    wire [3:0] s_pstrb;
    reg [DATA_W-1:0] smem;
    reg s_qreqn;
    wire s_pwrdn;

    assign s_prdata = smem;

    always @(posedge clk)
        if (s_psel && s_pen && s_pwrite)
            smem <= s_pwdata;

    function [EXT_CMD_W-1:0] mk_cmd;
        input [3:0] op;
        input [31:0] addr;
        begin
            mk_cmd = { {QOS_PW{1'b0}}, op, addr, {LEN_W{1'b0}}, {ID_W{1'b0}}, {USER_CMD_PW{1'b0}} };
        end
    endfunction

    adapter_slv_apb u_slv (
        .clk(clk), .rst_n(rst_n),
        .req_r_data(s_req_r), .req_r_valid(s_req_r_v), .req_r_ready(s_req_r_r),
        .req_w_data(s_req_w), .req_w_valid(s_req_w_v), .req_w_ready(s_req_w_r),
        .rsp_rd_data(s_rd_data), .rsp_rd_last(s_rd_last), .rsp_rd_resp(s_rd_resp),
        .rsp_rd_ext_txnid(), .rsp_rd_user(),
        .rsp_rd_valid(s_rd_v), .rsp_rd_ready(1'b1),
        .rsp_wr_resp(s_wr_resp), .rsp_wr_ext_txnid(), .rsp_wr_user(),
        .rsp_wr_valid(s_wr_v), .rsp_wr_ready(1'b1),
        .psel(s_psel), .penable(s_pen), .pwrite(s_pwrite),
        .paddr(s_paddr), .pwdata(s_pwdata), .pstrb(s_pstrb),
        .prdata(s_prdata), .pready(1'b1), .pslverr(1'b0),
        .qreqn(s_qreqn), .qacceptn(), .qdeny(), .qactive(), .lbus_pwrdn(s_pwrdn), .intercept()
    );

    initial clk = 0;
    always #5 clk = ~clk;

    initial begin
        err_cnt = 0;
        rst_n = 0;
        psel = 0; penable = 0; pwrite = 0; paddr = 0; pwdata = 0; pstrb = 0;
        qreqn = 1; err_en = 0;
        s_req_r_v = 0; s_req_w_v = 0; s_req_r = 0; s_req_w = 0; smem = 0;
        s_qreqn = 1;
        #30 rst_n = 1;
        @(negedge clk);

        $display("--- mst apb wr/rd ---");
        apb_write(32'h100, 32'hDEADBEEF);
        apb_read_check(32'h100, 32'hDEADBEEF);
        apb_write(32'h104, 32'hCAFE1234);
        apb_read_check(32'h104, 32'hCAFE1234);

        $display("--- mst apb SETUP capture (two addrs) ---");
        apb_write(32'h10, 32'h11111111);
        apb_write(32'h14, 32'h22222222);
        apb_read_check(32'h10, 32'h11111111);
        apb_read_check(32'h14, 32'h22222222);

        $display("--- mst apb ERR_ADDR PSLVERR ---");
        @(negedge clk);
        psel = 1; penable = 0; pwrite = 1; paddr = 32'h4000_0000; pwdata = 32'h1; pstrb = 4'hF;
        @(negedge clk);
        penable = 1;
        to = 0;
        while (!pready && to < 500) begin @(negedge clk); to = to + 1; end
        if (!pready) begin $display("TIMEOUT APB ERR_ADDR"); err_cnt = err_cnt + 1; end
        if (!pslverr) begin $display("FAIL ERR_ADDR no PSLVERR"); err_cnt = err_cnt + 1; end
        @(negedge clk);
        psel = 0; penable = 0;

        $display("--- mst apb QCH DOWN ---");
        @(negedge clk);
        psel = 1; penable = 0; pwrite = 1; paddr = 32'h200; pwdata = 32'h1;
        @(negedge clk);
        penable = 1;
        to = 0;
        while (!pready && to < 20) begin @(negedge clk); to = to + 1; end
        while (!pready && to < 500) begin @(negedge clk); to = to + 1; end
        @(negedge clk);
        psel = 0; penable = 0;
        repeat (4) @(posedge clk);
        qreqn = 0; err_en = 0;
        @(posedge clk); @(negedge clk);
        if (!lbus_pwrdn) begin $display("FAIL apb lbus_pwrdn"); err_cnt = err_cnt + 1; end
        psel = 1; penable = 0; pwrite = 0; paddr = 32'h100;
        @(negedge clk);
        penable = 1;
        repeat (5) @(negedge clk);
        if (pready) begin $display("FAIL down should not PREADY"); err_cnt = err_cnt + 1; end
        psel = 0; penable = 0; qreqn = 1;
        repeat (3) @(posedge clk);

        $display("--- slv apb wr/rd ---");
        s_req_w = { mk_cmd(`LB_OP_WR, 32'h20), {MOD_PW{1'b0}},
                    32'hA5A5A5A5, 4'hF, 1'b1, {ID_W{1'b0}} };
        s_req_w_v = 1;
        @(posedge clk);
        to = 0;
        while (!s_req_w_r && to < 200) begin @(posedge clk); to = to + 1; end
        @(negedge clk);
        s_req_w_v = 0;
        to = 0;
        while (!s_wr_v && to < 200) begin @(posedge clk); to = to + 1; end
        if (!s_wr_v || s_wr_resp !== `LB_RESP_OK) begin $display("FAIL slv apb wr"); err_cnt = err_cnt + 1; end
        @(negedge clk);
        s_req_r = mk_cmd(`LB_OP_RD, 32'h20);
        s_req_r_v = 1;
        @(posedge clk);
        to = 0;
        while (!s_req_r_r && to < 200) begin @(posedge clk); to = to + 1; end
        @(negedge clk);
        s_req_r_v = 0;
        to = 0;
        while (!s_rd_v && to < 200) begin @(posedge clk); to = to + 1; end
        if (s_rd_data !== 32'hA5A5A5A5) begin $display("FAIL slv apb rd %h", s_rd_data); err_cnt = err_cnt + 1; end
        @(negedge clk);

        $display("--- slv apb DOWN then REQ FAIL, no PSEL ---");
        s_qreqn = 0;
        @(posedge clk); @(negedge clk);
        if (!s_pwrdn) begin $display("FAIL slv apb pwrdn"); err_cnt = err_cnt + 1; end
        slv_psel_seen = 0;
        s_req_r = mk_cmd(`LB_OP_RD, 32'h20);
        s_req_r_v = 1;
        @(posedge clk);
        to = 0;
        while (!s_req_r_r && to < 80) begin
            if (s_psel) slv_psel_seen = 1;
            @(posedge clk); to = to + 1;
        end
        @(negedge clk);
        s_req_r_v = 0;
        to = 0;
        while (!s_rd_v && to < 80) begin
            if (s_psel) slv_psel_seen = 1;
            @(posedge clk); to = to + 1;
        end
        if (!s_rd_v || s_rd_resp !== `LB_RESP_FAIL) begin
            $display("FAIL slv apb down resp"); err_cnt = err_cnt + 1;
        end
        if (slv_psel_seen) begin $display("FAIL slv apb PSEL while DOWN"); err_cnt = err_cnt + 1; end
        s_qreqn = 1;

        if (err_cnt == 0) $display("=== ALL TESTS PASSED ===");
        else $display("=== %0d TEST(S) FAILED ===", err_cnt);
        $finish;
    end

    initial begin
        #400000;
        $display("TIMEOUT tb_adapter_simple_apb");
        $finish;
    end
endmodule
