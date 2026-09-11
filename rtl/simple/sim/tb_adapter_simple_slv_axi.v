//============================================================================
// Filename    : tb_adapter_simple_slv_axi.v
// Description : simple slv AXI4/AXI5 vs axi_slave_mem; QCH; DOWN FAIL;
//               ERR_ADDR; AXI5 STORE/LOAD.
//============================================================================
`include "adapter_defs.vh"
`default_nettype none
`timescale 1ns/1ps

module tb_adapter_simple_slv_axi;
    parameter ADDR_W = 32;
    parameter DATA_W = 64;
    parameter LEN_W  = 8;
    parameter ID_W   = 8;
    parameter USER_CMD_PW = 1;
    parameter USER_RSP_PW = 1;
    parameter QOS_PW = 1;
    parameter MOD_PW = 1;
    parameter W_BYTES = 8;
    parameter EXT_CMD_W = QOS_PW + 4 + ADDR_W + LEN_W + ID_W + USER_CMD_PW;
    parameter EXT_WD_W  = DATA_W + W_BYTES + 1 + ID_W;
    parameter EXT_REQ_W = EXT_CMD_W + MOD_PW + EXT_WD_W;
    parameter MOD_PW5 = 3;
    parameter EXT_REQ_W5 = EXT_CMD_W + MOD_PW5 + EXT_WD_W;

    reg clk, rst_n;
    integer err_cnt, to;
    integer ar_seen;

    reg [EXT_CMD_W-1:0] req_r_data;
    reg req_r_valid;
    wire req_r_ready;
    reg [EXT_REQ_W-1:0] req_w_data;
    reg req_w_valid;
    wire req_w_ready;
    wire [DATA_W-1:0] rsp_rd_data;
    wire rsp_rd_last;
    wire [1:0] rsp_rd_resp;
    wire [ID_W-1:0] rsp_rd_txn;
    wire [USER_RSP_PW-1:0] rsp_rd_user;
    wire rsp_rd_valid;
    reg rsp_rd_ready;
    wire [1:0] rsp_wr_resp;
    wire [ID_W-1:0] rsp_wr_txn;
    wire [USER_RSP_PW-1:0] rsp_wr_user;
    wire rsp_wr_valid;
    reg rsp_wr_ready;

    wire [ID_W-1:0] arid, awid, rid, bid;
    wire [ADDR_W-1:0] araddr, awaddr;
    wire [LEN_W-1:0] arlen, awlen;
    wire [2:0] arsize, awsize;
    wire [1:0] arburst, awburst, rresp, bresp;
    wire arvalid, arready, rvalid, rlast, rready;
    wire awvalid, awready, wvalid, wready, wlast, bvalid, bready;
    wire [DATA_W-1:0] wdata, rdata;
    wire [W_BYTES-1:0] wstrb;
    reg qreqn_slv;
    wire qacceptn_slv, lbus_pwrdn_slv, intercept_slv;

    function [EXT_CMD_W-1:0] mk_cmd;
        input [3:0] op;
        input [31:0] addr;
        input [7:0] len;
        input [7:0] txn;
        begin
            mk_cmd = { {QOS_PW{1'b0}}, op, addr, len, txn, {USER_CMD_PW{1'b0}} };
        end
    endfunction

    function [EXT_WD_W-1:0] mk_wd;
        input [63:0] data;
        input [7:0]  strb;
        input        last;
        input [7:0]  txn;
        begin
            mk_wd = {data, strb, last, txn};
        end
    endfunction

    adapter_slv_axi4 #(
        .MAX_RD_OST(4), .MAX_WR_OST(4)
    ) u_slv (
        .clk(clk), .rst_n(rst_n),
        .req_r_data(req_r_data), .req_r_valid(req_r_valid), .req_r_ready(req_r_ready),
        .req_w_data(req_w_data), .req_w_valid(req_w_valid), .req_w_ready(req_w_ready),
        .rsp_rd_data(rsp_rd_data), .rsp_rd_last(rsp_rd_last), .rsp_rd_resp(rsp_rd_resp),
        .rsp_rd_ext_txnid(rsp_rd_txn), .rsp_rd_user(rsp_rd_user),
        .rsp_rd_valid(rsp_rd_valid), .rsp_rd_ready(rsp_rd_ready),
        .rsp_wr_resp(rsp_wr_resp), .rsp_wr_ext_txnid(rsp_wr_txn),
        .rsp_wr_user(rsp_wr_user), .rsp_wr_valid(rsp_wr_valid),
        .rsp_wr_ready(rsp_wr_ready),
        .arid(arid), .araddr(araddr), .arlen(arlen), .arsize(arsize),
        .arburst(arburst), .arvalid(arvalid), .arready(arready),
        .rid(rid), .rdata(rdata), .rresp(rresp), .rlast(rlast),
        .rvalid(rvalid), .rready(rready),
        .awid(awid), .awaddr(awaddr), .awlen(awlen), .awsize(awsize),
        .awburst(awburst), .awvalid(awvalid), .awready(awready),
        .wdata(wdata), .wstrb(wstrb), .wlast(wlast), .wvalid(wvalid),
        .wready(wready),
        .bid(bid), .bresp(bresp), .bvalid(bvalid), .bready(bready),
        .qreqn(qreqn_slv), .qacceptn(qacceptn_slv), .qdeny(), .qactive(),
        .lbus_pwrdn(lbus_pwrdn_slv), .intercept(intercept_slv)
    );

    axi_slave_mem #(
        .ADDR_W(ADDR_W), .DATA_W(DATA_W), .LEN_W(LEN_W), .ID_W(ID_W),
        .MEM_BYTES(16384)
    ) u_axi (
        .clk(clk), .rst_n(rst_n),
        .awid(awid), .awaddr(awaddr), .awlen(awlen), .awsize(awsize),
        .awburst(awburst), .awvalid(awvalid), .awready(awready),
        .wdata(wdata), .wstrb(wstrb), .wlast(wlast), .wvalid(wvalid),
        .wready(wready),
        .bid(bid), .bresp(bresp), .bvalid(bvalid), .bready(bready),
        .arid(arid), .araddr(araddr), .arlen(arlen), .arsize(arsize),
        .arburst(arburst), .arvalid(arvalid), .arready(arready),
        .rid(rid), .rdata(rdata), .rresp(rresp), .rlast(rlast),
        .rvalid(rvalid), .rready(rready)
    );

    wire [4:0] awatop5;
    reg [EXT_REQ_W5-1:0] req_w5;
    reg req_w5_valid;
    wire req_w5_ready, rsp_wr5_valid, rsp_rd5_valid;
    wire [1:0] rsp_wr5_resp, rsp_rd5_resp;
    wire [DATA_W-1:0] rsp_rd5_data;
    wire awvalid5, awready5, wvalid5, wready5, wlast5, bvalid5, bready5;
    wire arvalid5, arready5, rvalid5, rready5, rlast5;
    wire [ID_W-1:0] awid5, bid5, arid5, rid5;
    wire [ADDR_W-1:0] awaddr5, araddr5;
    wire [LEN_W-1:0] awlen5, arlen5;
    wire [2:0] awsize5, arsize5;
    wire [1:0] awburst5, arburst5, bresp5, rresp5;
    wire [DATA_W-1:0] wdata5, rdata5;
    wire [W_BYTES-1:0] wstrb5;

    adapter_slv_axi5 #(
        .MAX_RD_OST(4), .MAX_WR_OST(4)
    ) u_slv5 (
        .clk(clk), .rst_n(rst_n),
        .req_r_data({EXT_CMD_W{1'b0}}), .req_r_valid(1'b0), .req_r_ready(),
        .req_w_data(req_w5), .req_w_valid(req_w5_valid), .req_w_ready(req_w5_ready),
        .rsp_rd_data(rsp_rd5_data), .rsp_rd_last(), .rsp_rd_resp(rsp_rd5_resp),
        .rsp_rd_ext_txnid(), .rsp_rd_user(), .rsp_rd_valid(rsp_rd5_valid),
        .rsp_rd_ready(1'b1),
        .rsp_wr_resp(rsp_wr5_resp), .rsp_wr_ext_txnid(), .rsp_wr_user(),
        .rsp_wr_valid(rsp_wr5_valid), .rsp_wr_ready(1'b1),
        .arid(arid5), .araddr(araddr5), .arlen(arlen5), .arsize(arsize5),
        .arburst(arburst5), .arvalid(arvalid5), .arready(arready5),
        .rid(rid5), .rdata(rdata5), .rresp(rresp5), .rlast(rlast5),
        .rvalid(rvalid5), .rready(rready5),
        .awid(awid5), .awaddr(awaddr5), .awlen(awlen5), .awsize(awsize5),
        .awburst(awburst5), .awvalid(awvalid5), .awready(awready5),
        .wdata(wdata5), .wstrb(wstrb5), .wlast(wlast5), .wvalid(wvalid5),
        .wready(wready5),
        .bid(bid5), .bresp(bresp5), .bvalid(bvalid5), .bready(bready5),
        .awatop(awatop5),
        .qreqn(1'b1), .qacceptn(), .qdeny(), .qactive(), .lbus_pwrdn(), .intercept()
    );

    axi_slave_mem #(
        .ADDR_W(ADDR_W), .DATA_W(DATA_W), .LEN_W(LEN_W), .ID_W(ID_W),
        .MEM_BYTES(16384)
    ) u_axi5 (
        .clk(clk), .rst_n(rst_n),
        .awid(awid5), .awaddr(awaddr5), .awlen(awlen5), .awsize(awsize5),
        .awburst(awburst5), .awvalid(awvalid5), .awready(awready5),
        .wdata(wdata5), .wstrb(wstrb5), .wlast(wlast5), .wvalid(wvalid5),
        .wready(wready5),
        .bid(bid5), .bresp(bresp5), .bvalid(bvalid5), .bready(bready5),
        .arid(arid5), .araddr(araddr5), .arlen(arlen5), .arsize(arsize5),
        .arburst(arburst5), .arvalid(arvalid5), .arready(arready5),
        .rid(rid5), .rdata(rdata5), .rresp(rresp5), .rlast(rlast5),
        .rvalid(rvalid5), .rready(rready5)
    );

    initial clk = 0;
    always #5 clk = ~clk;

    initial begin
        err_cnt = 0;
        rst_n = 0;
        req_r_valid = 0; req_w_valid = 0; req_r_data = 0; req_w_data = 0;
        rsp_rd_ready = 1; rsp_wr_ready = 1; req_w5_valid = 0; req_w5 = 0;
        qreqn_slv = 1;
        #30 rst_n = 1;
        @(negedge clk);

        $display("--- slv axi4 write/read ---");
        req_w_data = { mk_cmd(`LB_OP_WR, 32'h100, 8'd0, 8'h21),
                       {MOD_PW{1'b0}}, mk_wd(64'hDEADBEEFCAFEBABE, 8'hFF, 1'b1, 8'h21) };
        req_w_valid = 1;
        @(posedge clk);
        to = 0;
        while (!req_w_ready && to < 400) begin @(posedge clk); to = to + 1; end
        if (!req_w_ready) begin $display("TIMEOUT slv W"); err_cnt = err_cnt + 1; end
        @(negedge clk);
        req_w_valid = 0;
        to = 0;
        while (!rsp_wr_valid && to < 400) begin @(posedge clk); to = to + 1; end
        if (!rsp_wr_valid || rsp_wr_resp !== `LB_RESP_OK) begin
            $display("FAIL slv wr resp"); err_cnt = err_cnt + 1;
        end
        @(negedge clk);

        req_r_data = mk_cmd(`LB_OP_RD, 32'h100, 8'd0, 8'h21);
        req_r_valid = 1;
        @(posedge clk);
        to = 0;
        while (!req_r_ready && to < 400) begin @(posedge clk); to = to + 1; end
        @(negedge clk);
        req_r_valid = 0;
        to = 0;
        while (!rsp_rd_valid && to < 400) begin @(posedge clk); to = to + 1; end
        if (rsp_rd_data !== 64'hDEADBEEFCAFEBABE) begin
            $display("FAIL slv rd %h", rsp_rd_data); err_cnt = err_cnt + 1;
        end
        @(negedge clk);

        $display("--- slv axi4 multi outstanding wr ---");
        rsp_wr_ready = 0;
        req_w_data = { mk_cmd(`LB_OP_WR, 32'h200, 8'd0, 8'h51),
                       {MOD_PW{1'b0}}, mk_wd(64'hA1, 8'hFF, 1'b1, 8'h51) };
        req_w_valid = 1;
        @(posedge clk);
        to = 0;
        while (!req_w_ready && to < 400) begin @(posedge clk); to = to + 1; end
        @(negedge clk);
        req_w_valid = 0;
        req_w_data = { mk_cmd(`LB_OP_WR, 32'h208, 8'd0, 8'h52),
                       {MOD_PW{1'b0}}, mk_wd(64'hA2, 8'hFF, 1'b1, 8'h52) };
        req_w_valid = 1;
        @(posedge clk);
        to = 0;
        while (!req_w_ready && to < 400) begin @(posedge clk); to = to + 1; end
        if (!req_w_ready) begin $display("TIMEOUT slv ost W1"); err_cnt = err_cnt + 1; end
        @(negedge clk);
        req_w_valid = 0;
        rsp_wr_ready = 1;
        to = 0;
        while (!rsp_wr_valid && to < 400) begin @(posedge clk); to = to + 1; end
        @(negedge clk);
        to = 0;
        while (!rsp_wr_valid && to < 400) begin @(posedge clk); to = to + 1; end
        @(negedge clk);

        $display("--- slv QCH waits for AXI ost ---");
        rsp_wr_ready = 0;
        req_w_data = { mk_cmd(`LB_OP_WR, 32'h210, 8'd0, 8'h61),
                       {MOD_PW{1'b0}}, mk_wd(64'hB1, 8'hFF, 1'b1, 8'h61) };
        req_w_valid = 1;
        @(posedge clk);
        to = 0;
        while (!req_w_ready && to < 400) begin @(posedge clk); to = to + 1; end
        @(negedge clk);
        req_w_valid = 0;
        qreqn_slv = 0;
        repeat (8) @(posedge clk);
        if (lbus_pwrdn_slv) begin $display("FAIL slv pwrdn while ost"); err_cnt = err_cnt + 1; end
        if (!qacceptn_slv) begin $display("FAIL slv qacceptn while ost"); err_cnt = err_cnt + 1; end
        rsp_wr_ready = 1;
        to = 0;
        while (!rsp_wr_valid && to < 400) begin @(posedge clk); to = to + 1; end
        @(negedge clk);
        to = 0;
        while (!lbus_pwrdn_slv && to < 40) begin @(posedge clk); to = to + 1; end
        if (!lbus_pwrdn_slv) begin $display("FAIL slv no pwrdn after ost"); err_cnt = err_cnt + 1; end
        if (intercept_slv !== lbus_pwrdn_slv) begin
            $display("FAIL intercept != lbus_pwrdn"); err_cnt = err_cnt + 1;
        end

        $display("--- slv DOWN then REQ_R FAIL, no AR ---");
        ar_seen = 0;
        req_r_data = mk_cmd(`LB_OP_RD, 32'h100, 8'd0, 8'h77);
        req_r_valid = 1;
        @(posedge clk);
        to = 0;
        while (!req_r_ready && to < 80) begin
            if (arvalid) ar_seen = 1;
            @(posedge clk); to = to + 1;
        end
        @(negedge clk);
        req_r_valid = 0;
        to = 0;
        while (!rsp_rd_valid && to < 80) begin
            if (arvalid) ar_seen = 1;
            @(posedge clk); to = to + 1;
        end
        if (!rsp_rd_valid || rsp_rd_resp !== `LB_RESP_FAIL) begin
            $display("FAIL slv down rd resp %h", rsp_rd_resp); err_cnt = err_cnt + 1;
        end
        if (ar_seen) begin $display("FAIL slv AR while DOWN"); err_cnt = err_cnt + 1; end
        qreqn_slv = 1;
        repeat (4) @(posedge clk);

        $display("--- slv ERR_ADDR FAIL ---");
        req_w_data = { mk_cmd(`LB_OP_WR, 32'h4000_0000, 8'd0, 8'hE1),
                       {MOD_PW{1'b0}}, mk_wd(64'h1, 8'hFF, 1'b1, 8'hE1) };
        req_w_valid = 1;
        @(posedge clk);
        to = 0;
        while (!req_w_ready && to < 400) begin @(posedge clk); to = to + 1; end
        @(negedge clk);
        req_w_valid = 0;
        to = 0;
        while (!rsp_wr_valid && to < 400) begin @(posedge clk); to = to + 1; end
        if (rsp_wr_resp !== `LB_RESP_FAIL) begin
            $display("FAIL slv ERR_ADDR resp %h", rsp_wr_resp); err_cnt = err_cnt + 1;
        end
        @(negedge clk);

        $display("--- slv axi5 awatop STORE ---");
        req_w5 = { mk_cmd(`LB_OP_ATOMIC_STORE, 32'h40, 8'd0, 8'h09),
                   3'b101, mk_wd(64'h55, 8'hFF, 1'b1, 8'h09) };
        req_w5_valid = 1;
        @(posedge clk);
        to = 0;
        while (!req_w5_ready && to < 400) begin @(posedge clk); to = to + 1; end
        if (awatop5 !== 5'b10100) begin
            $display("FAIL awatop got %b exp 10100", awatop5);
            err_cnt = err_cnt + 1;
        end
        @(negedge clk);
        req_w5_valid = 0;
        to = 0;
        while (!rsp_wr5_valid && to < 400) begin @(posedge clk); to = to + 1; end
        if (!rsp_wr5_valid) begin $display("FAIL axi5 wr rsp"); err_cnt = err_cnt + 1; end

        if (err_cnt == 0) $display("=== ALL TESTS PASSED ===");
        else $display("=== %0d TEST(S) FAILED ===", err_cnt);
        $finish;
    end

    initial begin
        #400000;
        $display("TIMEOUT tb_adapter_simple_slv_axi");
        $finish;
    end
endmodule
