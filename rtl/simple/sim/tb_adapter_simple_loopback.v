//============================================================================
// Filename    : tb_adapter_simple_loopback.v
// Description : mst_axi4 <-> slv_axi4 <-> axi_slave_mem (1-beat, 2-beat, ost)
//============================================================================
`include "adapter_defs.vh"
`default_nettype none
`timescale 1ns/1ps

module tb_adapter_simple_loopback;
    parameter ADDR_W = 32;
    parameter DATA_W = 64;
    parameter LEN_W  = 8;
    parameter ID_W   = 8;
    parameter W_BYTES = 8;
    parameter EXT_CMD_W = 1 + 4 + ADDR_W + LEN_W + ID_W + 1;
    parameter EXT_WD_W  = DATA_W + W_BYTES + 1 + ID_W;
    parameter EXT_REQ_W = EXT_CMD_W + 1 + EXT_WD_W;

    reg clk, rst_n;
    integer err_cnt, to;

    reg [ID_W-1:0] awid, arid;
    reg [ADDR_W-1:0] awaddr, araddr;
    reg [LEN_W-1:0] awlen, arlen;
    reg awvalid, wvalid, wlast, arvalid, bready, rready;
    reg [DATA_W-1:0] wdata;
    reg [W_BYTES-1:0] wstrb;
    wire awready, wready, bvalid, arready, rvalid, rlast;
    wire [ID_W-1:0] bid, rid;
    wire [1:0] bresp, rresp;
    wire [DATA_W-1:0] rdata;

    wire [EXT_CMD_W-1:0] req_r_data;
    wire req_r_valid, req_r_ready, req_w_valid, req_w_ready;
    wire [EXT_REQ_W-1:0] req_w_data;
    wire [DATA_W-1:0] rsp_rd_data;
    wire rsp_rd_last, rsp_rd_valid, rsp_rd_ready;
    wire [1:0] rsp_rd_resp, rsp_wr_resp;
    wire [ID_W-1:0] rsp_rd_txn, rsp_wr_txn;
    wire rsp_wr_valid, rsp_wr_ready;

    adapter_mst_axi4 #(
        .MAX_RD_OST(4), .MAX_WR_OST(4)
    ) u_mst (
        .clk(clk), .rst_n(rst_n),
        .awid(awid), .awaddr(awaddr), .awlen(awlen), .awsize(3'd3),
        .awburst(2'b01), .awvalid(awvalid), .awready(awready),
        .wdata(wdata), .wstrb(wstrb), .wlast(wlast), .wvalid(wvalid),
        .wready(wready),
        .bid(bid), .bresp(bresp), .bvalid(bvalid), .bready(bready),
        .arid(arid), .araddr(araddr), .arlen(arlen), .arsize(3'd3),
        .arburst(2'b01), .arvalid(arvalid), .arready(arready),
        .rid(rid), .rdata(rdata), .rresp(rresp), .rlast(rlast),
        .rvalid(rvalid), .rready(rready),
        .req_r_data(req_r_data), .req_r_valid(req_r_valid), .req_r_ready(req_r_ready),
        .req_w_data(req_w_data), .req_w_valid(req_w_valid), .req_w_ready(req_w_ready),
        .rsp_rd_data(rsp_rd_data), .rsp_rd_last(rsp_rd_last), .rsp_rd_resp(rsp_rd_resp),
        .rsp_rd_ext_txnid(rsp_rd_txn), .rsp_rd_valid(rsp_rd_valid),
        .rsp_rd_ready(rsp_rd_ready),
        .rsp_wr_resp(rsp_wr_resp), .rsp_wr_ext_txnid(rsp_wr_txn),
        .rsp_wr_valid(rsp_wr_valid), .rsp_wr_ready(rsp_wr_ready),
        .qreqn(1'b1), .reg_qdeny_en(1'b0), .reg_err_en(1'b0),
        .qacceptn(), .qdeny(), .qactive(), .lbus_pwrdn()
    );

    wire [ID_W-1:0] s_arid, s_awid, s_rid, s_bid;
    wire [ADDR_W-1:0] s_araddr, s_awaddr;
    wire [LEN_W-1:0] s_arlen, s_awlen;
    wire [2:0] s_arsize, s_awsize;
    wire [1:0] s_arburst, s_awburst, s_rresp, s_bresp;
    wire s_arvalid, s_arready, s_rvalid, s_rlast, s_rready;
    wire s_awvalid, s_awready, s_wvalid, s_wready, s_wlast, s_bvalid, s_bready;
    wire [DATA_W-1:0] s_wdata, s_rdata;
    wire [W_BYTES-1:0] s_wstrb;

    adapter_slv_axi4 #(
        .MAX_RD_OST(4), .MAX_WR_OST(4)
    ) u_slv (
        .clk(clk), .rst_n(rst_n),
        .req_r_data(req_r_data), .req_r_valid(req_r_valid), .req_r_ready(req_r_ready),
        .req_w_data(req_w_data), .req_w_valid(req_w_valid), .req_w_ready(req_w_ready),
        .rsp_rd_data(rsp_rd_data), .rsp_rd_last(rsp_rd_last), .rsp_rd_resp(rsp_rd_resp),
        .rsp_rd_ext_txnid(rsp_rd_txn), .rsp_rd_user(),
        .rsp_rd_valid(rsp_rd_valid), .rsp_rd_ready(rsp_rd_ready),
        .rsp_wr_resp(rsp_wr_resp), .rsp_wr_ext_txnid(rsp_wr_txn),
        .rsp_wr_user(), .rsp_wr_valid(rsp_wr_valid), .rsp_wr_ready(rsp_wr_ready),
        .arid(s_arid), .araddr(s_araddr), .arlen(s_arlen), .arsize(s_arsize),
        .arburst(s_arburst), .arvalid(s_arvalid), .arready(s_arready),
        .rid(s_rid), .rdata(s_rdata), .rresp(s_rresp), .rlast(s_rlast),
        .rvalid(s_rvalid), .rready(s_rready),
        .awid(s_awid), .awaddr(s_awaddr), .awlen(s_awlen), .awsize(s_awsize),
        .awburst(s_awburst), .awvalid(s_awvalid), .awready(s_awready),
        .wdata(s_wdata), .wstrb(s_wstrb), .wlast(s_wlast), .wvalid(s_wvalid),
        .wready(s_wready),
        .bid(s_bid), .bresp(s_bresp), .bvalid(s_bvalid), .bready(s_bready),
        .qreqn(1'b1), .qacceptn(), .qdeny(), .qactive(), .lbus_pwrdn(), .intercept()
    );

    axi_slave_mem #(
        .ADDR_W(ADDR_W), .DATA_W(DATA_W), .LEN_W(LEN_W), .ID_W(ID_W)
    ) u_mem (
        .clk(clk), .rst_n(rst_n),
        .awid(s_awid), .awaddr(s_awaddr), .awlen(s_awlen), .awsize(s_awsize),
        .awburst(s_awburst), .awvalid(s_awvalid), .awready(s_awready),
        .wdata(s_wdata), .wstrb(s_wstrb), .wlast(s_wlast), .wvalid(s_wvalid),
        .wready(s_wready),
        .bid(s_bid), .bresp(s_bresp), .bvalid(s_bvalid), .bready(s_bready),
        .arid(s_arid), .araddr(s_araddr), .arlen(s_arlen), .arsize(s_arsize),
        .arburst(s_arburst), .arvalid(s_arvalid), .arready(s_arready),
        .rid(s_rid), .rdata(s_rdata), .rresp(s_rresp), .rlast(s_rlast),
        .rvalid(s_rvalid), .rready(s_rready)
    );

    initial clk = 0;
    always #5 clk = ~clk;

    initial begin
        err_cnt = 0;
        rst_n = 0;
        awvalid = 0; wvalid = 0; arvalid = 0; bready = 1; rready = 1;
        awid = 0; arid = 0; awaddr = 0; araddr = 0; awlen = 0; arlen = 0;
        wdata = 0; wstrb = 8'hFF; wlast = 1;
        #30 rst_n = 1;
        @(negedge clk);

        $display("--- loopback wr/rd ---");
        awid = 8'h4; awaddr = 32'h80; awlen = 0;
        wdata = 64'h1122334455667788; wlast = 1;
        awvalid = 1; wvalid = 1;
        @(posedge clk);
        to = 0;
        while (!(awready && wready) && to < 500) begin @(posedge clk); to = to + 1; end
        if (!(awready && wready)) begin $display("TIMEOUT loop AW"); err_cnt = err_cnt + 1; end
        @(negedge clk);
        awvalid = 0; wvalid = 0;
        to = 0;
        while (!bvalid && to < 500) begin @(posedge clk); to = to + 1; end
        if (bresp !== 2'b00) begin $display("FAIL loop B"); err_cnt = err_cnt + 1; end
        @(negedge clk);

        arid = 8'h4; araddr = 32'h80; arlen = 0; arvalid = 1;
        @(posedge clk);
        to = 0;
        while (!arready && to < 500) begin @(posedge clk); to = to + 1; end
        @(negedge clk);
        arvalid = 0;
        to = 0;
        while (!rvalid && to < 500) begin @(posedge clk); to = to + 1; end
        if (rdata !== 64'h1122334455667788 || !rlast) begin
            $display("FAIL loop R %h", rdata); err_cnt = err_cnt + 1;
        end
        @(negedge clk);

        $display("--- loopback 2-beat ---");
        awid = 8'h5; awaddr = 32'hC0; awlen = 1;
        wdata = 64'h1111111111111111; wlast = 0;
        awvalid = 1; wvalid = 1;
        @(posedge clk);
        to = 0;
        while (!(awready && wready) && to < 500) begin @(posedge clk); to = to + 1; end
        if (!(awready && wready)) begin $display("TIMEOUT loop beat0"); err_cnt = err_cnt + 1; end
        @(negedge clk);
        awvalid = 0; wdata = 64'h2222222222222222; wlast = 1;
        @(posedge clk);
        to = 0;
        while (!wready && to < 500) begin @(posedge clk); to = to + 1; end
        if (!wready) begin $display("TIMEOUT loop beat1"); err_cnt = err_cnt + 1; end
        @(negedge clk);
        wvalid = 0;
        to = 0;
        while (!bvalid && to < 500) begin @(posedge clk); to = to + 1; end
        if (bresp !== 2'b00) begin $display("FAIL loop 2beat B"); err_cnt = err_cnt + 1; end
        @(negedge clk);
        arid = 8'h5; araddr = 32'hC0; arlen = 1; arvalid = 1;
        @(posedge clk);
        to = 0;
        while (!arready && to < 500) begin @(posedge clk); to = to + 1; end
        @(negedge clk);
        arvalid = 0;
        to = 0;
        while (!rvalid && to < 500) begin @(posedge clk); to = to + 1; end
        if (rdata !== 64'h1111111111111111) begin $display("FAIL loop 2beat R0 %h", rdata); err_cnt = err_cnt + 1; end
        @(posedge clk);
        to = 0;
        while (!rvalid && to < 500) begin @(posedge clk); to = to + 1; end
        if (rdata !== 64'h2222222222222222 || !rlast) begin
            $display("FAIL loop 2beat R1 %h", rdata); err_cnt = err_cnt + 1;
        end
        @(negedge clk);

        $display("--- loopback multi outstanding wr ---");
        bready = 0;
        awid = 8'h71; awaddr = 32'h90; awlen = 0;
        wdata = 64'h71; wlast = 1; awvalid = 1; wvalid = 1;
        @(posedge clk);
        to = 0;
        while (!(awready && wready) && to < 500) begin @(posedge clk); to = to + 1; end
        @(negedge clk);
        awvalid = 0; wvalid = 0;
        awid = 8'h72; awaddr = 32'h98; wdata = 64'h72; awvalid = 1; wvalid = 1;
        @(posedge clk);
        to = 0;
        while (!(awready && wready) && to < 500) begin @(posedge clk); to = to + 1; end
        if (!(awready && wready)) begin $display("TIMEOUT loop ost W1"); err_cnt = err_cnt + 1; end
        @(negedge clk);
        awvalid = 0; wvalid = 0;
        bready = 1;
        to = 0;
        while (!bvalid && to < 500) begin @(posedge clk); to = to + 1; end
        @(negedge clk);
        to = 0;
        while (!bvalid && to < 500) begin @(posedge clk); to = to + 1; end
        @(negedge clk);

        if (err_cnt == 0) $display("=== ALL TESTS PASSED ===");
        else $display("=== %0d TEST(S) FAILED ===", err_cnt);
        $finish;
    end

    initial begin
        #400000;
        $display("TIMEOUT tb_adapter_simple_loopback");
        $finish;
    end
endmodule
