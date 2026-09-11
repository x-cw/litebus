//============================================================================
// Filename    : tb_adapter_simple_mst_axi.v
// Description : simple mst AXI4 + AXI5 vs adapter_slv_model; QCH; same-ID;
//               write txnid; ERR_ADDR SLVERR; AXI5 LOAD/SWAP/COMPARE.
//============================================================================
`include "adapter_defs.vh"
`default_nettype none
`timescale 1ns/1ps

module tb_adapter_simple_mst_axi;
    parameter ADDR_W = 32;
    parameter DATA_W = 64;
    parameter LEN_W  = 8;
    parameter ID_W   = 8;
    parameter W_BYTES = 8;
    parameter EXT_CMD_W = 1 + 4 + ADDR_W + LEN_W + ID_W + 1;
    parameter EXT_WD_W  = DATA_W + W_BYTES + 1 + ID_W;
    parameter EXT_REQ_W = EXT_CMD_W + 1 + EXT_WD_W;
    parameter EXT_REQ_W5 = EXT_CMD_W + 3 + EXT_WD_W;
    parameter MAX_RD_OST = 4;

    reg clk, rst_n;
    integer err_cnt, to, k;
    integer saw_same;

    reg [ID_W-1:0] awid, arid;
    reg [ADDR_W-1:0] awaddr, araddr;
    reg [LEN_W-1:0] awlen, arlen;
    reg [2:0] awsize, arsize;
    reg [1:0] awburst, arburst;
    reg awvalid, wvalid, wlast, arvalid, bready, rready;
    reg [DATA_W-1:0] wdata;
    reg [W_BYTES-1:0] wstrb;
    wire awready, wready, bvalid, arready, rvalid, rlast;
    wire [ID_W-1:0] bid, rid;
    wire [1:0] bresp, rresp;
    wire [DATA_W-1:0] rdata;
    reg qreqn, qdeny_en, err_en;
    wire qacceptn, qdeny, lbus_pwrdn;

    wire [EXT_CMD_W-1:0] req_r_data;
    wire req_r_valid, req_r_ready, req_w_valid, req_w_ready;
    wire [EXT_REQ_W-1:0] req_w_data;
    wire [DATA_W-1:0] rsp_rd_data;
    wire rsp_rd_last, rsp_rd_valid, rsp_rd_ready, rsp_wr_valid, rsp_wr_ready;
    wire [1:0] rsp_rd_resp, rsp_wr_resp;
    wire [ID_W-1:0] rsp_rd_txn, rsp_wr_txn;

    adapter_mst_axi4 #(
        .MAX_RD_OST(MAX_RD_OST), .MAX_WR_OST(MAX_RD_OST)
    ) u_mst (
        .clk(clk), .rst_n(rst_n),
        .awid(awid), .awaddr(awaddr), .awlen(awlen), .awsize(awsize),
        .awburst(awburst), .awvalid(awvalid), .awready(awready),
        .wdata(wdata), .wstrb(wstrb), .wlast(wlast), .wvalid(wvalid),
        .wready(wready),
        .bid(bid), .bresp(bresp), .bvalid(bvalid), .bready(bready),
        .arid(arid), .araddr(araddr), .arlen(arlen), .arsize(arsize),
        .arburst(arburst), .arvalid(arvalid), .arready(arready),
        .rid(rid), .rdata(rdata), .rresp(rresp), .rlast(rlast),
        .rvalid(rvalid), .rready(rready),
        .req_r_data(req_r_data), .req_r_valid(req_r_valid), .req_r_ready(req_r_ready),
        .req_w_data(req_w_data), .req_w_valid(req_w_valid), .req_w_ready(req_w_ready),
        .rsp_rd_data(rsp_rd_data), .rsp_rd_last(rsp_rd_last), .rsp_rd_resp(rsp_rd_resp),
        .rsp_rd_ext_txnid(rsp_rd_txn), .rsp_rd_valid(rsp_rd_valid),
        .rsp_rd_ready(rsp_rd_ready),
        .rsp_wr_resp(rsp_wr_resp), .rsp_wr_ext_txnid(rsp_wr_txn),
        .rsp_wr_valid(rsp_wr_valid), .rsp_wr_ready(rsp_wr_ready),
        .qreqn(qreqn), .reg_qdeny_en(qdeny_en), .reg_err_en(err_en),
        .qacceptn(qacceptn), .qdeny(qdeny), .qactive(), .lbus_pwrdn(lbus_pwrdn)
    );

    adapter_slv_model #(
        .ADDR_W(ADDR_W), .DATA_W(DATA_W), .LEN_W(LEN_W), .ID_W(ID_W),
        .MEM_BYTES(65536), .ERR_ADDR(32'h4000_0000), .MAX_BEATS(64), .MOD_PW(1)
    ) u_mem (
        .clk(clk), .rst_n(rst_n),
        .req_r_data(req_r_data), .req_r_valid(req_r_valid), .req_r_ready(req_r_ready),
        .req_w_data(req_w_data), .req_w_valid(req_w_valid), .req_w_ready(req_w_ready),
        .rsp_rd_data(rsp_rd_data), .rsp_rd_last(rsp_rd_last), .rsp_rd_resp(rsp_rd_resp),
        .rsp_rd_ext_txnid(rsp_rd_txn), .rsp_rd_valid(rsp_rd_valid),
        .rsp_rd_ready(rsp_rd_ready),
        .rsp_wr_resp(rsp_wr_resp), .rsp_wr_ext_txnid(rsp_wr_txn),
        .rsp_wr_valid(rsp_wr_valid), .rsp_wr_ready(rsp_wr_ready)
    );

    wire [EXT_CMD_W-1:0] r5_data;
    wire r5_valid, r5_ready, w5_valid, w5_ready;
    wire [EXT_REQ_W5-1:0] w5_data;
    wire [DATA_W-1:0] rd5_data;
    wire rd5_last, rd5_valid, rd5_ready, wr5_valid, wr5_ready;
    wire [1:0] rd5_resp, wr5_resp;
    wire [ID_W-1:0] rd5_txn, wr5_txn;
    wire awready5, wready5, bvalid5, arready5, rvalid5, rlast5;
    wire [1:0] bresp5, rresp5;
    wire [ID_W-1:0] bid5, rid5;
    wire [DATA_W-1:0] rdata5;
    reg awvalid5, wvalid5, wlast5, arvalid5, bready5, rready5;
    reg [4:0] awatop5;
    reg [ID_W-1:0] awid5;
    reg [ADDR_W-1:0] awaddr5;
    reg [LEN_W-1:0] awlen5;
    reg [DATA_W-1:0] wdata5;
    reg [W_BYTES-1:0] wstrb5;

    adapter_mst_axi5 #(
        .MAX_RD_OST(MAX_RD_OST), .MAX_WR_OST(MAX_RD_OST)
    ) u_mst5 (
        .clk(clk), .rst_n(rst_n),
        .awid(awid5), .awaddr(awaddr5), .awlen(awlen5), .awsize(3'd3),
        .awburst(2'b01), .awvalid(awvalid5), .awready(awready5),
        .wdata(wdata5), .wstrb(wstrb5), .wlast(wlast5), .wvalid(wvalid5),
        .wready(wready5),
        .bid(bid5), .bresp(bresp5), .bvalid(bvalid5), .bready(bready5),
        .arid(8'h0), .araddr(32'h0), .arlen(8'h0), .arsize(3'd3),
        .arburst(2'b01), .arvalid(arvalid5), .arready(arready5),
        .rid(rid5), .rdata(rdata5), .rresp(rresp5), .rlast(rlast5),
        .rvalid(rvalid5), .rready(rready5),
        .awatop(awatop5),
        .req_r_data(r5_data), .req_r_valid(r5_valid), .req_r_ready(r5_ready),
        .req_w_data(w5_data), .req_w_valid(w5_valid), .req_w_ready(w5_ready),
        .rsp_rd_data(rd5_data), .rsp_rd_last(rd5_last), .rsp_rd_resp(rd5_resp),
        .rsp_rd_ext_txnid(rd5_txn), .rsp_rd_valid(rd5_valid),
        .rsp_rd_ready(rd5_ready),
        .rsp_wr_resp(wr5_resp), .rsp_wr_ext_txnid(wr5_txn),
        .rsp_wr_valid(wr5_valid), .rsp_wr_ready(wr5_ready),
        .qreqn(1'b1), .reg_qdeny_en(1'b0), .reg_err_en(1'b0),
        .qacceptn(), .qdeny(), .qactive(), .lbus_pwrdn()
    );

    adapter_slv_model #(
        .ADDR_W(ADDR_W), .DATA_W(DATA_W), .LEN_W(LEN_W), .ID_W(ID_W),
        .MEM_BYTES(65536), .ERR_ADDR(32'h4000_0000), .MAX_BEATS(64), .MOD_PW(3)
    ) u_mem5 (
        .clk(clk), .rst_n(rst_n),
        .req_r_data(r5_data), .req_r_valid(r5_valid), .req_r_ready(r5_ready),
        .req_w_data(w5_data), .req_w_valid(w5_valid), .req_w_ready(w5_ready),
        .rsp_rd_data(rd5_data), .rsp_rd_last(rd5_last), .rsp_rd_resp(rd5_resp),
        .rsp_rd_ext_txnid(rd5_txn), .rsp_rd_valid(rd5_valid),
        .rsp_rd_ready(rd5_ready),
        .rsp_wr_resp(wr5_resp), .rsp_wr_ext_txnid(wr5_txn),
        .rsp_wr_valid(wr5_valid), .rsp_wr_ready(wr5_ready)
    );

    task wait_hi;
        input integer which;
        begin
            to = 0;
            if (which == 0) begin
                while (!bvalid && to < 800) begin
                    @(posedge clk); to = to + 1;
                end
                if (!bvalid) begin $display("TIMEOUT B"); err_cnt = err_cnt + 1; end
            end else begin
                while (!rvalid && to < 800) begin
                    @(posedge clk); to = to + 1;
                end
                if (!rvalid) begin $display("TIMEOUT R"); err_cnt = err_cnt + 1; end
            end
        end
    endtask

    task axi4_aww;
        input [7:0] id;
        input [31:0] addr;
        input [63:0] data;
        begin
            awid = id; awaddr = addr; awlen = 0; awvalid = 1;
            wdata = data; wlast = 1; wvalid = 1;
            @(posedge clk);
            to = 0;
            while (!(awvalid && awready && wvalid && wready) && to < 400) begin
                @(posedge clk); to = to + 1;
            end
            if (!(awready && wready)) begin $display("TIMEOUT AW/W @%h", addr); err_cnt = err_cnt + 1; end
            @(negedge clk);
            awvalid = 0; wvalid = 0;
        end
    endtask

    task axi5_aww;
        input [7:0] id;
        input [31:0] addr;
        input [4:0] atop;
        input [63:0] data;
        begin
            awid5 = id; awaddr5 = addr; awlen5 = 0; awatop5 = atop;
            wdata5 = data; wlast5 = 1; wstrb5 = 8'hFF;
            awvalid5 = 1; wvalid5 = 1;
            @(posedge clk);
            to = 0;
            while (!(awready5 && wready5) && to < 400) begin
                @(posedge clk); to = to + 1;
            end
            if (!(awready5 && wready5)) begin $display("TIMEOUT a5 AW @%h", addr); err_cnt = err_cnt + 1; end
            @(negedge clk);
            awvalid5 = 0; wvalid5 = 0;
        end
    endtask

    initial clk = 0;
    always #5 clk = ~clk;

    initial begin
        err_cnt = 0;
        rst_n = 0;
        awvalid = 0; wvalid = 0; arvalid = 0; bready = 1; rready = 1;
        awid = 0; arid = 0; awaddr = 0; araddr = 0; awlen = 0; arlen = 0;
        awsize = 3; arsize = 3; awburst = 1; arburst = 1;
        wdata = 0; wstrb = 8'hFF; wlast = 1;
        qreqn = 1; qdeny_en = 0; err_en = 0;
        awvalid5 = 0; wvalid5 = 0; arvalid5 = 0; bready5 = 1; rready5 = 1;
        awatop5 = 0; awid5 = 0; awaddr5 = 0; awlen5 = 0; wdata5 = 0;
        wstrb5 = 8'hFF; wlast5 = 1;
        #30 rst_n = 1;
        @(negedge clk);

        $display("--- mst axi4 wr/rd 1 beat ---");
        awid = 8'hA; awaddr = 32'h100; awlen = 0; awvalid = 1;
        wdata = 64'h0123456789ABCDEF; wlast = 1; wvalid = 1;
        @(posedge clk);
        to = 0;
        while (!(awvalid && awready && wvalid && wready) && to < 200) begin
            @(posedge clk); to = to + 1;
        end
        if (!(awready && wready)) begin $display("TIMEOUT AW/W"); err_cnt = err_cnt + 1; end
        @(negedge clk);
        awvalid = 0; wvalid = 0;
        wait_hi(0);
        if (bresp !== 2'b00 || bid !== 8'hA) begin
            $display("FAIL B %h id %h", bresp, bid); err_cnt = err_cnt + 1;
        end
        @(negedge clk);

        arid = 8'hA; araddr = 32'h100; arlen = 0; arvalid = 1;
        @(posedge clk);
        to = 0;
        while (!(arvalid && arready) && to < 200) begin
            @(posedge clk); to = to + 1;
        end
        if (!arready) begin $display("TIMEOUT AR"); err_cnt = err_cnt + 1; end
        @(negedge clk);
        arvalid = 0;
        wait_hi(1);
        if (rdata !== 64'h0123456789ABCDEF || rresp !== 2'b00 || !rlast || rid !== 8'hA) begin
            $display("FAIL R %h", rdata); err_cnt = err_cnt + 1;
        end
        @(negedge clk);

        $display("--- mst wr txnid = write slot 0 ---");
        awid = 8'h5; awaddr = 32'h280; awlen = 0; awvalid = 1;
        wdata = 64'h55; wlast = 1; wvalid = 1;
        @(posedge clk);
        to = 0;
        while (!(req_w_valid && req_w_ready) && to < 200) begin
            @(posedge clk); to = to + 1;
        end
        if (req_w_data[ID_W-1:0] !== {ID_W{1'b0}}) begin
            $display("FAIL wr txnid %h exp 0", req_w_data[ID_W-1:0]);
            err_cnt = err_cnt + 1;
        end
        to = 0;
        while (!(awready && wready) && to < 200) begin @(posedge clk); to = to + 1; end
        @(negedge clk);
        awvalid = 0; wvalid = 0;
        wait_hi(0);
        @(negedge clk);

        $display("--- mst axi4 2-beat ---");
        awid = 8'h3; awaddr = 32'h200; awlen = 1; awvalid = 1;
        wdata = 64'h1111111111111111; wlast = 0; wvalid = 1;
        @(posedge clk);
        to = 0;
        while (!(awready && wready) && to < 200) begin
            @(posedge clk); to = to + 1;
        end
        if (!(awready && wready)) begin $display("TIMEOUT beat0"); err_cnt = err_cnt + 1; end
        @(negedge clk);
        awvalid = 0; wdata = 64'h2222222222222222; wlast = 1;
        @(posedge clk);
        to = 0;
        while (!wready && to < 200) begin
            @(posedge clk); to = to + 1;
        end
        if (!wready) begin $display("TIMEOUT beat1"); err_cnt = err_cnt + 1; end
        @(negedge clk);
        wvalid = 0;
        wait_hi(0);
        if (bresp !== 2'b00) begin $display("FAIL 2beat B"); err_cnt = err_cnt + 1; end
        @(negedge clk);
        arid = 8'h3; araddr = 32'h200; arlen = 1; arvalid = 1;
        @(posedge clk);
        to = 0;
        while (!arready && to < 200) begin @(posedge clk); to = to + 1; end
        if (!arready) begin $display("TIMEOUT 2beat AR"); err_cnt = err_cnt + 1; end
        @(negedge clk);
        arvalid = 0;
        wait_hi(1);
        if (rdata !== 64'h1111111111111111) begin $display("FAIL 2beat R0 %h", rdata); err_cnt = err_cnt + 1; end
        @(posedge clk);
        to = 0;
        while (!rvalid && to < 200) begin @(posedge clk); to = to + 1; end
        if (!rvalid) begin $display("TIMEOUT 2beat R1"); err_cnt = err_cnt + 1; end
        if (rdata !== 64'h2222222222222222 || !rlast) begin $display("FAIL 2beat R1 %h", rdata); err_cnt = err_cnt + 1; end
        @(negedge clk);

        $display("--- mst axi4 multi outstanding wr ---");
        bready = 0;
        awid = 8'h11; awaddr = 32'h400; awlen = 0; awvalid = 1;
        wdata = 64'h1111000011110000; wlast = 1; wvalid = 1;
        @(posedge clk);
        to = 0;
        while (!(awready && wready) && to < 200) begin @(posedge clk); to = to + 1; end
        if (!(awready && wready)) begin $display("TIMEOUT ost W0"); err_cnt = err_cnt + 1; end
        @(negedge clk);
        awvalid = 0; wvalid = 0;
        awid = 8'h22; awaddr = 32'h408; awvalid = 1; wdata = 64'h2222000022220000; wvalid = 1;
        @(posedge clk);
        to = 0;
        while (!(awready && wready) && to < 200) begin @(posedge clk); to = to + 1; end
        if (!(awready && wready)) begin $display("TIMEOUT ost W1 (need 2-ost)"); err_cnt = err_cnt + 1; end
        @(negedge clk);
        awvalid = 0; wvalid = 0;
        bready = 1;
        @(posedge clk);
        to = 0;
        while (!bvalid && to < 200) begin @(posedge clk); to = to + 1; end
        if (!bvalid || (bid !== 8'h11 && bid !== 8'h22)) begin
            $display("FAIL ost B0 id %h", bid); err_cnt = err_cnt + 1;
        end
        @(negedge clk);
        @(posedge clk);
        to = 0;
        while (!bvalid && to < 200) begin @(posedge clk); to = to + 1; end
        if (!bvalid || (bid !== 8'h11 && bid !== 8'h22)) begin
            $display("FAIL ost B1 id %h", bid); err_cnt = err_cnt + 1;
        end
        @(negedge clk);

        $display("--- mst axi4 multi outstanding rd ---");
        arid = 8'h31; araddr = 32'h400; arlen = 0; arvalid = 1;
        @(posedge clk);
        to = 0;
        while (!arready && to < 200) begin @(posedge clk); to = to + 1; end
        @(negedge clk);
        arvalid = 0;
        arid = 8'h32; araddr = 32'h408; arvalid = 1;
        @(posedge clk);
        to = 0;
        while (!arready && to < 200) begin @(posedge clk); to = to + 1; end
        if (!arready) begin $display("TIMEOUT ost AR1"); err_cnt = err_cnt + 1; end
        @(negedge clk);
        arvalid = 0;
        wait_hi(1);
        if (!rlast) begin $display("FAIL ost R0 last"); err_cnt = err_cnt + 1; end
        @(negedge clk);
        wait_hi(1);
        if (!rlast) begin $display("FAIL ost R1 last"); err_cnt = err_cnt + 1; end
        @(negedge clk);

        $display("--- mst same AXI ID stall ---");
        bready = 0;
        axi4_aww(8'hA5, 32'h600, 64'hA5A5);
        awid = 8'hA5; awaddr = 32'h608; awlen = 0; awvalid = 1;
        wdata = 64'h5A5A; wlast = 1; wvalid = 1;
        saw_same = 0;
        for (k = 0; k < 8; k = k + 1) begin
            @(posedge clk);
            if (awready) saw_same = 1;
        end
        if (saw_same) begin $display("FAIL same-id awready"); err_cnt = err_cnt + 1; end
        awid = 8'hA6;
        @(posedge clk);
        to = 0;
        while (!(awready && wready) && to < 200) begin @(posedge clk); to = to + 1; end
        if (!(awready && wready)) begin $display("TIMEOUT other-id W"); err_cnt = err_cnt + 1; end
        @(negedge clk);
        awvalid = 0; wvalid = 0;
        bready = 1;
        wait_hi(0);
        @(negedge clk);
        wait_hi(0);
        @(negedge clk);

        $display("--- mst ERR_ADDR SLVERR ---");
        axi4_aww(8'hE0, 32'h4000_0000, 64'h1);
        wait_hi(0);
        if (bresp !== `AXI_RESP_SLVERR) begin
            $display("FAIL ERR_ADDR B %h", bresp); err_cnt = err_cnt + 1;
        end
        @(negedge clk);
        arid = 8'hE0; araddr = 32'h4000_0000; arlen = 0; arvalid = 1;
        @(posedge clk);
        to = 0;
        while (!arready && to < 200) begin @(posedge clk); to = to + 1; end
        @(negedge clk);
        arvalid = 0;
        wait_hi(1);
        if (rresp !== `AXI_RESP_SLVERR) begin
            $display("FAIL ERR_ADDR R %h", rresp); err_cnt = err_cnt + 1;
        end
        @(negedge clk);

        $display("--- mst QCH waits for outstanding ---");
        awid = 8'h41; awaddr = 32'h500; awlen = 0; awvalid = 1;
        wdata = 64'h5; wlast = 1; wvalid = 1; bready = 0;
        @(posedge clk);
        to = 0;
        while (!(awready && wready) && to < 200) begin @(posedge clk); to = to + 1; end
        @(negedge clk);
        awvalid = 0; wvalid = 0;
        qdeny_en = 0; err_en = 0; qreqn = 0;
        repeat (8) @(posedge clk);
        if (lbus_pwrdn) begin $display("FAIL QCH down while ost"); err_cnt = err_cnt + 1; end
        if (!qacceptn) begin $display("FAIL QCH qacceptn while ost"); err_cnt = err_cnt + 1; end
        bready = 1;
        @(posedge clk);
        to = 0;
        while (!bvalid && to < 200) begin @(posedge clk); to = to + 1; end
        if (!bvalid) begin $display("TIMEOUT QCH ost B"); err_cnt = err_cnt + 1; end
        @(negedge clk);
        to = 0;
        while (!lbus_pwrdn && to < 40) begin @(posedge clk); to = to + 1; end
        if (!lbus_pwrdn) begin $display("FAIL QCH no pwrdn after ost"); err_cnt = err_cnt + 1; end
        qreqn = 1;
        repeat (3) @(posedge clk);

        $display("--- mst QCH err_mode ---");
        awaddr = 32'h300; awlen = 0; awid = 8'h1; awvalid = 1;
        wvalid = 0;
        @(posedge clk);
        to = 0;
        while (!awready && to < 200) begin @(posedge clk); to = to + 1; end
        if (!awready) begin $display("TIMEOUT QCH AW"); err_cnt = err_cnt + 1; end
        @(negedge clk);
        awvalid = 0;
        err_en = 1; qreqn = 0;
        @(posedge clk); @(negedge clk);
        arid = 8'h7; araddr = 32'h0; arlen = 0; arvalid = 1;
        @(posedge clk);
        to = 0;
        while (!arready && to < 200) begin @(posedge clk); to = to + 1; end
        if (!arready) begin $display("TIMEOUT err AR"); err_cnt = err_cnt + 1; end
        @(negedge clk);
        arvalid = 0;
        wait_hi(1);
        if (rresp !== `AXI_RESP_SLVERR || !rlast || rid !== 8'h7) begin
            $display("FAIL QCH SLVERR rresp=%h", rresp); err_cnt = err_cnt + 1;
        end
        @(negedge clk);
        wdata = 64'h0; wlast = 1; wvalid = 1;
        @(posedge clk);
        to = 0;
        while (!wready && to < 400) begin @(posedge clk); to = to + 1; end
        if (!wready) begin $display("TIMEOUT err W"); err_cnt = err_cnt + 1; end
        @(negedge clk);
        wvalid = 0;
        wait_hi(0);
        qreqn = 1; err_en = 0;
        repeat (3) @(posedge clk);

        $display("--- mst axi5 atomic store ---");
        axi5_aww(8'h2, 32'h80, 5'b00100, 64'hA5A5A5A5A5A5A5A5);
        to = 0;
        while (!bvalid5 && to < 400) begin @(posedge clk); to = to + 1; end
        if (!bvalid5) begin $display("TIMEOUT a5 B"); err_cnt = err_cnt + 1; end
        if (bresp5 !== 2'b00) begin $display("FAIL axi5 B %h", bresp5); err_cnt = err_cnt + 1; end
        @(negedge clk);

        $display("--- mst axi5 LOAD (B+R) ---");
        axi5_aww(8'h3, 32'hA0, 5'b00000, 64'h1111_2222_3333_4444);
        to = 0;
        while (!bvalid5 && to < 400) begin @(posedge clk); to = to + 1; end
        @(negedge clk);
        axi5_aww(8'h4, 32'hA0, 5'b00001, 64'h0);
        to = 0;
        while (!rvalid5 && to < 400) begin @(posedge clk); to = to + 1; end
        if (!rvalid5) begin $display("TIMEOUT a5 LOAD R"); err_cnt = err_cnt + 1; end
        if (rdata5 !== 64'h1111_2222_3333_4444 || rid5 !== 8'h4 || !rlast5) begin
            $display("FAIL LOAD R %h id %h", rdata5, rid5); err_cnt = err_cnt + 1;
        end
        @(negedge clk);
        to = 0;
        while (!bvalid5 && to < 400) begin @(posedge clk); to = to + 1; end
        if (!bvalid5 || bid5 !== 8'h4) begin $display("FAIL LOAD B"); err_cnt = err_cnt + 1; end
        @(negedge clk);

        $display("--- mst axi5 SWAP ---");
        axi5_aww(8'h5, 32'hA8, 5'b00000, 64'hAAAA_AAAA_AAAA_AAAA);
        to = 0;
        while (!bvalid5 && to < 400) begin @(posedge clk); to = to + 1; end
        @(negedge clk);
        axi5_aww(8'h6, 32'hA8, 5'b01010, 64'hBBBB_BBBB_BBBB_BBBB);
        to = 0;
        while (!rvalid5 && to < 400) begin @(posedge clk); to = to + 1; end
        if (rdata5 !== 64'hAAAA_AAAA_AAAA_AAAA) begin
            $display("FAIL SWAP old %h", rdata5); err_cnt = err_cnt + 1;
        end
        @(negedge clk);
        to = 0;
        while (!bvalid5 && to < 400) begin @(posedge clk); to = to + 1; end
        @(negedge clk);
        axi5_aww(8'h8, 32'hA8, 5'b00001, 64'h0);
        to = 0;
        while (!rvalid5 && to < 400) begin @(posedge clk); to = to + 1; end
        if (rdata5 !== 64'hBBBB_BBBB_BBBB_BBBB) begin
            $display("FAIL SWAP new via LOAD %h", rdata5); err_cnt = err_cnt + 1;
        end
        @(negedge clk);
        to = 0;
        while (!bvalid5 && to < 400) begin @(posedge clk); to = to + 1; end
        @(negedge clk);

        $display("--- mst axi5 COMPARE match ---");
        axi5_aww(8'h9, 32'hB0, 5'b00000, 64'hC0DE_C0DE_C0DE_C0DE);
        to = 0;
        while (!bvalid5 && to < 400) begin @(posedge clk); to = to + 1; end
        @(negedge clk);
        axi5_aww(8'hA, 32'hB0, 5'b11111, 64'hC0DE_C0DE_C0DE_C0DE);
        to = 0;
        while (!rvalid5 && to < 400) begin @(posedge clk); to = to + 1; end
        if (rdata5 !== 64'hC0DE_C0DE_C0DE_C0DE) begin
            $display("FAIL CMP old %h", rdata5); err_cnt = err_cnt + 1;
        end
        @(negedge clk);
        to = 0;
        while (!bvalid5 && to < 400) begin @(posedge clk); to = to + 1; end
        if (bresp5 !== 2'b00) begin $display("FAIL CMP B %h", bresp5); err_cnt = err_cnt + 1; end
        @(negedge clk);

        if (err_cnt == 0) $display("=== ALL TESTS PASSED ===");
        else $display("=== %0d TEST(S) FAILED ===", err_cnt);
        $finish;
    end

    initial begin
        #800000;
        $display("TIMEOUT tb_adapter_simple_mst_axi");
        $finish;
    end
endmodule
