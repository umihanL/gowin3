`timescale 1ns/1ps
module fix_source_tb;
    reg clk = 0;
    always #5 clk = ~clk;
    reg rst_n = 1, ready = 0, frame = 0, req = 0, empty = 1;
    reg [11:0] x = 4095;
    wire rd;
    wire [15:0] pixel;
    reg [15:0] bars [0:7];
    integer i;
    tft_video_source dut (.clk(clk), .rst_n(rst_n), .source_ready(ready),
        .frame_begin(frame), .data_req(req), .x_addr(x), .fifo_empty(empty),
        .fifo_data(16'h1234), .fifo_read(rd), .pixel_data(pixel));
    initial begin
        bars[0]=16'hffff; bars[1]=16'hffe0; bars[2]=16'h07ff; bars[3]=16'h07e0;
        bars[4]=16'hf81f; bars[5]=16'hf800; bars[6]=16'h001f; bars[7]=16'h0000;
        #1; rst_n=0;
        repeat (3) @(negedge clk);
        rst_n=1; req=1;
        for (i=0; i<800; i=i+1) begin
            x=i; #1;
            if (pixel !== bars[i/100] || rd !== 0)
                $fatal(1,"startup bar x=%0d pixel=%h expected=%h read=%b", i,pixel,bars[i/100],rd);
        end
        @(negedge clk); x=0; ready=1; empty=0;
        repeat (4) @(negedge clk);
        if (pixel !== 16'hffff || rd !== 1) $fatal(1,"camera switched mid-frame");
        frame=1; @(negedge clk); frame=0;
        if (pixel !== 16'h1234) $fatal(1,"camera did not switch at frame boundary");
        empty=1; #1;
        if (pixel !== 16'hffff || rd !== 0) $fatal(1,"underflow protection failed");
        empty=0; ready=0;
        repeat (4) @(negedge clk);
        if (pixel !== 16'hffff) $fatal(1,"failed to return to bars");
        req=0; #1;
        if (rd !== 0) $fatal(1,"read during blanking");
        $display("PASS: all color bars, frame-boundary camera switch, empty FIFO protection");
        $finish;
    end
endmodule
