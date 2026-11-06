`timescale 1ns/1ps
module portrait_display_tb;
    reg clk=0;always #5 clk=~clk;
    reg rst_n=0,fb=0,req=1,ready=0,empty=1;
    reg [11:0] x=80,y=0;
    wire rd,bad;wire [15:0] pixel;
    portrait_display dut(.clk(clk),.rst_n(rst_n),.frame_begin(fb),.data_req(req),
        .x_addr(x),.y_addr(y),.frame_available(ready),.fifo_empty(empty),.fifo_data(16'h1234),
        .fifo_read(rd),.pixel_data(pixel),.underflow(bad));
    integer i;
    initial begin
        repeat(3)@(negedge clk);rst_n=1;
        ready=1;empty=0;repeat(4)@(negedge clk);
        for(i=0;i<800;i=i+1)begin x=i;#1;if(i>=80 && i<720)begin if(pixel!==16'h1234 || !rd)$fatal(1,"video viewport x=%0d",i);end else if(pixel!==0 || rd)$fatal(1,"pillarbox consumed video x=%0d",i);end
        @(negedge clk);x=80;empty=1;@(negedge clk);empty=0;#1;
        if(rd || !bad)$fatal(1,"underflow must discard remaining frame");
        fb=1;@(negedge clk);fb=0;#1;if(!rd)$fatal(1,"frame recovery");
        req=0;#1;if(rd || pixel!=0)$fatal(1,"blanking");
        $display("PASS 640x480 4:3 landscape RGB with 80px side bars, underflow discard/recovery, blanking");$finish;
    end
endmodule
