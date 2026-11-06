`timescale 1ns/1ps
module nav_video_timing_tb;
    reg clk=0;always #5 clk=~clk;
    reg rst_n=0;
    wire req,de,hs,vs,fb,pclk;
    wire [11:0] x,y;wire [15:0] rgb;
    reg [15:0] pipeline;
    always @(posedge clk)pipeline<={x[7:0],y[7:0]};
    disp_driver #(.DATA_LATENCY(1)) dut(.ClkDisp(clk),.Rst_n(rst_n),.Data(pipeline),
        .DataReq(req),.H_Addr(x),.V_Addr(y),.Disp_HS(hs),.Disp_VS(vs),
        .Disp_Red(rgb[4:0]),.Disp_Green(rgb[10:5]),.Disp_Blue(rgb[15:11]),
        .Frame_Begin(fb),.Disp_DE(de),.Disp_PCLK(pclk));
    integer pixels=0,frames=0,xx,yy;
    reg prev_de=0;
    always @(negedge clk)if(rst_n)begin
        if(de)begin
            xx=pixels%800;yy=pixels/800;
            if({rgb[4:0],rgb[10:5],rgb[15:11]}!=={xx[7:0],yy[7:0]})$fatal(1,"pipeline pixel %0d,%0d rgb=%h",xx,yy,rgb);
            pixels=pixels+1;
            if(pixels==384000)begin pixels=0;frames=frames+1;end
        end
        if(frames==2)begin
            $display("PASS delayed TFT DE/HS/VS and RGB: every pixel of two 800x480 frames");$finish;
        end
    end
    initial begin repeat(5)@(negedge clk);rst_n=1;end
    initial begin #12000000;$fatal(1,"TFT timing timeout");end
endmodule
