`timescale 1ns/1ps
module vision_zebra_exposure_tb;
 reg clk=0;always #5 clk=~clk;
 reg rst=0,fb=0,req=0;reg [11:0] x=0,y=0;reg [15:0] rgb=0;
 reg [7:0] slider=0;wire update,detected;
 vision_video dut(.clk(clk),.rst_n(rst),.frame_begin(fb),.data_req(req),.pixel_valid(req),
  .x(x),.y(y),.rgb(rgb),.threshold(slider),.zebra_update(update),.zebra_detected(detected));
 function [15:0] gray(input integer v);begin gray={v[7:3],v[7:2],v[7:3]};end endfunction
 task frame(input integer low,high,bars,expected);
  integer xx,yy,sx,sy,white;
  begin
   for(yy=0;yy<480;yy=yy+1)for(xx=0;xx<640;xx=xx+1)begin
    @(negedge clk);req=1;x=80+xx;y=yy;sx=xx*104/640;sy=yy*78/480;
    white=bars && sy>=32 && sy<=56 && sx>=24 && sx<=79 && ((sx-24)/4)%2==0;
    rgb=gray(white?high:low);
   end
   @(negedge clk);req=0;fb=1;@(negedge clk);fb=0;#1;
   if(!update || detected!==expected[0])$fatal(1,"exposure test low=%0d high=%0d bars=%0d slider=%0d threshold=%0d detected=%b",low,high,bars,slider,dut.zebra_threshold,detected);
   @(negedge clk);
  end
 endtask
 initial begin
  repeat(5)@(negedge clk);rst=1;
  // Both values initially below 128; the first frame calibrates exposure.
  frame(24,104,1,0);frame(24,104,1,1);
  slider=255;frame(152,232,1,0);frame(152,232,1,1);
  frame(200,200,0,0);
  $display("PASS zebra exposure: dim/bright longitudinal bars, preview slider 0/255 isolation, clear release");$finish;
 end
 initial begin #30000000;$fatal(1,"exposure timeout");end
endmodule
