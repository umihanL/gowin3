`timescale 1ns/1ps
`include "disp_parameter_cfg.v"
module display_latency_check #(parameter LATENCY=0)(input clk,rst_n);
 wire req,de,hs,vs;wire [11:0] x,y;wire [15:0] rgb;
 reg [15:0] one_cycle;
 always @(posedge clk)one_cycle<={x[7:0],y[7:0]};
 disp_driver #(.DATA_LATENCY(LATENCY)) d(.ClkDisp(clk),.Rst_n(rst_n),
  .Data(LATENCY==0?{x[7:0],y[7:0]}:one_cycle),.DataReq(req),.H_Addr(x),.V_Addr(y),
  .Disp_DE(de),.Disp_HS(hs),.Disp_VS(vs),.Disp_Red(rgb[15:11]),.Disp_Green(rgb[10:5]),.Disp_Blue(rgb[4:0]));
 reg [1:0] eq,eh,ev;reg [15:0] ep[0:1];integer i;
 always @(posedge clk)if(rst_n)begin
  eq[1]=eq[0];eh[1]=eh[0];ev[1]=ev[0];ep[1]=ep[0];
  eq[0]=req;eh[0]=d.hcount_r>`H_Sync_Time-1;ev[0]=d.vcount_r>`V_Sync_Time-1;
  ep[0]=req?{x[7:0],y[7:0]}:0;
  #1;
  if(de!==eq[LATENCY] || hs!==eh[LATENCY] || vs!==ev[LATENCY] || rgb!==ep[LATENCY])
   $fatal(1,"legacy TFT latency %0d mismatch",LATENCY);
 end else begin eq=0;eh=0;ev=0;ep[0]=0;ep[1]=0;end
endmodule

// Continuous raster, matching the real input register -> overlay -> renderer
// -> synchronous row font -> TFT pipeline. No idle cycles between samples.
module vision_display_timing_tb;
 reg clk=0;always #5 clk=~clk;
 reg rst=0;wire req,de,hs,vs,fb;wire [11:0] x,y;wire [15:0] rgb;
 reg [11:0] ux=0,uy=0;reg ur=0,uf=0;reg [15:0] camera=0,raw_preview=0,up=0;
 wire [15:0] overlay,pixel;reg [1:0] page=1;
 function [15:0] pattern(input [11:0] a,b);begin pattern={a[7:0],b[7:0]};end endfunction
 always @(posedge clk)begin
  ux<=x;uy<=y;ur<=req;uf<=fb;camera<=pattern(x,y);
  raw_preview<=pattern(x,y)^16'h5a3c;up<=raw_preview;
 end
 attitude_overlay osd(.clk(clk),.rst_n(rst),.frame_begin(uf),.data_req(ur),.x_addr(ux),.y_addr(uy),
  .portrait_mode(page==2),.background(page==2?raw_preview:camera),.online(1'b1),.calibrated(1'b1),.available(1'b1),.sensor_status(3'd2),
  .cal_state(2'd2),.roll(16'sd123),.pitch(-16'sd456),.yaw(16'sd789),.pixel(overlay));
 nav_renderer renderer(.clk(clk),.rst_n(rst),.data_req(ur),.x_addr(ux),.y_addr(uy),
  .video(overlay),.preview(up),.live_page(page),.screen(4'd1),.binary_threshold(8'd96),.pixel(pixel));
 disp_driver #(.DATA_LATENCY(3)) d(.ClkDisp(clk),.Rst_n(rst),.Data(pixel),.DataReq(req),
  .H_Addr(x),.V_Addr(y),.Disp_DE(de),.Disp_HS(hs),.Disp_VS(vs),.Frame_Begin(fb),
  .Disp_Red(rgb[15:11]),.Disp_Green(rgb[10:5]),.Disp_Blue(rgb[4:0]));
 display_latency_check #(.LATENCY(0)) legacy0(clk,rst);
 display_latency_check #(.LATENCY(1)) legacy1(clk,rst);
 reg [3:0] eq=0,eh=0,ev=0,check=0;reg [15:0] ep[0:3];
 integer i,pixels=0,frames=0,verified=0;
 always @(posedge clk)if(rst)begin
  for(i=3;i>0;i=i-1)begin eq[i]=eq[i-1];eh[i]=eh[i-1];ev[i]=ev[i-1];ep[i]=ep[i-1];check[i]=check[i-1];end
  eq[0]=req;eh[0]=d.hcount_r>`H_Sync_Time-1;ev[0]=d.vcount_r>`V_Sync_Time-1;
  ep[0]=pattern(x,y);check[0]=req && !(x<72 && y<72) &&
    !(x>=88 && x<216 && y>=24 && y<40) && !(x>=556 && x<712 && y>=10 && y<90);
  if(page==2)begin
   check[0]=req && ((x>=28 && x<388) || (x>=396 && x<756)) && y<480 &&
      !(x<72 && y>=408);
   ep[0]=pattern(x,y)^16'h5a3c;
  end
  #1;
  if(de!==eq[3] || hs!==eh[3] || vs!==ev[3])$fatal(1,"three-cycle DE/HS/VS misalignment");
  if(!de && rgb!==0)$fatal(1,"RGB outside DE");
  if(check[3])begin
   if(rgb!==ep[3])$fatal(1,"continuous UI camera/preview pixel at %0d,%0d got %h expected %h",x,y,rgb,ep[3]);
   verified=verified+1;
  end
  if(de)begin
   pixels=pixels+1;
   if(pixels==384000)begin pixels=0;frames=frames+1;page=2;end
    if(frames==2)begin
    if(verified<450000)$fatal(1,"unexpected pixel coverage %0d",verified);
    $display("PASS vision display timing: continuous 800x480 normal/preview, latency 0/1/3, RGB/DE/HS/VS and line/frame edges (%0d pixels)",verified);$finish;
   end
  end
 end
 initial begin repeat(5)@(negedge clk);rst=1;end
 initial begin #12000000;$fatal(1,"display integration timeout");end
endmodule
