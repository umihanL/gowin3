`timescale 1ns/1ps
// Independent convolution oracle, including interruption by active preview.
module vision_gaussian_tb;
 reg clk=0;always #5 clk=~clk;
 reg rst=0,fb=0,req=0;wire update;wire [15:0] preview;
 vision_video dut(.clk(clk),.rst_n(rst),.frame_begin(fb),.data_req(req),.pixel_valid(1'b0),
   .x(12'd100),.y(12'd100),.rgb(16'd0),.threshold(8'd96),.turn_hint(2'sd0),
   .preview(preview),.road_update(update));
 reg [5:0] picture[0:8111];
 integer a,b,k,dx,dy,cx,cy,weight,sum,expected,checked=0,cycles=0,interruptions=0;
 function integer gray(input [5:0] p);
   begin gray=(85*p[5:4])/4+(85*p[3:2])/2+(85*p[1:0])/4;end
 endfunction
 always @(negedge clk)if(rst && !fb)begin
   if(!req && dut.scan_state==3 && dut.column<104)begin
     sum=0;cy=70-2*dut.row;cx=dut.column;
     for(dy=-1;dy<=1;dy=dy+1)for(dx=-1;dx<=1;dx=dx+1)begin
       a=cx+dx;if(a<0)a=0;if(a>103)a=103;
       weight=(dx==0?2:1)*(dy==0?2:1);
       sum=sum+weight*gray(picture[(cy+dy)*104+a]);
     end
     expected=(sum+8)/16;
     if(dut.filtered_luma!==expected)
       $fatal(1,"Gaussian mismatch row=%0d col=%0d expected=%0d actual=%0d",dut.row,cx,expected,dut.filtered_luma);
     checked=checked+1;
   end
   // Force preview to interrupt every possible vertical read phase.
   if(dut.scan_state==13 && cycles%11==0)interruptions=interruptions+1;
   req=cycles%11<3;cycles=cycles+1;
 end
 initial begin
   repeat(4)@(negedge clk);rst=1;
   for(k=0;k<8112;k=k+1)begin
     picture[k]=(k*17+(k/104)*29+(k%7)*3)%64;
     dut.bank0[k]=picture[k];dut.bank1[k]=picture[k];
   end
   force dut.captured=19'd307200;
   fb=1;@(negedge clk);fb=0;release dut.captured;
   wait(update);@(negedge clk);
   if(checked!=2808 || interruptions<100)$fatal(1,"coverage checked=%0d interruptions=%0d",checked,interruptions);
   $display("PASS Gaussian: all 2808 scan pixels match independent 3x3 convolution, clamped borders, %0d interrupted reads",interruptions);
   $finish;
 end
 initial begin #5000000;$fatal(1,"Gaussian scan exceeded frame blanking budget");end
endmodule
