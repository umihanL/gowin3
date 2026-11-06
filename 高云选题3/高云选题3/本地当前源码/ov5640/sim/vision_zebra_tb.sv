`timescale 1ns/1ps
module vision_zebra_tb;
 reg clk=0;always #5 clk=~clk;
 reg rst_n=0,frame_begin=0,frame_complete=1,sample_valid=0,dark=1;
 reg [6:0] x=0,y=0,centre=52;
 wire update,detected;
 vision_zebra dut(.clk(clk),.rst_n(rst_n),.enable(1'b1),.frame_begin(frame_begin),
   .frame_complete(frame_complete),.sample_valid(sample_valid),.dark(dark),
   .sample_x(x),.sample_y(y),.centre(centre),.update(update),.detected(detected));
 function stripe_white(input integer row);
   begin stripe_white=(row>=34 && row<=35)||(row>=38 && row<=39)||
       (row>=42 && row<=43)||(row>=46 && row<=47)||(row>=60 && row<=62);end
 endfunction
 task draw_frame(input integer pattern);
   integer xx,yy;
   begin
     for(yy=34;yy<=74;yy=yy+1)begin
       for(xx=0;xx<104;xx=xx+1)begin
         @(negedge clk);sample_valid=1;x=xx;y=yy;
         if(pattern==1)dark=!stripe_white(yy);
         else if(pattern==2)dark=!(yy>=60 && yy<=62);
         else dark=1;
       end
     end
     @(negedge clk);sample_valid=0;frame_begin=1;
     @(negedge clk);frame_begin=0;
     #1;
   end
 endtask
 initial begin
   repeat(4)@(negedge clk);rst_n=1;@(negedge clk);frame_begin=1;@(negedge clk);frame_begin=0;
   draw_frame(1);
   if(!update || !detected)$fatal(1,"transverse zebra bands not detected: w=%0d d=%0d near=%b",dut.white_bands,dut.dark_bands,dut.near_white);
   draw_frame(2);
   if(!update || detected)$fatal(1,"single white marking falsely detected as zebra");
   draw_frame(0);
   if(!update || detected)$fatal(1,"ordinary dark road falsely detected as zebra");
   $display("PASS vision zebra: multi-band crosswalk detected; single marking and ordinary road rejected");$finish;
 end
 initial begin #10000000;$fatal(1,"zebra detector timeout");end
endmodule
