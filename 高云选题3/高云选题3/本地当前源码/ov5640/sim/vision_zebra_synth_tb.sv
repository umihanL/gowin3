`timescale 1ns/1ps
module vision_zebra_tb;
 GSR GSR(.GSRI(1'b1));
 reg clk=0;always #5 clk=~clk;
 reg rst_n=0,frame_begin=0,frame_complete=1,sample_valid=0,dark=1;
 reg [6:0] x=0,y=0,centre=52;
 wire update,detected;
 wire ref_update,ref_detected;
 vision_zebra_reference ref_dut(.clk(clk),.rst_n(rst_n),.frame_begin(frame_begin),
   .frame_complete(frame_complete),.sample_valid(sample_valid),.dark(dark),
   .sample_x(x),.sample_y(y),.centre(centre),.update(ref_update),.detected(ref_detected));
 integer comparisons=0;
 always @(posedge clk) begin
  #1;
  if(rst_n) begin
   if(update!==ref_update || detected!==ref_detected)
    $fatal(1,"netlist mismatch at x=%d y=%d fb=%b complete=%b got=%b%b ref=%b%b",x,y,frame_begin,frame_complete,update,detected,ref_update,ref_detected);
   comparisons=comparisons+1;
  end
 end
 vision_zebra dut(.clk(clk),.rst_n(rst_n),.frame_begin(frame_begin),
   .frame_complete(frame_complete),.sample_valid(sample_valid),.dark(dark),
   .sample_x(x),.sample_y(y),.centre(centre),.update(update),.detected(detected));
 function stripe_white(input integer row);
   begin stripe_white=(row>=34 && row<=35)||(row>=38 && row<=39)||
       (row>=42 && row<=43)||(row>=46 && row<=47);end
 endfunction
 task draw_frame(input integer pattern);
   integer xx,yy;
   begin
     for(yy=0;yy<78;yy=yy+1)begin
       for(xx=0;xx<104;xx=xx+1)begin
         @(negedge clk);sample_valid=1;x=xx;y=yy;
         if(pattern==1)dark=!(stripe_white(yy) && xx>=12 && xx<=91);
         else if(pattern==2)dark=!(yy>=42 && yy<=44 && xx>=12 && xx<=91);
         else if(pattern==3)dark=0;
         else if(pattern==4)begin
           dark=!(stripe_white(yy) && xx>=8 && xx<=72);
           if((xx==17 || xx==39 || xx==63) && yy%3==0)dark=!dark;
         end else if(pattern==5)dark=!(yy>=34 && yy<=56 && xx>=24 && xx<=79 && ((xx-24)/4)%2==0);
         else if(pattern==6)dark=!(yy>=34 && yy<=56 && xx>=24 && xx<=79 && (((xx-24+(yy-34)/4)/4)%2==0));
         else if(pattern==7)dark=!(xx>=45 && xx<=49 && yy>=30 && yy<=70);
         else if(pattern==8)dark=!((xx>=20 && xx<=24 || xx>=80 && xx<=84) && yy>=30 && yy<=70);
         else if(pattern==9)dark=!(xx>=12 && xx<=91 && stripe_white(yy-(xx-52)/8));
         else if(pattern==10)dark=!((xx+yy*7)%17==0);
         else if(pattern==11)dark=!(xx>=24 && xx<=79 && ((xx-24)/4)%2==0 && yy>=34 && yy<=36);
         else dark=1;
       end
     end
     @(negedge clk);sample_valid=0;frame_begin=1;
     @(negedge clk);frame_begin=0;
     #1;
   end
 endtask
 initial begin
   repeat(4)@(negedge clk);rst_n=1;
   draw_frame(1);
   if(!update || !detected)$fatal(1,"transverse zebra bands away from the image bottom not detected");
   draw_frame(2);
   if(!update || detected)$fatal(1,"single white marking falsely detected as zebra");
   draw_frame(3);
   if(!update || detected)$fatal(1,"all-white image falsely detected as zebra");
   draw_frame(4);
   if(!update || !detected)$fatal(1,"offset/noisy crosswalk was not detected");
   draw_frame(5);if(!update || !detected)$fatal(1,"course longitudinal bars were not detected");
   frame_complete=0;draw_frame(0);if(update || !detected)$fatal(1,"incomplete frame cleared detection");frame_complete=1;
   draw_frame(6);if(!update || !detected)$fatal(1,"slanted longitudinal bars were not detected");
   draw_frame(7);if(!update || detected)$fatal(1,"single longitudinal line was misdetected");
   draw_frame(8);if(!update || detected)$fatal(1,"two lane boundaries were misdetected");
   draw_frame(9);if(!update || !detected)$fatal(1,"slanted transverse bars were not detected");
   draw_frame(10);if(!update || detected)$fatal(1,"isolated noise was misdetected");
   draw_frame(11);if(!update || detected)$fatal(1,"too few repeated rows were misdetected");
   draw_frame(0);
   if(!update || detected)$fatal(1,"ordinary dark road falsely detected as zebra");
   $display("PASS netlist equivalence: %0d cycles checked",comparisons);
   $display("PASS vision zebra: both bar directions, slant, offset, noise, partial frame hold, clear release; single marking, lane boundaries, all-white image, noise and short texture rejected");$finish;
 end
 initial begin #10000000;$fatal(1,"zebra detector timeout");end
endmodule
