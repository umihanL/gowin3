`timescale 1ns/1ps
module bypass_single_front_tb;
 reg clk=0;always #5 clk=~clk;
 reg rst=0,tick=0,start=0,cancel=0,choose_left=0,single_entry=0;
 reg signed [15:0] px=0,py=0,heading=0;
 reg signed [31:0] travel=0;
 reg [15:0] lf=300,rf=300,left=500,right=500;
 reg [31:0] diagnostics=0;reg [3:0] obs=0;
 wire active,done,blocked,tail;wire [3:0] phase;
 wire signed [15:0] steer,lx,ly;
 vehicle_bypass dut(.clk(clk),.rst_n(rst),.tick(tick),.start(start),.cancel(cancel),
 .choose_left(choose_left),.single_front_entry(single_entry),.px(px),.py(py),.heading(heading),
 .cosine(17'sd32768),.sine(17'sd0),.travel_q8(travel),.lf(lf),.rf(rf),
 .left_mm(left),.right_mm(right),.diagnostics(diagnostics),.observations(obs),
 .active(active),.done(done),.blocked(blocked),.phase(phase),.steering(steer),
 .local_x(lx),.local_y(ly),.tail_confirmed(tail),.left_turn());
 integer side,n;
 task step;begin
  @(negedge clk);tick=1;travel=travel+256;
  @(negedge clk);tick=0;repeat(198)@(negedge clk);
 end endtask
 // Sensor completion occurs between control ticks, on different clocks.
 task sample(input integer value,input integer code,input [3:0] mask);begin
  @(negedge clk);
  if(mask[0] || mask[3])begin
   if(choose_left)begin right=value;diagnostics[31:24]=code;end
   else begin left=value;diagnostics[7:0]=code;end
  end
  obs=mask;@(negedge clk);obs=0;step;
 end endtask
 task refresh_front;begin sample(0,0,2);sample(0,0,4);end endtask
 task side_sample(input integer value,input integer code);begin sample(value,code,choose_left?8:1);end endtask
 task launch(input bit classified_single);begin
  @(negedge clk);single_entry=classified_single;lf=300;rf=300;diagnostics=0;
  left=500;right=500;px=0;py=0;heading=0;start=1;
  if(classified_single)begin
   if(choose_left)begin lf=65535;diagnostics[15:8]=8'h11;end
   else begin rf=65535;diagnostics[23:16]=8'h11;end
  end
  @(negedge clk);start=0;py=choose_left?60:-60;heading=choose_left?350:-350;
  refresh_front;for(n=0;n<12;n=n+1)step;
  if(phase!=7)$fatal(1,"single-front test failed to settle phase=%0d",phase);
 end endtask
 task stop_case;begin @(negedge clk);cancel=1;@(negedge clk);cancel=0;end endtask
 initial begin
  repeat(4)@(negedge clk);rst=1;
  for(side=0;side<2;side=side+1)begin
   choose_left=side;launch(1);
   side_sample(26,0);side_sample(26,0);
   if(phase!=7)$fatal(1,"aligned above 25mm threshold");
   side_sample(25,0);for(n=0;n<3;n=n+1)step;
   if(phase!=7)$fatal(1,"held range counted as fresh second sample");
   side_sample(25,9);side_sample(24,0);
   if(phase!=7)$fatal(1,"bus fault did not reset confirmation");
   side_sample(24,0);
   if(phase!=6 || blocked)$fatal(1,"asynchronous 25mm trigger missing side=%0d phase=%0d",side,phase);
   stop_case;
   launch(1);for(n=0;n<32;n=n+1)step;
   side_sample(24,0);side_sample(24,0);
   if(phase!=7)$fatal(1,"stale forward ranges authorised alignment");
   refresh_front;side_sample(24,0);side_sample(24,0);
   if(phase!=6)$fatal(1,"fresh forward channels did not restore eligibility");
   stop_case;
   launch(0);
   if(choose_left)begin lf=65535;diagnostics[15:8]=8'h11;end
   else begin rf=65535;diagnostics[23:16]=8'h11;end
   refresh_front;side_sample(25,0);side_sample(24,0);side_sample(24,0);
   if(phase!=7 || dut.single_front_latched)$fatal(1,"dual-beam entry became single-beam shortcut");
   stop_case;
  end
  $display("PASS single-front bypass: left/right 25mm two-sample trigger, asynchronous channels, held/fault/stale rejection and latched dual-front exclusion");$finish;
 end
 initial begin #2000000;$fatal(1,"single-front timeout");end
endmodule
