`timescale 1ns/1ps
// Range observations include the real failure: a falling/rising range during
// yaw, then another rebound while both forward beams still see the box.
module bypass_edge_trend_tb;
 reg clk=0;always #5 clk=~clk;
 reg rst=0,tick=0,start=0,cancel=0,choose_left=0;
 reg signed [15:0] px=0,py=0,heading=0;
 reg signed [31:0] travel=0;
 reg [15:0] lf=300,rf=300,left=500,right=500;
 reg [31:0] diagnostics=0;reg [3:0] obs=0;
 wire active,done,blocked,tail;wire [3:0] phase;
 wire signed [15:0] steer,lx,ly;
 vehicle_bypass dut(.clk(clk),.rst_n(rst),.tick(tick),.start(start),.cancel(cancel),
 .choose_left(choose_left),.px(px),.py(py),.heading(heading),.cosine(17'sd32768),.sine(17'sd0),
 .travel_q8(travel),.lf(lf),.rf(rf),.left_mm(left),.right_mm(right),
 .diagnostics(diagnostics),.observations(obs),.active(active),.done(done),.blocked(blocked),
 .phase(phase),.steering(steer),.local_x(lx),.local_y(ly),.tail_confirmed(tail),.left_turn());
 integer side,n,advance_mm=1;
 task step(input integer inner,input integer code,input bit sample);begin
  @(negedge clk);left=inner;right=inner;diagnostics[7:0]=code;diagnostics[31:24]=code;
  obs=sample?4'hf:0;tick=1;travel=travel+advance_mm*256;
  @(negedge clk);tick=0;obs=0;repeat(198)@(negedge clk);
 end endtask
 task fall;begin step(500,0,1);step(470,0,1);step(440,0,1);step(400,0,1);end endtask
 task clear_front;begin
  lf=65535;rf=65535;diagnostics[15:8]=8'h11;diagnostics[23:16]=8'h11;
  step(400,0,1);step(400,0,1);step(400,0,1);
 end endtask
 initial begin
  repeat(4)@(negedge clk);rst=1;
  for(side=0;side<4;side=side+1)begin
   @(negedge clk);choose_left=side%2;lf=300;rf=300;start=1;px=0;py=0;heading=0;diagnostics=0;
   @(negedge clk);start=0;
   py=side%2?60:-60;
   // Strong rotation-induced minimum is not the obstacle edge.
   fall;step(430,0,1);step(450,0,1);
   if(phase!=1 || dut.side_edge_rising)$fatal(1,"rotating range rebound aligned into front obstacle");
   heading=side%2?350:-350;
   for(n=0;n<12;n=n+1)step(500,0,1);
   if(phase!=7)$fatal(1,"settled outward heading did not enter diagonal phase %0d",phase);
   fall;step(430,0,1);step(450,0,1);
   if(phase!=7 || dut.side_edge_rising)$fatal(1,"side rebound overrode blocked front beams");
   clear_front;
   // First finite return alone, even followed by a jump, is insufficient.
   step(65535,9,1);step(400,0,1);step(450,0,1);step(470,0,1);
   if(phase!=7)$fatal(1,"no-echo/first-hit jump masqueraded as decreasing side trend");
   fall;
   step(410,0,0);step(420,0,0);
   if(dut.side_edge_rising)$fatal(1,"held readings created an edge without completed samples");
   step(410,0,1);step(65535,9,1);
   if(phase!=7)$fatal(1,"bus fault confirmed a rising edge");
   // High-rate observations can fall by just 1mm per sample.
   step(500,0,1);
   for(n=499;n>=400;n=n-1)step(n,0,1);
   if(!dut.decline_established)$fatal(1,"slow cumulative side decline was lost");
   if(side<2)begin
    step(435,0,1);
    if(phase!=7 || dut.rising_samples!=0)$fatal(1,"first millimetre bypassed 2mm guard");
    step(440,0,1);
    if(phase!=7 || dut.rising_samples!=1)$fatal(1,"first qualified rise missing");
    step(445,0,1);
   end else begin
    step(400,0,1);step(65535,8'h11,1);step(65535,8'h11,1);
   end
   if(phase!=6 || blocked)$fatal(1,"confirmed side edge did not align promptly side=%0d phase=%0d",side,phase);
   if(dut.required_offset>50)$fatal(1,"45mm floor replaced by oversized lateral gate");
   @(negedge clk);cancel=1;@(negedge clk);cancel=0;
  end
  $display("PASS bypass side trend: both directions reject rotating minima, blocked-front rebounds, first-hit jumps, stale/fault samples; fresh finite/no-echo edge aligns within 3mm with 45mm floor");$finish;
 end
 initial begin #2000000;$fatal(1,"side trend timeout");end
endmodule
