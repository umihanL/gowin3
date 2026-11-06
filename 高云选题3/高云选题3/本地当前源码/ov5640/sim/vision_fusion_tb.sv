`timescale 1ns/1ps
module vision_fusion_tb;
 reg clk=0;always #5 clk=~clk;
 reg rst=0,tick=0,enabled=1,update=0,valid=1,junction=0;
 reg signed [8:0] near_error=0,heading=0;
 reg signed [15:0] route=0;wire signed [15:0] steer;wire active;
 vehicle_vision_fusion dut(.turn_entry_ready(1'b1),.mapped_entry_due(1'b0),.incoming_heading_error(16'sd0),.planned_exit_valid(1'b0),.planned_exit_heading(16'sd0),.terminal_approach(1'b0),.planned_turn(route>10?2'sd1:route < -10?-2'sd1:2'sd0),.clk(clk),.rst_n(rst),.tick(tick),.enabled(enabled),
 .road_visibility({1'b1,{2{valid}}}),.road_update(update),.road_valid(valid),.road_junction(junction),.road_near(near_error),.road_heading(heading),
 .moving(1'b0),.pose_heading(16'sd0),.travel_q8(32'sd0),.route_steer(route),.steering(steer),.active(active));
 integer n,last_steer=0;real lateral=30,angle=0;
 task step(input bit new_frame);begin
  @(negedge clk);tick=1;update=new_frame;
  @(negedge clk);tick=0;update=0;
  if(steer-last_steer>1 || last_steer-steer>1)$fatal(1,"steering discontinuity");
  last_steer=steer;
 end endtask
 initial begin
  repeat(4)@(negedge clk);rst=1;near_error=30;
  for(n=0;n<80;n=n+1)step(n%3==0);
  if(!active || steer > -3 || steer < -5)$fatal(1,"right-side road not steering right %d",steer);
  // Independent lateral bicycle model, pixel scale ~6 mm; road image shifts
  // with vehicle displacement and camera lookahead, rather than DUT feedback.
  for(n=0;n<1500;n=n+1)begin
   near_error=$rtoi((lateral+210*angle));heading=$rtoi(110*angle);
   step(n%3==0);
   angle=angle+steer*0.0174533*200.0/142.0*0.01;
   lateral=lateral+angle*200.0*0.01;
  end
  if(lateral>5 || lateral < -5 || angle>0.1 || angle < -0.1)$fatal(1,"closed-loop road did not converge %f %f",lateral,angle);
  $display("INFO normalized bicycle residual: %f mm, yaw %f rad",lateral,angle);
  near_error=0;heading=0;route=25;
  for(n=0;n<150;n=n+1)step(n%3==0);
  if(!active || steer!=0)$fatal(1,"map overrode a narrow centred visible lane");
  junction=1;
  for(n=0;n<100;n=n+1)step(n%3==0);
  if(!active || steer!=0)$fatal(1,"left junction tag steered a centred visible lane %d",steer);
  route=-25;
  for(n=0;n<100;n=n+1)step(n%3==0);
  if(!active || steer!=0)$fatal(1,"right junction tag steered a centred visible lane %d",steer);
  junction=0;route=25;near_error=-30;heading=0;
  for(n=0;n<150;n=n+1)step(n%6==0);
  if(!active || steer<3 || steer>5)$fatal(1,"lower camera cadence changed steady correction %d",steer);
  route=0;
  valid=0;for(n=0;n<100;n=n+1)step(n%3==0);
  if(active)$fatal(1,"lost road remained active");
  enabled=0;route=25;last_steer=25;step(0);if(steer!=25)$fatal(1,"corner/bypass override");
  $display("PASS vision fusion: normalized gentle PD, fixed-time derivative, map conflict/junction evidence, independent convergence and fallback");$finish;
 end
endmodule
