`timescale 1ns/1ps
// Physical bicycle yaw, with a limited-FOV camera around a 90 degree turn.
module vision_blind_turn_tb;
 reg clk=0;always #5 clk=~clk;
 reg rst=0,tick=0,enabled=1,update=0,valid=1,moving=0,junction=0;
 reg signed [8:0] near_error=0,trend=0;
 reg signed [15:0] route=0,yaw=0;
 reg signed [31:0] travel=0;
 wire signed [15:0] steer;wire active,blocked;wire [15:0] limit;
 wire [2:0] state; reg [2:0] visibility=3'b111; reg terminal=0;
 vehicle_vision_fusion dut(.turn_entry_ready(1'b1),.mapped_entry_due(1'b0),.incoming_heading_error(16'sd0),.planned_exit_valid(1'b0),.planned_exit_heading(16'sd0),.terminal_approach(terminal),.planned_turn(route>10?2'sd1:route < -10?-2'sd1:2'sd0),.clk(clk),.rst_n(rst),.tick(tick),.enabled(enabled),
  .road_visibility(visibility),.road_update(update),.road_valid(valid),.road_junction(junction),.road_near(near_error),.road_heading(trend),
  .route_steer(route),.pose_heading(yaw),.travel_q8(travel),.moving(moving),
  .steering(steer),.active(active),.blocked(blocked),.speed_limit(limit),.guidance_state(state));
 real angle=0;integer n,side,direction;
 task step(input bit frame);begin
  @(negedge clk);tick=1;update=frame;
  @(negedge clk);tick=0;update=0;
  if(moving && !blocked)begin
   angle=angle+$tan(steer*3.141592654/180.0)*100.0/142.0*0.01;
   yaw=$rtoi(angle*1800/3.141592654);travel=travel+256;
  end
 end endtask
 task reset_case;begin
  @(negedge clk);rst=0;moving=0;valid=1;visibility=3'b111;terminal=0;angle=0;yaw=0;travel=0;route=0;near_error=0;trend=0;
  repeat(5)@(negedge clk);rst=1;
  for(n=0;n<20;n=n+1)step(n%3==0);
  if(!active)$fatal(1,"visible lane was not acquired before start");
 end endtask
 initial begin
  for(side=0;side<2;side=side+1)begin
   reset_case;direction=side==0?1:-1;moving=1;route=direction*25;trend=-direction*20;
   for(n=0;n<12;n=n+1)step(n%3==0);
   // The inner boundary vanishes first. Even invalid centre estimates
   // must not commit a blind turn while the measured outer edge survives.
   visibility=direction==1?3'b110:3'b101;valid=0;
   for(n=0;n<80;n=n+1)step(n%3==0);
   if(state==2 || blocked || $abs(steer)>8)$fatal(1,"single-edge loss caused early full lock/stop side=%0d",side);
   $display("PASS single-edge approach side=%0d: outer boundary survives, no full lock or false deviation stop",side);
   visibility=3'b100;
   for(n=0;n<45;n=n+1)step(n%3==0);
   if(state!=2 || steer!=direction*28 || blocked || limit!=100)
    $fatal(1,"limited FOV did not acquire full measured turn, side=%0d state=%0d steer=%0d",side,state,steer);
   while($abs(yaw)<650 && !blocked)step(n%3==0);
   // The exit lane now re-enters the camera. Wheel control must return to
   // measured lane centre rather than completing an encoder turn distance.
   valid=1;visibility=3'b111;near_error=0;trend=0;route=direction*25;
   for(n=0;n<350;n=n+1)step(n%3==0);
   if(state!=1 || !active || blocked || $abs(steer)>1)
    $fatal(1,"exit-lane reacquisition failed side=%0d state=%0d steer=%0d",side,state,steer);
   $display("PASS limited-FOV turn side=%0d: full 28deg wheel, IMU-bounded turn, visible exit retakes authority",side);
  end
  reset_case;moving=1;route=25;trend=-20;visibility=3'b110;
  for(n=0;n<15;n=n+1)step(n%3==0);
  valid=0;visibility=3'b100;
  for(n=0;n<3100 && !blocked;n=n+1)step(n%3==0);
  if(!blocked || limit!=0 || $abs(yaw)>1030)$fatal(1,"unseen exit allowed endless blind circles yaw=%0d ticks=%0d blocked=%b",yaw,dut.blind_ticks,blocked);
  $display("PASS unseen exit: bounded yaw/travel/time stop; no repeated blind circle");
  reset_case;moving=1;route=0;valid=0;visibility=3'b100;
  for(n=0;n<90;n=n+1)step(n%3==0);
  if(blocked || dut.blind_turn || steer!=0)$fatal(1,"straight camera loss prevented inertial route fallback");
  $display("PASS straight road loss: no invented turn; inertial route fallback");
  reset_case;moving=1;route=0;valid=0;visibility=3'b110;
  for(n=0;n<90;n=n+1)step(n%3==0);
  if(blocked || dut.blind_turn || steer!=0)$fatal(1,"unarmed single-edge loss prevented route fallback");
  $display("PASS unarmed single edge: unreliable centre falls back to inertial route");
  reset_case;moving=1;route=25;trend=-20;valid=0;
  for(n=0;n<12;n=n+1)begin visibility=n%2?3'b110:3'b101;step(1);end
  visibility=3'b100;for(n=0;n<20;n=n+1)step(n%3==0);
  if(dut.blind_turn)$fatal(1,"alternating boundary losses armed a blind turn");
  reset_case;moving=1;route=25;trend=-20;visibility=3'b110;
  for(n=0;n<15;n=n+1)step(n%3==0);
  valid=0;visibility=3'b000;
  for(n=0;n<30;n=n+1)step(n%3==0);
  if(dut.blind_turn)$fatal(1,"incomplete camera frames committed a blind turn");
  reset_case;moving=1;terminal=1;route=25;trend=-20;visibility=3'b110;
  for(n=0;n<15;n=n+1)step(n%3==0);
  valid=0;visibility=3'b100;
  for(n=0;n<25;n=n+1)step(n%3==0);
  if(dut.blind_turn)$fatal(1,"bend beyond the terminal approach acquired full lock");
  $display("PASS frame/goal qualification: camera frame loss and road beyond goal cannot start a blind turn");$finish;
 end
 initial begin #10000000;$fatal(1,"blind turn timeout");end
endmodule
