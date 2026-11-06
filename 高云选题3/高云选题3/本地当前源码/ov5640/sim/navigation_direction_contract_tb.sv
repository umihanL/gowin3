`timescale 1ns/1ps
// Public interface: a quarter-turn must not retain an opposite visual slew;
// far-only or widened observations are not a starting body-pose measurement.
module navigation_direction_contract_tb;
 parameter CASE=0;
 reg clk=0;always #5 clk=~clk;
 reg rst=0,tick=0,update=0,moving=0,due=0,entry=0,junction=0,valid=1;
 reg [2:0] visibility=7;
 reg signed [1:0] turn=0;
 reg signed [15:0] route=0,yaw=0,exit_yaw=0,incoming=0;
 reg signed [8:0] lane=0,heading=0;
 wire blocked,recovery,ready;wire signed [15:0] steer;
 wire signed [15:0] alignment_yaw,alignment_offset;
 vehicle_vision_fusion dut(.clk(clk),.rst_n(rst),.tick(tick),.enabled(1'b1),
  .road_update(update),.road_valid(valid),.road_junction(junction),.road_visibility(visibility),
  .road_near(lane),.road_heading(heading),.terminal_approach(1'b0),
  .route_steer(route),.pose_heading(yaw),.travel_q8(32'sd0),.moving(moving),
  .planned_turn(turn),.planned_exit_valid(1'b1),.planned_exit_heading(exit_yaw),
  .route_guidance_valid(1'b1),.route_target_behind(1'b0),.turn_entry_ready(entry),
  .incoming_heading_error(incoming),.incoming_lateral_error(16'sd0),.mapped_entry_due(due),
  .blocked(blocked),.recovery_active(recovery),.steering(steer),.start_ready(ready),
  .alignment_yaw(alignment_yaw),.alignment_offset(alignment_offset));
 task step;begin @(negedge clk);tick=1;update=1;@(negedge clk);tick=0;update=0;end endtask
 integer side,k;
 initial begin
  for(side=-1;side<=1;side=side+2)begin
   rst=0;moving=0;due=0;entry=0;turn=0;route=0;yaw=0;incoming=0;
   lane=0;heading=0;visibility=7;junction=0;valid=1;exit_yaw=side*900;
   repeat(4)@(negedge clk);rst=1;repeat(20)step;
   if(CASE==0)begin
    // A wrong branch fit just BEFORE the reached bend has already loaded
    // a contrary visual command. Natural lost-centre recovery is not the
    // mapped-entry event, and used to leave that command slewing for 0.3s.
    moving=1;lane=side*180;heading=side*80;repeat(60)step;
    if(steer*side>=-10)$fatal(1,"opposite precondition not established");
    turn=side;route=-side*28;entry=1;valid=0;
    for(k=0;k<35;k=k+1)begin
     step;
     if(recovery && steer*side<0)
      $fatal(1,"QUARTER_OUTPUT_REVERSED side=%0d steer=%0d",side,steer);
    end
    if(blocked || !recovery || steer*side<15)
     $fatal(1,"QUARTER_CORNER_REVERSED side=%0d steer=%0d",side,steer);
    due=0;yaw=side*200;repeat(30)step;
    if(blocked || !recovery || steer*side<10)
     $fatal(1,"QUARTER_CORNER_REVERSED_MIDTURN side=%0d steer=%0d",side,steer);
    // Actual exit ends direction ownership and restores signed centring.
    yaw=side*900;valid=1;lane=side*60;heading=0;repeat(55)step;
    if(blocked || recovery || steer*side>=-2)
     $fatal(1,"EXIT_CENTRING_LOST side=%0d steer=%0d",side,steer);
   end else begin
    // The video extractor can still fit the far road after BOTH near
    // edges disappear. Its extrapolated slope is not starting vehicle yaw.
    lane=side*100;heading=side*50;visibility=CASE==1?4:7;junction=CASE==2;
    repeat(45)step;
    if(!ready || $abs(steer)>1 || $abs(alignment_yaw)>1 || $abs(alignment_offset)>1)
     $fatal(1,"UNTRUSTED_START_PRELOAD case=%0d side=%0d steer=%0d yaw=%0d offset=%0d",CASE,side,steer,alignment_yaw,alignment_offset);
    // One normal near lane restores immediate correction in both signs.
    visibility=7;junction=0;lane=side*30;heading=0;repeat(45)step;
    if(steer*side>=-2)$fatal(1,"NEAR_LANE_CORRECTION_DISABLED side=%0d steer=%0d",side,steer);
   end
   $display("PASS direction contract case=%0d side=%0d",CASE,side);
  end
  $finish;
 end
 initial begin #2000000;$fatal(1,"direction contract timeout");end
endmodule
