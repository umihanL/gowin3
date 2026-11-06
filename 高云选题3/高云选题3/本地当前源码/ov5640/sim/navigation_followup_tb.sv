`timescale 1ns/1ps
// New faults exercised through the fusion's public interfaces.
module navigation_followup_tb;
 parameter CASE=0;
 reg clk=0;always #5 clk=~clk;
 reg rst=0,tick=0,enabled=0,update=0,valid=1,moving=0,trust=1,entry=0,due=0,behind=0;
 reg [2:0] visibility=7;
 reg signed [15:0] route=0,yaw=0,exit_heading=900;
 reg signed [8:0] near_error=0,heading_error=0;
 reg signed [1:0] planned=0;
 wire blocked,active,recovery;wire signed [15:0] steer;
 wire start_ready,accepted;
 vehicle_vision_fusion dut(.clk(clk),.rst_n(rst),.tick(tick),.enabled(enabled),
 .road_update(update),.road_valid(valid),.road_junction(1'b0),.road_visibility(visibility),
 .terminal_approach(1'b0),.road_near(near_error),.road_heading(heading_error),
 .route_steer(route),.pose_heading(yaw),.travel_q8(32'sd0),.moving(moving),
 .planned_turn(planned),.planned_exit_valid(1'b1),.planned_exit_heading(exit_heading),
 .turn_entry_ready(entry),.mapped_entry_due(due),.incoming_heading_error(16'sd0),
`ifndef BASELINE
 .start_ready(start_ready),.mapped_entry_accepted(accepted),
 .route_target_behind(behind),
`endif
 .route_guidance_valid(trust),.blocked(blocked),.active(active),.recovery_active(recovery),.steering(steer));
 integer n=0,side,k;
 task step(input bit frame);begin
  @(negedge clk);tick=1;update=frame;
  @(negedge clk);tick=0;update=0;n=n+1;
 end endtask
 task steps(input integer count);integer j;begin for(j=0;j<count;j=j+1)step(j%3==0);end endtask
 task reset_case;begin
  @(negedge clk);rst=0;enabled=0;moving=0;trust=1;entry=0;due=0;behind=0;
  valid=1;visibility=7;planned=0;route=0;yaw=0;near_error=0;heading_error=0;
  repeat(4)@(negedge clk);rst=1;steps(10);
 end endtask
 initial begin
  for(side=-1;side<=1;side=side+2)begin
   reset_case;exit_heading=side*900;
   if(CASE==0)begin
    // The disabled tracker and the first frame point to the wrong side.
    route=side*28;near_error=-side*110;heading_error=-side*50;steps(40);
    enabled=1;step(1);repeat(2)step(0);
    if($abs(steer)>1)$fatal(1,"STARTUP_STALE_KICK side=%0d steer=%0d",side,steer);
    route=0;near_error=0;heading_error=0;steps(60);
    if(blocked || !active || $abs(steer)>1)$fatal(1,"STABLE_CENTRED_START side=%0d steer=%0d",side,steer);
    near_error=side*30;steps(70);
    if(steer*side>=-2 || $abs(steer)>6)$fatal(1,"STARTUP_CORRECTION_SIGN side=%0d steer=%0d",side,steer);
    $display("PASS qualified startup side=%0d: stale tracker/single frame suppressed, stable lane correction retained",side);
   end else if(CASE==1)begin
    enabled=1;steps(30);moving=1;planned=side;entry=1;due=1;
    route=0;trust=0;steps(35);
    if(blocked || !recovery || steer*side<15)
     $fatal(1,"MAPPED_JUNCTION_MISSED side=%0d steer=%0d recovery=%b",side,steer,recovery);
    due=0;trust=1;route=side*20;steps(20);
    yaw=side*900;planned=0;route=0;near_error=side*60;steps(65);
    if(blocked || recovery || !active || steer*side>=-2)
     $fatal(1,"MAPPED_EXIT_CENTRING_FAILED side=%0d steer=%0d",side,steer);
    $display("PASS mapped junction side=%0d: valid straight-looking image/brief corridor handoff cannot veto reached entry, visible exit centres",side);
   end else begin
    enabled=1;steps(30);moving=1;entry=1;due=1;planned=side;
    route=side*28;steps(40);due=0;
    // A still-ahead S target calls for countersteer to keep the nose on the
    // arc while the rear axle catches up; exit yaw alone cannot decide sign.
    yaw=side*300;route=-side*18;steps(55);
    if(blocked || !recovery || steer*side>=-12)
     $fatal(1,"S_AHEAD_COUNTERSTEER_SUPPRESSED side=%0d steer=%0d",side,steer);
    behind=1;steps(55);
    if(blocked || !recovery || steer*side<15)
     $fatal(1,"BEHIND_TARGET_REVERSED_TURN side=%0d steer=%0d",side,steer);
    $display("PASS S target side=%0d: ahead geometric correction retained, behind target cannot reverse committed corner",side);
   end
  end
  $finish;
 end
 initial begin #10000000;$fatal(1,"followup fusion timeout");end
endmodule
