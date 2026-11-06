`timescale 1ns/1ps
// Public inputs: old recovery still owns the outgoing lane, but the next
// target already asks to turn. Only its physical entry may release that turn.
module navigation_next_entry_tb;
 reg clk=0;always #5 clk=~clk;reg rst=0,tick=0,update=0,moving=0;
 reg signed [1:0] planned=1;reg signed [15:0] yaw=0,exit_yaw=900,route=20,incoming=0;
 reg entry=1,due=1;reg valid=1;wire recovery,blocked;wire signed [15:0] steer;
 vehicle_vision_fusion dut(.clk(clk),.rst_n(rst),.tick(tick),.enabled(1'b1),
 .moving(moving),.road_update(update),.road_valid(valid),.road_junction(1'b0),.road_visibility(3'b111),
 .road_near(9'sd0),.road_heading(9'sd0),.route_steer(route),.pose_heading(yaw),.travel_q8(32'sd0),
 .planned_turn(planned),.planned_exit_valid(1'b1),.planned_exit_heading(exit_yaw),
 .route_guidance_valid(1'b1),.route_target_behind(1'b0),.turn_entry_ready(entry),
 .mapped_entry_due(due),.incoming_heading_error(incoming),.terminal_approach(1'b0),
 .recovery_active(recovery),.blocked(blocked),.steering(steer));
 task step;begin @(negedge clk);tick=1;update=1;@(negedge clk);tick=0;update=0;end endtask
 integer side,j;
 initial begin
 for(side=-1;side<=1;side=side+2)begin
  rst=0;moving=0;planned=side;exit_yaw=side*900;yaw=0;route=side*20;entry=1;due=1;
  repeat(5)@(negedge clk);rst=1;repeat(10)step;moving=1;repeat(30)step;
  if(!recovery || steer*side<10)$fatal(1,"first bend not committed");
  // Exit is almost straight, but has not yet met the visual release band.
  due=0;entry=0;yaw=side*760;planned=-side;exit_yaw=0;route=-side*28;incoming=0;
  repeat(40)step;
  if(blocked || !recovery || $abs(steer)>1)$fatal(1,"NEXT_ENTRY_EARLY side=%0d steer=%0d",side,steer);
  yaw=side*900;entry=1;due=1;repeat(40)step;
  if(blocked || steer*side> -10)$fatal(1,"reached next entry did not turn");
  $display("PASS next physical entry side=%0d: future pursuit cannot steer early from old recovery, reached junction turns",side);
 end
 $finish;end
endmodule
