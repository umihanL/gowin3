`timescale 1ns/1ps
// A reached 90-degree branch must start in its planned direction even when
// the pursuit target/centroid requests the opposite branch. Public ports only.
module navigation_branch_direction_tb;
 reg clk=0;always #5 clk=~clk;
 reg rst=0,tick=0,update=0,due=0,moving=0;
 reg signed [15:0] route=0,yaw=0,exit_heading=0;
 reg signed [1:0] turn=0;
 wire blocked,recovery;wire signed [15:0] steering;
 vehicle_vision_fusion dut(.clk(clk),.rst_n(rst),.tick(tick),.enabled(1'b1),
 .road_update(update),.road_valid(1'b1),.road_junction(1'b1),.road_visibility(3'b111),
 .terminal_approach(1'b0),.road_near(9'sd0),.road_heading(9'sd0),
 .route_steer(route),.pose_heading(yaw),.travel_q8(32'sd0),.moving(moving),
 .planned_turn(turn),.planned_exit_valid(1'b1),.planned_exit_heading(exit_heading),
 .route_guidance_valid(1'b1),.route_target_behind(1'b0),.turn_entry_ready(due),
 .incoming_heading_error(16'sd0),.incoming_lateral_error(16'sd0),.mapped_entry_due(due),
 .blocked(blocked),.recovery_active(recovery),.steering(steering));
 integer k,side;
 task step;begin @(negedge clk);tick=1;update=1;@(negedge clk);tick=0;update=0;end endtask
 initial begin
  for(side=-1;side<=1;side=side+2)begin
   rst=0;due=0;moving=0;turn=0;route=0;yaw=0;exit_heading=side*900;
   repeat(5)@(negedge clk);rst=1;
   repeat(20)step;
   moving=1;turn=side;route=-side*28;
   // The branch is still ahead; no route-induced early steering.
   repeat(20)step;
   if(recovery || $abs(steering)>1)$fatal(1,"unreached branch steered early");
   due=1;repeat(25)step;
   if(blocked || !recovery || steering*side<20)
    $fatal(1,"selected branch reversed side=%0d steering=%0d",side,steering);
   // Reaching 15 degrees must not release direction ownership: the old
   // controller could now chase the other branch at a 90-degree junction.
   due=0;yaw=side*300;repeat(35)step;
   if(blocked || !recovery || steering*side<10)
    $fatal(1,"committed branch reversed mid-turn side=%0d yaw=%0d steering=%0d",side,yaw,steering);
   yaw=side*600;repeat(35)step;
   if(blocked || !recovery || steering*side<10)
    $fatal(1,"committed branch abandoned before exit side=%0d",side);
   $display("PASS reached 90deg branch side=%0d overrides contradictory pursuit without early turn",side);
  end
  $finish;
 end
 initial begin #1000000;$fatal(1,"branch timeout");end
endmodule
