`timescale 1ns/1ps
// Public ports only: anticipatory map steering, off-centre exit and IMU turn.
module navigation_authority_tb;
 parameter CASE=0;
 reg clk=0;always #5 clk=~clk;
 reg rst=0,tick=0,update=0,valid=1,moving=0,trust=1;
 reg [2:0] visibility=7;
 reg signed [15:0] route=0,yaw=0,exit_heading=900;
 reg signed [8:0] near_error=0,heading=0;
 reg signed [1:0] planned=0;
 reg signed [31:0] travel=0;
 wire active,blocked,recovery;wire signed [15:0] steer;
 vehicle_vision_fusion dut(.turn_entry_ready(1'b1),.mapped_entry_due(1'b0),.incoming_heading_error(16'sd0),.clk(clk),.rst_n(rst),.tick(tick),.enabled(1'b1),
 .road_update(update),.road_valid(valid),.road_junction(1'b0),.road_visibility(visibility),
 .terminal_approach(1'b0),.road_near(near_error),.road_heading(heading),
 .route_steer(route),.pose_heading(yaw),.travel_q8(travel),.moving(moving),
 .planned_turn(planned),.planned_exit_valid(1'b1),.planned_exit_heading(exit_heading),
 .route_guidance_valid(trust),.blocked(blocked),.active(active),.recovery_active(recovery),.steering(steer));
 integer n,side;
 task step;begin
  @(negedge clk);tick=1;update=n%3==0;
  @(negedge clk);tick=0;update=0;
 end endtask
 task steps(input integer count);integer k;begin for(k=0;k<count;k=k+1)begin n=n+1;step; if(CASE==1 && yaw!=0 && n%5==0)$display("n=%d steer=%d rec=%b out=%d goal=%d near=%d head=%d curve=%b mapframes=%d blind=%b",n,steer,recovery,dut.output_q4,dut.goal,dut.near_q4,dut.heading_q4,dut.curve_qualified,dut.map_frames,dut.blind_turn);end end endtask
 task reset_case(input integer direction);begin
  @(negedge clk);rst=0;n=0;moving=0;valid=1;visibility=7;trust=1;
  planned=0;route=0;yaw=0;exit_heading=direction*900;near_error=0;heading=0;travel=0;
  repeat(5)@(negedge clk);rst=1;steps(30);moving=1;
 end endtask
 initial begin
  for(side=-1;side<=1;side=side+2)begin
   reset_case(side);
   if(CASE==0)begin
    // Future route sample already lies on the arc; the near road is straight.
    planned=side;route=side*28;steps(100);
    if(blocked || recovery || $abs(steer)>1)$fatal(1,"EARLY_MAP_TURN side=%0d steer=%0d",side,steer);
    // A displaced straight lane must still win over the opposite map angle.
    near_error=side*45;steps(60);
    if(blocked || !active || steer*side>=-2)$fatal(1,"VISIBLE_LANE_OVERRIDDEN side=%0d steer=%0d",side,steer);
    $display("PASS authority: future curve cannot steer a centred near lane; signed lane correction wins side=%0d",side);
   end else begin
    planned=side;route=side*28;valid=0;visibility=6;steps(60);
    if(!recovery || blocked)$fatal(1,"test did not enter route recovery");
    if(CASE==1)begin
     // At the actual outgoing yaw, the lane is 80mm to image right/left.
     yaw=side*900;valid=1;visibility=side==1?5:6;
     near_error=side*80;heading=side*10;steps(65);
     if(blocked || recovery || !active || steer*side>=-2)
      $fatal(1,"OFFCENTRE_EXIT_LOCKOUT side=%0d active=%b recovery=%b steer=%0d",side,active,recovery,steer);
     $display("PASS off-centre exit: one measured boundary and +/-80mm error reacquire correcting vision side=%0d",side);
    end else if(CASE==2)begin
     // Pose drift or target handoff invalidates the 100mm map corridor mid turn.
     yaw=side*450;trust=0;steps(45);
     if(blocked || !recovery || steer*side<10)$fatal(1,"CORNER_CORRIDOR_ABORT side=%0d",side);
     yaw=side*900;valid=1;visibility=7;near_error=side*40;steps(65);
     if(blocked || recovery || !active || steer*side>=0)$fatal(1,"IMU_EXIT_HANDOVER side=%0d",side);
     $display("PASS corner corridor: IMU completes latched outgoing yaw during map mismatch, then visible lane centres side=%0d",side);
    end
   end
  end
  $finish;
 end
 initial begin #5000000;$fatal(1,"authority timeout");end
endmodule
