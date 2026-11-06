`timescale 1ns/1ps
// Public-interface regression: complete frames, trusted map and IMU feedback.
module navigation_handoff_tb;
 reg clk=0;always #5 clk=~clk;
 reg rst=0,tick=0,update=0,valid=1,moving=0,send_frames=1,trust=1;
 reg [2:0] visibility=7;
 reg signed [15:0] yaw=0,route=0,exit_heading=900;
 reg signed [8:0] near_error=0,heading_error=0;
 reg signed [31:0] travel=0;reg signed [1:0] planned=0;
 wire blocked,active,recovery;wire signed [15:0] steer;wire [2:0] state;
 vehicle_vision_fusion dut(.turn_entry_ready(1'b1),.mapped_entry_due(1'b0),.incoming_heading_error(16'sd0),.clk(clk),.rst_n(rst),.tick(tick),.enabled(1'b1),
 .planned_exit_valid(1'b1),.planned_exit_heading(exit_heading),.planned_turn(planned),
 .route_guidance_valid(trust),.recovery_active(recovery),
 .road_update(update),.road_valid(valid),.road_junction(1'b0),.road_visibility(visibility),
 .terminal_approach(1'b0),.road_near(near_error),.road_heading(heading_error),
 .route_steer(route),.pose_heading(yaw),.travel_q8(travel),.moving(moving),
 .blocked(blocked),.active(active),.steering(steer),.guidance_state(state));
 integer n,clock_ticks=0,edge_mode;
 task step;begin
  @(negedge clk);tick=1;update=send_frames && clock_ticks%3==0;
  clock_ticks=clock_ticks+1;@(negedge clk);tick=0;update=0;
 end endtask
 task wait_ticks(input integer count);integer k;begin for(k=0;k<count;k=k+1)step;end endtask
 task reset_case;begin
  @(negedge clk);rst=0;moving=0;valid=1;visibility=7;send_frames=1;trust=1;
  yaw=0;route=0;travel=0;planned=0;exit_heading=900;clock_ticks=0;near_error=0;heading_error=0;
  repeat(4)@(negedge clk);rst=1;wait_ticks(30);moving=1;
 end endtask
 task expect_recovery;begin
  wait_ticks(40);if(blocked || !recovery || steer<20)
   $fatal(1,"MAP_HANDOFF_FAILED recovery=%b steer=%0d state=%0d",recovery,steer,state);
 end endtask
 initial begin
  // No profile curve tag yet: usable world segment already supplies pursuit.
  for(edge_mode=0;edge_mode<3;edge_mode=edge_mode+1)begin
   reset_case;valid=0;visibility=edge_mode==0?4:edge_mode==1?6:7;route=28;
   expect_recovery;wait_ticks(100);
   if(blocked || !recovery)$fatal(1,"persistent fragments aborted recovery");
   $display("PASS persistent fragments mode=%0d: invalid centre uses trusted route before curve tag",edge_mode);
  end
  reset_case;planned=1;route=28;wait_ticks(70);
  if(blocked || $abs(steer)>1 || recovery)$fatal(1,"future route curve overrode visible incoming lane");
  near_error=-40;heading_error=-20;wait_ticks(70);
  if(blocked || steer<20 || !recovery)$fatal(1,"matching near bend did not confirm mapped turn");
  yaw=900;planned=0;route=0;near_error=0;heading_error=0;wait_ticks(130);
  if(blocked || !active || $abs(steer)>1)$fatal(1,"curve authority did not return to visual lane");
  $display("PASS confirmed route turn: incoming straight stays visual; matching near bend commits; outgoing vision resumes");
  reset_case;valid=0;visibility=6;planned=1;route=28;expect_recovery;
  yaw=300;valid=1;visibility=7;wait_ticks(50);
  if(blocked || !recovery || active)$fatal(1,"false lane released at 30deg");
  yaw=900;route=0;planned=0;wait_ticks(70);
  if(blocked || recovery || !active || $abs(steer)>1)$fatal(1,"outgoing lane did not reacquire");
  $display("PASS visual handback: outgoing heading required, then visual centring resumes");
  reset_case;valid=0;visibility=7;planned=1;route=28;exit_heading=450;expect_recovery;
  yaw=400;planned=-1;route=-28;exit_heading=0;wait_ticks(80);
  if(blocked || !recovery || steer > -20)$fatal(1,"S lobe did not reverse steering");
  yaw=0;valid=1;visibility=7;planned=0;route=0;wait_ticks(70);
  if(blocked || recovery || !active)$fatal(1,"S outgoing lane not reacquired");
  $display("PASS S reversal: follow actual second exit and recover vision without forced 90deg");
  reset_case;valid=0;visibility=6;route=28;expect_recovery;
  trust=0;wait_ticks(10);if(blocked)$fatal(1,"transient segment handoff stopped navigation");
  trust=1;wait_ticks(10);if(blocked)$fatal(1,"corridor did not recover");
  trust=0;wait_ticks(25);if(!blocked)$fatal(1,"invalid corridor allowed unbounded motion");
  $display("PASS map corridor: short handoff tolerated; persistent invalid guidance stops");
  reset_case;valid=0;visibility=6;route=28;expect_recovery;
  send_frames=0;wait_ticks(25);if(!blocked)$fatal(1,"camera disconnect escaped watchdog");
  reset_case;valid=0;visibility=6;route=28;expect_recovery;
  visibility=2;wait_ticks(25);if(!blocked)$fatal(1,"incomplete frames escaped watchdog");
  $display("PASS camera freshness: disconnected and incomplete streams stop bounded recovery");
  reset_case;valid=0;visibility=6;route=28;expect_recovery;
  travel=-31*256;wait_ticks(2);if(!blocked)$fatal(1,"reverse travel escaped watchdog");
  reset_case;valid=0;visibility=6;route=28;expect_recovery;
  travel=2401*256;wait_ticks(2);if(!blocked)$fatal(1,"travel escaped watchdog");
  reset_case;valid=0;visibility=6;route=28;expect_recovery;
  wait_ticks(6010);if(!blocked)$fatal(1,"time escaped watchdog");
  reset_case;trust=0;planned=0;valid=0;visibility=7;wait_ticks(80);
  if(!blocked)$fatal(1,"untrusted straight vision loss did not stop");
  $display("PASS bounded recovery: reverse, travel, time and untrusted guidance watchdogs");
  $finish;
 end
 initial begin #10000000;$fatal(1,"handoff timeout");end
endmodule
