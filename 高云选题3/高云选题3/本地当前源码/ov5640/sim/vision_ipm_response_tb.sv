`timescale 1ns/1ps
module vision_ipm_response_tb;
 reg clk=0;always #5 clk=~clk;
 reg rst=0,tick=0,update=0;
 reg signed [8:0] near_error=0,heading=0;
 wire active,blocked;wire signed [15:0] steer,align_yaw,align_offset;
 vehicle_vision_fusion dut(.turn_entry_ready(1'b1),.mapped_entry_due(1'b0),.incoming_heading_error(16'sd0),.clk(clk),.rst_n(rst),.tick(tick),.enabled(1'b1),
 .road_update(update),.road_valid(1'b1),.road_junction(1'b0),.road_visibility(3'b111),
 .terminal_approach(1'b0),.road_near(near_error),.road_heading(heading),
 .route_steer(16'sd0),.pose_heading(16'sd0),.travel_q8(32'sd0),.moving(1'b0),
 .planned_turn(2'sd0),.planned_exit_valid(1'b0),.planned_exit_heading(16'sd0),
 .route_guidance_valid(1'b0),.steering(steer),.active(active),.blocked(blocked),
 .alignment_yaw(align_yaw),.alignment_offset(align_offset));
 task step;begin @(negedge clk);tick=1;@(negedge clk);tick=0;end endtask
 task reset_case(input integer lateral,input integer trend);begin
  @(negedge clk);rst=0;near_error=lateral;heading=trend;update=0;
  repeat(4)@(negedge clk);rst=1;
  @(negedge clk);update=1;@(negedge clk);update=0;
 end endtask
 integer side,n;
 initial begin
  for(side=-1;side<=1;side=side+2)begin
   reset_case(side*5,0);step;
   if(!active || blocked)$fatal(1,"first good frame did not acquire vision");
   step;step;
   if(steer!=-side)$fatal(1,"5mm offset did not steer within 30ms steer=%0d",steer);
   // Pure yaw must not be interpreted as a lateral map-origin offset.
   reset_case(side*21,side*11);
   if(align_yaw*side<48 || align_yaw*side>60 || align_offset>1 || align_offset < -1)
    $fatal(1,"IPM startup frame origin error yaw=%0d offset=%0d",align_yaw,align_offset);
   for(n=0;n<20;n=n+1)step;
   if(steer*side>=0 || steer>6 || steer < -6)$fatal(1,"yaw response direction/amplitude");
  end
  $display("PASS IPM response: first-frame acquisition, +/-5mm correction within 30ms, pure-yaw startup origin and bounded steering");$finish;
 end
endmodule
