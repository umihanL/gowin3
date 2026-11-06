`timescale 1ns/1ps
// Both reported branches, delayed entry acquisition, false opposite lane,
// and early preview of the next S lobe. Only public control inputs are used.
module navigation_turn_ownership_tb;
 reg clk=0;always #5 clk=~clk;
 reg rst=0,tick=0,update=0,moving=0,entry=0,due=0;
 reg signed [1:0] planned=0;
 reg signed [15:0] yaw=0,exit_yaw=0,route=0,incoming=0;
 reg signed [8:0] lane=0,heading=0;
 wire blocked,recovery,ready;wire signed [15:0] steer;
 vehicle_vision_fusion dut(.clk(clk),.rst_n(rst),.tick(tick),.enabled(1'b1),
  .road_update(update),.road_valid(1'b1),.road_junction(1'b1),.road_visibility(3'b111),
  .road_near(lane),.road_heading(heading),.terminal_approach(1'b0),
  .moving(moving),.pose_heading(yaw),.travel_q8(32'sd0),.route_steer(route),
  .planned_turn(planned),.planned_exit_valid(1'b1),.planned_exit_heading(exit_yaw),
  .route_guidance_valid(1'b1),.route_target_behind(1'b0),.turn_entry_ready(entry),
  .incoming_heading_error(incoming),.mapped_entry_due(due),
  .blocked(blocked),.recovery_active(recovery),.start_ready(ready),.steering(steer));
 task step;begin @(negedge clk);tick=1;update=1;@(negedge clk);tick=0;update=0;end endtask
 integer side,delay_yaw,k;
 initial begin
  for(side=-1;side<=1;side=side+2)for(delay_yaw=0;delay_yaw<=300;delay_yaw=delay_yaw+150)begin
   rst=0;moving=0;entry=0;due=0;planned=0;yaw=0;route=0;incoming=0;lane=0;heading=0;
   exit_yaw=side*900;repeat(4)@(negedge clk);rst=1;repeat(20)step;
   if(!ready)$fatal(1,"complete valid lane failed to acquire");
   moving=1;planned=side;route=-side*28;
   repeat(20)step;
   if(recovery || $abs(steer)>1)$fatal(1,"unreached curve turned early");
   // Entry can be acquired late after image/RAM latency. It is still a
   // 90-degree planned corner, rather than a 60-degree S lobe.
   yaw=side*delay_yaw;incoming=-yaw;entry=1;due=1;
   lane=side*80;heading=side*30;repeat(35)step;
   if(blocked || !recovery || steer*side<15)
    $fatal(1,"opposite branch commanded side=%0d delayedYaw=%0d steer=%0d",side,delay_yaw,steer);
   due=0;entry=0;planned=-side;exit_yaw=0;route=-side*28;
   // Next metadata is a preview; it cannot reverse the current corner.
   repeat(40)step;
   if(blocked || steer*side<10)$fatal(1,"future S lobe reversed unfinished corner side=%0d",side);
   yaw=side*600;repeat(35)step;
   if(blocked || steer*side<10)$fatal(1,"corner abandoned before outgoing yaw");
   // At the NEXT entry, even a short S join can legitimately reverse.
   yaw=side*760;incoming=side*140;entry=1;due=1;repeat(40)step;
   if(blocked || steer*side> -10)$fatal(1,"next physical entry failed to reverse");
   $display("PASS reported branch side=%0d late acquisition=%0d: no early turn, no opposite corner, next reached entry releases",side,delay_yaw);
  end
  $finish;
 end
 initial begin #5000000;$fatal(1,"ownership timeout");end
endmodule
