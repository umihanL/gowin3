`timescale 1ns/1ps
module navigation_corner_tb;
 reg clk=0;always #5 clk=~clk;
 reg rst=0,tick=0,update=0,valid=1,moving=0,exit_valid=1;reg [2:0] visibility=7;
 reg signed [15:0] yaw=0,exit_heading=900;reg signed [1:0] planned=1;
 wire blocked,active;wire signed [15:0] steer;wire [2:0] state;
 vehicle_vision_fusion dut(.turn_entry_ready(1'b1),.mapped_entry_due(1'b0),.incoming_heading_error(16'sd0),.clk(clk),.rst_n(rst),.tick(tick),.enabled(1'b1),
 .planned_exit_valid(exit_valid),.planned_exit_heading(exit_heading),
 .road_update(update),.road_valid(valid),.road_junction(1'b0),.road_visibility(visibility),
 .terminal_approach(1'b0),.road_near(9'sd0),.road_heading(9'sd0),.route_steer(16'sd25),
 .pose_heading(yaw),.travel_q8(32'sd0),.moving(moving),.planned_turn(planned),
 .blocked(blocked),.active(active),.steering(steer),.guidance_state(state));
 integer n,side,start_yaw;
 function integer wrap(input integer a);begin wrap=a>1800?a-3600:a < -1800?a+3600:a;end endfunction
 task step;begin @(negedge clk);tick=1;update=1;@(negedge clk);tick=0;update=0;end endtask
 task reset_case(input integer origin,input integer direction,input integer extent);begin
  @(negedge clk);rst=0;moving=0;valid=1;visibility=7;yaw=origin;planned=direction;
  exit_valid=1;exit_heading=wrap(origin+direction*extent);
  repeat(5)@(negedge clk);rst=1;for(n=0;n<20;n=n+1)step;
  moving=1;valid=0;visibility=4;for(n=0;n<40;n=n+1)step;
  if(!dut.blind_turn || blocked)$fatal(1,"turn not committed");
 end endtask
 initial begin
 for(side=-1;side<=1;side=side+2)begin
  start_yaw=side==1?1700:-1700;reset_case(start_yaw,side,900);
  yaw=wrap(start_yaw+side*300);valid=1;visibility=7;
  for(n=0;n<20;n=n+1)step;
  if(!dut.blind_turn || active || blocked)$fatal(1,"EARLY_RELEASE_30_DEGREES");
  exit_heading=wrap(start_yaw-side*900);planned=-side;
  valid=0;visibility=4;for(n=0;n<100;n=n+1)step;
  if(!dut.blind_turn || blocked || dut.loss_ticks!=0)$fatal(1,"camera gap aborted committed turn");
  yaw=wrap(start_yaw+side*850);valid=1;visibility=7;for(n=0;n<15;n=n+1)step;
  if(!dut.blind_turn || blocked)$fatal(1,"turn handed back before outgoing heading");
  yaw=wrap(start_yaw+side*900);for(n=0;n<40;n=n+1)step;
  if(dut.blind_turn || !active || blocked || $abs(steer)>1)$fatal(1,"90deg exit did not return to visual centre");
  valid=0;visibility=0;for(n=0;n<25;n=n+1)step;
  if(blocked)$fatal(1,"pre-turn loss carried over after release");
  $display("PASS committed quarter turn side=%0d: early false lane, target latch, transient loss, heading wrap and clean handback",side);
 end
 reset_case(0,1,450);yaw=450;valid=1;visibility=7;for(n=0;n<40;n=n+1)step;
 if(dut.blind_turn || !active || blocked)$fatal(1,"S lobe was forced to 90deg");
 $display("PASS S geometry: planned 45deg exit remains 45deg");
 reset_case(0,1,900);yaw=-160;for(n=0;n<2;n=n+1)step;
 if(!blocked)$fatal(1,"wrong-way IMU yaw escaped watchdog");
 reset_case(0,1,900);yaw=900;valid=0;visibility=4;
 for(n=0;n<120;n=n+1)step;
 if(blocked || !dut.blind_turn || steer!=0)$fatal(1,"missing exit did not hold heading");
 for(n=0;n<3100 && !blocked;n=n+1)step;
 if(!blocked)$fatal(1,"unseen exit permitted unbounded navigation");
 $display("PASS bounded fallback: heading hold, wrong-way rejection and unseen-exit watchdog");$finish;
 end
 initial begin #10000000;$fatal(1,"corner regression timeout");end
endmodule
