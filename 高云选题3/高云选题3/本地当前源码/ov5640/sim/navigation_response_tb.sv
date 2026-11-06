`timescale 1ns/1ps
module navigation_response_tb;
 reg clk=0;always #5 clk=~clk;
 reg rst=0,tick=0,enabled=1,update=0,valid=1,moving=0,terminal=0;
 reg [2:0] visibility=7;
 reg signed [8:0] near_error=0,trend=0;
 reg signed [1:0] planned=0;
 wire signed [15:0] steer;wire blocked,active;wire [2:0] state;
 vehicle_vision_fusion dut(.turn_entry_ready(1'b1),.mapped_entry_due(1'b0),.incoming_heading_error(16'sd0),.planned_exit_valid(1'b0),.planned_exit_heading(16'sd0),.clk(clk),.rst_n(rst),.tick(tick),.enabled(enabled),
  .road_update(update),.road_valid(valid),.road_junction(1'b0),
  .road_visibility(visibility),.terminal_approach(terminal),
  .road_near(near_error),.road_heading(trend),.route_steer(16'sd0),
  .pose_heading(16'sd0),.travel_q8(32'sd0),.moving(moving),.planned_turn(planned),
  .blocked(blocked),.steering(steer),.active(active),.guidance_state(state));
 integer n,side;
 task step(input bit frame);begin
  @(negedge clk);tick=1;update=frame;
  @(negedge clk);tick=0;update=0;
 end endtask
 task reset_case;begin
  @(negedge clk);rst=0;moving=0;valid=1;visibility=7;terminal=0;planned=0;near_error=0;trend=0;
  repeat(4)@(negedge clk);rst=1;
  for(n=0;n<20;n=n+1)step(n%3==0);
  if(!active)$fatal(1,"visible lane not acquired");
 end endtask
 initial begin
  for(side=-1;side<=1;side=side+2)begin
   reset_case;moving=1;planned=side;
   for(n=0;n<6;n=n+1)step(n%3==0);
   // Both edges leave between camera frames, with no single-edge frame.
   valid=0;visibility=4;
   step(1);step(0);
   if(dut.blind_turn)$fatal(1,"one empty frame invented full lock");
   step(1);for(n=0;n<36;n=n+1)step(n%3==0);
   if(!dut.blind_turn || blocked || steer!=side*28)
    $fatal(1,"skipped-single-edge bend failed side=%0d steer=%0d state=%0d",side,steer,state);
  end
  $display("PASS skipped edge phase: left/right planned bends survive direct complete image loss without false deviation stop");
  for(side=-1;side<=1;side=side+2)begin
   reset_case;moving=1;trend=-side*20;
   for(n=0;n<18;n=n+1)step(n%3==0);
   trend=0;valid=0;visibility=4;
   for(n=0;n<42;n=n+1)step(n%3==0);
   if(!dut.blind_turn || blocked || steer!=side*28)$fatal(1,"visual bend memory lost direction");
  end
  $display("PASS visual bend memory: measured heading retains direction through FOV loss when the route tag is delayed");
  reset_case;moving=1;planned=1;
  for(n=0;n<80;n=n+1)step(n%3==0);
  if(dut.blind_turn || steer!=0)$fatal(1,"map hint overrode visible straight centre");
  planned=0;for(n=0;n<130;n=n+1)step(n%3==0);
  valid=0;visibility=4;for(n=0;n<70;n=n+1)step(n%3==0);
  if(dut.blind_turn || !blocked)$fatal(1,"expired bend hint invented a turn");
  $display("PASS bend expiry: visible-road authority preserved; old curve hint cannot trigger a later blind turn");
  reset_case;moving=1;planned=1;for(n=0;n<6;n=n+1)step(n%3==0);
  valid=0;visibility=0;for(n=0;n<40;n=n+1)step(n%3==0);
  if(dut.blind_turn)$fatal(1,"dropped camera frames caused turn");
  $display("PASS navigation response: missing frame is not a bend; two complete lost-road frames are required");$finish;
 end
 initial begin #2000000;$fatal(1,"navigation response timeout");end
endmodule
