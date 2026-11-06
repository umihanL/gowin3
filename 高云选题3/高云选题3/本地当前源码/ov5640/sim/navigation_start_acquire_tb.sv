`timescale 1ns/1ps
// Camera noise/isolated rejected frames must not restart the launch handshake.
module navigation_start_acquire_tb;
 reg clk=0;always #5 clk=~clk;
 reg rst=0,tick=0,update=0,valid=0,enabled=1;
 reg signed [8:0] near_error=0,heading=0;
 wire ready,active,blocked;wire [2:0] state;
 vehicle_vision_fusion dut(.clk(clk),.rst_n(rst),.tick(tick),.enabled(enabled),
 .road_update(update),.road_valid(valid),.road_junction(1'b0),.road_visibility(3'b111),
 .terminal_approach(1'b0),.road_near(near_error),.road_heading(heading),
 .route_steer(16'sd0),.pose_heading(16'sd0),.travel_q8(32'sd0),.moving(1'b0),
 .planned_turn(2'sd0),.planned_exit_valid(1'b0),.planned_exit_heading(16'sd0),
 .route_guidance_valid(1'b1),.route_target_behind(1'b0),.turn_entry_ready(1'b1),
 .incoming_heading_error(16'sd0),.incoming_lateral_error(16'sd0),.mapped_entry_due(1'b0),
 .start_ready(ready),.active(active),.blocked(blocked),.guidance_state(state));
 integer i;
 task step(input bit frame);begin
  @(negedge clk);tick=1;update=frame;@(negedge clk);tick=0;update=0;
 end endtask
 initial begin
  repeat(5)@(negedge clk);rst=1;
  repeat(60)step(1);
  if(ready || active)$fatal(1,"absent road authorized launch");
  // Plausible +/-7mm image variation: the old consecutive-delta test never acquires.
  valid=1;
  for(i=0;i<24;i=i+1)begin near_error=i%2?7:-7;heading=i%2?5:-5;step(1);end
  if(!ready || !active)$fatal(1,"START_NOISE_WAIT: usable lane never authorizes start");
  // A rejected frame must retain both readiness and the previous short-term track.
  valid=0;step(1);step(0);
  if(!ready || !active || state==4)$fatal(1,"START_FRAME_WAIT: one rejection restarted launch");
  // Complete camera loss still withdraws permission; a timeout cannot invent a road.
  repeat(24)step(0);
  if(ready || active)$fatal(1,"stale road authorized launch");
  enabled=0;step(0);enabled=1;valid=1;near_error=0;heading=0;
  repeat(15)step(1);
  if(!ready || !active || blocked)$fatal(1,"new request failed to reacquire");
  $display("PASS noisy launch, rejected frame continuity, stale-camera interlock and reacquisition");$finish;
 end
endmodule
