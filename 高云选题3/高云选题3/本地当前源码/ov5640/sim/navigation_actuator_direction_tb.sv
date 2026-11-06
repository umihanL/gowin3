`timescale 1ns/1ps
// Production navigation output through the unchanged subsystem's slew,
// measured left/right calibration ROM and actual servo PWM output pin.
module navigation_actuator_direction_tb;
 reg clk=0;always #5 clk=~clk;
 reg rst=0,tick=0;integer clocks=0;
 reg due=0;reg signed [1:0] turn=0;reg signed [15:0] exit_yaw=0;
 wire signed [15:0] steer,command;wire pulse;
 tri1 sda,scl;
 always @(negedge clk)begin
  if(!rst)begin clocks=0;tick=0;end
  else if(clocks==9999)begin clocks=0;tick=1;end
  else begin clocks=clocks+1;tick=0;end
 end
 vehicle_vision_fusion fusion(.clk(clk),.rst_n(rst),.tick(tick),.enabled(1'b1),
  .road_update(tick),.road_valid(1'b1),.road_junction(1'b0),.road_visibility(3'b111),
  .road_near(9'sd0),.road_heading(9'sd0),.terminal_approach(1'b0),
  .route_steer(-16'sd28*turn),.pose_heading(16'sd0),.travel_q8(32'sd0),.moving(1'b1),
  .planned_turn(turn),.planned_exit_valid(1'b1),.planned_exit_heading(exit_yaw),
  .route_guidance_valid(1'b1),.route_target_behind(1'b0),.turn_entry_ready(due),
  .incoming_heading_error(16'sd0),.incoming_lateral_error(16'sd0),.mapped_entry_due(due),
  .steering(steer));
 vehicle_subsystem #(.CLK_HZ(1000000)) car(.clk(clk),.rst_n(rst),.mode(4'd2),
  .navigation_request(1'b0),.route_valid(1'b0),.route_count(12'd0),
  .route_x_q8(17'd0),.route_y_q8(17'd0),.route_lock(1'b0),
  .control_remote(1'b0),.control_host(1'b0),.host_rx(1'b1),
  .configured_speed(16'd100),.slowdown_cm(16'd65),.bypass_cm(16'd65),.brake_speed(16'd100),
  .touch_run(1'b0),.touch_speed(16'sd0),.touch_steer(steer),
  .road_update(1'b0),.road_valid(1'b0),.road_junction(1'b0),.road_visibility(3'b000),
  .road_near(9'sd0),.road_heading(9'sd0),.road_fit_valid(1'b0),
  .road_fit_forward0(10'd0),.road_fit_forward1(10'd0),.road_fit_error0(9'sd0),.road_fit_error1(9'sd0),
  .zebra_update(1'b0),.zebra_detected(1'b0),.imu_available(1'b1),.imu_update(1'b1),
  .imu_restart(1'b0),.imu_yaw(16'sd0),.encoder_A(1'b0),.encoder_B(1'b0),
  .encoder2_A(1'b0),.encoder2_B(1'b0),.ps2_miso(1'b1),.tof_sda(sda),.tof_scl(scl),
  .commanded_steer(command),.servo_pwm_out(pulse));
 integer side,width;
 task measure;begin
  @(posedge pulse);width=0;
  while(pulse)begin @(negedge clk);if(pulse)width=width+1;end
 end endtask
 initial begin
  for(side=-1;side<=1;side=side+2)begin
   rst=0;due=0;turn=0;exit_yaw=0;repeat(5)@(negedge clk);rst=1;
   repeat(60000)@(negedge clk);measure;
   if(width<1499 || width>1501)$fatal(1,"servo neutral pulse=%0d",width);
   turn=side;exit_yaw=side*900;due=1;
   repeat(800000)@(negedge clk);measure;
   if(steer!=side*28 || command!=side*28 ||
      (side==1 && (width<2165 || width>2168)) ||
      (side==-1 && (width<942 || width>946)))
    $fatal(1,"SERVO_DIRECTION side=%0d nav=%0d command=%0d pulseUs=%0d",side,steer,command,width);
   $display("PASS navigation-to-servo pin: side=%0d steer=%0d measuredPulseUs=%0d",side,steer,width);
  end
  $finish;
 end
 initial begin #30000000;$fatal(1,"actuator timeout");end
endmodule
