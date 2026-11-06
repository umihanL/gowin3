`timescale 1ns/1ps
module imu_epoch_motion_tb;
 reg clk=0;always #5 clk=~clk;reg rst=0;reg [3:0] mode=0;
 reg available=1,update=1,restart=0,rezero=0;reg signed [15:0] yaw=375;
 wire drive;wire signed [15:0] speed,heading;wire [3:0] status;
 vehicle_motion #(.CLK_HZ(20000),.START_DELAY_MS(0),.STALL_MS(3000)) dut(
 .clk(clk),.rst_n(rst),.mode(mode),.navigation_request(1'b0),.navigation_avoidance(1'b0),
 .configured_speed(16'd200),.slowdown_cm(16'd65),.bypass_cm(16'd65),.brake_speed(16'd200),
 .imu_available(available),.imu_update(update),.imu_restart(restart),.imu_rezero(rezero),.imu_yaw(yaw),
 .encoder1(32'sd0),.encoder2(32'sd0),.host_pause(1'b0),
 .range_frame(1'b1),.range_samples(4'hf),.range_observations(4'hf),.range_diagnostics(32'd0),
 .left_mm(16'd1500),.left_front_mm(16'd1500),.right_front_mm(16'd1500),.right_mm(16'd1500),
 .drive_enabled(drive),.speed_mm_s(speed),.pose_heading(heading),.status(status));
 initial begin
  repeat(5)@(negedge clk);rst=1;repeat(1000)@(negedge clk);mode=7;
  repeat(2000)@(negedge clk);
  if(!drive || status!=2 || speed<=0)$fatal(1,"mode7 fixture did not drive status=%d",status);
  @(negedge clk);update=0;available=0;restart=1;
  @(negedge clk);restart=0;
  if(dut.imu_pending || !dut.imu_epoch_pending || dut.imu_age==65535)$fatal(1,"transport restart lost previous timeout policy or stale sample consumed");
  repeat(1000)@(negedge clk);
  if(!drive || status!=2)$fatal(1,"short transport reset stopped non-navigation mode");
  yaw=0;available=1;update=1;repeat(1000)@(negedge clk);
  if(heading>2 || heading < -2 || !drive)$fatal(1,"new sensor epoch changed map direction: %d",heading);
  @(negedge clk);rezero=1;@(negedge clk);rezero=0;update=0;available=0;
  if(drive || speed || dut.imu_age!=65535 || dut.imu_pending)$fatal(1,"manual zero failed to stop and reject old epoch");
  $display("PASS mode7 transport recovery retains timeout and heading; manual zero stops and discards stale epoch");$finish;
 end
 initial begin #2000000;$fatal(1,"epoch timeout");end
endmodule
