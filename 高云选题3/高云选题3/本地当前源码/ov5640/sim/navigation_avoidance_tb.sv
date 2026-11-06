`timescale 1ns/1ps
// Optional map detours use production range memory, bypass and route tracker.
module navigation_avoidance_tb;
 reg clk=0;always #5 clk=~clk;
 reg rst=0,avoidance=1,pause=0,imu=1;
 reg [3:0] mode=0;
 reg signed [31:0] e1=0,e2=0;reg signed [15:0] yaw=0;
 reg [15:0] left=1500,right=1500,front=1500;
 reg [16:0] rx,ry;wire [10:0] index;
 reg [11:0] route_count=2;
 wire tick,drive,brake;wire signed [15:0] speed,steer;
 wire [3:0] status,phase;
 always @(posedge clk)begin rx<=(index==0?500:route_count==3 && index==1?1000:2500)*256/10;ry<=2700*256/10;end
 vehicle_motion #(.CLK_HZ(20000),.START_DELAY_MS(0)) dut(
  .clk(clk),.rst_n(rst),.mode(mode),.navigation_avoidance(avoidance),
  .navigation_request(mode==9),.route_valid(1'b1),.profile_ready(1'b1),
  .route_start_heading_valid(1'b1),.route_start_heading(16'sd0),
  .route_count(route_count),.route_index(index),.route_x_q8(rx),.route_y_q8(ry),
  .route_lock(1'b1),.route_curve(2'sd0),.configured_speed(16'd160),
  .slowdown_cm(16'd65),.bypass_cm(16'd65),.brake_speed(16'd200),
  .encoder1(e1),.encoder2(e2),.imu_available(imu),.imu_update(imu),.imu_yaw(yaw),.imu_restart(1'b0),
  .range_frame(1'b1),.range_samples(4'hf),.range_observations(4'hf),.range_diagnostics(32'd0),
  .left_mm(left),.right_mm(right),.left_front_mm(front),.right_front_mm(front),
  .remote_link(1'b0),.remote_run(1'b0),.remote_speed(16'sd0),.remote_steer(16'sd0),.host_pause(pause),
  .zebra_update(1'b1),.zebra_detected(1'b1),
  .drive_enabled(drive),.brake_enable(brake),.speed_mm_s(speed),.steer_deg(steer),
  .status(status),.phase(phase),.tick_10ms(tick));
 real travel=0;integer ticks=0,side;
 always @(negedge clk)if(rst && tick)begin
  ticks=ticks+1;if(drive)travel=travel+speed*0.01;
  e1=$rtoi(travel*1560/(66*3.141592654));e2=e1;
 end
 task wait_ticks(input integer n);integer goal;begin
  goal=ticks+n;wait(ticks>=goal);@(negedge clk);
 end endtask
 task start_run(input integer avoid);begin
  rst=0;mode=0;avoidance=avoid;pause=0;imu=1;travel=0;e1=0;e2=0;yaw=0;
  left=1500;right=1500;front=1500;
  repeat(8)@(negedge clk);rst=1;wait_ticks(3);mode=9;wait_ticks(100);
  if(!drive || status!=2 || speed!=160 || dut.avoidance_selected!=avoid)
   $fatal(1,"map startup failed avoid=%0d status=%0d speed=%0d",avoid,status,speed);
 end endtask
 initial begin
  start_run(0);avoidance=1;front=40;wait_ticks(30);
  if(dut.avoidance_selected || !drive || speed!=160 || dut.bypass_active || status!=2)
   $fatal(1,"OFF changed during active run");
  mode=0;front=1500;wait_ticks(3);mode=9;wait_ticks(90);front=500;wait_ticks(30);
  if(!dut.avoidance_selected || !dut.bypass_active || steer>=0 || dut.bypass_choose_left)
   $fatal(1,"next entry ON did not choose right bypass");
  avoidance=0;wait_ticks(4);
  if(!dut.avoidance_selected || !dut.bypass_active)$fatal(1,"live switch cancelled latched bypass");
  pause=1;wait_ticks(3);if(drive || speed)$fatal(1,"host pause ignored during bypass");
  pause=0;wait_ticks(10);if(!drive || !dut.bypass_active)$fatal(1,"host resume lost bypass");
  $display("PASS map avoidance entry latch, next-run switch, right bypass and host pause/resume");
  start_run(1);left=2000;right=300;wait_ticks(5);front=500;wait_ticks(30);
  if(!dut.bypass_active || !dut.bypass_choose_left || steer<=0)$fatal(1,"ON left side selection failed");
  mode=0;wait_ticks(3);if(drive || speed || dut.bypass_active)$fatal(1,"exit did not cancel bypass");
  $display("PASS ON left bypass and task-exit cancellation");
  start_run(1);left=100;right=100;wait_ticks(5);front=500;wait_ticks(110);
  if(status!=5 || drive || speed || !brake || dut.bypass_active)
   $fatal(1,"no-clear-side did not stop blocked status=%0d phase=%0d",status,phase);
  front=1500;left=1500;right=1500;wait_ticks(20);
  if(drive || speed || status!=5)$fatal(1,"blocked task restarted without user re-entry");
  start_run(1);front=40;wait_ticks(4);
  if(status!=5 || drive || speed || !brake)$fatal(1,"ON immediate obstacle stop removed");
  $display("PASS ON blocked/no-space and near-range stop, no automatic restart");
  for(side=-1;side<=1;side=side+2)begin
   start_run(1);yaw=-side*60;wait_ticks(35);
   if(side==1)left=40;else right=40;
   wait_ticks(5);
   if(dut.route_candidate!==dut.tracker_steer || side*steer<=0 || dut.bypass_active)
    $fatal(1,"ON side ranging replaced map tracker steering");
  end
  $display("PASS ON retains inertial steering authority; camera zebra remains non-authoritative");
  // Isolate recovery orientation: map goal remains ahead of its segment
  // plane even when the nozzle/car heading has rotated 180 degrees. Such a
  // body-frame negative forward distance cannot finish or skip a waypoint.
  start_run(1);
  force dut.nav_resume=1'b1;force dut.pose_heading=16'sd1800;
  force dut.pose_x_q8=24'sd204800;force dut.pose_y_q8=24'sd128000;
  wait_ticks(10);
  if(status!=2 || !drive || dut.forward_mm>=0 || dut.goal_plane_reached)
   $fatal(1,"rejoin confused body-behind target with terminal map arrival");
  release dut.nav_resume;release dut.pose_heading;release dut.pose_x_q8;release dut.pose_y_q8;
  route_count=3;start_run(1);
  force dut.nav_resume=1'b1;force dut.pose_heading=16'sd1800;
  force dut.pose_x_q8=24'sd204800;force dut.pose_y_q8=24'sd128000;
  wait_ticks(10);
  if(status!=2 || !drive || index!=1 || dut.forward_mm>=0 || dut.goal_plane_reached)
   $fatal(1,"rejoin skipped a not-yet-passed intermediate map waypoint");
  release dut.nav_resume;release dut.pose_heading;release dut.pose_x_q8;release dut.pose_y_q8;
  $display("PASS rejoin uses map-plane arrival rather than temporary body orientation");
  $finish;
 end
 initial begin #40000000;$fatal(1,"optional avoidance regression timeout");end
endmodule
