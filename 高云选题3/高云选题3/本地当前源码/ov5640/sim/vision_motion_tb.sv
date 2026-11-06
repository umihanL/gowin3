`timescale 1ns/1ps
// Independent bicycle/road projection through the actual navigation arbiter.
module vision_motion_tb;
 parameter INITIAL_YAW_TENTHS=0;
 reg clk=0;always #5 clk=~clk;
 reg rst=0,request=0;reg [3:0] mode=0;wire tick,drive;
 wire [10:0] index;reg [16:0] rx=0,ry=0;
 reg signed [31:0] e1=0,e2=0;reg signed [15:0] yaw=0;
 wire signed [15:0] speed,steer;wire [3:0] status;
 reg update=0,valid=1;reg signed [8:0] near_error=0,road_heading=0;
 reg stop_frames=0,close_side=0;
 reg zebra=0;
 always @(posedge clk)begin rx<=(500+20*index)*256/10;ry<=2700*256/10;end
 vehicle_motion #(.CLK_HZ(20000),.STALL_MS(3000)) dut(.route_exit_valid(1'b0),.route_exit_heading(16'sd0),.route_curve(2'sd0),.route_upcoming_turn(2'sd0),.route_entry_mm(8'd0),.route_tangent(16'sd0),.imu_restart(1'b0),
  .clk(clk),.rst_n(rst),.mode(mode),.navigation_request(request),.route_valid(1'b1),.profile_ready(1'b1),
  .configured_speed(16'd200),.slowdown_cm(16'd65),.bypass_cm(16'd65),.brake_speed(16'd200),
  .route_count(12'd201),.route_index(index),.route_x_q8(rx),.route_y_q8(ry),.route_lock(index==0 || index==200),
  .encoder1(e1),.encoder2(e2),.imu_available(1'b1),.imu_update(1'b1),.imu_yaw(yaw),
  .range_frame(1'b1),.range_samples(4'hf),.range_observations(4'hf),.range_diagnostics(32'd0),
  .left_mm(close_side?16'd70:16'd1200),.right_mm(16'd1200),.left_front_mm(16'd1500),.right_front_mm(16'd1500),
  .remote_link(1'b0),.remote_run(1'b0),.remote_speed(16'sd0),.remote_steer(16'sd0),
  .road_visibility({1'b1,{2{valid}}}),.road_update(update),.road_valid(valid),.road_junction(1'b0),.road_near(near_error),.road_heading(road_heading),
  .zebra_update(update),.zebra_detected(zebra),
  .drive_enabled(drive),.speed_mm_s(speed),.steer_deg(steer),.status(status),.tick_10ms(tick));
 real wx=281,wy=500,angle=INITIAL_YAW_TENTHS*3.141592654/1800.0,w1=0,w2=0,ds,kappa,previous_angle,noise;
 integer applied=0,ticks=0,frames=0,max_steer=0,startup_command,startup_delta;reg enabled_seen=0,first_drive_checked=0;
 always @(negedge clk)if(rst && tick)begin
  ticks=ticks+1;
  if(drive && !first_drive_checked)begin
   first_drive_checked=1;
   startup_command=steer;
   startup_delta=applied-startup_command;
   if(ticks>15)$fatal(1,"navigation retained visual startup interlock");
   $display("PASS immediate startup: initial yaw=%0d first driven tick=%0d",INITIAL_YAW_TENTHS,ticks);
  end
  if(applied<steer)applied=applied+1;else if(applied>steer)applied=applied-1;
  kappa=$tan(applied*0.01745329252)/142.0;
  ds=drive?speed*0.01/(1+84*$abs(kappa)):0;previous_angle=angle;angle=angle+ds*kappa;
  wx=wx+ds*$cos((angle+previous_angle)/2);wy=wy+ds*$sin((angle+previous_angle)/2);
  w1=w1+ds*(1-84*kappa);w2=w2+ds*(1+84*kappa);
  e1=$rtoi(w1*1560/(66*3.141592654));e2=$rtoi(w2*1560/(66*3.141592654));
  yaw=$rtoi(angle*1800/3.141592654);
  if(ticks%3==0 && !stop_frames)begin
   // Physical road is 30mm left of finger-selected map line; 1px noise.
   noise=frames%2==0?1:-1;frames=frames+1;
   near_error=$rtoi(((wy-530)+210*angle+noise*2));road_heading=$rtoi(110*angle);update=1;
  end
  if(dut.vision_guidance.active)enabled_seen=1;
  if($abs(steer)>max_steer)max_steer=$abs(steer);
 end else update=0;
 task wait_ticks(input integer n);integer t;begin t=ticks+n;wait(ticks>=t);@(negedge clk);end endtask
 initial begin
  repeat(5)@(negedge clk);rst=1;wait_ticks(5);stop_frames=1;mode=9;request=1;
  wait_ticks(250);
  if(!drive || status!=2 || speed!=80)$fatal(1,"navigation did not start without road observations");
  stop_frames=0;
  wait_ticks(650);
  $display("INFO skew run: initialYaw=%0d pose=%0d,%0d poseYaw=%0d initHeading=%0d target=%0d,%0d distance=%0d forward=%0d lateral=%0d index=%0d physical=%f,%f",INITIAL_YAW_TENTHS,dut.pose_x_q8>>>8,dut.pose_y_q8>>>8,dut.pose_heading,dut.initial_heading,dut.target_x_mm,dut.target_y_mm,dut.distance_mm,dut.forward_mm,dut.lateral_mm,index,wx,wy);
  // With no stationary pose alignment and 2.5s without images, an initial
  // 8-degree skew needs more correction than the old pre-aligned fixture.
  if(status!=2 || !enabled_seen || wy<510 || wy>545 || $abs(angle)>0.10 || max_steer>16)
   $fatal(1,"navigation/vision integration status=%0d y=%f yaw=%f active=%b peak=%0d",status,wy,angle,enabled_seen,max_steer);
  zebra=1;wait_ticks(100);
  if(!dut.zebra_latched || speed>100 || speed<95 || status!=2)
   $fatal(1,"zebra did not reduce navigation speed: %0d",speed);
  zebra=0;wait_ticks(30);
  if(!dut.zebra_latched || speed>100)$fatal(1,"zebra slowdown released before body cleared");
  wait_ticks(360);
  if(dut.zebra_latched || speed<=100 || status!=2)$fatal(1,"zebra cruise speed did not restore after tail clearance");
  $display("PASS zebra speed: cap at 100 mm/s, camera-to-tail clearance hold and automatic cruise restoration");
  stop_frames=1;wait_ticks(30);
  if(dut.vision_guidance.active || status!=2 || speed>=200 || dut.vision_speed_limit!=80 || dut.requested_speed>80)
   $fatal(1,"short straight visibility gap did not slow down %0d %0d",status,speed);
  stop_frames=0;wait_ticks(60);close_side=1;wait_ticks(3);
  if(!dut.vision_guidance.active)
   $fatal(1,"side laser unnecessarily disabled visible-road centering");
  stop_frames=1;wait_ticks(90);
  if(status!=2 || !drive || speed!=80)$fatal(1,"prolonged vision loss stopped inertial fallback");
  $display("PASS vision motion: immediate startup, independent encoder/IMU bicycle, noisy lane convergence, short loss/reacquire, prolonged-loss route fallback and visual authority beside a valid side echo");$finish;
 end
 initial begin #100000000;$fatal(1,"vision motion timeout");end
endmodule
