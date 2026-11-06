`timescale 1ns/1ps
// Real subsystem/profile/motion/host controller; pin-level UART commands.
module host_navigation_integration_tb;
 reg clk=0;always #5 clk=~clk;reg rst=0,host_rx=1;reg [3:0] mode=0;reg selected=1;
 wire host_tx;wire [10:0] index;wire profile_ready;reg [16:0] x=0,y=0;
 wire [3:0] status;wire signed [15:0] speed,steer;wire signed [23:0] px,py;
 wire [1:0] dir,dir2;wire pwm,pwm2;tri1 sda,scl;reg update=0;
 vehicle_subsystem #(.CLK_HZ(20000),.HOST_BAUD(2000)) dut(.clk(clk),.rst_n(rst),.mode(mode),
 .control_remote(1'b0),.control_host(selected),.host_rx(host_rx),.host_tx(host_tx),
 .configured_speed(16'd160),.slowdown_cm(16'd65),.bypass_cm(16'd65),.brake_speed(16'd200),
 .touch_run(1'b1),.touch_speed(16'sd800),.touch_steer(16'sd28),
 .navigation_request(mode==9),.route_valid(1'b1),.route_count(12'd3),.route_index(index),
 .route_x_q8(x),.route_y_q8(y),.route_lock(index==0 || index==2),
 .imu_available(1'b1),.imu_update(1'b1),.imu_restart(1'b0),.imu_yaw(16'sd0),
 .encoder_A(1'b0),.encoder_B(1'b0),.encoder2_A(1'b0),.encoder2_B(1'b0),
 .ps2_miso(1'b1),.tof_sda(sda),.tof_scl(scl),.dir(dir),.dir2(dir2),.pwm_out(pwm),.pwm_out2(pwm2),
 
 .zebra_update(1'b0),.zebra_detected(1'b0),.status(status),.commanded_speed(speed),.commanded_steer(steer),
 .profile_ready(profile_ready),.pose_x_q8(px),.pose_y_q8(py));
 always @(posedge clk)begin x<=(500+index*200)*256/10;y<=2700*256/10;end
 always @(negedge clk)update<=dut.tick;
 task byte_send(input [7:0] b);integer j;begin
  @(negedge clk);host_rx=0;repeat(10)@(negedge clk);
  for(j=0;j<8;j=j+1)begin host_rx=b[j];repeat(10)@(negedge clk);end
  host_rx=1;repeat(11)@(negedge clk);
 end endtask
 task command(input [7:0] op,b0,b1);begin
  byte_send(8'ha5);byte_send(8'h5a);byte_send(1);byte_send(1);byte_send(op);
  byte_send(b0);byte_send(b1);byte_send(0);byte_send(0);byte_send(op^b0^b1);
  repeat(1100)@(negedge clk);
 end endtask
 task ticks(input integer n);integer j;begin
  for(j=0;j<n;j=j+1)begin @(negedge dut.tick);repeat(5)@(negedge clk);end
 end endtask
 integer saved_index;reg [3:0] saved_state;reg signed [23:0] saved_x;
 initial begin
  force dut.encoder_position1=0;force dut.encoder_position2=0;
  force dut.tof_debug0=0;force dut.tof_debug1=0;force dut.tof_debug2=0;force dut.tof_debug3=0;
  force dut.left_mm=1500;force dut.left_front_mm=1500;force dut.right_front_mm=1500;force dut.right_mm=1500;
  force dut.range_frame=1;force dut.range_samples=4'hf;force dut.range_observations=4'hf;
  repeat(5)@(negedge clk);rst=1;wait(profile_ready);mode=2;ticks(3);
  if(dut.drive_enabled || speed || steer)$fatal(1,"host entry inherited manual command");
  command(2,160,0);command(3,0,4);ticks(15);
  if(!dut.drive_enabled || speed<20 || speed>60 || steer)$fatal(1,"host source/ramp speed=%0d steer=%0d",speed,steer);
  command(4,8,0);ticks(20);if(speed!=8)$fatal(1,"host slowdown did not ramp to 8");
  command(5,0,0);ticks(1);if(dut.drive_enabled || speed || !dut.host_brake)$fatal(1,"host STOP not connected");
  mode=0;ticks(2);mode=9;ticks(3);
  if(dut.drive_enabled || dut.motion_mode!=0 || speed)$fatal(1,"navigation started without host START");
  command(2,100,0);command(3,1,5);
  // Navigation starts immediately. This fixture holds the encoders still,
  // so inspect startup before the real stalled-wheel watchdog must stop it.
  ticks(8);command(1,0,0);ticks(20);
  if(status!=2 || !dut.drive_enabled || speed==0)$fatal(1,"host NAV START status=%0d state=%0d speed=%0d",status,dut.motion.state,speed);
  saved_index=dut.motion_index;saved_state=dut.motion.state;saved_x=px;
  command(5,0,0);ticks(10);
  if(dut.drive_enabled || speed || dut.motion_index!=saved_index || px!=saved_x || dut.motion.state!=saved_state)
   $fatal(1,"pause lost route or pose");
  if(!dut.host_nav_session || !dut.host_brake)$fatal(1,"paused nav session/brake lost");
  command(7,0,0);ticks(5);
  if(!dut.drive_enabled || speed==0 || dut.motion_index!=saved_index || px!=saved_x)$fatal(1,"resume rebased/restarted navigation");
  selected=0;ticks(2);if(dut.host_run)$fatal(1,"leaving communication retained host run");
  $display("PASS host integration: third source arbitration, ramp/STOP/brake, no implicit NAV START, real route/profile startup, pause/resume without reset of pose/route");$finish;
 end
 initial begin #60000000;$fatal(1,"host integration timeout");end
endmodule
