`timescale 1ns/1ps
// Existing UART STOP/RESUME plus B13: no hardware access, no fake ACK path.
module host_patrol_pause_tb;
 reg clk=0;always #5 clk=~clk;reg rst=0,host_rx=1;reg [3:0] mode=0;reg selected=1;
 wire host_tx,pump;wire [10:0] index;wire profile_ready;reg [16:0] x=0,y=0;
 wire [3:0] status;wire signed [15:0] speed,steer;wire signed [23:0] px,py;
 wire [1:0] dir,dir2;wire pwm,pwm2;tri1 sda,scl;
 vehicle_subsystem #(.CLK_HZ(20000),.HOST_BAUD(2000)) dut(.clk(clk),.rst_n(rst),.mode(mode),
 .control_remote(1'b0),.control_host(selected),.host_rx(host_rx),.host_tx(host_tx),.water_pump(pump),
 .configured_speed(16'd100),.slowdown_cm(16'd65),.bypass_cm(16'd65),.brake_speed(16'd200),
 .touch_run(1'b0),.touch_speed(16'sd0),.touch_steer(16'sd0),
 .navigation_request(mode==9),.route_valid(1'b1),.route_count(12'd3),.route_index(index),
 .route_x_q8(x),.route_y_q8(y),.route_lock(index==0 || index==2),
 .imu_available(1'b1),.imu_update(1'b1),.imu_restart(1'b0),.imu_yaw(16'sd0),
 .encoder_A(1'b0),.encoder_B(1'b0),.encoder2_A(1'b0),.encoder2_B(1'b0),
 .ps2_miso(1'b1),.tof_sda(sda),.tof_scl(scl),.dir(dir),.dir2(dir2),.pwm_out(pwm),.pwm_out2(pwm2),
 .zebra_update(1'b0),.zebra_detected(1'b0),.status(status),.commanded_speed(speed),.commanded_steer(steer),
 .profile_ready(profile_ready),.pose_x_q8(px),.pose_y_q8(py));
 always @(posedge clk)begin x<=(500+index*200)*256/10;y<=2700*256/10;end
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
 integer saved_index,round;reg [3:0] saved_state,saved_phase;reg signed [23:0] saved_x,saved_y;
 initial begin
  force dut.encoder_position1=0;force dut.encoder_position2=0;
  force dut.actual_speed1=0;force dut.actual_speed2=0;
  repeat(5)@(negedge clk);rst=1;wait(profile_ready);mode=9;ticks(3);
  command(2,100,0);command(3,1,45);ticks(8);command(1,0,0);ticks(20);
  if(status!=2 || !dut.drive_enabled || speed<=0)$fatal(1,"route did not start");
  for(round=0;round<2;round=round+1)begin
   saved_index=dut.motion_index;saved_state=dut.motion.state;saved_phase=dut.motion.phase;saved_x=px;saved_y=py;
   command(5,0,0);ticks(15);
   if(dut.drive_enabled || speed || dut.host_run || !dut.host_nav_session || !dut.host_brake)
    $fatal(1,"pause did not retain stopped route session");
   if(dut.motion_index!=saved_index || px!=saved_x || py!=saved_y || dut.motion.state!=saved_state || dut.motion.phase!=saved_phase || status!=2 || dut.host.task_id!=45)
    $fatal(1,"pause changed route pose/phase/task");
   // STOP intentionally latches pump OFF. An explicit OFF and measured still
   // interval must precede any bounded automatic burst.
   command(8'h19,0,0);ticks(12);
   if(!dut.pump.ready)$fatal(1,"stopped pump was not ready after explicit OFF");
   command(8'h18,8'h90,1); // 400 ms, not continuous water
   if(!pump || !dut.pump.gate_high || dut.host_run)$fatal(1,"bounded burst failed");
   command(7,0,0);
   if(dut.host_run || dut.drive_enabled || speed || !pump)$fatal(1,"RESUME must reject while B13 HIGH");
   command(8'h19,0,0);
   if(pump)$fatal(1,"explicit OFF failed");
   command(7,0,0);ticks(5);
   if(!dut.host_run || !dut.host_nav_session || !dut.drive_enabled || speed<=0 || pump)
    $fatal(1,"verified OFF did not permit resume");
   if(dut.motion_index!=saved_index || px!=saved_x || py!=saved_y || dut.host.task_id!=45)
    $fatal(1,"resume restarted route/pose/task");
  end
  command(5,0,0);selected=0;ticks(2);selected=1;ticks(2);
  command(7,0,0);
  if(dut.host_run || dut.host_nav_session)$fatal(1,"resume revived session after mode exit");
  $display("PASS patrol pause: two STOP/OFF/burst/OFF/RESUME rounds retain route pose/task; resume rejects pump HIGH and exited session");
  $finish;
 end
 initial begin #100000000;$fatal(1,"patrol pause timeout");end
endmodule
