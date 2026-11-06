`timescale 1ns/1ps
// UART and production subsystem/motion/odometry, with ideal encoder feedback.
// Only external wheel feedback is modelled; ACKs and route control are real RTL.
module host_retained_jog_tb;
 reg clk=0;always #5 clk=~clk;reg rst=0,host_rx=1;reg [3:0] mode=0;reg selected=1;
 wire host_tx,pump;wire [10:0] index;wire profile_ready;reg [16:0] x=0,y=0;
 wire [3:0] status;wire signed [15:0] speed,steer;wire signed [23:0] px,py;
 wire [1:0] dir,dir2;wire pwm,pwm2;tri1 sda,scl;
 reg signed [31:0] enc=0;reg signed [15:0] actual=0;integer fraction=0,delta=0;
 vehicle_subsystem #(.CLK_HZ(20000),.HOST_BAUD(2000)) dut(.clk(clk),.rst_n(rst),.mode(mode),
 .control_remote(1'b0),.control_host(selected),.host_rx(host_rx),.host_tx(host_tx),.water_pump(pump),
 .configured_speed(16'd100),.slowdown_cm(16'd65),.bypass_cm(16'd65),.brake_speed(16'd200),
 .touch_run(1'b0),.touch_speed(16'sd0),.touch_steer(16'sd0),.servo_center_tenths(11'd900),
 .navigation_avoidance(1'b0),.start_heading_valid(1'b1),.start_heading(16'sd0),
 .navigation_request(mode==9),.route_valid(1'b1),.route_count(12'd5),.route_index(index),
 .route_x_q8(x),.route_y_q8(y),.route_lock(index==0 || index==4),
 .map_route_valid(1'b1),.map_planning(1'b0),.map_generation(16'd17),.map_signature(32'h11223344),
 .imu_available(1'b1),.imu_update(1'b1),.imu_restart(1'b0),.imu_rezero(1'b0),.imu_yaw(16'sd0),
 .encoder_A(1'b0),.encoder_B(1'b0),.encoder2_A(1'b0),.encoder2_B(1'b0),
 .ps2_miso(1'b1),.tof_sda(sda),.tof_scl(scl),.dir(dir),.dir2(dir2),.pwm_out(pwm),.pwm_out2(pwm2),
 .zebra_update(1'b0),.zebra_detected(1'b0),.status(status),.commanded_speed(speed),.commanded_steer(steer),
 .profile_ready(profile_ready),.pose_x_q8(px),.pose_y_q8(py));
 always @(posedge clk)begin x<=(500+index*250)*256/10;y<=2700*256/10;end
 always @(negedge dut.tick)if(rst)begin
  fraction=fraction+speed*1560;delta=fraction/20735;fraction=fraction-delta*20735;
  enc=enc+delta;actual=speed;
 end
 wire received,error;wire [7:0] data;integer bytes=0;reg [7:0] received_bytes[0:16383];
 host_uart_rx #(.CLK_HZ(20000),.BAUD(2000)) monitor(.clk(clk),.rst_n(rst),.rx(host_tx),
 .valid(received),.data(data),.framing_error(error));
 always @(negedge clk)if(received)begin received_bytes[bytes]=data;bytes=bytes+1;end
 task byte_send(input [7:0] b);integer j;begin
  @(negedge clk);host_rx=0;repeat(10)@(negedge clk);
  for(j=0;j<8;j=j+1)begin host_rx=b[j];repeat(10)@(negedge clk);end
  host_rx=1;repeat(11)@(negedge clk);
 end endtask
 reg [31:0] reply;integer serial=0,reply_start;
 task command(input [7:0] seq,op,input [31:0] args);integer timeout,n;reg [7:0] check;begin
  reply_start=bytes;
  byte_send(8'ha5);byte_send(8'h5a);byte_send(1);byte_send(seq);byte_send(op);
  byte_send(args[7:0]);byte_send(args[15:8]);byte_send(args[23:16]);byte_send(args[31:24]);
  byte_send(1^seq^op^args[7:0]^args[15:8]^args[23:16]^args[31:24]);
  timeout=0;
  while(bytes<reply_start+10 && timeout<5000)begin @(negedge clk);timeout=timeout+1;end
  if(bytes<reply_start+10)$fatal(1,"UART missing reply op=%h",op);
  repeat(30)@(negedge clk);
  if(received_bytes[reply_start+3]!=seq || received_bytes[reply_start+4]!=(op|8'h80))
    $fatal(1,"wrong UART reply op=%h received=%h",op,received_bytes[reply_start+4]);
  check=0;for(n=2;n<=8;n=n+1)check=check^received_bytes[reply_start+n];
  if(check!=received_bytes[reply_start+9])$fatal(1,"bad UART checksum");
  reply={received_bytes[reply_start+8],received_bytes[reply_start+7],received_bytes[reply_start+6],received_bytes[reply_start+5]};
 end endtask
 task ticks(input integer n);integer j;begin
  for(j=0;j<n;j=j+1)begin @(negedge dut.tick);repeat(5)@(negedge clk);end
 end endtask
 task finish_jog(input [7:0] wanted);integer n;begin
  n=0;
  while(dut.jog_active && n<1100)begin
   ticks(20);n=n+20;command(200,1,0);
   if(status!=2 || dut.host_run || dut.motion_index!=saved_index || dut.host.task_id!=45 || dut.motion.rebase)
     $fatal(1,"jog altered route lifecycle");
  end
  if(n>=1100)$fatal(1,"jog never finished");
  command(201,8'h3d,0);
  if(reply[7:0]!=3 || reply[23:16]!=wanted || !reply[24] || !reply[25] || reply[26])
    $fatal(1,"jog completion wrong %h wanted seq=%0d",reply,wanted);
 end endtask
 integer saved_index;reg signed [23:0] saved_x,saved_y;integer start_count;
 initial begin
  force dut.encoder_position1=enc;force dut.encoder_position2=enc;
  force dut.actual_speed1=actual;force dut.actual_speed2=actual;
  repeat(5)@(negedge clk);rst=1;wait(profile_ready);mode=2;ticks(15);
  command(1,8'h12,2);ticks(15);command(2,8'h15,0);
  if(reply[15:8]!=2 || !reply[22])$fatal(1,"jog capability/version missing %h",reply);
  mode=9;ticks(3);command(3,2,100);command(4,3,{16'd0,8'd45,8'd1});ticks(20);
  if(status!=2 || !dut.host_run)$fatal(1,"route startup failed %0d",status);
  command(5,8'h3c,{16'd40,16'd50});if(reply[7:0]!=1)$fatal(1,"jog allowed during cruise");
  command(6,5,0);ticks(15);command(7,8'h19,0);ticks(15);
  saved_index=dut.motion_index;saved_x=px;saved_y=py;start_count=enc;
  command(8,8'h3c,{16'd40,16'd50});if(reply[7:0]!=0)$fatal(1,"jog rejected %h",reply);
  ticks(5);command(9,7,0);if(reply[7:0]!=4 || dut.host_run)$fatal(1,"resume allowed during jog");
  command(10,8'h18,400);if(reply[7:0]!=4 || pump)$fatal(1,"pump allowed during jog");
  finish_jog(8);
  if(px-saved_x<45*256 || px-saved_x>55*256 || py!=saved_y || enc-start_count<340)
   $fatal(1,"forward jog did not update original map pose pxdelta=%0d counts=%0d",px-saved_x,enc-start_count);
  command(8,8'h3c,{16'd40,16'd50});ticks(20);
  if(dut.jog_active)$fatal(1,"replayed completed sequence restarted jog");
  command(11,8'h3c,{16'd40,-16'sd50});if(reply[7:0]!=0)$fatal(1,"reverse rejected");finish_jog(11);
  if(px-saved_x>3*256 || px-saved_x< -3*256 || py!=saved_y)$fatal(1,"reverse did not return anchor");
  if(dut.motion_index!=saved_index || dut.host.task_id!=45)$fatal(1,"return lost route cursor/task");
  command(12,7,0);ticks(5);
  if(reply[7:0]!=0 || !dut.host_run || !dut.drive_enabled || speed<=0 || dut.motion.rebase)
   $fatal(1,"return did not resume old route");
  if(px<saved_x || px-saved_x>20*256)$fatal(1,"resume rebased pose");
  command(13,5,0);ticks(15);command(14,8'h3c,{16'd40,16'd100});ticks(5);
  command(15,5,0);ticks(15);command(16,8'h3d,0);
  if(reply[7:0]!=4 || reply[15:8]!=1 || reply[23:16]!=14 || speed || dut.drive_enabled)
    $fatal(1,"UART STOP did not cancel jog %h",reply);
  command(14,8'h3c,{16'd40,16'd100});ticks(10);if(dut.jog_active || speed)$fatal(1,"cancelled UART replay started motion");
  // STATUS is read-only: flooding it does not renew the driving lease.
  command(17,8'h3c,{16'd20,16'd100});
  repeat(28)command(18,8'h3d,0);
  ticks(210);command(19,8'h3d,0);
  if(reply[7:0]!=4 || reply[15:8]!=2 || speed || dut.drive_enabled)$fatal(1,"query renewed jog lease %h",reply);
  $display("PASS retained jog UART+motion: capabilities, cruise refusal, signed pose update/anchor return, fixed route cursor/task, pump/resume exclusion, STOP replay protection, query-only watchdog");$finish;
 end
 initial begin #100000000;$fatal(1,"retained integration timeout");end
endmodule
