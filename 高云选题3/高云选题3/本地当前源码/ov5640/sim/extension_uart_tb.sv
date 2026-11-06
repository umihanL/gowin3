`timescale 1ns/1ps
// Pin-level UART -> real subsystem -> real signed wheel commands.
module extension_uart_tb;
 reg clk=0;always #5 clk=~clk;
 reg rst=0,rx=1;reg [3:0] mode=2;wire tx;wire signed [15:0] speed,steer;
 reg signed [31:0] e1=0,e2=0;wire [3:0] status;tri1 sda,scl;
 vehicle_subsystem #(.CLK_HZ(1000000),.HOST_BAUD(100000)) dut(
 .clk(clk),.rst_n(rst),.mode(mode),.control_remote(1'b0),.control_host(1'b1),.host_rx(rx),.host_tx(tx),
 .host_route_page(1'b0),.host_route_result(8'd5),.host_route_status(8'd0),
 .navigation_avoidance(1'b0),.configured_speed(16'd100),.slowdown_cm(16'd65),.bypass_cm(16'd65),.brake_speed(16'd100),
 .touch_run(1'b0),.touch_speed(16'sd0),.touch_steer(16'sd0),.navigation_request(1'b0),.route_valid(1'b0),.route_count(12'd0),
 .route_x_q8(17'd0),.route_y_q8(17'd0),.route_lock(1'b0),.imu_available(1'b1),.imu_update(1'b1),.imu_restart(1'b0),.imu_yaw(16'sd0),
 .encoder_A(1'b0),.encoder_B(1'b0),.encoder2_A(1'b0),.encoder2_B(1'b0),.ps2_miso(1'b1),.tof_sda(sda),.tof_scl(scl),
 .zebra_update(1'b0),.zebra_detected(1'b0),.status(status),.commanded_speed(speed),.commanded_steer(steer));
 wire received;wire [7:0] byte_data;integer bytes=0;reg [7:0] received_bytes[0:2047];
 host_uart_rx #(.CLK_HZ(1000000),.BAUD(100000)) monitor(.clk(clk),.rst_n(rst),.rx(tx),.valid(received),.data(byte_data),.framing_error());
 always @(negedge clk)if(received)begin received_bytes[bytes]=byte_data;bytes=bytes+1;end
 task uart(input [7:0] b);integer j;begin
  @(negedge clk);rx=0;repeat(10)@(negedge clk);
  for(j=0;j<8;j=j+1)begin rx=b[j];repeat(10)@(negedge clk);end
  rx=1;repeat(13)@(negedge clk);
 end endtask
 task command(input [7:0] seq,op,input [31:0] arg);begin
  uart(8'ha5);uart(8'h5a);uart(1);uart(seq);uart(op);
  uart(arg[7:0]);uart(arg[15:8]);uart(arg[23:16]);uart(arg[31:24]);
  uart(1^seq^op^arg[7:0]^arg[15:8]^arg[23:16]^arg[31:24]);
  repeat(1200)@(negedge clk);
 end endtask
 task ticks(input integer n,input integer delta);integer j;begin
  for(j=0;j<n;j=j+1)begin @(negedge dut.tick);e1=e1+delta;e2=e2+delta;repeat(5)@(negedge clk);end
 end endtask
 initial begin
  force dut.encoder_position1=e1;force dut.encoder_position2=e2;force dut.actual_speed1=0;force dut.actual_speed2=0;
  force dut.tof_debug0=0;force dut.tof_debug1=0;force dut.tof_debug2=0;force dut.tof_debug3=0;
  force dut.left_mm=1000;force dut.left_front_mm=1000;force dut.right_front_mm=1000;force dut.right_mm=1000;
  force dut.range_frame=1;force dut.range_samples=15;force dut.range_observations=15;
  repeat(5)@(negedge clk);rst=1;ticks(14,0);
  command(1,8'h15,0);if(received_bytes[4]!=8'h95 || received_bytes[6]!=2 || received_bytes[7]!=127)$fatal(1,"capability wire");
  command(2,8'h14,0);if(!received_bytes[16][1] || !received_bytes[16][3])$fatal(1,"stopped/scope flags");
  command(30,8'h12,3);ticks(14,0);
  command(31,8'h03,0);if(received_bytes[bytes-5]!=3 || dut.host_run || dut.drive_enabled)$fatal(1,"traffic started with zebra OFF");
  command(32,8'h17,1);command(33,8'h14,0);if(!received_bytes[bytes-4][4])$fatal(1,"17 switch actual state not returned");
  command(34,8'h17,0);command(35,8'h14,0);if(received_bytes[bytes-4][4])$fatal(1,"17 disable did not return OFF");
  command(3,8'h12,2);ticks(14,0);if(dut.extension_mode!=2)$fatal(1,"task mode wire");
  command(4,8'h10,{16'd80,-16'sd100});ticks(20,-2);
  if(!dut.drive_enabled || speed>=0 || steer!=0)$fatal(1,"UART reverse not connected speed%0d",speed);
  command(5,8'h05,0);ticks(14,0);if(dut.drive_enabled || speed!=0 || !dut.host_brake)$fatal(1,"UART stop/brake");
  command(6,8'h11,{16'd0,16'd60});ticks(20,2);
  if(!dut.drive_enabled || speed<=0)$fatal(1,"UART velocity no motion");
  command(6,8'h11,{16'd0,16'd30});ticks(10,1);if(speed!=30)$fatal(1,"same SEQ speed adjustment failed");
  ticks(33,1);ticks(14,0);if(dut.drive_enabled || speed!=0 || dut.extension_phase!=5)$fatal(1,"lease not connected");
  command(7,8'h01,0);ticks(5,0);if(dut.drive_enabled)$fatal(1,"heartbeat rearmed expired motion");
  $display("PASS extension pin UART: 15 capabilities, 14 measured-stop scope, 12 mode, reverse 10, STOP brake, 11 speed renewal/400ms timeout");$finish;
 end
 initial begin #300000000;$fatal(1,"extension UART watchdog");end
endmodule
