`timescale 1ns/1ps
// Real shared settings -> complete vehicle actuator path -> PWM pin.
module servo_center_tb;
 reg clk=0;always #5 clk=~clk;
 reg rst=0,tv=0,td=0;reg [3:0] screen=12;reg [15:0] tx=0,ty=0;
 reg signed [15:0] request=0;
 wire [10:0] center;wire pulse;wire signed [15:0] command,speed1,speed2;
 wire [2:0] editing;tri1 sda,scl;
 vehicle_settings settings(.clk(clk),.rst_n(rst),.screen(screen),
  .touch_valid(tv),.touch_down(td),.touch_x(tx),.touch_y(ty),
  .remote_increase(1'b0),.remote_decrease(1'b0),.servo_center_tenths(center),.edit_parameter(editing));
 vehicle_subsystem #(.CLK_HZ(1000000)) car(.host_point_select_ready(1'b0),.host_point_route_ready(1'b0),.host_point_planning(1'b0),.selected_point(4'd0),.host_point_error(3'd0),.clk(clk),.rst_n(rst),.mode(4'd2),
  .navigation_request(1'b0),.route_valid(1'b0),.route_count(12'd0),
  .route_x_q8(17'd0),.route_y_q8(17'd0),.route_lock(1'b0),
  .control_remote(1'b0),.control_host(1'b0),.navigation_avoidance(1'b0),.host_rx(1'b1),
  .configured_speed(16'd200),.slowdown_cm(16'd65),.bypass_cm(16'd65),.brake_speed(16'd200),
  .servo_center_tenths(center),.touch_run(1'b1),.touch_speed(16'sd200),.touch_steer(request),
  .zebra_update(1'b0),.zebra_detected(1'b0),.imu_available(1'b1),.imu_update(1'b1),
  .imu_restart(1'b0),.imu_yaw(16'sd0),.encoder_A(1'b0),.encoder_B(1'b0),
  .encoder2_A(1'b0),.encoder2_B(1'b0),.ps2_miso(1'b1),.tof_sda(sda),.tof_scl(scl),
  .commanded_steer(command),.servo_pwm_out(pulse),.target_speed1(speed1),.target_speed2(speed2));
 task tap(input integer a,b);begin
  @(negedge clk);tx=a;ty=b;tv=1;td=1;repeat(3)@(negedge clk);
  td=0;repeat(3)@(negedge clk);tv=0;
 end endtask
 task measure(input integer expected);integer n;begin
  // Let any previous frame finish; then measure a complete updated frame.
  repeat(40010)@(negedge clk);@(posedge pulse);n=0;
  while(pulse)begin @(negedge clk);if(pulse)n=n+1;end
  if(n!=expected)$fatal(1,"PWM center=%0d request=%0d measured=%0d expected=%0d",center,request,n,expected);
 end endtask
 task settled(input integer wheel,input integer angle);integer delta;begin
  delta=wheel-car.applied_steer;if(delta<0)delta=-delta;
  request=wheel;repeat((delta+1)*10000+10)@(negedge clk);
  if(command!=wheel || car.applied_steer!=wheel || car.servo_angle_tenths!=angle)
   $fatal(1,"linkage center=%0d wheel=%0d applied=%0d angle=%0d expected=%0d",center,wheel,car.applied_steer,car.servo_angle_tenths,angle);
  if(wheel==28 && (speed2!=200 || speed1<103 || speed1>106))$fatal(1,"left differential changed");
  if(wheel==-28 && (speed1!=200 || speed2<103 || speed2>106))$fatal(1,"right differential changed");
  measure(500+(2000*angle+900)/1800);
 end endtask
 integer center_number,w,k,expected,offset;reg [7:0] reference_rom[0:127];
 initial $readmemh("src/vehicle/servo_offset.mem",reference_rom);
 initial begin
  repeat(8)@(negedge clk);rst=1;repeat(20)@(negedge clk);
  if(center!=900)$fatal(1,"reset center");
  tap(200,340);screen=14;repeat(8)@(negedge clk);
  if(editing!=4)$fatal(1,"settings selector");
  settled(0,900);tap(490,280);settled(0,901);
  tap(310,280);repeat(2)tap(650,280);repeat(3)tap(490,280);
  if(center!=923)$fatal(1,"shared center 92.3");
  settled(28,1523);settled(-28,423);settled(0,923);
  // Sweep every wheel degree for four centres; verify both nonlinear halves
  // and the unchanged rear differential using actual steering slew.
  for(center_number=0;center_number<4;center_number=center_number+1)begin
   case(center_number)
    0:begin repeat(80)tap(150,280);expected=500;end
    1:begin repeat(40)tap(650,280);expected=900;end
    2:begin repeat(2)tap(650,280);repeat(3)tap(490,280);expected=923;end
    3:begin repeat(80)tap(650,280);expected=1200;end
   endcase
   if(center!=expected)$fatal(1,"centre sweep setup");
   settled(-28,expected-500);
   for(w=-28;w<=28;w=w+1)begin
    request=w;repeat(11010)@(negedge clk);
    k=w<0?64-w:w;offset=reference_rom[k]*10;
    if(car.servo_angle_tenths!=(w<0?expected-offset:expected+offset) || car.applied_steer!=w)
     $fatal(1,"nonlinear full sweep center=%0d wheel=%0d servo=%0d",center,w,car.servo_angle_tenths);
    if(car.servo_min_tenths!=expected-500 || car.servo_max_tenths!=expected+600)
     $fatal(1,"adaptive bounds");
   end
   settled(28,expected+600);
   $display("PASS full nonlinear sweep: center=%0d lower=%0d upper=%0d wheel=-28..28",expected,expected-500,expected+600);
  end
  // Defense in depth for an invalid upstream value: finite electrical limits.
  force car.servo_center_tenths=11'd0;repeat(20)@(negedge clk);
  if(car.bounded_center!=500 || car.servo_angle_tenths!=1100)$fatal(1,"invalid low center clamp");
  force car.servo_center_tenths=11'd2047;repeat(20)@(negedge clk);
  if(car.bounded_center!=1200 || car.servo_angle_tenths!=1800)$fatal(1,"invalid high center clamp");
  release car.servo_center_tenths;
  $display("PASS shared settings/actuator: 0.1 degree PWM, asymmetric translated bounds, 228 nonlinear points, unchanged slew/differential and invalid-input clamps");
  $finish;
 end
 initial begin #200000000;$fatal(1,"servo center timeout");end
endmodule
