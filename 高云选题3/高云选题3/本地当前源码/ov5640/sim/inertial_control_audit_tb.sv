`timescale 1ns/1ps
// Diagnostic contracts: reproduce non-visual overrides without editing RTL.
module inertial_control_audit_tb;
 reg clk=0;always #5 clk=~clk;
 reg rst=0;reg [3:0] mode=0;
 reg signed [31:0] e1=0,e2=0;reg signed [15:0] yaw=0;
 reg [15:0] left=1500,right=1500,front=1500;
 reg [16:0] rx,ry;wire [10:0] index;
 wire tick,drive;wire signed [15:0] speed,steer;wire [3:0] status;
 always @(posedge clk)begin rx<=(index==0?500:2500)*256/10;ry<=2700*256/10;end
 vehicle_motion #(.CLK_HZ(20000),.STALL_MS(3000)) dut(
 .clk(clk),.rst_n(rst),.mode(mode),.navigation_fusion(1'b0),.navigation_request(mode==9),
 .route_valid(1'b1),.profile_ready(1'b1),.route_count(12'd2),.route_index(index),
 .route_x_q8(rx),.route_y_q8(ry),.route_lock(1'b1),.route_tangent(16'sd0),.route_curve(2'sd0),
 .route_exit_valid(1'b0),.route_exit_heading(16'sd0),.route_upcoming_turn(2'sd0),.route_entry_mm(8'd0),
 .route_entry_valid(1'b0),.route_entry_index_valid(1'b0),
 .configured_speed(16'd160),.slowdown_cm(16'd65),.bypass_cm(16'd65),.brake_speed(16'd200),
 .encoder1(e1),.encoder2(e2),.imu_available(1'b1),.imu_update(1'b1),.imu_yaw(yaw),.imu_restart(1'b0),
 .range_frame(1'b1),.range_samples(4'hf),.range_observations(4'hf),.range_diagnostics(32'd0),
 .left_mm(left),.right_mm(right),.left_front_mm(front),.right_front_mm(front),
 .remote_link(1'b0),.remote_run(1'b0),.remote_speed(16'sd0),.remote_steer(16'sd0),.host_pause(1'b0),
 .road_update(1'b1),.road_valid(1'b1),.road_junction(1'b1),.road_visibility(3'b111),
 .road_near(9'sd180),.road_heading(9'sd100),.zebra_update(1'b1),.zebra_detected(1'b1),
 .drive_enabled(drive),.speed_mm_s(speed),.steer_deg(steer),.status(status),.tick_10ms(tick));
 real travel=0;integer ticks=0,side,reported;
 always @(negedge clk)if(rst && tick)begin
  ticks=ticks+1;if(drive)travel=travel+speed*0.01;
  e1=$rtoi(travel*1560/(66*3.141592654));e2=e1;
 end
 task wait_ticks(input integer n);integer end_tick;begin
  end_tick=ticks+n;wait(ticks>=end_tick);@(negedge clk);
 end endtask
 task reset_run;begin
  rst=0;mode=0;travel=0;e1=0;e2=0;yaw=0;left=1500;right=1500;front=1500;
  repeat(8)@(negedge clk);rst=1;wait_ticks(3);mode=9;wait_ticks(10);
  if(!drive || status!=2 || dut.visual_navigation)$fatal(1,"pure mode failed");
 end endtask
 initial begin
  for(side=-1;side<=1;side=side+2)begin
   reset_run;
   // Change only the incoming IMU yaw. The path remains an eastbound line.
   yaw=-side*60;wait_ticks(35);
   if(side*dut.tracker_steer<=0 || $abs(dut.tracker_steer)>=8)
    $fatal(1,"mild map correction not established tracker=%0d",dut.tracker_steer);
   if(side==1)left=50;else right=50;
   wait_ticks(4);
   if(steer!=-side*3 || side*dut.tracker_steer<=0 || dut.bypass_active)
    $fatal(1,"side override changed tracker=%0d command=%0d",dut.tracker_steer,steer);
   $display("REPRODUCED side ToF override: pure=1 vision=0 mapSteer=%0d output=%0d left=%0d right=%0d",dut.tracker_steer,steer,left,right);
  end
  reset_run;wait_ticks(70);front=500;wait_ticks(35);
  if(!dut.bypass_active || dut.bypass_choose_left || steer>=0)
   $fatal(1,"right bypass not reproduced active=%0d left=%0d steer=%0d",dut.bypass_active,dut.bypass_choose_left,steer);
  $display("REPRODUCED front ToF bypass: pure=1 vision=0 plannedLine=east front=500 left=1500 right=1500 output=%0d phase=%0d",steer,dut.phase);
  $display("PASS diagnosis: pure navigation excludes vision but both ToF steering overrides remain active");$finish;
 end
 initial begin #10000000;$fatal(1,"control audit timeout");end
endmodule
