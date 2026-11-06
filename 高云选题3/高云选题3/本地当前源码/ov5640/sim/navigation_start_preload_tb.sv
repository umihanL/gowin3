`timescale 1ns/1ps
// Public-port integration: rejected frames cannot erase a stopped preload
// then let the wheels launch before the actuator has caught up again.
module navigation_start_preload_tb;
 parameter NO_LANE=0;
 parameter JUNCTION_FIT=0;
 reg clk=0;always #5 clk=~clk;reg rst=0;reg [3:0] mode=0;
 wire tick,drive;wire [10:0] index,read_index;wire signed [15:0] speed,steer;wire [3:0] status;
 reg [16:0] rx=0,ry=0;reg update=0,valid=0;
 always @(posedge clk)begin rx<=index==0?17'd12800:17'd64000;ry<=17'd69120;end
 vehicle_motion #(.CLK_HZ(20000),.START_DELAY_MS(100)) dut(
 .clk(clk),.rst_n(rst),.mode(mode),.navigation_request(mode==9),.route_valid(1'b1),.profile_ready(1'b1),
 .configured_speed(16'd160),.slowdown_cm(16'd65),.bypass_cm(16'd65),.brake_speed(16'd200),
 .route_count(JUNCTION_FIT?12'd3:12'd2),.route_index(index),.route_read_index(read_index),.route_x_q8(rx),.route_y_q8(ry),.route_lock(1'b1),
 .route_tangent(16'sd0),.route_curve(2'sd0),.route_upcoming_turn(2'sd0),.route_entry_mm(8'd0),
 .route_entry_valid(1'b0),.route_entry_index_valid(1'b0),.route_exit_valid(1'b0),.route_exit_heading(16'sd0),
 .encoder1(32'sd0),.encoder2(32'sd0),.imu_available(1'b1),.imu_update(1'b1),.imu_restart(1'b0),.imu_yaw(16'sd0),
 .range_frame(1'b1),.range_samples(4'hf),.range_observations(4'hf),.range_diagnostics(32'd0),
 .left_mm(16'd1500),.right_mm(16'd1500),.left_front_mm(16'd1500),.right_front_mm(16'd1500),
 .remote_link(1'b0),.remote_run(1'b0),.remote_speed(16'sd0),.remote_steer(16'sd0),.host_pause(1'b0),
 .road_fit_valid(JUNCTION_FIT!=0),.road_fit_forward0(10'd188),.road_fit_forward1(10'd420),
 .road_fit_error0(-9'sd30),.road_fit_error1(-9'sd100),
 .road_visibility(3'b111),.road_update(update),.road_valid(valid),.road_junction(JUNCTION_FIT!=0),
 .road_near(9'sd90),.road_heading(9'sd0),.zebra_update(1'b0),.zebra_detected(1'b0),
 .drive_enabled(drive),.speed_mm_s(speed),.steer_deg(steer),.status(status),.tick_10ms(tick));
 integer ticks=0,applied=0,first=0;
 always @(negedge clk)if(rst && JUNCTION_FIT && drive && read_index!==index)
  $fatal(1,"junction ground fit acquired path RAM despite excluded registration frame");
 always @(negedge clk)if(rst && tick)begin
  ticks=ticks+1;
  if(drive)begin
   if(NO_LANE)$fatal(1,"missing starting lane launched vehicle");
   if(!JUNCTION_FIT)begin
    if($abs(applied-steer)>1)$fatal(1,"START_PRELOAD_LOST: driven before preload recovered ticks=%0d applied=%0d steer=%0d",ticks,applied,steer);
    $display("PASS stopped steering survives rejected frames; first drive ticks=%0d applied=%0d steer=%0d",ticks,applied,steer);$finish;
   end else begin
    if(first==0)first=ticks;
    if(ticks-first>=40)begin $display("PASS navigation_junction_fit: complete valid widened junction frames cannot register or borrow route RAM");$finish;end
   end
  end
  if(applied<steer)applied=applied+1;else if(applied>steer)applied=applied-1;
  update=ticks>=100 && ticks%3==1;
  valid=!NO_LANE && (ticks<109 || ticks>=118);
  if(NO_LANE && status==4)begin
   $display("PASS absent starting lane: bounded acquisition fault ticks=%0d; no indefinite WAIT or blind launch",ticks);$finish;
  end
  if(ticks>(NO_LANE?340:180))$fatal(1,"rejected launch frames caused indefinite waiting status=%0d",status);
 end else update=0;
 initial begin repeat(5)@(negedge clk);rst=1;repeat(100)@(negedge clk);mode=9;end
endmodule
