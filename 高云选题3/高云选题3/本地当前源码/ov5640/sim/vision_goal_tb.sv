`timescale 1ns/1ps
// Independent bicycle, a lane shifted 30mm from the selected map, and a bend
// visible beyond the chosen endpoint. Test sparse and dense vertical/diagonal
// routes; successful finish must not depend on entering an 18mm radius circle.
module vision_goal_tb;
 parameter DENSE=1,VERTICAL=1;
 localparam COUNT=DENSE?64:2;
 reg clk=0;always #5 clk=~clk;
 reg rst=0,request=0;reg [3:0] mode=0;
 reg [16:0] rx=0,ry=0;wire [10:0] index;
 reg signed [31:0] e1=0,e2=0;reg signed [15:0] yaw=0;
 reg update=0,valid=1;reg [2:0] visibility=3'b111;
 reg signed [8:0] near_error=0,trend=0;
 wire tick,drive;wire signed [15:0] speed,steer;wire [3:0] status;
 real pi=3.141592654,h0,wx,wy,angle,w1=0,w2=0,ds,kappa,old_angle,along,lateral,nose_x,nose_y,goal_length;
 integer ticks=0,applied=0,finished_tick=0;
 reg beyond_bend=0,saw_single=0,saw_empty=0;
 always @(posedge clk)begin
  if(VERTICAL)begin rx<=$rtoi((3250.0+20.0*index/(COUNT-1))*256/10);ry<=$rtoi((1410.0+1260.0*index/(COUNT-1))*256/10);end
  else begin rx<=$rtoi((500.0+900.0*index/(COUNT-1))*256/10);ry<=$rtoi((2700.0-900.0*index/(COUNT-1))*256/10);end
 end
 vehicle_motion #(.CLK_HZ(20000),.STALL_MS(3000)) dut(.route_exit_valid(1'b0),.route_exit_heading(16'sd0),.route_curve(2'sd0),.route_upcoming_turn(2'sd0),.route_entry_mm(8'd0),.route_tangent(VERTICAL?-16'sd891:16'sd450),.imu_restart(1'b0),
 .clk(clk),.rst_n(rst),.mode(mode),.navigation_request(request),.route_valid(1'b1),.profile_ready(1'b1),
 .configured_speed(16'd180),.slowdown_cm(16'd65),.bypass_cm(16'd65),.brake_speed(16'd200),
 .route_count(12'(COUNT)),.route_index(index),.route_x_q8(rx),.route_y_q8(ry),.route_lock(index==0 || index==COUNT-1),
 .encoder1(e1),.encoder2(e2),.imu_available(1'b1),.imu_update(1'b1),.imu_yaw(yaw),
 .range_frame(1'b1),.range_samples(4'hf),.range_observations(4'hf),.range_diagnostics(32'd0),
 .left_mm(16'd1200),.right_mm(16'd1200),.left_front_mm(16'd2500),.right_front_mm(16'd2500),
 .remote_link(1'b0),.remote_run(1'b0),.remote_speed(16'sd0),.remote_steer(16'sd0),
 .road_visibility(visibility),.road_update(update),.road_valid(valid),.road_junction(1'b0),.road_near(near_error),.road_heading(trend),
 .zebra_update(1'b0),.zebra_detected(1'b0),
 .drive_enabled(drive),.speed_mm_s(speed),.steer_deg(steer),.status(status),.tick_10ms(tick));
 always @(negedge clk)if(rst && tick)begin
  ticks=ticks+1;
  if(applied<steer)applied=applied+1;else if(applied>steer)applied=applied-1;
  kappa=$tan(applied*pi/180.0)/142.0;
  ds=drive?speed*0.01/(1+84*$abs(kappa)):0;old_angle=angle;angle=angle+ds*kappa;
  wx=wx+ds*$cos((old_angle+angle)/2);wy=wy+ds*$sin((old_angle+angle)/2);
  w1=w1+ds*(1-84*kappa);w2=w2+ds*(1+84*kappa);
  e1=$rtoi(w1*1560/(66*pi));e2=$rtoi(w2*1560/(66*pi));yaw=$rtoi((angle-h0)*1800/pi);
  nose_x=wx+219*$cos(angle);nose_y=wy+219*$sin(angle);
  along=(nose_x-(VERTICAL?3250:500))*$cos(h0)+(nose_y-(VERTICAL?1790:500))*$sin(h0);
  lateral=-(nose_x-(VERTICAL?3250:500))*$sin(h0)+(nose_y-(VERTICAL?1790:500))*$cos(h0);
  if(ticks%3==0)begin
   update=1;
   near_error=$rtoi(256.0*(lateral-(along>300?30:0)+210*(angle-h0))/254.0);
   trend=$rtoi(256.0*110*(angle-h0)/254.0);
   if(along>goal_length-120)begin
    // The camera sees the next bend, but navigation ends before it.
    trend=-24;visibility=3'b110;saw_single=1;
    if(along>goal_length-35)begin valid=0;visibility=3'b100;saw_empty=1;end
   end
  end
  if(dut.vision_guidance.blind_turn)$fatal(1,"straight goal acquired a bend beyond endpoint");
 end else update=0;
 initial begin
  h0=VERTICAL?$atan2(-1260.0,20.0):pi/4;goal_length=VERTICAL?$sqrt(1260.0*1260.0+400.0):$sqrt(2.0*900.0*900.0);
  angle=h0;wx=(VERTICAL?3250:500)-219*$cos(h0);wy=(VERTICAL?1790:500)-219*$sin(h0);
  repeat(5)@(negedge clk);rst=1;mode=9;request=1;
  wait(status==2 || status>=3);wait(status>=3 || ticks>5000);
  $display("INFO goal dense=%0d vertical=%0d status=%0d along=%f lateral=%f index=%0d dist=%0d dot=%0d",DENSE,VERTICAL,status,along,lateral,index,dut.distance_mm,dut.goal_dot);
  if(status!=3 || drive || speed!=0 || !saw_single || !saw_empty || along<goal_length-55 || along>goal_length+10)
   $fatal(1,"goal plane failed to stop a visually shifted lane");
  finished_tick=ticks;wait(ticks>finished_tick+100);
  if(drive || speed!=0 || steer!=0 || status!=3)$fatal(1,"finished run restarted/kept steering");
  $display("PASS terminal arrival: dense=%0d vertical=%0d shifted lane stops at goal plane; no beyond-goal bend, restart or circling",DENSE,VERTICAL);$finish;
 end
 initial begin #150000000;$fatal(1,"goal test timeout");end
endmodule
