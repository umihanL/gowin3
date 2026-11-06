`timescale 1ns/1ps
// Real production tracker/control with explicit odometry-coordinate stimuli.
// These inputs reproduce accumulated inertial drift; this is not an assertion
// of real-world positioning accuracy or a closed-loop vehicle plant test.
module navigation_drift_continue_tb;
 reg clk=0;always #5 clk=~clk;
 reg rst=0,pause=0,imu=1,inject=0;
 reg [3:0] mode=0;reg [1:0] orientation=0;
 reg [11:0] count=3;
 reg signed [15:0] start_heading=0;
 reg signed [31:0] e1=0,e2=0;
 reg signed [23:0] injected_x=0,injected_y=0;
 reg [16:0] rx,ry;
 wire [10:0] index;
 wire tick,drive,brake;wire signed [15:0] speed,steer;
 wire [3:0] status;wire signed [23:0] px,py;
 always @(posedge clk)begin
  rx<=(orientation==0?(index==0?500:index==1?1000:2500):500)*256/10;
  ry<=(orientation==1?(index==0?2700:index==1?2200:700):
        orientation==2?(index==0?700:index==1?1200:2700):2700)*256/10;
 end
 vehicle_motion #(.CLK_HZ(20000),.START_DELAY_MS(0)) dut(
  .clk(clk),.rst_n(rst),.mode(mode),.navigation_avoidance(1'b0),
  .route_start_heading_valid(1'b1),.route_start_heading(start_heading),
  .navigation_request(mode==9),.route_valid(1'b1),.profile_ready(1'b1),
  .route_count(count),.route_index(index),.route_x_q8(rx),.route_y_q8(ry),
  .route_lock(1'b1),.route_curve(2'sd0),.configured_speed(16'd200),
  .slowdown_cm(16'd65),.bypass_cm(16'd65),.brake_speed(16'd200),
  .encoder1(e1),.encoder2(e2),.imu_available(imu),.imu_update(imu),
  .imu_yaw(16'sd0),.imu_restart(1'b0),
  .range_frame(1'b1),.range_samples(4'hf),.range_observations(4'hf),.range_diagnostics(32'd0),
  .left_mm(16'd1500),.right_mm(16'd1500),.left_front_mm(16'd1500),.right_front_mm(16'd1500),
  .remote_link(1'b0),.remote_run(1'b0),.remote_speed(16'sd0),.remote_steer(16'sd0),.host_pause(pause),
  .zebra_update(1'b0),.zebra_detected(1'b0),
  .drive_enabled(drive),.brake_enable(brake),.speed_mm_s(speed),.steer_deg(steer),
  .status(status),.pose_x_q8(px),.pose_y_q8(py),.tick_10ms(tick));
 integer ticks=0;real travel=0;
 always @(negedge clk)if(rst && tick)begin
  ticks=ticks+1;
  if(drive)travel=travel+speed*0.01;
  e1=$rtoi(travel*1560.0/(66.0*3.141592654));e2=e1;
 end
 task wait_ticks(input integer n);integer last;begin
  last=ticks+n;wait(ticks>=last);@(negedge clk);
 end endtask
 task pose(input integer x,y);begin
  injected_x=x*256;injected_y=y*256;
  force dut.pose_x_q8=injected_x;force dut.pose_y_q8=injected_y;
 end endtask
 task start_run(input integer points,heading);begin
  release dut.pose_x_q8;release dut.pose_y_q8;
  rst=0;mode=0;e1=0;e2=0;travel=0;pause=0;imu=1;count=points;start_heading=heading;
  orientation=heading==900?1:heading==-900?2:0;
  repeat(8)@(negedge clk);rst=1;wait_ticks(3);mode=9;wait_ticks(12);
 end endtask
 task require_running(input integer stage);begin
  if(!drive || status!=2 || brake)$fatal(1,"drift stopped navigation stage=%0d status=%0d speed=%0d index=%0d lateral=%0d forward=%0d",stage,status,speed,index,dut.lateral_mm,dut.forward_mm);
 end endtask
 integer side,direction;
 initial begin
  for(side=-1;side<=1;side=side+2)begin
   start_run(3,0);require_running(0);
   // A target 400 mm to either side and still 200 mm ahead is valid
   // pursuit input, not evidence of a sensor failure.
   pose(800,500+side*400);wait_ticks(30);require_running(1);
   if(index!=1 || side*steer>=0 || $abs(dut.lateral_mm)<350)
    $fatal(1,"large lateral error did not continue signed correction");
   // Pass the intermediate point 70 mm longitudinally while remaining
   // outside its old 60 mm acquisition window. The next point must load.
   pose(1070,500+side*400);wait_ticks(5);require_running(2);
   if(index!=2 || dut.target_x_mm!=2500 || dut.forward_mm<=0)
    $fatal(1,"passed intermediate point was not advanced index=%0d target=%0d",index,dut.target_x_mm);
  end
  $display("PASS lateral +/-400mm keeps correcting; passed intermediate target advances without fault or reverse chasing");
  // Rotate the route and map heading by +/-90 degrees. Segment-plane
  // passage must advance the proper Y target, independent of X progress.
  for(direction=-1;direction<=1;direction=direction+2)
   for(side=-1;side<=1;side=side+2)begin
    start_run(3,direction*900);require_running(8);
    pose(500+side*400,direction==1?800:2200);wait_ticks(30);require_running(9);
    if(index!=1 || $signed(steer)*$signed(dut.lateral_mm)<=0 || $abs(dut.lateral_mm)<350)
     $fatal(1,"rotated route lost signed lateral correction direction=%0d",direction);
    pose(500+side*400,direction==1?1070:1930);wait_ticks(5);require_running(10);
    if(index!=2 || dut.target_y_mm!=(direction==1?2500:500) || dut.forward_mm<=0)
     $fatal(1,"rotated segment-plane passage failed direction=%0d index=%0d targetY=%0d",direction,index,dut.target_y_mm);
   end
  $display("PASS rotated north/south segment-plane progression with both lateral drift signs");
  start_run(2,0);require_running(3);
  // The final target is (1000,500). Hold position to isolate the real
  // tracker steering and controller speed response at each curvature.
  pose(900,400);wait_ticks(110);require_running(4);
  if(steer<=12 || steer>24 || speed!=140)
   $fatal(1,"moderate curve speed wrong steer=%0d speed=%0d",steer,speed);
  pose(950,400);wait_ticks(25);require_running(5);
  if(steer<=24 || speed!=120)
   $fatal(1,"tight curve speed wrong steer=%0d speed=%0d",steer,speed);
  pose(700,500);wait_ticks(55);require_running(6);
  if(steer!=0 || speed!=200)$fatal(1,"straight cruise did not recover steer=%0d speed=%0d",steer,speed);
  pause=1;wait_ticks(3);if(drive || speed)$fatal(1,"operator pause lost");
  pause=0;wait_ticks(8);require_running(7);
  // Final segment-plane arrival remains a stop. It represents odometric
  // route completion, not externally verified physical arrival.
  pose(1010,900);wait_ticks(5);
  if(status!=3 || drive || speed || !brake)$fatal(1,"terminal-plane completion lost status=%0d",status);
  $display("PASS measured steering speed caps 140/120, straight recovery 200, operator pause and odometric arrival stop");
  start_run(2,1800);
  if(status!=6 || drive || speed)$fatal(1,"start heading/route mismatch no longer rejected");
  $display("PASS start-heading mismatch remains a non-driving fault");
  $display("PASS navigation drift continuation and gentler corner speeds");$finish;
 end
 initial begin #100000000;$fatal(1,"navigation drift test timeout");end
endmodule
