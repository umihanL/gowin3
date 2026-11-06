`timescale 1ns/1ps
// With navigation avoidance OFF, range inputs cannot control map navigation. Only the clock
// is scaled; production sensor/stall timeout settings are retained.
module navigation_uninterrupted_tb;
 reg clk=0;always #5 clk=~clk;
 reg rst=0,avoidance=0,north=0,imu=1,pause=0,valid_route=1,freeze_wheels=0;
 reg [3:0] mode=0;
 reg signed [31:0] e1=0,e2=0;reg signed [15:0] yaw=0;
 reg [15:0] left=1500,right=1500,front=1500,configured=160;
 reg frame=1;reg [3:0] samples=15,observations=15;reg [31:0] diagnostics=0;
 reg [16:0] rx,ry;wire [10:0] index;
 wire tick,drive,brake;wire signed [15:0] speed,steer;
 wire signed [23:0] px,py;wire [3:0] status,phase;
 always @(posedge clk)begin
  rx<=(north?500:index==0?500:2500)*256/10;
  ry<=(north?(index==0?2700:700):2700)*256/10;
 end
 vehicle_motion #(.CLK_HZ(20000),.START_DELAY_MS(0)) dut(
  .clk(clk),.rst_n(rst),.mode(mode),.navigation_avoidance(avoidance),
  .navigation_request(mode==9),.route_valid(valid_route),.profile_ready(1'b1),
  .route_count(12'd2),.route_index(index),.route_x_q8(rx),.route_y_q8(ry),
  .route_lock(1'b1),.route_curve(2'sd0),
  .configured_speed(configured),.slowdown_cm(16'd65),.bypass_cm(16'd65),.brake_speed(16'd200),
  .encoder1(e1),.encoder2(e2),.imu_available(imu),.imu_update(imu),.imu_yaw(yaw),.imu_restart(1'b0),
  .range_frame(frame),.range_samples(samples),.range_observations(observations),.range_diagnostics(diagnostics),
  .left_mm(left),.right_mm(right),.left_front_mm(front),.right_front_mm(front),
  .remote_link(1'b0),.remote_run(1'b0),.remote_speed(16'sd0),.remote_steer(16'sd0),.host_pause(pause),
  .zebra_update(1'b1),.zebra_detected(1'b1),
  .drive_enabled(drive),.brake_enable(brake),.speed_mm_s(speed),.steer_deg(steer),
  .status(status),.phase(phase),.pose_x_q8(px),.pose_y_q8(py),.tick_10ms(tick));
 real travel=0;integer ticks=0,jump=0,checks=0,stage=0;
 reg enforce_cruise=0;
 always @(negedge clk)if(rst && tick)begin
  ticks=ticks+1;
  if(!freeze_wheels)begin
   if(drive)travel=travel+speed*0.01;
   e1=$rtoi(travel*1560.0/(66.0*3.141592654))+jump;e2=e1;
  end
  if(enforce_cruise)begin
   checks=checks+1;
   if(!drive || status!=2 || speed!=configured || steer!=0 || phase!=0 ||
      dut.bypass_active || dut.bypass_start || dut.nav_resume || dut.avoidance_selected)
    $fatal(1,"range affected map motion stage=%0d speed=%0d steer=%0d status=%0d phase=%0d",stage,speed,steer,status,phase);
   if(dut.target_x_mm!=(north?500:2500) || dut.target_y_mm!=(north?2500:500))
    $fatal(1,"range changed route target");
  end
 end
 task wait_ticks(input integer n);integer last;begin
  last=ticks+n;wait(ticks>=last);@(negedge clk);
 end endtask
 task reset_run(input integer avoid,vertical,runmode,range_at_start);begin
  enforce_cruise=0;rst=0;mode=0;travel=0;e1=0;e2=0;jump=0;yaw=0;
  left=1500;right=1500;front=range_at_start;configured=160;imu=1;pause=0;valid_route=1;freeze_wheels=0;
  frame=1;samples=15;observations=15;diagnostics=0;avoidance=avoid;north=vertical;
  repeat(8)@(negedge clk);rst=1;wait_ticks(3);mode=runmode;wait_ticks(10);
  if(!drive || status!=2)$fatal(1,"failed to start mode=%0d status=%0d state=%0d",runmode,status,dut.state);
 end endtask
 task stopped(input integer expected);begin
  if(drive || speed!=0 || status!=expected || !brake)
   $fatal(1,"required stop missing expected=%0d actual=%0d speed=%0d",expected,status,speed);
 end endtask
 integer a,v,side,timeout,saved_index;reg signed [23:0] saved_x,saved_y;
 initial begin
  for(a=0;a<1;a=a+1)for(v=0;v<=1;v=v+1)begin
   reset_run(a,v,9,40);wait_ticks(90);enforce_cruise=1;
   stage=1;left=40;right=50;wait_ticks(25);
   stage=2;front=400;wait_ticks(25);
   stage=3;front=65535;diagnostics=32'h11111111;wait_ticks(25);
   stage=4;diagnostics=32'h09090909;wait_ticks(25);
   stage=5;frame=0;samples=0;observations=0;wait_ticks(100);
   stage=6;frame=1;samples=15;observations=15;diagnostics=0;front=40;wait_ticks(25);
   stage=7;avoidance=!avoidance;front=1500;left=1500;right=1500;wait_ticks(25);
   enforce_cruise=0;
  end
  $display("PASS avoidance OFF range isolation: east/north routes, initial/recovered 40mm, slow zone, no echo, bus fault, stale scan, clear and in-run flag changes (%0d checked ticks)",checks);
  for(a=0;a<1;a=a+1)for(side=-1;side<=1;side=side+2)begin
   reset_run(a,0,9,40);yaw=-side*60;wait_ticks(35);
   if(side*dut.tracker_steer<=0 || $abs(dut.tracker_steer)>=8)$fatal(1,"missing mild map correction");
   if(side==1)left=40;else right=40;
   wait_ticks(5);
   if(dut.route_candidate!==dut.tracker_steer || side*steer<=0 || dut.bypass_active)
    $fatal(1,"side ranging replaced map steering");
  end
  $display("PASS side sensors preserve IMU/map left/right correction");
  reset_run(0,0,9,40);wait_ticks(90);
  configured=0;wait_ticks(90);if(speed || !drive || status!=2)$fatal(1,"configured zero ignored");
  configured=80;wait_ticks(50);if(speed!=80)$fatal(1,"configured speed lost");
  pause=1;wait_ticks(3);saved_index=index;saved_x=px;saved_y=py;
  if(speed || drive)$fatal(1,"explicit pause ignored");
  wait_ticks(12);if(index!=saved_index || px!=saved_x || py!=saved_y)$fatal(1,"pause lost pose/route");
  pause=0;wait_ticks(50);if(speed!=80 || !drive || status!=2 || index!=saved_index)$fatal(1,"pause resume failed");
  mode=0;wait_ticks(2);if(speed || drive)$fatal(1,"manual task exit ignored");
  $display("PASS configured speed/zero, manual pause/resume and task exit retain authority");
  reset_run(0,0,9,40);wait_ticks(90);imu=0;wait_ticks(1010);stopped(4);
  reset_run(0,0,9,40);wait_ticks(90);jump=1000;wait_ticks(5);stopped(8);
  reset_run(0,0,9,40);wait_ticks(90);freeze_wheels=1;wait_ticks(160);stopped(9);
  reset_run(0,0,9,40);wait_ticks(90);valid_route=0;wait_ticks(3);
  if(drive || speed || status!=7)$fatal(1,"invalid route did not stop");
  $display("PASS production IMU timeout, encoder jump, wheel stall and route loss stop");
  reset_run(0,0,9,40);timeout=0;
  while(status<3 && timeout<1600)begin wait_ticks(1);timeout=timeout+1;end
  stopped(3);if(travel<1950 || travel>2050)$fatal(1,"normal route arrival wrong travel=%f",travel);
  $display("PASS actual encoder/IMU route arrival despite 40mm range (travel=%f)",travel);
  reset_run(0,0,7,1500);wait_ticks(110);front=400;wait_ticks(70);
  if(speed>=160 || speed==0 || !drive)$fatal(1,"mode7 slowdown removed speed=%0d",speed);
  front=40;wait_ticks(3);stopped(5);
  reset_run(0,0,8,1500);wait_ticks(90);front=40;wait_ticks(3);stopped(5);
  reset_run(0,0,8,1500);wait_ticks(90);front=500;wait_ticks(30);
  if(!dut.bypass_active || steer>=0)$fatal(1,"independent mode8 detour removed");
  // Re-enter mode9 directly from an active detour; no reset is supplied.
  mode=9;front=40;wait_ticks(100);enforce_cruise=1;stage=8;wait_ticks(10);enforce_cruise=0;
  // Repeat from mode8's blocked phase5.
  reset_run(0,0,8,1500);left=100;right=100;wait_ticks(90);front=500;wait_ticks(30);
  if(phase!=5)$fatal(1,"mode8 blocked setup did not enter phase5");
  mode=9;front=40;wait_ticks(100);enforce_cruise=1;stage=9;wait_ticks(10);enforce_cruise=0;
  // The opposite handoff must reacquire the independent near-range stop.
  mode=8;wait_ticks(6);stopped(5);
  $display("PASS mode7 slowdown/near stop, mode8 near stop/detour, detour/blocked-to-map and map-to-mode8 handoffs");
  $display("PASS uninterrupted map navigation contract; no hardware operations");$finish;
 end
 initial begin #100000000;$fatal(1,"uninterrupted navigation timeout");end
endmodule
