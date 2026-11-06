`timescale 1ns/1ps
// Actual production profile + motion, independent rear-axle bicycle physics.
module navigation_inertial_regression_tb;
 parameter SIDE_CLOSE=0;
 parameter HOSTILE_VISION=0;
 reg clk=0;always #5 clk=~clk;
 reg rst=0;reg [3:0] mode=0;reg [11:0] count=0;
 reg [16:0] points_x[0:2047],points_y[0:2047],rx,ry;
 reg locks[0:2047],rl;
 wire [10:0] index,motion_index,profile_index,entry_index;
 wire profile_busy,profile_ready,exit_valid,entry_valid;
 wire signed [15:0] tangent,exit_heading,entry_x,entry_y;
 wire signed [1:0] curve,upcoming;wire [7:0] entry_mm;
 assign index=profile_busy?profile_index:motion_index;
 always @(posedge clk)begin rx<=points_x[index];ry<=points_y[index];rl<=locks[index];end
 vehicle_route_profile profile(.clk(clk),.rst_n(rst),.route_valid(1'b1),.route_count(count),
 .route_index(profile_index),.route_x_q8(rx),.route_y_q8(ry),.route_lock(rl),.selected_segment(4'd0),
 .execution_index(motion_index),.execution_tangent(tangent),.execution_curve(curve),
 .execution_upcoming_turn(upcoming),.execution_entry_mm(entry_mm),
 .execution_entry_valid(entry_valid),.execution_entry_x_mm(entry_x),.execution_entry_y_mm(entry_y),
 .execution_entry_index(entry_index),.execution_exit_valid(exit_valid),.execution_exit_heading(exit_heading),
 .busy(profile_busy),.ready(profile_ready));
 reg signed [31:0] e1=0,e2=0;reg signed [15:0] yaw=0;
 wire tick,drive;wire signed [15:0] speed,steer,heading;wire [3:0] status,phase;
 wire signed [23:0] pose_x,pose_y;
 reg [15:0] left_range=SIDE_CLOSE?50:1500,right_range=1500,front_range=1500;
 wire [2:0] visibility=HOSTILE_VISION?3'b111:3'b000;
 vehicle_motion #(.CLK_HZ(20000),.STALL_MS(3000)) dut(.clk(clk),.rst_n(rst),.mode(mode),
 .navigation_fusion(1'b0),.navigation_avoidance(1'b0),.navigation_request(mode==9),.route_valid(1'b1),.profile_ready(profile_ready),
 .configured_speed(16'd160),.slowdown_cm(16'd65),.bypass_cm(16'd65),.brake_speed(16'd200),
 .route_count(count),.route_index(motion_index),.route_x_q8(rx),.route_y_q8(ry),.route_lock(rl),
 .route_upcoming_turn(upcoming),.route_entry_mm(entry_mm),.route_curve(curve),.route_tangent(tangent),
 .route_exit_valid(exit_valid),.route_exit_heading(exit_heading),.route_entry_valid(entry_valid),
 .route_entry_x_mm(entry_x),.route_entry_y_mm(entry_y),.route_entry_index_valid(entry_valid),.route_entry_index(entry_index),
 .encoder1(e1),.encoder2(e2),.imu_available(1'b1),.imu_update(1'b1),.imu_restart(1'b0),.imu_yaw(yaw),
 .range_frame(1'b1),.range_samples(4'hf),.range_observations(4'hf),.range_diagnostics(32'd0),
 .left_mm(left_range),.right_mm(right_range),.left_front_mm(front_range),.right_front_mm(front_range),
 .remote_link(1'b0),.remote_run(1'b0),.remote_speed(16'sd0),.remote_steer(16'sd0),.host_pause(1'b0),
 .road_visibility(visibility),.road_update(HOSTILE_VISION!=0),.road_valid(HOSTILE_VISION!=0),
 .road_junction(HOSTILE_VISION!=0),.road_near(-9'sd180),.road_heading(-9'sd100),
 .zebra_update(HOSTILE_VISION!=0),.zebra_detected(HOSTILE_VISION!=0),
 .drive_enabled(drive),.speed_mm_s(speed),.steer_deg(steer),.status(status),.phase(phase),.tick_10ms(tick),
 .pose_x_q8(pose_x),.pose_y_q8(pose_y),.pose_heading(heading));
 real wx,wy,angle,w1=0,w2=0,ds,kappa,oldangle,nx,ny;
 real ax,ay,bx,by,dx,dy,f,ex,ey,d,best,maxerror=0,pose_error=0,maxpose=0;
 integer applied=0,ticks=0,j,bestj,fd,fields,qx,qy,ql,out;
 integer wrong=0,lastindex=-1,tracker_checks=0;
 reg [1023:0] filename,tracefile;
 function real norm(input real a);begin
  while(a>3.141592654)a=a-6.283185307;while(a < -3.141592654)a=a+6.283185307;norm=a;
 end endfunction
 always @(negedge clk)if(rst && tick)begin
  ticks=ticks+1;
  if(applied<steer)applied=applied+1;else if(applied>steer)applied=applied-1;
  kappa=$tan(applied*0.01745329252)/142.0;
  ds=drive?speed*0.01/(1+84*$abs(kappa)):0;oldangle=angle;angle=angle+ds*kappa;
  wx=wx+ds*$cos((angle+oldangle)/2);wy=wy+ds*$sin((angle+oldangle)/2);
  w1=w1+ds*(1-84*kappa);w2=w2+ds*(1+84*kappa);
  e1=$rtoi(w1*1560/(66*3.141592654));e2=$rtoi(w2*1560/(66*3.141592654));
  yaw=$rtoi(norm(angle)*1800/3.141592654);
  nx=wx+219*$cos(angle);ny=wy+219*$sin(angle);best=1e12;bestj=0;
  for(j=0;j<count-1;j=j+1)begin
   ax=points_x[j]*10.0/256;ay=3200-points_y[j]*10.0/256;
   bx=points_x[j+1]*10.0/256;by=3200-points_y[j+1]*10.0/256;
   dx=bx-ax;dy=by-ay;
   if(dx*dx+dy*dy>=0.01)begin
    f=((nx-ax)*dx+(ny-ay)*dy)/(dx*dx+dy*dy);if(f<0)f=0;if(f>1)f=1;
    ex=ax+f*dx;ey=ay+f*dy;d=(nx-ex)*(nx-ex)+(ny-ey)*(ny-ey);
    if(d<best)begin best=d;bestj=j;end
   end
  end
  if(mode==9 && status>=2)begin
   if($sqrt(best)>maxerror)maxerror=$sqrt(best);
   pose_error=$sqrt((pose_x/256.0-nx)*(pose_x/256.0-nx)+(pose_y/256.0-ny)*(pose_y/256.0-ny));
   if(pose_error>maxpose)maxpose=pose_error;
   if(dut.visual_navigation || dut.zebra_latched)$fatal(1,"vision escaped inertial selection");
   if(curve==1 && dut.route_steer<0 || curve==-1 && dut.route_steer>0)wrong=wrong+1;
  end
  if(ticks%2==0)$fwrite(out,"%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%f,%f,%f,%f,%f,%0d,%0d\n",
    ticks,motion_index,bestj,steer,dut.tracker_steer,dut.route_steer,curve,tangent,exit_heading,
    heading,dut.forward_mm,dut.lateral_mm,dut.target_x_mm,dut.target_y_mm,phase,nx,ny,angle,$sqrt(best),pose_error,status,dut.state);
 end
 always @(posedge clk)if(rst && dut.state==7 && dut.current_tracker_done &&
                         !dut.nav_resume && !dut.bypass_active)begin
  tracker_checks=tracker_checks+1;
  if(dut.target_x_mm!==(points_x[motion_index]*10>>8) ||
     dut.target_y_mm!==(3200-(points_y[motion_index]*10>>8)))$fatal(1,"wrong route target");
  if(lastindex>=0 && (motion_index<lastindex || motion_index>lastindex+1))$fatal(1,"route index order changed");
  lastindex=motion_index;
 end
 initial begin
  if(!$value$plusargs("ROUTE=%s",filename))$fatal(1,"missing route");
  if(!$value$plusargs("CSV=%s",tracefile))$fatal(1,"missing trace");
  fd=$fopen(filename,"r");if(!fd)$fatal(1,"route absent");
  while(!$feof(fd))begin
   fields=$fscanf(fd,"%d,%d,%d\n",qx,qy,ql);
   if(fields==3)begin points_x[count]=qx;points_y[count]=qy;locks[count]=ql;count=count+1;end
  end
  $fclose(fd);out=$fopen(tracefile,"w");
  $fwrite(out,"tick,index,nearest,command,tracker,route,curve,tangent,exit,heading,forward,lateral,target_x,target_y,phase,nose_x,nose_y,yaw,cross_error,pose_error,status,state\n");
  angle=$atan2($itor(points_y[0])-$itor(points_y[1]),$itor(points_x[1])-$itor(points_x[0]));
  wx=points_x[0]*10.0/256-219*$cos(angle);wy=3200-points_y[0]*10.0/256-219*$sin(angle);
  repeat(5)@(negedge clk);rst=1;wait(profile_ready);repeat(1000)@(negedge clk);mode=9;
  wait(status>=3 || ticks>=15000);repeat(2)@(negedge clk);$fclose(out);
  $display("RESULT inertial: status=%0d index=%0d/%0d maxCross=%f maxPose=%f oppositeCurveTicks=%0d ticks=%0d targetChecks=%0d",status,motion_index,count,maxerror,maxpose,wrong,ticks,tracker_checks);
  if(status!=3 || tracker_checks<count)$fatal(1,"inertial route did not finish or checked no targets");
  $display("PASS inertial route execution with hostileVision=%0d",HOSTILE_VISION);$finish;
 end
 initial begin #500000000;$fatal(1,"route audit timeout");end
endmodule
