`timescale 1ns/1ps
module road_motion_tb #(parameter TEST_LOOKAHEAD=60,TEST_SEGMENTS=2);
 reg clk=0;always #5 clk=~clk;
 reg rst=0;reg [3:0] mode=0;
 reg [16:0] points_x[0:2047],points_y[0:2047],rx,ry;
 reg locks[0:2047],read_lock;
 reg [11:0] count=0;wire [10:0] index,motion_index,profile_index;
 wire profile_busy,profile_ready;wire [3:0] profile_segments;
 assign index=profile_busy?profile_index:motion_index;
 always @(posedge clk)begin rx<=points_x[index];ry<=points_y[index];read_lock<=locks[index];end
 vehicle_route_profile profile(.execution_curve(),.execution_index(11'd1),.execution_tangent(),.clk(clk),.rst_n(rst),.route_valid(1'b1),.route_count(count),
 .route_index(profile_index),.route_x_q8(rx),.route_y_q8(ry),.route_lock(read_lock),
 .selected_segment(4'd0),.busy(profile_busy),.ready(profile_ready),.segment_count(profile_segments));
 reg signed [31:0] e1=0,e2=0;reg signed [15:0] yaw=0;
 wire tick,drive;wire signed [15:0] speed,steer;wire [3:0] status;
 vehicle_motion #(.REQUIRE_VISION(0),.CLK_HZ(20000),.STALL_MS(3000),.LOOKAHEAD_MM(TEST_LOOKAHEAD)) dut(.route_exit_valid(1'b0),.route_exit_heading(16'sd0),.route_curve(2'sd0),.route_upcoming_turn(2'sd0),.route_entry_mm(8'd0),.route_tangent(16'sd0),.imu_restart(1'b0),.clk(clk),.rst_n(rst),.mode(mode),
 .navigation_request(mode==9),.route_valid(1'b1),.profile_ready(profile_ready),
 .configured_speed(16'd200),.slowdown_cm(16'd65),.bypass_cm(16'd65),.brake_speed(16'd200),
 .route_count(count),.route_index(motion_index),.route_x_q8(rx),.route_y_q8(ry),.route_lock(read_lock),
 .encoder1(e1),.encoder2(e2),.imu_available(1'b1),.imu_update(1'b1),.imu_yaw(yaw),
 .range_frame(1'b1),.range_samples(4'h0),.range_observations(4'hf),.range_diagnostics(32'h11111111),
 .left_mm(16'hffff),.right_mm(16'hffff),.left_front_mm(16'hffff),.right_front_mm(16'hffff),
 .remote_link(1'b0),.remote_run(1'b0),.remote_speed(16'sd0),.remote_steer(16'sd0),
 .road_visibility(3'b000),.road_update(1'b0),.road_valid(1'b0),.road_junction(1'b0),.road_near(9'sd0),.road_heading(9'sd0),
 .zebra_update(1'b0),.zebra_detected(1'b0),
 .drive_enabled(drive),.speed_mm_s(speed),.steer_deg(steer),.status(status),.tick_10ms(tick));
 real wx,wy,angle,w1=0,w2=0,ds,kappa,previous_angle,nose_x,nose_y,route_error,best_error,max_error=0;
 real dx,dy,fraction,ax,ay,bx,by,projection_x,projection_y;
 integer applied=0,ticks=0,j,fd,fields,qx,qy,ql;integer log;
 reg [1023:0] route_file,trace_file;
 always @(negedge clk)if(rst && tick)begin
  ticks=ticks+1;
  if(applied<steer)applied=applied+1;else if(applied>steer)applied=applied-1;
  kappa=$tan(applied*0.01745329252)/142.0;
  ds=drive?speed*0.01/(1+84*$abs(kappa)):0;previous_angle=angle;angle=angle+ds*kappa;
  wx=wx+ds*$cos((angle+previous_angle)/2);wy=wy+ds*$sin((angle+previous_angle)/2);
  w1=w1+ds*(1-84*kappa);w2=w2+ds*(1+84*kappa);
  e1=$rtoi(w1*1560/(66*3.141592654));e2=$rtoi(w2*1560/(66*3.141592654));
  yaw=$rtoi(angle*1800/3.141592654);
  while(yaw>1800)yaw=yaw-3600;while(yaw < -1800)yaw=yaw+3600;
  nose_x=wx+219*$cos(angle);nose_y=wy+219*$sin(angle);best_error=1e9;
  for(j=0;j<count-1;j=j+1)begin
   ax=points_x[j]*10.0/256;ay=3200-points_y[j]*10.0/256;
   bx=points_x[j+1]*10.0/256;by=3200-points_y[j+1]*10.0/256;
   dx=bx-ax;dy=by-ay;
   fraction=((nose_x-ax)*dx+(nose_y-ay)*dy)/(dx*dx+dy*dy);
   if(fraction<0)fraction=0;if(fraction>1)fraction=1;
   projection_x=ax+fraction*dx;projection_y=ay+fraction*dy;
   route_error=$sqrt((projection_x-nose_x)*(projection_x-nose_x)+(projection_y-nose_y)*(projection_y-nose_y));
   if(route_error<best_error)best_error=route_error;
  end
  if(best_error>max_error)max_error=best_error;
  if(ticks%10==0)$fwrite(log,"%f,%f,%d,%d,%f\n",nose_x,nose_y,index,steer,best_error);
 end
 initial begin
  if(!$value$plusargs("ROUTE=%s",route_file))route_file="sim/road_route.csv";
  if(!$value$plusargs("CSV=%s",trace_file))trace_file="sim/road_motion.csv";
  fd=$fopen(route_file,"r");if(!fd)$fatal(1,"route CSV absent");
  while(!$feof(fd))begin
   fields=$fscanf(fd,"%d,%d,%d\n",qx,qy,ql);
   if(fields==3)begin points_x[count]=qx;points_y[count]=qy;locks[count]=ql;count=count+1;end
  end
  $fclose(fd);log=$fopen(trace_file,"w");
  dx=$itor(points_x[1])-$itor(points_x[0]);dy=$itor(points_y[0])-$itor(points_y[1]);angle=$atan2(dy,dx);
  wx=points_x[0]*10.0/256-219*$cos(angle);wy=3200-points_y[0]*10.0/256-219*$sin(angle);
  repeat(5)@(negedge clk);rst=1;repeat(1000)@(negedge clk);mode=9;
  wait(status>=3 || ticks>20000);@(negedge clk);$fclose(log);
  if(status!=3 || !profile_ready || profile_segments!=TEST_SEGMENTS)$fatal(1,"generated line/arc route execution failed status=%d index=%d/%d x=%f y=%f heading=%f profile=%d",status,index,count,nose_x,nose_y,angle,profile_segments);
  dx=nose_x-points_x[count-1]*10.0/256;dy=nose_y-(3200-points_y[count-1]*10.0/256);
  if($sqrt(dx*dx+dy*dy)>35 || max_error>100)$fatal(1,"road follow/goal error max=%f goal=%f",max_error,$sqrt(dx*dx+dy*dy));
  $display("PASS road motion: actual generated %d-node line/arc route, encoder/IMU bicycle, max nose cross error=%fmm final=%fmm",count,max_error,$sqrt(dx*dx+dy*dy));$finish;
 end
endmodule
