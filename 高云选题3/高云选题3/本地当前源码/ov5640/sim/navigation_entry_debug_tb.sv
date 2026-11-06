`timescale 1ns/1ps
// Actual graph paths, independent bicycle, and centre estimates from geometry.
module navigation_entry_debug_tb;
 parameter EDGE_MODE=1;parameter NAV_SPEED=160;parameter CAMERA_MODE=0;parameter EXPECT_HANDBACK=0;parameter STARTUP_GAP_TICKS=0;
 reg clk=0;always #5 clk=~clk;reg rst=0;reg [3:0] mode=0;
 reg [16:0] points_x[0:2047],points_y[0:2047],rx,ry;reg locks[0:2047],rl;
 reg [11:0] count=0;wire [10:0] index,motion_index,profile_index;
 wire signed [1:0] upcoming_turn;wire [7:0] entry_mm;
 wire profile_busy,profile_ready,exit_valid;wire signed [15:0] tangent,exit_heading;wire signed [1:0] curve;
 assign index=profile_busy?profile_index:motion_index;
 always @(posedge clk)begin rx<=points_x[index];ry<=points_y[index];rl<=locks[index];end
 vehicle_route_profile profile(.clk(clk),.rst_n(rst),.route_valid(1'b1),.route_count(count),
 .route_index(profile_index),.route_x_q8(rx),.route_y_q8(ry),.route_lock(rl),.selected_segment(4'd0),
 .execution_index(motion_index),.execution_tangent(tangent),.execution_curve(curve),
 .execution_upcoming_turn(upcoming_turn),.execution_entry_mm(entry_mm),.execution_exit_valid(exit_valid),.execution_exit_heading(exit_heading),.busy(profile_busy),.ready(profile_ready));
 reg signed [31:0] e1=0,e2=0;reg signed [15:0] yaw=0;
 wire tick,drive;wire signed [15:0] speed,steer;wire [3:0] status;wire [2:0] visual_state;
 reg update=0,valid=1,junction=0;reg [2:0] visibility=7;reg signed [8:0] near_error=0,heading_error=0;
 vehicle_motion #(.CLK_HZ(20000),.STALL_MS(3000)) dut(.clk(clk),.rst_n(rst),.mode(mode),
 .navigation_request(mode==9),.route_valid(1'b1),.profile_ready(profile_ready),
 .configured_speed(NAV_SPEED[15:0]),.slowdown_cm(16'd65),.bypass_cm(16'd65),.brake_speed(16'd200),
 .route_count(count),.route_index(motion_index),.route_x_q8(rx),.route_y_q8(ry),.route_lock(rl),
 .route_upcoming_turn(upcoming_turn),.route_entry_mm(entry_mm),.route_curve(curve),.route_tangent(tangent),.route_exit_valid(exit_valid),.route_exit_heading(exit_heading),
 .encoder1(e1),.encoder2(e2),.imu_available(1'b1),.imu_update(1'b1),.imu_restart(1'b0),.imu_yaw(yaw),
 .range_frame(1'b1),.range_samples(4'hf),.range_observations(4'hf),.range_diagnostics(32'd0),
 .left_mm(16'd1500),.right_mm(16'd1500),.left_front_mm(16'd1500),.right_front_mm(16'd1500),
 .remote_link(1'b0),.remote_run(1'b0),.remote_speed(16'sd0),.remote_steer(16'sd0),
 .road_visibility(visibility),.road_update(update),.road_valid(valid),.road_junction(junction),
 .road_near(near_error),.road_heading(heading_error),.zebra_update(1'b0),.zebra_detected(1'b0),
 .drive_enabled(drive),.speed_mm_s(speed),.steer_deg(steer),.status(status),.vision_state(visual_state),.tick_10ms(tick));
 real wx,wy,angle,w1=0,w2=0,ds,kappa,oldangle,nx,ny,ax,ay,bx,by,dx,dy,f,ex,ey,d,best,besttheta;
 real px,py,roadtheta,err,nearvalue,headvalue,maxerror=0,starttheta,upcoming,maxupcoming;
 real c0,c1,fl0,fl1,ll0,ll1,row_y[0:1],row_side[0:1],u; integer row_n,hits;
 integer applied=0,ticks=0,j,bestj,fd,fields,qx,qy,ql,out,laststate=0;integer fallback_ticks=0,handbacks=0;
 reg previous_active=0,seen_recovery=0;
 reg [1023:0] filename,tracefile;
 function real norm(input real a);begin
  while(a>3.141592654)a=a-6.283185307;while(a < -3.141592654)a=a+6.283185307;norm=a;end endfunction
 always @(negedge clk)if(rst && tick)begin
  ticks=ticks+1;if(applied<steer)applied=applied+1;else if(applied>steer)applied=applied-1;
  kappa=$tan(applied*0.01745329252)/142.0;ds=drive?speed*0.01/(1+84*$abs(kappa)):0;oldangle=angle;angle=angle+ds*kappa;
  wx=wx+ds*$cos((angle+oldangle)/2);wy=wy+ds*$sin((angle+oldangle)/2);
  w1=w1+ds*(1-84*kappa);w2=w2+ds*(1+84*kappa);
  e1=$rtoi(w1*1560/(66*3.141592654));e2=$rtoi(w2*1560/(66*3.141592654));yaw=$rtoi(norm(angle)*1800/3.141592654);
  nx=wx+219*$cos(angle);ny=wy+219*$sin(angle);best=1e12;bestj=0;
  for(j=0;j<count-1;j=j+1)begin
   ax=points_x[j]*10.0/256;ay=3200-points_y[j]*10.0/256;bx=points_x[j+1]*10.0/256;by=3200-points_y[j+1]*10.0/256;
   dx=bx-ax;dy=by-ay;f=((nx-ax)*dx+(ny-ay)*dy)/(dx*dx+dy*dy);if(f<0)f=0;if(f>1)f=1;
   ex=ax+f*dx;ey=ay+f*dy;d=(nx-ex)*(nx-ex)+(ny-ey)*(ny-ey);
   if(dx*dx+dy*dy>=1 && d<best)begin best=d;bestj=j;px=ex;py=ey;besttheta=$atan2(dy,dx);end
  end
  if($sqrt(best)>maxerror)maxerror=$sqrt(best);
  if(ticks%3==0)begin
   update=1;roadtheta=besttheta;maxupcoming=0;
   for(j=bestj+1;j<count-1 && j<bestj+5;j=j+1)begin
    dx=$itor(points_x[j+1])-$itor(points_x[j]);dy=$itor(points_y[j])-$itor(points_y[j+1]);
    upcoming=dx*dx+dy*dy>=655? $abs(norm($atan2(dy,dx)-roadtheta)):0;if(upcoming>maxupcoming)maxupcoming=upcoming;
   end
   // Tight curves and junctions may retain an outer edge yet lack a centre.
   valid=CAMERA_MODE==1?1:!(maxupcoming>0.13 || $abs(norm(besttheta-angle))>0.20);junction=maxupcoming>0.13;
   if(CAMERA_MODE==2)begin
    // Ground-plane intersections at the actual two local IPM fit bands.
    // A road segment outside the camera cone cannot supply a centre.
    row_y[0]=188;row_y[1]=274;hits=0;
    for(row_n=0;row_n<2;row_n=row_n+1)begin
     c0=1e12;row_side[row_n]=0;
     for(j=bestj>2?bestj-2:0;j<count-1 && j<bestj+25;j=j+1)begin
      ax=points_x[j]*10.0/256-nx;ay=3200-points_y[j]*10.0/256-ny;
      bx=points_x[j+1]*10.0/256-nx;by=3200-points_y[j+1]*10.0/256-ny;
      // The user-selected destination is a point on the road, not the end
      // of the physical asphalt visible beyond that point.
      if(j==count-2 && (bx-ax)*(bx-ax)+(by-ay)*(by-ay)>1)begin
       c1=$sqrt((bx-ax)*(bx-ax)+(by-ay)*(by-ay));
       bx=ax+(bx-ax)*1500/c1;by=ay+(by-ay)*1500/c1;
      end
      fl0=ax*$cos(angle)+ay*$sin(angle);fl1=bx*$cos(angle)+by*$sin(angle);
      ll0=-ax*$sin(angle)+ay*$cos(angle);ll1=-bx*$sin(angle)+by*$cos(angle);
      if($abs(fl1-fl0)>0.01)begin
       u=(row_y[row_n]-fl0)/(fl1-fl0);
       if(u>=0 && u<=1 && $abs(ll0+u*(ll1-ll0))<c0)begin
        row_side[row_n]=ll0+u*(ll1-ll0);c0=$abs(row_side[row_n]);
       end
      end
     end
     if(c0<row_y[row_n]*0.57735)hits=hits+1;
    end
    valid=hits==2;visibility=valid?7:4;
   end
   if(ticks<STARTUP_GAP_TICKS)valid=0;
   visibility=valid?7:EDGE_MODE==0?4:EDGE_MODE==1?(curve==1?6:5):7;
   err=(nx-px)*(-$sin(besttheta))+(ny-py)*$cos(besttheta);
   nearvalue=(err+210*norm(angle-besttheta));headvalue=110*norm(angle-besttheta);
   if(CAMERA_MODE==2 && valid)begin
    headvalue=-(row_side[1]-row_side[0])*110/86;
    nearvalue=-row_side[0]+(210-188)*headvalue/110;
   end
   near_error=$rtoi(nearvalue>127?127:nearvalue < -127?-127:nearvalue);
   heading_error=$rtoi(headvalue>127?127:headvalue < -127?-127:headvalue);
  end else update=0;
  if(visual_state!=laststate || ticks%30==0)begin
   $display("STATE t=%0d index=%0d nearest=%0d visual=%0d steer=%0d curve=%0d valid=%0d err=%f nose=%f,%f yaw=%f",ticks,motion_index,bestj,visual_state,steer,curve,valid,$sqrt(best),nx,ny,angle);$display("DEBUG next=%0d dist=%0d hint=%0d exitvalid=%0d map=%0d wait=%0d entry=%0d entryTravel=%0d travel=%0d direction=%0d trust=%0d incoming=%0d near=%0d head=%0d",upcoming_turn,entry_mm,dut.vision_turn_hint,exit_valid,dut.vision_guidance.map_recovery,dut.vision_guidance.approach_wait,dut.turn_entry_ready,dut.entry_travel_q8>>>8,dut.travel_q8>>>8,dut.entry_direction,dut.route_guidance_valid,dut.incoming_error,near_error,heading_error);laststate=visual_state;
  end
  if(dut.vision_recovery)fallback_ticks=fallback_ticks+1;
  if(ticks<STARTUP_GAP_TICKS && (drive || status==6))$fatal(1,"unseen starting lane must wait stationary, status=%0d drive=%b",status,drive);
  if(dut.vision_recovery)seen_recovery=1;
  if(seen_recovery && !dut.vision_recovery && dut.vision_active && !previous_active)handbacks=handbacks+1;
  previous_active=dut.vision_active;
  if(ticks%10==0)$fwrite(out,"%0d,%f,%f,%f,%0d,%0d,%0d,%0d,%f\n",ticks,nx,ny,angle,motion_index,steer,visual_state,valid,$sqrt(best));
 end else update=0;
 initial begin
  if(!$value$plusargs("ROUTE=%s",filename))filename="sim/navigation_robust_outer_route.csv";
  if(!$value$plusargs("CSV=%s",tracefile))tracefile="sim/navigation_robust_route_trace.csv";
  fd=$fopen(filename,"r");if(!fd)$fatal(1,"route absent");
  while(!$feof(fd))begin fields=$fscanf(fd,"%d,%d,%d\n",qx,qy,ql);if(fields==3)begin points_x[count]=qx;points_y[count]=qy;locks[count]=ql;count=count+1;end end
  $fclose(fd);out=$fopen(tracefile,"w");starttheta=$atan2($itor(points_y[0])-$itor(points_y[1]),$itor(points_x[1])-$itor(points_x[0]));angle=starttheta;
  wx=points_x[0]*10.0/256-219*$cos(angle);wy=3200-points_y[0]*10.0/256-219*$sin(angle);
  repeat(5)@(negedge clk);rst=1;repeat(1000)@(negedge clk);mode=9;
  wait(status>=3 || ticks>18000);@(negedge clk);$fclose(out);
  if(status!=3 || maxerror>50 || (CAMERA_MODE==0 && fallback_ticks<20) || (EXPECT_HANDBACK && handbacks==0))$fatal(1,"ACTUAL_ROUTE_CAMERA status=%0d index=%0d/%0d error=%f fallback=%0d handbacks=%0d valid=%0d visible=%b target=%0d,%0d fwd=%0d lat=%0d",status,motion_index,count,maxerror,fallback_ticks,handbacks,valid,visibility,dut.target_x_mm,dut.target_y_mm,dut.forward_mm,dut.lateral_mm);
  $display("PASS actual graph navigation with limited-FOV camera: count=%0d maxError=%f fallbackTicks=%0d handbacks=%0d",count,maxerror,fallback_ticks,handbacks);$finish;
 end
 initial begin #500000000;$fatal(1,"route camera timeout");end
endmodule
