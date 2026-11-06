`timescale 1ns/1ps
module vision_turn_motion_tb;
 parameter SIDE=1;
 parameter SKIP_SINGLE=0;
 parameter FALSE_LANE=0;
 parameter STALE_POINT=0;
 parameter USE_PROFILE=0;
 parameter NAV_SPEED=160;
 reg clk=0;always #5 clk=~clk;
 reg rst=0,request=0;reg [3:0] mode=0;
 reg [16:0] path_x[0:127],path_y[0:127];
 wire [10:0] index,profile_index;wire signed [1:0] upcoming_turn;wire [7:0] entry_mm;wire profile_busy,profile_ready,profile_exit_valid;
 wire profile_entry_valid;wire [10:0] profile_entry_index;
 wire signed [15:0] profile_heading,profile_exit_heading;wire signed [1:0] profile_curve;
 wire [10:0] read_index=USE_PROFILE && profile_busy?profile_index:index;
 reg [16:0] rx=0,ry=0;
 vehicle_route_profile actual_profile(.clk(clk),.rst_n(rst),.route_valid(1'b1),.route_count(12'd80),
 .route_index(profile_index),.route_x_q8(rx),.route_y_q8(ry),.route_lock(read_index==0 || read_index==79),
 .selected_segment(4'd0),.execution_index(index),.execution_tangent(profile_heading),.execution_curve(profile_curve),
 .execution_upcoming_turn(upcoming_turn),.execution_entry_mm(entry_mm),.execution_entry_valid(profile_entry_valid),
 .execution_entry_index(profile_entry_index),.execution_exit_valid(profile_exit_valid),.execution_exit_heading(profile_exit_heading),.busy(profile_busy),.ready(profile_ready));
 reg signed [31:0] e1=0,e2=0;reg signed [15:0] yaw=0;
 reg signed [15:0] planned_heading=0;
 reg update=0,valid=1;reg [2:0] visibility=3'b111;reg signed [8:0] near_error=0,trend=0;
 wire tick,drive;wire signed [15:0] speed,steer;wire [3:0] status;wire [2:0] visual_state;
 always @(posedge clk)begin
  rx<=path_x[read_index];ry<=path_y[read_index];
  if(index>0)planned_heading<=$rtoi($atan2($itor(path_y[index-1])-$itor(path_y[index]),$itor(path_x[index])-$itor(path_x[index-1]))*1800/3.141592654);
 end
 vehicle_motion #(.CLK_HZ(20000),.STALL_MS(3000)) dut(.route_exit_valid(USE_PROFILE?profile_exit_valid:1'b1),.route_exit_heading(USE_PROFILE?profile_exit_heading:SIDE==1?16'sd900:-16'sd900),.route_curve(USE_PROFILE?profile_curve:index>=16 && index<48?(SIDE==1?2'sd1:-2'sd1):2'sd0),.route_upcoming_turn(USE_PROFILE?upcoming_turn:2'sd0),.route_entry_mm(USE_PROFILE?entry_mm:8'd0),.route_tangent(USE_PROFILE?profile_heading:planned_heading),.imu_restart(1'b0),
 .clk(clk),.rst_n(rst),.mode(mode),.navigation_request(request),.route_valid(1'b1),.profile_ready(USE_PROFILE?profile_ready:1'b1),
 .configured_speed(NAV_SPEED[15:0]),.slowdown_cm(16'd65),.bypass_cm(16'd65),.brake_speed(16'd200),
 .route_entry_valid(USE_PROFILE?profile_entry_valid:1'b0),.route_entry_index_valid(USE_PROFILE?profile_entry_valid:1'b0),
 .route_entry_index(profile_entry_index),
 .route_count(12'd80),.route_index(index),.route_x_q8(rx),.route_y_q8(ry),.route_lock(index==0 || index==79),
 .encoder1(e1),.encoder2(e2),.imu_available(1'b1),.imu_update(1'b1),.imu_yaw(yaw),
 .range_frame(1'b1),.range_samples(4'hf),.range_observations(4'hf),.range_diagnostics(32'd0),
 .left_mm(16'd1200),.right_mm(16'd1200),.left_front_mm(16'd1500),.right_front_mm(16'd1500),
 .remote_link(1'b0),.remote_run(1'b0),.remote_speed(16'sd0),.remote_steer(16'sd0),
 .road_visibility(visibility),.road_update(update),.road_valid(valid),.road_junction(1'b0),.road_near(near_error),.road_heading(trend),
 .zebra_update(1'b0),.zebra_detected(1'b0),.vision_state(visual_state),
 .drive_enabled(drive),.speed_mm_s(speed),.steer_deg(steer),.status(status),.tick_10ms(tick));
 real wx=281,wy=500,angle=0,w1=0,w2=0,ds,kappa,old_angle,t;
 integer applied=0,ticks=0,n;reg probed_stale=0;reg saw_blind=0,saw_reacquire=0;
 always @(negedge clk)if(rst && tick)begin
  ticks=ticks+1;
  if(applied<steer)applied=applied+1;else if(applied>steer)applied=applied-1;
  kappa=$tan(applied*3.141592654/180.0)/142.0;
  ds=drive?speed*0.01/(1+84*$abs(kappa)):0;old_angle=angle;angle=angle+ds*kappa;
  wx=wx+ds*$cos((old_angle+angle)/2);wy=wy+ds*$sin((old_angle+angle)/2);
  // The first curved edge starts at nose x=800 mm. The route target can
  // already name that edge while the nose is still 60 mm before it.
  // A mapped turn must wait for the physical entry, not its RAM address.
  if(USE_PROFILE && !saw_blind && dut.entry_ready_current &&
     wx+219*$cos(angle)<790)
   $fatal(1,"mapped entry accepted before physical plane: nose=%f index=%0d",wx+219*$cos(angle),index);
  w1=w1+ds*(1-84*kappa);w2=w2+ds*(1+84*kappa);
  e1=$rtoi(w1*1560/(66*3.141592654));e2=$rtoi(w2*1560/(66*3.141592654));yaw=$rtoi(angle*1800/3.141592654);
  if(ticks%3==0)begin
   update=1;
   if(wx<581 && !saw_blind)begin valid=1;visibility=SKIP_SINGLE || wx<521?3'b111:SIDE==1?3'b110:3'b101;near_error=0;trend=0;end
   else if(FALSE_LANE && SIDE*angle>0.45 && SIDE*angle<0.75)begin valid=1;visibility=3'b111;near_error=0;trend=0;end
   else if(SIDE*angle<1.35)begin valid=0;visibility=3'b100;end
   else begin
    valid=1;visibility=3'b111;
    // Outgoing physical road x=848; lane errors come from actual pose.
    near_error=$rtoi(256.0*(SIDE*(848-wx)+210*(angle-SIDE*1.570796327))/254.0);
    trend=$rtoi(256.0*110*(angle-SIDE*1.570796327)/254.0);
   end
  end
  if(visual_state==2 && !saw_blind)$display("BLIND entry planned=%0d hint=%0d route=%0d left=%b physicalYaw=%f",planned_heading,dut.vision_turn_hint,dut.route_steer,dut.vision_guidance.blind_left,angle);
  if(visual_state==2)saw_blind=1;
  if(STALE_POINT && visual_state==2 && !probed_stale)begin
   force dut.lateral_mm=16'sd360;force dut.forward_mm=-16'sd100;probed_stale=1;
  end else if(STALE_POINT && probed_stale)begin release dut.lateral_mm;release dut.forward_mm;end
  if(saw_blind && dut.vision_active)saw_reacquire=1;
 end else update=0;
 initial begin
  for(n=0;n<16;n=n+1)begin path_x[n]=(500+n*20)*256/10;path_y[n]=2700*256/10;end
  for(n=16;n<48;n=n+1)begin
   t=(n-15)*1.570796327/32;
   path_x[n]=$rtoi((581+267*$sin(t)+219*$cos(t))*256/10);
   path_y[n]=$rtoi((3200-(500+SIDE*(267*(1-$cos(t))+219*$sin(t))))*256/10);
  end
  for(n=48;n<80;n=n+1)begin
   path_x[n]=848*256/10;path_y[n]=$rtoi((3200-(500+SIDE*(486+(n-47)*20)))*256/10);
  end
  repeat(5)@(negedge clk);rst=1;mode=9;request=1;
  wait(status==2);
  wait(status>=3 || ticks>8000);
  if(status!=3 || !saw_blind || !saw_reacquire || $abs(wx-848)>35 || $abs(angle-SIDE*1.570796327)>0.12)
   $fatal(1,"integrated blind turn side=%0d status=%0d visual=%0d pose=%f,%f yaw=%f index=%0d blind=%b reacquire=%b target=%0d,%0d",SIDE,status,visual_state,wx,wy,angle,index,saw_blind,saw_reacquire,dut.target_x_mm,dut.target_y_mm);
  $display("PASS integrated limited-FOV turn side=%0d: real route progression, blind 28deg turn, visual exit centering, original goal",SIDE);$finish;
 end
 initial begin #200000000;$fatal(1,"integrated turn timeout");end
endmodule
