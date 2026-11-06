`timescale 1ns/1ps
module host_points_tb;
 reg clk=0;always #5 clk=~clk;
 reg rst=0,rx=1,tv=0,td=0,req=0;reg [15:0] tx=0,ty=0;reg [11:0] vx=0,vy=0;
 wire [15:0] pixel;wire [3:0] mode,point_id,selected_point,status,phase;wire host,remote;
 wire select_point,start_point,cancel_point,select_ready,point_ready,planning,active,route_valid;
 wire route_command,route_page;wire [7:0] route_opcode,route_result,route_status;wire [31:0] route_arguments;
 wire zebra_enabled,zebra_can_change,zebra_toggle;
 wire [2:0] point_error;wire [10:0] route_index,center;wire [11:0] route_count;
 wire [16:0] route_x,route_y;wire route_lock,profile_ready;wire uart_tx;
 wire [15:0] speed_limit,slowdown,bypass,brake;wire navigation_avoidance;
 wire signed [15:0] speed,steer;wire signed [23:0] pose_x,pose_y;tri1 sda,scl;
 wire start_heading_valid;wire signed [15:0] start_heading;
 wire [31:0] map_signature;wire [15:0] map_generation,barrier_generation;
 wire map_route_valid,map_planning;wire [10:0] map_index;wire [16:0] map_x,map_y;
 wire [3:0] barrier_valid;wire [35:0] bx0,by0,bx1,by1;
 navigation_ui ui(.imu_available(1'b1),.imu_restart(1'b0),.imu_yaw(16'sd0),.start_heading_valid(start_heading_valid),.start_heading(start_heading),.clk(clk),.rst_n(rst),.data_req(req),.x_addr(vx),.y_addr(vy),.video(16'd0),.preview(16'd0),.zebra_enable(zebra_enabled),.zebra_can_change(zebra_can_change),.zebra_toggle(zebra_toggle),
 .host_route_command(route_command),.host_route_opcode(route_opcode),.host_route_arguments(route_arguments),
 .host_route_page(route_page),.host_route_result(route_result),.host_route_status(route_status),
 .touch_valid(tv),.touch_down(td),.touch_x(tx),.touch_y(ty),.pixel(pixel),.operation_mode(mode),
 .speed_edit(1'b0),.speed_increase(1'b0),.speed_decrease(1'b0),.control_remote(remote),.control_host(host),
 .navigation_avoidance(navigation_avoidance),.configured_speed(speed_limit),.slowdown_cm(slowdown),.bypass_cm(bypass),.brake_speed(brake),
 .servo_center_tenths(center),.navigation_request(active),.route_valid(route_valid),
 .host_point_select(select_point),.host_point_start(start_point),.host_point_cancel(cancel_point),.host_point_id(point_id),
 .host_point_select_ready(select_ready),.host_point_route_ready(point_ready),.host_point_planning(planning),
 .selected_point(selected_point),.host_point_error(point_error),
 .execution_index(route_index),.execution_count(route_count),.execution_x_q8(route_x),.execution_y_q8(route_y),.execution_lock(route_lock),
 .map_signature(map_signature),.map_generation(map_generation),.map_route_valid(map_route_valid),.map_planning(map_planning),
 .map_route_index(map_index),.map_route_x_q8(map_x),.map_route_y_q8(map_y),
 .map_barrier_generation(barrier_generation),.map_barrier_valid(barrier_valid),.map_barrier_x0(bx0),.map_barrier_y0(by0),.map_barrier_x1(bx1),.map_barrier_y1(by1),
 .profile_ready(profile_ready),.motion_status(status),.motion_phase(phase));
 vehicle_subsystem #(.CLK_HZ(1000000),.HOST_BAUD(100000)) car(.start_heading_valid(start_heading_valid),.start_heading(start_heading),.clk(clk),.rst_n(rst),.mode(mode),
 .zebra_toggle(zebra_toggle),.zebra_enable(zebra_enabled),.zebra_can_change(zebra_can_change),
 .host_route_command(route_command),.host_route_opcode(route_opcode),.host_route_arguments(route_arguments),
 .host_route_page(route_page),.host_route_result(route_result),.host_route_status(route_status),
 .control_remote(remote),.control_host(host),.navigation_avoidance(navigation_avoidance),.host_rx(rx),.host_tx(uart_tx),
 .host_point_select(select_point),.host_point_start(start_point),.host_point_cancel(cancel_point),.host_point_id(point_id),
 .host_point_select_ready(select_ready),.host_point_route_ready(point_ready),.host_point_planning(planning),
 .selected_point(selected_point),.host_point_error(point_error),
 .configured_speed(speed_limit),.slowdown_cm(slowdown),.bypass_cm(bypass),.brake_speed(brake),.servo_center_tenths(center),
 .touch_run(1'b0),.touch_speed(16'sd0),.touch_steer(16'sd0),.navigation_request(active),.route_valid(route_valid),
 .route_count(route_count),.route_index(route_index),.route_x_q8(route_x),.route_y_q8(route_y),.route_lock(route_lock),
 .map_signature(map_signature),.map_generation(map_generation),.map_route_valid(map_route_valid),.map_planning(map_planning),
 .map_route_index(map_index),.map_route_x_q8(map_x),.map_route_y_q8(map_y),
 .map_barrier_generation(barrier_generation),.map_barrier_valid(barrier_valid),.map_barrier_x0(bx0),.map_barrier_y0(by0),.map_barrier_x1(bx1),.map_barrier_y1(by1),
 .imu_available(1'b1),.imu_update(1'b1),.imu_restart(1'b0),.imu_yaw(16'sd0),
 .encoder_A(1'b0),.encoder_B(1'b0),.encoder2_A(1'b0),.encoder2_B(1'b0),
 .ps2_miso(1'b1),.tof_sda(sda),.tof_scl(scl),.zebra_update(1'b0),.zebra_detected(1'b0),
 .profile_ready(profile_ready),.status(status),.phase(phase),.commanded_speed(speed),.commanded_steer(steer),.pose_x_q8(pose_x),.pose_y_q8(pose_y));
 wire received,rx_error;wire [7:0] data;integer byte_count=0;reg [7:0] rb[0:16383];
 host_uart_rx #(.CLK_HZ(1000000),.BAUD(100000)) monitor(.clk(clk),.rst_n(rst),.rx(uart_tx),.valid(received),.data(data),.framing_error(rx_error));
 always @(negedge clk)if(received)begin rb[byte_count]=data;byte_count=byte_count+1;end
 task tap(input integer a,b);begin
  @(negedge clk);tx=a;ty=b;tv=1;td=1;@(negedge clk);tv=0;repeat(7)@(negedge clk);
  tv=1;td=0;@(negedge clk);tv=0;repeat(7)@(negedge clk);
 end endtask
 task uart(input [7:0] b);integer i;begin
  @(negedge clk);rx=0;repeat(10)@(negedge clk);
  for(i=0;i<8;i=i+1)begin rx=b[i];repeat(10)@(negedge clk);end
  rx=1;repeat(13)@(negedge clk);
 end endtask
 integer sequence_no=0,last_frame;
 task command(input [7:0] op,a,b,input integer expected,input [7:0] c=0,d=0);integer before_bytes,k,j,timeout;reg [7:0] sum;reg found;begin
  before_bytes=byte_count;sequence_no=sequence_no+1;
  uart(8'ha5);uart(8'h5a);uart(1);uart(sequence_no);uart(op);uart(a);uart(b);uart(c);uart(d);uart(1^sequence_no^op^a^b^c^d);
  timeout=0;found=0;
  while(!found && timeout<10000)begin
   @(negedge clk);timeout=timeout+1;
   for(k=before_bytes/10;k<byte_count/10;k=k+1)if(rb[k*10+3]==(sequence_no&255) && rb[k*10+4]==(op|8'h80))begin found=1;last_frame=k;end
  end
  if(!found)$fatal(1,"no ACK op=%h bytes=%0d",op,byte_count);
  sum=0;for(j=2;j<=8;j=j+1)sum=sum^rb[last_frame*10+j];
  if(sum!=rb[last_frame*10+9] || rb[last_frame*10+5]!=expected)$fatal(1,"ACK op=%h result=%0d expected=%0d",op,rb[last_frame*10+5],expected);
  repeat(40)@(negedge clk);
 end endtask
 task wait_plan;integer chunks;begin
  chunks=0;
  while(planning || !profile_ready)begin
   repeat(20000)@(negedge clk);command(1,0,0,0);chunks=chunks+1;
   if(speed || active || car.drive_enabled)$fatal(1,"motion during planning");
   if(chunks>900)$fatal(1,"planner stuck state=%0d error=%0d",ui.road_planner.state,point_error);
   if(!planning && point_error)$fatal(1,"planner failed error=%0d",point_error);
  end
  if(!route_valid || route_count<2 || !point_ready)$fatal(1,"missing ready route count=%0d",route_count);
 end endtask
 task snapshot;integer f,a,b;begin
  f=$fopen("sim/host_points_map.ppm","wb");$fwrite(f,"P6\n800 480\n255\n");req=1;
  for(b=0;b<480;b=b+1)for(a=0;a<800;a=a+1)begin
   @(negedge clk);vx=a;vy=b;repeat(2)@(negedge clk);#1;
   $fwrite(f,"%c%c%c",{pixel[4:0],pixel[4:2]},{pixel[10:5],pixel[10:9]},{pixel[15:11],pixel[15:13]});
  end
  $fclose(f);req=0;
 end endtask
 reg [8*128-1:0] route_filename;integer target=1,route_file,i,saved_index,expected_x,expected_y;reg [15:0] saved_xy,saved_generation;reg signed [23:0] saved_pose;
 initial begin
  if(!$value$plusargs("point=%d",target))target=1;
  force car.encoder_position1=0;force car.encoder_position2=0;
  repeat(5)@(negedge clk);rst=1;wait(ui.control.state==1);
  // All interactions use the production touch and serial paths.
  tap(700,40);tap(200,180);tap(200,400);if(!host)$fatal(1,"communication mode selection");
  tap(32,32);tap(32,32);tap(180,452);if(mode!=2 || !host)$fatal(1,"extension entry unavailable");
  command(8'h15,0,0,0);if(rb[last_frame*10+6]!=1 || rb[last_frame*10+7]!=63)$fatal(1,"numbered merge lost extension capabilities");
  command(8'h30,0,0,0);command(8'h36,0,0,6);
  if({rb[last_frame*10+8],rb[last_frame*10+7],rb[last_frame*10+6],rb[last_frame*10+5]}!=32'h44be0006)$fatal(1,"map identity mismatch");
  command(8'h12,0,0,0);if(car.extension_mode!=0 || car.host_run)$fatal(1,"mode selection unexpectedly started car");
  tap(32,32);tap(580,340);tap(180,164);if(mode!=5)$fatal(1,"map entry");
  command(8,target,41,7);if(selected_point || planning)$fatal(1,"missing start accepted");
  tap(255,139);wait(ui.control.state==1);repeat(5)@(negedge clk);
  if(!ui.have_start || ui.have_goal)$fatal(1,"user start only setup");
  command(8'h24,0,0,0); // Explicit map-right start heading before selecting a route.
  command(8,0,41,2);command(8,10,41,2);
  command(8,target,41,0);
  if(selected_point!=target || !planning || ui.count!=0 || !ui.have_goal)$fatal(1,"destination not committed");
  command(3,1,41,3);command(8,target,41,4);
  wait_plan;command(9,0,0,0);
  if(rb[last_frame*10+6]!=target || rb[last_frame*10+7]!=2)$fatal(1,"route query not ready");
  $sformat(route_filename,"sim/host_point_%0d_route.csv",target);route_file=$fopen(route_filename,"w");$fwrite(route_file,"index,x_q8,y_q8,lock\n");
  for(i=0;i<route_count;i=i+1)begin
   force car.route_index=i;repeat(4)@(negedge clk);
   $fwrite(route_file,"%0d,%0d,%0d,%0d\n",i,route_x,route_y,route_lock);
  end
  release car.route_index;$fclose(route_file);
  // UART map samples use their own read port and do not disturb profile/motion.
  saved_generation=map_generation;expected_x=(route_x*10)>>8;expected_y=(route_y*10)>>8;
  command(8'h35,saved_generation[7:0],saved_generation[15:8],expected_x&255,(route_count-1)&255,(route_count-1)>>8);
  if({rb[last_frame*10+6],rb[last_frame*10+5]}!=expected_x || {rb[last_frame*10+8],rb[last_frame*10+7]}!=expected_y)$fatal(1,"UART route differs from real planner RAM");
  command(8'h35,saved_generation[7:0]-1,saved_generation[15:8],255,0,0);
  if(target==9 && $test$plusargs("snapshot"))begin snapshot;command(1,0,0,0);end
  command(3,1,41,0);repeat(130000)@(negedge clk);
  if(mode!=9 || !active || !car.host_nav_session || !car.host_run || !car.host_link_ok || status!=2 || speed<=0 || !car.drive_enabled)
   $fatal(1,"auto handoff lost session mode=%0d active=%b session=%b run=%b link=%b status=%0d speed=%0d",mode,active,car.host_nav_session,car.host_run,car.host_link_ok,status,speed);
  saved_index=car.motion_index;saved_pose=pose_x;
  command(8'h31,0,0,1);if(!rb[last_frame*10+7][0])$fatal(1,"map pose not valid after actual odometry commit");
  command(8'h32,1,0,(pose_x>>>8)&255);
  if({rb[last_frame*10+6],rb[last_frame*10+5]}!=(pose_x>>>8) || {rb[last_frame*10+8],rb[last_frame*10+7]}!=(pose_y>>>8))$fatal(1,"map pose does not match measured odometry");
  command(3,1,41,4);command(8,1,42,1);
  command(5,0,0,0);repeat(20000)@(negedge clk);
  if(speed || car.drive_enabled || !car.host_nav_session || car.motion_index!=saved_index || pose_x!=saved_pose)$fatal(1,"pause changed route/pose");
  command(7,0,0,0);repeat(20000)@(negedge clk);if(speed<=0)$fatal(1,"resume failed");
  command(1,0,0,0);
  force car.status=3;repeat(2400)@(negedge clk);
  if(car.host_run || rb[(byte_count/10-1)*10+4]!=8'hf0)$fatal(1,"completion event missing");
  release car.status;command(3,1,41,4);
  tap(560,396);if(mode!=4 || active)$fatal(1,"navigation stop page");
  repeat(5)@(negedge clk);if(car.host_nav_session || car.host_run)$fatal(1,"leaving page retained session");
  // A numbered-point session must not remove the original uploaded-route API.
  if(target==9)begin
   tap(32,32);tap(180,452);repeat(150000)@(negedge clk);
   command(8'h12,2,0,0);repeat(150000)@(negedge clk);
   // 14 returns the phase in D0, not a generic ACK error code.
   // Every stop request starts a fresh 100 ms wheel-still confirmation.
   command(8'h14,0,0,0);if(!rb[last_frame*10+6][1])$fatal(1,"missing measured-stop flag before route upload");
   command(8'h20,115,13,0);wait(ui.control.state==1);repeat(5)@(negedge clk);
   if(mode!=5 || !route_page || selected_point || !ui.have_start || car.host_run)$fatal(1,"uploaded BEGIN route after numbered session");
   command(8'h24,0,0,0);
   command(8'h21,163,80,0);command(8'h22,88,73,0);
   if(ui.count!=1 || !ui.have_goal)$fatal(1,"uploaded waypoint/goal API lost");
   command(8'h23,0,0,0);if(!planning || car.drive_enabled)$fatal(1,"uploaded plan not accepted or moving early");
   command(5,0,0,0);repeat(150000)@(negedge clk);
   if(planning || active || car.host_run)$fatal(1,"uploaded route STOP cancellation");
   command(8,2,42,0);if(selected_point!=2 || route_page || !planning)$fatal(1,"numbered select did not replace upload ownership");
   command(5,0,0,0);if(planning || car.drive_enabled)$fatal(1,"final plan STOP failed");
   $display("PASS combined real UART/UI: extension 12/14/15, numbered 08/09, explicit 03 handoff, and uploaded 20/21/22/23 cancellation coexist");
  end
  $display("PASS numbered point %0d: user start only, UART target, real planner/profile, auto handoff, no premature motion, pause/resume, duplicate rejection and completion",target);$finish;
 end
 initial begin #300000000;$fatal(1,"point integration timeout mode=%0d state=%0d",mode,ui.road_planner.state);end
endmodule
