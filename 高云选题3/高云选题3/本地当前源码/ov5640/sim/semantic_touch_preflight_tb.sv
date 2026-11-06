`timescale 1ns/1ps
module semantic_touch_preflight_tb;
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
 reg signed [15:0] imu_yaw=375;reg imu_restart=0;wire [31:0] map_start_edit_generation;wire map_start_valid;wire [7:0] map_start_x,map_start_y;wire start_heading_valid;wire signed [15:0] start_heading;
 wire [31:0] map_signature;wire [15:0] map_generation,barrier_generation;
 wire map_route_valid,map_planning;wire [10:0] map_index;wire [16:0] map_x,map_y;
 wire [3:0] barrier_valid;wire [35:0] bx0,by0,bx1,by1;
 navigation_ui ui(.imu_available(1'b1),.imu_restart(imu_restart),.imu_yaw(imu_yaw),.start_heading_valid(start_heading_valid),.start_heading(start_heading),.map_start_valid(map_start_valid),.map_start_x(map_start_x),.map_start_y(map_start_y),.map_start_edit_generation(map_start_edit_generation),.clk(clk),.rst_n(rst),.data_req(req),.x_addr(vx),.y_addr(vy),.video(16'd0),.preview(16'd0),.zebra_enable(zebra_enabled),.zebra_can_change(zebra_can_change),.zebra_toggle(zebra_toggle),
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
 .profile_ready(profile_ready),.motion_status(status),.motion_phase(phase),.pose_x_q8(pose_x),.pose_y_q8(pose_y),.pose_heading(start_heading));
 vehicle_subsystem #(.CLK_HZ(1000000),.HOST_BAUD(100000)) car(.start_heading_valid(start_heading_valid),.start_heading(start_heading),.map_start_valid(map_start_valid),.map_start_x(map_start_x),.map_start_y(map_start_y),.map_start_edit_generation(map_start_edit_generation),.clk(clk),.rst_n(rst),.mode(mode),
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
 .imu_available(1'b1),.imu_update(1'b1),.imu_restart(imu_restart),.imu_yaw(imu_yaw),
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
   if(speed || active || car.drive_enabled)$fatal(1,"motion during planning speed=%d active=%b drive=%b mode=%d",speed,active,car.drive_enabled,mode);
   if(chunks>900)$fatal(1,"planner stuck state=%0d error=%0d",ui.road_planner.state,point_error);
   if(!planning && point_error)$fatal(1,"planner failed error=%0d",point_error);
  end
  if(!route_valid || route_count<2 || !point_ready)$fatal(1,"missing ready route count=%0d",route_count);
 end endtask
 task snapshot(input [1023:0] filename);integer f,a,b;begin
  f=$fopen(filename,"wb");$fwrite(f,"P6\n800 480\n255\n");req=1;
  for(b=0;b<480;b=b+1)for(a=0;a<800;a=a+1)begin
   @(negedge clk);vx=a;vy=b;repeat(2)@(negedge clk);#1;
   $fwrite(f,"%c%c%c",{pixel[4:0],pixel[4:2]},{pixel[10:5],pixel[10:9]},{pixel[15:11],pixel[15:13]});
  end
  $fclose(f);req=0;
 end endtask

 defparam car.motion.START_DELAY_MS=1;
 reg signed [23:0] saved_x,saved_y;integer n,default_heading,expected_heading;
 initial begin
  force car.encoder_position1=0;force car.encoder_position2=0;
  force car.actual_speed1=0;force car.actual_speed2=0;
  repeat(5)@(negedge clk);rst=1;wait(ui.control.state==1);
  tap(700,40);tap(200,180);tap(200,400);if(!host)$fatal(1,"communication mode selection");
  tap(32,32);tap(32,32);tap(180,452);
  command(8'h30,0,0,0);if((rb[last_frame*10+7]&96)!=96)$fatal(1,"area/region preference API capability absent");
  command(8'h3a,0,0,255);if(rb[last_frame*10+6]!=255)$fatal(1,"invalid start query");
  command(8'h12,2,0,0);repeat(150000)@(negedge clk);
  command(8'h20,88,13,0);wait(ui.control.state==1);repeat(5)@(negedge clk);
  // Host SET_START still needs an explicit heading. Touching existing start
  // declares right, then successive distinct taps turn left 90 degrees.
  if(start_heading_valid || map_start_edit_generation!=0)$fatal(1,"host start invented heading/edit");
  tap(201,139);wait(ui.control.state==1);repeat(5)@(negedge clk);
  if(!start_heading_valid)$fatal(1,"touch road heading initialize");
  default_heading=start_heading;expected_heading=default_heading;
  for(n=0;n<4;n=n+1)begin
   expected_heading=expected_heading>900?expected_heading-2700:expected_heading+900;
   tap(201,139);if(start_heading!=expected_heading)$fatal(1,"CCW 90 rotation / wrap");
  end
  expected_heading=default_heading>900?default_heading-2700:default_heading+900;
  @(negedge clk);tx=201;ty=139;tv=1;td=1;repeat(100)@(negedge clk);
  if(start_heading!=expected_heading)$fatal(1,"held touch repeated rotation");
  td=0;@(negedge clk);tv=0;repeat(10)@(negedge clk);
  if(map_start_edit_generation!=6)$fatal(1,"physical taps generation");
  command(8'h24,8'h08,7,0);
  command(8'h3b,0,0,6);
  command(8'h3a,0,0,88);
  if(rb[last_frame*10+6]!=13 || {rb[last_frame*10+8],rb[last_frame*10+7]}!=1800)$fatal(1,"start reference fields");
  command(8'h20,88,13,0);wait(ui.control.state==1);
  if(start_heading_valid)$fatal(1,"host reset heading unexpectedly valid");
  command(8'h3b,0,0,6);command(8'h3a,0,0,255);
  command(8'h26,100,0,2,80,159);
  if(ui.control.region_preference_enabled)$fatal(1,"invalid region bounds accepted");
  command(8'h26,0,80,0,88,159);
  if(!ui.control.region_preference_enabled || ui.control.region_y0!=80 || ui.control.region_x1!=88)
    $fatal(1,"region preference not routed from UART to planner config");
  command(8'h20,56,13,0);wait(ui.control.state==1);
  if(!ui.control.region_preference_enabled)$fatal(1,"SET_START lost region preference");
  command(8'h26,255,255,0,255,255);
  if(ui.control.region_preference_enabled)$fatal(1,"explicit region clear ignored");
  command(8'h26,0,80,0,88,159);
  command(8'h25,0,0,0);wait(ui.control.state==1);
  if(ui.control.region_preference_enabled)$fatal(1,"RESET_NAV retained prior region preference");
  if(speed || car.drive_enabled || active)$fatal(1,"region configuration caused vehicle movement");
  $display("PASS UART region preference: capability, validation, set/start retention, explicit clear/reset; no motion");
  $display("PASS semantic touch preflight: snapped initial tangent, four CCW rotations, hold debounce, 3A/3B and host changes preserve physical generation");$finish;
 end
 initial begin #1000000000;$fatal(1,"semantic touch timeout");end
endmodule
