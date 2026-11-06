`timescale 1ns/1ps
module heading_navigation_tb;
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
 reg signed [15:0] imu_yaw=375;reg imu_restart=0;wire start_heading_valid;wire signed [15:0] start_heading;
 wire [31:0] map_signature;wire [15:0] map_generation,barrier_generation;
 wire map_route_valid,map_planning;wire [10:0] map_index;wire [16:0] map_x,map_y;
 wire [3:0] barrier_valid;wire [35:0] bx0,by0,bx1,by1;
 navigation_ui ui(.imu_available(1'b1),.imu_restart(imu_restart),.imu_yaw(imu_yaw),.start_heading_valid(start_heading_valid),.start_heading(start_heading),.clk(clk),.rst_n(rst),.data_req(req),.x_addr(vx),.y_addr(vy),.video(16'd0),.preview(16'd0),.zebra_enable(zebra_enabled),.zebra_can_change(zebra_can_change),.zebra_toggle(zebra_toggle),
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
 integer heading=1800,first_x,first_y,second_x,second_y,loops;
 reg [15:0] heading_word;
 initial begin
  if(!$value$plusargs("heading=%d",heading))heading=1800;
  force car.encoder_position1=0;force car.encoder_position2=0;
  repeat(5)@(negedge clk);rst=1;wait(ui.control.state==1);
  tap(700,40);tap(200,180);tap(200,400);if(!host)$fatal(1,"communication mode selection");
  tap(32,32);tap(32,32);tap(180,452);if(mode!=2 || !host)$fatal(1,"extension entry unavailable");
  command(8'h30,0,0,0);if((rb[last_frame*10+7]&16)==0)$fatal(1,"heading capability absent");
  command(8'h12,2,0,0);repeat(150000)@(negedge clk);
  command(8'h20,88,13,0);wait(ui.control.state==1);repeat(5)@(negedge clk);
  command(8,4,41,7);if(planning || active)$fatal(1,"unknown heading planned/moved");
  command(8'h24,9,7,2); // 180.1 degrees rejected, not clamped.
  // Anchor a slightly different initial angle, then rotate the stationary car
  // through IMU feedback before planning. This exercises the actual reference,
  // not a route-derived heading or forced internal planner register.
  heading_word=heading-100;
  command(8'h24,heading_word[7:0],heading_word[15:8],0);
  if({rb[last_frame*10+7],rb[last_frame*10+6]}!=heading_word || rb[last_frame*10+8]!=0)$fatal(1,"heading ACK echo");
  imu_yaw=475;repeat(10)@(negedge clk);
  if(!start_heading_valid || start_heading!=heading)$fatal(1,"IMU map-reference tracking got %d",start_heading);
  command(8,4,41,0);
  if($test$plusargs("unsupported"))begin
   wait(!planning);repeat(5)@(negedge clk);
   if(point_error!=2 || route_valid || active || car.drive_enabled)$fatal(1,"perpendicular start created executable route");
   $display("PASS no legal start tangent: perpendicular heading rejected without fallback");$finish;
  end
  wait_plan;
  force car.route_index=0;repeat(4)@(negedge clk);first_x=route_x;first_y=route_y;
  force car.route_index=1;repeat(4)@(negedge clk);second_x=route_x;second_y=route_y;
  release car.route_index;
  if((heading==1800 && second_x>=first_x)||(heading==0 && second_x<=first_x))
   $fatal(1,"8->4 wrong start direction heading=%d first=%d second=%d",heading,first_x,second_x);
  if(second_y!=first_y)$fatal(1,"8->4 did not start tangent to top straight");
  if($test$plusargs("rotate_after_plan"))begin
   imu_yaw=2275;repeat(10)@(negedge clk);
   command(3,1,41,0);repeat(130000)@(negedge clk);
   if(status!=6 || car.drive_enabled || speed)$fatal(1,"car rotated after plan was silently re-aligned");
   $display("PASS stale route direction: 180-degree physical rotation after planning refused at motion startup");$finish;
  end
  command(3,1,41,0);repeat(130000)@(negedge clk);
  if(mode!=9 || status!=2 || !car.drive_enabled || speed<=0)$fatal(1,"navigation did not arm status=%d state=%d",status,car.motion.state);
  if(car.motion.initial_heading!=heading || car.motion.pose_heading!=heading)$fatal(1,"route overwrote real heading init=%d pose=%d",car.motion.initial_heading,car.motion.pose_heading);
  command(5,0,0,0);repeat(20000)@(negedge clk);
  if(speed || car.drive_enabled)$fatal(1,"stop failed");
  $display("PASS heading-aware UART/UI/planner/motion: 8->4 heading=%d first_dx=%d path_count=%d length_cm=%d",heading,second_x-first_x,route_count,ui.refined_length);
  // A sensor reset requires a fresh physical heading declaration.
  imu_restart=1;@(negedge clk);imu_restart=0;repeat(3)@(negedge clk);
  if(start_heading_valid)$fatal(1,"IMU restart retained stale map heading");
  $finish;
 end
 initial begin #400000000;$fatal(1,"heading integration timeout state=%d",ui.road_planner.state);end
endmodule
