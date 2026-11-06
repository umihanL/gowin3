`timescale 1ns/1ps
module map_telemetry_tb;
 reg clk=0;always #5 clk=~clk;reg rst=0,rx=1,enabled=1,selected=0;
 reg valid=1,route_valid=1,planning=0;reg [15:0] generation=7,bgeneration=3;
 reg signed [23:0] px=350*256,py=2850*256;reg signed [15:0] heading=900;
 wire tx,run,nav,link;wire [15:0] speed;wire [10:0] index;
 reg [16:0] qx,qy;
 reg [10:0] execution_target=1;
 reg [11:0] execution_count=3;
 reg [3:0] telemetry_status=2;
 always @(posedge clk)begin qx<=(index+1)*2560;qy<=(index+2)*2560;end
 vehicle_host_link #(.CLK_HZ(1000000),.BAUD(100000),.LINK_TIMEOUT_MS(20)) dut(
 .clk(clk),.rst_n(rst),.rx(rx),.selected(selected),.drive_page(1'b1),.navigation_page(1'b0),.navigation_ready(1'b1),
 .planning_page(1'b0),.point_planning(1'b0),.selected_point(4'd2),.point_error(3'd0),
 .motion_status(telemetry_status),.motion_phase(4'd1),.event_valid(1'b0),.extension_event_valid(1'b0),.extension_status(32'd0),
 .map_telemetry_enabled(enabled),.map_mode(4'd9),.map_pose_valid(valid),.map_imu_available(1'b1),
 .map_pose_x_q8(px),.map_pose_y_q8(py),.map_heading(heading),.map_signature(32'h44be0006),
 .map_generation(generation),.map_route_valid(route_valid),.map_planning(planning),.map_route_count(execution_count),
 .map_route_index(index),.map_route_x_q8(qx),.map_route_y_q8(qy),
 .map_navigation_target_index(execution_target),
 .map_barrier_generation(bgeneration),.map_barrier_valid(4'b0010),
 .map_barrier_x0(36'd20<<9),.map_barrier_y0(36'd30<<9),.map_barrier_x1(36'd50<<9),.map_barrier_y1(36'd30<<9),
 .tx(tx),.run(run),.nav_session(nav),.speed_limit(speed),.link_ok(link));
 wire received,error;wire [7:0] data;integer bytes=0;reg [7:0] rb[0:8191];
 host_uart_rx #(.CLK_HZ(1000000),.BAUD(100000)) monitor(.clk(clk),.rst_n(rst),.rx(tx),.valid(received),.data(data),.framing_error(error));
 always @(negedge clk)if(received)begin rb[bytes]=data;bytes=bytes+1;end
 task uart(input [7:0] b);integer j;begin
  @(negedge clk);rx=0;repeat(10)@(negedge clk);
  for(j=0;j<8;j=j+1)begin rx=b[j];repeat(10)@(negedge clk);end
  rx=1;repeat(13)@(negedge clk);
 end endtask
 integer sequence_no=0,last_frame;reg permit_readonly_busy=0;
 task command(input [7:0] op,input [31:0] arg,input [31:0] expected);integer before_bytes,k,t,j;reg found;reg [7:0] sum;begin
  before_bytes=bytes;sequence_no=sequence_no+1;
  uart(8'ha5);uart(8'h5a);uart(1);uart(sequence_no);uart(op);
  uart(arg[7:0]);uart(arg[15:8]);uart(arg[23:16]);uart(arg[31:24]);
  uart(1^sequence_no^op^arg[7:0]^arg[15:8]^arg[23:16]^arg[31:24]);
  found=0;t=0;
  while(!found && t<10000)begin
   @(negedge clk);t=t+1;
   for(k=before_bytes/10;k<bytes/10;k=k+1)if(rb[k*10+3]==sequence_no && rb[k*10+4]==(op|8'h80))begin found=1;last_frame=k;end
  end
  if(!found)$fatal(1,"no map reply %h",op);
  if({rb[last_frame*10+8],rb[last_frame*10+7],rb[last_frame*10+6],rb[last_frame*10+5]}!==expected &&
     !(permit_readonly_busy && op==8'h37 && dut.link_age>=20 &&
       {rb[last_frame*10+8],rb[last_frame*10+7],rb[last_frame*10+6],rb[last_frame*10+5]}===32'hffffffff))
   $fatal(1,"map seq=%d op=%h got=%h expected=%h",sequence_no,op,{rb[last_frame*10+8],rb[last_frame*10+7],rb[last_frame*10+6],rb[last_frame*10+5]},expected);
  sum=0;for(j=2;j<=8;j=j+1)sum=sum^rb[last_frame*10+j];
  if(sum!==rb[last_frame*10+9])$fatal(1,"map reply checksum");
  repeat(30)@(negedge clk);
 end endtask
 integer i;
 initial begin
  repeat(5)@(negedge clk);rst=1;repeat(10)@(negedge clk);
  command(8'h30,0,32'h01ff0100);if(run || link)$fatal(1,"map probe enabled control");
  command(8'h36,0,32'h44be0006);
  command(8'h31,0,32'h00070001);
  command(8'h37,1,32'h00010007);
  execution_target=2;
  command(8'h37,1,32'h00010007); // held with the pose token, not live target
  command(8'h37,2,32'hffffffff);
  command(8'h37,32'h00010001,32'hffffffff); // reserved argument bytes
  px=-25600;py=0;heading=-450;
  command(8'h32,1,{16'd2850,16'd350});command(8'h33,1,32'h12090384);
  command(8'h32,2,32'hffffffff);
  command(8'h31,0,32'h00070002);command(8'h32,2,{16'd0,-16'sd100});
  command(8'h37,1,32'hffffffff);command(8'h37,2,32'h00020007);
  command(8'h33,2,32'h1209fe3e);
  command(8'h34,0,32'h80030007);command(8'h35,32'h00020007,{16'd400,16'd300});
  generation=8;command(8'h35,32'h00020007,32'hffffffff);
  command(8'h37,2,32'hffffffff); // never splice old pose into a new route
  command(8'h35,32'h00030008,32'hffffffff);
  planning=1;command(8'h35,32'h00000008,32'hffffffff);planning=0;
  route_valid=0;command(8'h35,32'h00000008,32'hffffffff);route_valid=1;
  command(8'h38,0,32'h00020003);command(8'h39,32'h00010003,{16'd300,16'd200});
  command(8'h39,32'h01010003,{16'd300,16'd500});bgeneration=4;
  command(8'h39,32'h00010003,32'hffffffff);command(8'h39,32'h00000004,32'hffffffff);
  valid=0;command(8'h31,0,32'h00060003);command(8'h32,3,32'hffffffff);
  command(8'h37,3,32'hffffffff);
  valid=1;execution_target=3;command(8'h31,0,32'h00070004);
  command(8'h37,4,32'hffffffff); // out of range target is not progress
  execution_target=1;command(8'h31,0,32'h00070005);
  command(8'h37,5,32'h00010008);
  planning=1;command(8'h37,5,32'hffffffff);planning=0;
  route_valid=0;command(8'h37,5,32'hffffffff);route_valid=1;
  telemetry_status=3;command(8'h31,0,32'h00070006);
  command(8'h37,6,32'h00010008);command(8'h33,6,32'h1309fe3e);
  telemetry_status=2; // final DONE pose/progress is readable without a new START
  execution_count=310;execution_target=300;command(8'h31,0,32'h00070007);
  command(8'h37,7,32'h012c0008); // multi-checkpoint routes exceed 255 samples
  execution_count=2048;execution_target=2047;command(8'h31,0,32'h00070008);
  command(8'h37,8,32'h07ff0008);
  selected=1;repeat(5)@(negedge clk);
  command(8'h03,32'h00000700,32'h00c80300);
  if(!run)$fatal(1,"legacy start failed");
  // Read-only map traffic must never refresh the driving watchdog.
  // Its timeout event may occupy TX; a busy read returns the documented FF
  // sentinel instead of fabricating progress. Normal queries above stay exact.
  permit_readonly_busy=1;
  for(i=0;i<12;i=i+1)command(8'h37,8,32'h07ff0008);
  if(run || link)$fatal(1,"map queries masked missing control heartbeat");
  $display("PASS map UART: coherent pose/target snapshots, stale token/generation/index rejection, real units, read-only watchdog isolation");$finish;
 end
 initial begin #10000000;$fatal(1,"map telemetry timeout");end
endmodule
