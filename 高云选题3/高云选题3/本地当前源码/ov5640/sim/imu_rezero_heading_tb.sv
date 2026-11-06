`timescale 1ns/1ps
module imu_rezero_heading_tb;
 reg clk=0;always #5 clk=~clk;
 reg rst=0,available=1,restart=0,rezero=0,command=0,tv=0,td=0,snap_done=0,point_start=0;
 reg [15:0] tx=0,ty=0;
 reg signed [15:0] yaw=375;reg [7:0] opcode=0;reg [31:0] args=0;
 wire valid;wire signed [15:0] heading;wire [7:0] result;
 nav_control dut(.clk(clk),.rst_n(rst),.edit_distance(1'b0),
   .host_selected(1'b1),.host_command(command),.host_opcode(opcode),.host_arguments(args),
   .host_motion_status(4'd0),.imu_available(available),.imu_restart(restart),.imu_yaw(yaw),
`ifdef REZERO_SUPPORTED
   .imu_rezero(rezero),
`endif
   .start_heading_valid(valid),.start_heading(heading),.host_result(result),
   .touch_valid(tv),.touch_down(td),.touch_x(tx),.touch_y(ty),.host_point_select(1'b0),.host_point_start(point_start),.host_point_cancel(1'b0),
   .planned_anchor_valid(1'b0),.refine_done(1'b0),.snap_done(snap_done),.snap_accept(1'b1),.snapped_x(8'd20),.snapped_y(8'd20),.snapped_heading_valid(1'b1),.snapped_heading(16'sd900),.zebra_can_change(1'b0));
 task send(input [7:0] op,input [31:0] value);begin
  wait(dut.state==1);@(negedge clk);opcode=op;args=value;#1;
  if(result!=0)$fatal(1,"unexpected command rejection %h %d",op,result);
  command=1;@(negedge clk);command=0;
 end endtask
 task set_heading(input integer value);begin send(8'h24,{16'd0,value[15:0]});end endtask
 task zero_sensor;begin
  @(negedge clk);rezero=1;@(negedge clk);rezero=0;available=0;yaw=0;
  repeat(6)@(negedge clk);
  if(valid)$fatal(1,"heading valid while calibrating");
  // Calibration's provisional raw orientation must not rotate the map arrow.
  yaw=-1200;repeat(4)@(negedge clk);yaw=0;available=1;
  repeat(4)@(negedge clk);
 end endtask
 task tap(input integer x,y);begin
  @(negedge clk);tx=x;ty=y;tv=1;td=1;@(negedge clk);td=0;@(negedge clk);tv=0;
 end endtask
 integer keep;
 initial begin
  repeat(5)@(negedge clk);rst=1;wait(dut.state==1);
  // Explicit map +90 at arbitrary sensor yaw +37.5, then a stationary S0.
  send(8'h20,32'h00000d58);wait(dut.state==1);set_heading(900);
  if(!valid || heading!=900)$fatal(1,"initial declaration wrong");
`ifdef REZERO_SUPPORTED
  // S0 and a conflicting heading write on the same clock must reject
  // the write and keep the last actual map direction. Availability may lag.
  @(negedge clk);rezero=1;opcode=8'h24;args=0;command=1;#1;
  if(result!=7 || valid)$fatal(1,"zero/heading-write race accepted stale input");
  @(negedge clk);rezero=0;command=0;repeat(4)@(negedge clk);
  if(valid)$fatal(1,"pending zero released before availability went low");
  available=0;yaw=0;repeat(4)@(negedge clk);available=1;repeat(4)@(negedge clk);
`else
  zero_sensor;
`endif
  if(!valid || heading!=900)$fatal(1,"REZERO_MAP_HEADING_CHANGED actual=%0d expected=900",heading);
  yaw=100;repeat(3)@(negedge clk);
  if(heading!=1000)$fatal(1,"new epoch relative turn lost: %0d",heading);
  // A second stationary zero preserves current map direction, not the old raw origin.
  zero_sensor;
  if(!valid || heading!=1000)$fatal(1,"repeat zero changed heading: %0d",heading);
  // Normal user order: zero first, then confirm the left-facing map arrow.
  set_heading(1800);yaw=100;repeat(3)@(negedge clk);
  if(heading!=-1700)$fatal(1,"+180 crossing wrong: %0d",heading);
  zero_sensor;if(heading!=-1700)$fatal(1,"wrapped direction lost");
  yaw=-200;repeat(3)@(negedge clk);
  if(heading!=1700)$fatal(1,"-180 crossing wrong: %0d",heading);
`ifdef REZERO_SUPPORTED
  // Exactly concurrent START/S0 cannot execute a previously ready route.
  force dut.route_ready=1'b1;force dut.have_goal=1'b1;
  @(negedge clk);point_start=1;rezero=1;#1;
  if(dut.host_point_route_ready)$fatal(1,"zero/start race still reports ready");
  @(negedge clk);point_start=0;rezero=0;available=0;yaw=0;
  repeat(3)@(negedge clk);
  if(dut.navigation_active)$fatal(1,"zero/start race launched movement");
  release dut.route_ready;release dut.have_goal;
  available=1;repeat(4)@(negedge clk);
  // Selecting and rotating a touchscreen start while S0 is still acquiring
  // zero must wait for the final raw yaw, never latch provisional orientation.
  @(negedge clk);rezero=1;@(negedge clk);rezero=0;available=0;yaw=-700;
  repeat(4)@(negedge clk);tap(140,180);wait(dut.state==3);
  @(negedge clk);snap_done=1;@(negedge clk);snap_done=0;repeat(3)@(negedge clk);
  if(valid || heading!=900)$fatal(1,"touch start bound to provisional calibration yaw");
  tap(65,153);repeat(3)@(negedge clk);
  if(valid || heading!=1800)$fatal(1,"pending touch rotation changed physical reference");
  yaw=0;available=1;repeat(4)@(negedge clk);
  if(!valid || heading!=1800)$fatal(1,"touch calibration completion wrong heading");
`endif
  // Sensor recovery is not an operator stationary zero.
  restart=1;@(negedge clk);restart=0;repeat(3)@(negedge clk);
  if(valid || dut.route_ready)$fatal(1,"unexpected restart kept reference/route");
  $display("PASS manual zero preserves map direction, relative turns and wrap; unexpected restart invalidates");$finish;
 end
 initial begin #3000000;$fatal(1,"zero test timeout");end
endmodule
