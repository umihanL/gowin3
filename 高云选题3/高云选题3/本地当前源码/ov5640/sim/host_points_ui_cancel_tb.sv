`timescale 1ns/1ps
module host_points_ui_cancel_tb;
 reg clk=0;always #5 clk=~clk;reg rst=0,tv=0,td=0,select_p=0,cancel_p=0,start_p=0;
 reg heading_cmd=0;reg [15:0] tx=0,ty=0;wire [3:0] mode,point;wire planning,ready,can_select,can_start,active;wire [2:0] error;
 navigation_ui ui(.clk(clk),.rst_n(rst),.data_req(1'b0),.x_addr(12'd0),.y_addr(12'd0),.video(16'd0),.preview(16'd0),
 .zebra_enable(1'b0),.zebra_can_change(1'b1),.imu_available(1'b1),.imu_restart(1'b0),.imu_yaw(16'sd0),.host_route_command(heading_cmd),.host_route_opcode(8'h24),.host_route_arguments(32'd0),.touch_valid(tv),.touch_down(td),.touch_x(tx),.touch_y(ty),.operation_mode(mode),
 .speed_edit(1'b0),.speed_increase(1'b0),.speed_decrease(1'b0),.execution_index(11'd0),
 .host_point_select(select_p),.host_point_start(start_p),.host_point_cancel(cancel_p),.host_point_id(4'd9),
 .host_point_select_ready(can_select),.host_point_route_ready(can_start),.host_point_planning(planning),
 .selected_point(point),.host_point_error(error),.route_valid(ready),.navigation_request(active));
 task tap(input integer a,b);begin
  @(negedge clk);tx=a;ty=b;tv=1;td=1;@(negedge clk);tv=0;repeat(6)@(negedge clk);
  tv=1;td=0;@(negedge clk);tv=0;repeat(6)@(negedge clk);
 end endtask
 task pulse_select;begin @(negedge clk);select_p=1;@(negedge clk);select_p=0;repeat(3)@(negedge clk);end endtask
 task pulse_cancel;begin @(negedge clk);cancel_p=1;@(negedge clk);cancel_p=0;repeat(3)@(negedge clk);end endtask
 initial begin
  repeat(5)@(negedge clk);rst=1;wait(ui.control.state==1);
  tap(700,40);tap(200,180);tap(200,400);tap(32,32);tap(32,32);tap(580,340);tap(180,164);
  tap(255,139);wait(ui.control.state==1);repeat(3)@(negedge clk);
  @(negedge clk);heading_cmd=1;@(negedge clk);heading_cmd=0;repeat(3)@(negedge clk);
  if(!can_select)$fatal(1,"missing user start");
  pulse_select;if(!planning || point!=9 || ready || active)$fatal(1,"select state");
  repeat(100)@(negedge clk);pulse_cancel;
  if(planning || ready || active || error!=3 || !can_select || ui.road_planner.busy)$fatal(1,"cancel during route wipe");
  pulse_select;wait(ui.refine_busy);repeat(200)@(negedge clk);pulse_cancel;
  if(planning || ready || active || error!=3 || ui.refine_busy)$fatal(1,"cancel during planner");
  pulse_select;repeat(100)@(negedge clk);tap(32,32);repeat(5)@(negedge clk);
  if(mode!=4 || planning || ready || active || ui.refine_busy)$fatal(1,"back while planning");
  tap(180,164);pulse_select;repeat(100)@(negedge clk);tap(452,328);wait(ui.control.state==1);
  if(ui.have_start || ui.have_goal || point || planning || ready || can_select)$fatal(1,"clear during plan");
  @(negedge clk);start_p=1;@(negedge clk);start_p=0;repeat(3)@(negedge clk);
  if(mode!=5 || active)$fatal(1,"invalid START jumped page");
  tap(255,139);wait(ui.control.state==1);repeat(3)@(negedge clk);
  @(negedge clk);heading_cmd=1;@(negedge clk);heading_cmd=0;repeat(3)@(negedge clk);
  force ui.barrier_valid=4'b0001;force ui.barrier_x0=0;force ui.barrier_y0=0;
  force ui.barrier_x1=36'd351;force ui.barrier_y1=36'd319;
  pulse_select;wait(!planning);repeat(5)@(negedge clk);
  if(ready || active || can_start || error!=2)$fatal(1,"blocked map produced executable route error=%0d",error);
  release ui.barrier_valid;release ui.barrier_x0;release ui.barrier_y0;release ui.barrier_x1;release ui.barrier_y1;
  $display("PASS real UI faults: cancel during raster wipe/planner, back and clear invalidate routes; invalid START and fully blocked map remain stopped");$finish;
 end
 initial begin #10000000;$fatal(1,"UI fault timeout");end
endmodule
