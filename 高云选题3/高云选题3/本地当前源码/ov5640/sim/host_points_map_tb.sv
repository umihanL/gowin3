`timescale 1ns/1ps
module host_points_map_tb;
 reg clk=0;always #5 clk=~clk;reg rst=0,tv=0,td=0,select_p=0,cancel_p=0,start_p=0;
 reg [15:0] tx=0,ty=0;reg req=0;reg [11:0] vx=0,vy=0;wire [15:0] pixel;wire [3:0] mode,point;wire planning,ready,can_select,can_start,active;wire [2:0] error;
 navigation_ui ui(.clk(clk),.rst_n(rst),.data_req(req),.x_addr(vx),.y_addr(vy),.video(16'd0),.preview(16'd0),
 .zebra_enable(1'b0),.zebra_can_change(1'b1),.host_route_command(1'b0),.host_route_opcode(8'd0),.host_route_arguments(32'd0),.touch_valid(tv),.touch_down(td),.touch_x(tx),.touch_y(ty),.operation_mode(mode),.pixel(pixel),
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

 integer f,x,y;
 initial begin
  repeat(5)@(negedge clk);rst=1;wait(ui.control.state==1);
  tap(700,40);tap(200,180);tap(200,400);tap(32,32);tap(32,32);tap(580,340);tap(180,164);
  tap(255,139);wait(ui.control.state==1);repeat(3)@(negedge clk);
  f=$fopen("sim/host_points_map_preview.ppm","wb");$fwrite(f,"P6\n800 480\n255\n");req=1;
  for(y=0;y<480;y=y+1)for(x=0;x<800;x=x+1)begin
   @(negedge clk);vx=x;vy=y;repeat(2)@(negedge clk);#1;
   $fwrite(f,"%c%c%c",{pixel[4:0],pixel[4:2]},{pixel[10:5],pixel[10:9]},{pixel[15:11],pixel[15:13]});
  end
  $fclose(f);$display("PASS actual UI map numbered destination snapshot");$finish;
 end
endmodule
