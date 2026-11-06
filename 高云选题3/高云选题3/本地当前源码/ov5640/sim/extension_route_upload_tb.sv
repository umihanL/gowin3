`timescale 1ns/1ps
module extension_route_upload_tb;
 reg clk=0;always #5 clk=~clk;
 reg rst=0,selected=1,command=0;reg [7:0] op=0;reg [31:0] args=0;reg [3:0] motion_status=0;
 wire route_page;wire [7:0] result,route_status;wire [3:0] screen,wc;
 wire [7:0] sx,sy,gx,gy;wire [63:0] wx,wy;wire ready,navigation,planning;
 wire clear,cancel,start,done;wire [2:0] error;wire [17:0] length;
 wire snap_start,snap_done,snap_accept;wire [7:0] snapx,snapy,xx,yy;
 wire av;wire [2:0] ai;wire [7:0] ax,ay;
 wire sample;wire [16:0] px,py;wire [11:0] count;wire [16:0] ex,ey;reg [10:0] index=0;
 nav_control control(.clk(clk),.rst_n(rst),.edit_distance(1'b0),.host_selected(selected),.host_command(command),
 .host_opcode(op),.host_arguments(args),.host_motion_status(motion_status),.host_route_page(route_page),.host_result(result),.host_route_status(route_status),
 .snap_start(snap_start),.snap_request_x(snapx),.snap_request_y(snapy),.snap_done(snap_done),.snap_accept(snap_accept),.snapped_x(xx),.snapped_y(yy),
 .planned_anchor_valid(av),.planned_anchor_index(ai),.planned_anchor_x(ax),.planned_anchor_y(ay),
 .touch_valid(1'b0),.touch_down(1'b0),.touch_x(16'd0),.touch_y(16'd0),.screen(screen),
 .sx(sx),.sy(sy),.gx(gx),.gy(gy),.waypoint_count(wc),.waypoint_x(wx),.waypoint_y(wy),.route_ready(ready),.navigation_active(navigation),.planning(planning),
 .map_blocked(1'b0),.map_directions(8'd0),.map_central(1'b1),.refine_clear(clear),.refine_cancel(cancel),.refine_start(start),.refine_done(done),.refine_error(error),.refined_length_cm(length));
 nav_road_planner planner(.clk(clk),.rst_n(rst),.clear(clear),.cancel(cancel),.start(start),
 .snap_start(snap_start),.snap_x(snapx),.snap_y(snapy),.snap_done(snap_done),.snap_accept(snap_accept),.snapped_x(xx),.snapped_y(yy),
 .anchor_valid(av),.anchor_index(ai),.anchor_x(ax),.anchor_y(ay),
 .sx(sx),.sy(sy),.gx(gx),.gy(gy),.waypoint_count(wc),.waypoint_x(wx),.waypoint_y(wy),
 .barrier_valid(4'd0),.barrier_x0(36'd0),.barrier_y0(36'd0),.barrier_x1(36'd0),.barrier_y1(36'd0),
 .done(done),.error(error),.length_cm(length),.sample_valid(sample),.sample_x_q8(px),.sample_y_q8(py),
 .execution_index(index),.execution_count(count),.execution_x_q8(ex),.execution_y_q8(ey));
 integer samples=0,lastx=0,lasty=0,dx,dy,minx,maxx,miny,maxy,fd,caseid=0;
 integer witness,junction1,junction2,finish;
 always @(posedge clk)if(sample)begin
   if(samples>0)begin
    dx=$signed({1'b0,px})-lastx;dy=$signed({1'b0,py})-lasty;
    if(dx*dx+dy*dy>25*65536)$fatal(1,"route discontinuity");
    if(samples<4 && dy>0)$fatal(1,"wrong initial heading");
   end
   if(px<minx)minx=px;if(px>maxx)maxx=px;if(py<miny)miny=py;if(py>maxy)maxy=py;
   if(py<267*256 && py>260*256 && px>100*256 && px<110*256 && witness<0)witness=samples;
   // The first topological junction is world Y=100 cm -> raster Y=220 cm.
   if(py<225*256 && py>215*256 && px>100*256 && px<110*256 && junction1<0)junction1=samples;
   if(py<177*256 && py>167*256 && px>100*256 && px<110*256 && junction2<0)junction2=samples;
   lastx=px;lasty=py;samples=samples+1;
   $fwrite(fd,"%0d,%0d\n",px,py);
 end
 task cmd(input [7:0] code,x,y,input [7:0] expected=0);begin
   @(negedge clk);op=code;args={16'd0,y,x};@(negedge clk);
   if(result!==expected)$fatal(1,"route cmd %h result%0d expected%0d",code,result,expected);
   command=result==0;@(negedge clk);command=0;repeat(30000)@(negedge clk);
 end endtask
 task reset_samples;begin samples=0;minx=999999;maxx=0;miny=999999;maxy=0;witness=-1;junction1=-1;junction2=-1;end endtask
 initial begin
  repeat(5)@(negedge clk);rst=1;repeat(30000)@(negedge clk);
  cmd(8'h20,52,140);cmd(8'h21,52,132);cmd(8'h21,52,120);cmd(8'h22,36,110);
  reset_samples;fd=$fopen("diagnostics/extension_tasks_20261029/route_first_left.csv","w");cmd(8'h23,0,0);
  wait(!planning);repeat(10)@(negedge clk);$fclose(fd);
  if(error || !ready || screen!=9 || !navigation || route_status!=8 || count!=samples)$fatal(1,"left route not ready err%0d status%0d",error,route_status);
  if(witness<0 || lastx>80*256 || lastx<68*256 || lasty<215*256 || lasty>228*256 || miny<205*256 || maxy>285*256)
    $fatal(1,"first left route geometry witness%0d goal%0d,%0d miny%0d maxy%0d",witness,lastx,lasty,miny,maxy);
  $display("PASS first intersection LEFT: actual planner %0d samples %0dcm, witness%0d, final cm %0d,%0d",samples,length,witness,lastx/256,lasty/256);
  motion_status=2;cmd(8'h20,52,140,4);motion_status=3;repeat(3)@(negedge clk);if(route_status!=11)$fatal(1,"completion state");
  motion_status=0;cmd(8'h20,52,140);cmd(8'h21,52,132);cmd(8'h21,52,98);cmd(8'h21,52,96);cmd(8'h22,67,86);
  reset_samples;fd=$fopen("diagnostics/extension_tasks_20261029/route_second_right.csv","w");cmd(8'h23,0,0);
  wait(!planning);repeat(10)@(negedge clk);$fclose(fd);
  if(error || !ready || route_status!=8 || count!=samples)$fatal(1,"right route not ready %0d",error);
  if(witness<0 || junction1<=witness || lastx<128*256 || lastx>143*256 || lasty<167*256 || lasty>180*256 || miny<160*256 || minx<95*256)
    $fatal(1,"second right geometry witness%0d j1%0d j2%0d goal%0d,%0d miny%0d",witness,junction1,junction2,lastx,lasty,miny);
  $display("PASS second intersection RIGHT: actual planner %0d samples %0dcm, ordered witness%0d first junction%0d, final cm %0d,%0d",samples,length,witness,junction1,lastx/256,lasty/256);
  cmd(8'h20,52,140);cmd(8'h22,67,86);cmd(8'h23,0,0);
  @(negedge clk);op=8'h05;args=0;command=1;@(negedge clk);command=0;
  repeat(100)@(negedge clk);if(planning || ready || navigation || route_status!=9)$fatal(1,"STOP during planning did not cancel");
  force done=1;repeat(2)@(negedge clk);release done;repeat(2)@(negedge clk);
  if(navigation || ready)$fatal(1,"late planner done restarted stopped task");
  selected=0;repeat(4)@(negedge clk);if(navigation || route_page)$fatal(1,"mode change retained remote route");
  $display("PASS route upload bounds/busy/planner/automatic navigation page/mode cancellation");$finish;
 end
 initial begin #2000000000;$fatal(1,"route planner watchdog");end
endmodule
