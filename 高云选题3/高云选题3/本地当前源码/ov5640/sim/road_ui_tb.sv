`timescale 1ns/1ps
module road_ui_tb;
 reg clk=0;always #5 clk=~clk;
 reg rst_n=0,req=1,tv=0,td=0;
 reg [11:0] x=200,y=200;reg [15:0] tx=0,ty=0;
 wire [15:0] pixel;wire [3:0] mode,segment;wire active,ready,pv;wire [7:0] px,py;
 navigation_ui dut(.clk(clk),.rst_n(rst_n),.data_req(req),.x_addr(x),.y_addr(y),.video(16'h1234),
 .preview(16'h5678),.touch_valid(tv),.touch_down(td),.touch_x(tx),.touch_y(ty),.pixel(pixel),
 .operation_mode(mode),.navigation_request(active),.route_valid(ready),.path_valid(pv),.path_x(px),.path_y(py),.path_segment(segment),
 .speed_edit(1'b0),.speed_increase(1'b0),.speed_decrease(1'b0),.execution_index(11'd0));
 task tap(input integer u,v);begin
  @(negedge clk);tx=u;ty=v;tv=1;td=1;@(negedge clk);tv=0;repeat(6)@(negedge clk);
  tv=1;td=0;@(negedge clk);tv=0;repeat(6)@(negedge clk);
 end endtask
 task point(input integer u,v);begin tap(25+u*2,113+v*2);wait(dut.control.state==1);repeat(3)@(negedge clk);end endtask
 integer cycles=0,samples=0,j,out,uu,vv;
 reg [7:0] planned_wx,planned_wy;
 always @(posedge clk)begin
  cycles=cycles+1;if(pv)begin
   samples=samples+1;
   if(segment==0)begin planned_wx=px;planned_wy=py;end
  end
  if(cycles>35000000)$fatal(1,"UI planning stuck state=%d engine=%d",dut.control.state,dut.road_planner.state);
  if(dut.control.planning && dut.control.message==37)$fatal(1,"obsolete smoothing status still active");
 end
 task snapshot;begin
  out=$fopen("sim/menu_road_map.ppm","wb");$fwrite(out,"P6\n800 480\n255\n");
  for(vv=0;vv<480;vv=vv+1)for(uu=0;uu<800;uu=uu+1)begin
   @(negedge clk);x=uu;y=vv;repeat(2)@(negedge clk);
   $fwrite(out,"%c%c%c",{pixel[4:0],pixel[4:2]},{pixel[10:5],pixel[10:9]},{pixel[15:11],pixel[15:13]});
  end
  $fclose(out);
 end endtask
 initial begin
  repeat(5)@(negedge clk);rst_n=1;wait(dut.control.state==1);repeat(4)@(negedge clk);
  if(mode!=0 || !dut.metadata_valid)$fatal(1,"startup metadata/menu");
  tap(580,340);tap(180,164);if(mode!=5)$fatal(1,"map menu");
  point(0,0);if(dut.have_start)$fatal(1,"far off-road selection accepted");
  point(16,78);if(!dut.have_start || dut.sx<11 || dut.sx>14)$fatal(1,"visible start not snapped to road centre");
  tap(452,200);point(88,13);if(!dut.have_goal)$fatal(1,"goal selection");
  tap(452,264);point(135,147);if(dut.count!=1)$fatal(1,"ordered waypoint");
  tap(452,392);wait(!dut.planning);repeat(4)@(negedge clk);
  if(!ready || samples<100 || dut.control.message!=19)$fatal(1,"route never ready or old warning remains %b %d",ready,dut.control.message);
  if(dut.wx[7:0]!==planned_wx || dut.wy[7:0]!==planned_wy)
    $fatal(1,"visible waypoint did not follow chosen line/fillet endpoint");
  // Keep DE permanently asserted: planning no longer waits on display blanking.
  $display("PASS road UI: centre snapping, ordered waypoint, planning completes with DE always high; ready in %d cycles",cycles);
  if($test$plusargs("snapshot"))snapshot;
  tap(32,32);tap(180,268);if(mode!=9 || !active)$fatal(1,"planned navigation failed to start");
  tap(560,396);if(active || mode!=4)$fatal(1,"navigation stop");
  tap(180,164);tap(452,136);point(12,80);if(ready)$fatal(1,"point edit kept stale path");
  tap(452,392);repeat(500)@(negedge clk);tap(452,328);wait(dut.control.state==1);
  if(ready || dut.planning || dut.count!=0 || dut.have_start || dut.have_goal)$fatal(1,"cancel/clear while planning");
  for(j=0;j<28160;j=j+1)if(dut.route[j]!==0)$fatal(1,"stale route raster");
  $display("PASS road UI start/stop: route navigation starts, edits invalidate, clear cancels and removes all route pixels");$finish;
 end
endmodule
