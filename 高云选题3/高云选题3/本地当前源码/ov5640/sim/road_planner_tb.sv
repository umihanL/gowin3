`timescale 1ns/1ps
module road_planner_tb;
 reg clk=0;always #5 clk=~clk;
 reg rst=0,clear=0,cancel=0,start=0,snap_start=0;
 reg [7:0] sx=12,sy=78,gx=88,gy=13,snap_x=16,snap_y=78;
 reg [3:0] wc=1,bv=0;reg [63:0] wx=0,wy=0;
 reg [35:0] bx0=0,by0=0,bx1=0,by1=0;
 wire sd,sa,busy,done,rw,rb,sv,lock;wire [7:0] xx,yy;
 wire [2:0] error;wire [17:0] length;wire [14:0] ra;
 wire [16:0] px,py,ex,ey;wire [3:0] seg;wire [11:0] count;reg [10:0] index=0;
 reg [10:0] telemetry_index=0;wire [16:0] telemetry_x,telemetry_y;
 nav_road_planner dut(.clk(clk),.rst_n(rst),.clear(clear),.cancel(cancel),.start(start),
 .snap_start(snap_start),.snap_x(snap_x),.snap_y(snap_y),.snap_done(sd),.snap_accept(sa),.snapped_x(xx),.snapped_y(yy),
 .sx(sx),.sy(sy),.gx(gx),.gy(gy),.waypoint_count(wc),.waypoint_x(wx),.waypoint_y(wy),
 .barrier_valid(bv),.barrier_x0(bx0),.barrier_y0(by0),.barrier_x1(bx1),.barrier_y1(by1),
 .busy(busy),.done(done),.error(error),.length_cm(length),.route_write(rw),.route_bit(rb),.route_addr(ra),
 .sample_valid(sv),.sample_x_q8(px),.sample_y_q8(py),.sample_segment(seg),
 .execution_index(index),.execution_count(count),.execution_x_q8(ex),.execution_y_q8(ey),.execution_lock(lock),
 .telemetry_index(telemetry_index),.telemetry_x_q8(telemetry_x),.telemetry_y_q8(telemetry_y));
 integer cycles=0,outputs=0,lastx,lasty,dx,dy,old_dx,old_dy,points,fd;
 reg [16:0] qx[0:2047],qy[0:2047];
 always @(posedge clk)begin
  cycles=cycles+1;
  if(cycles>35000000)$fatal(1,"bounded planner watchdog state=%d expansions=%d min=%d",dut.state,dut.expansions,dut.minimum);
  if(sv)begin
   if(outputs!=0)begin
    dx=$signed({1'b0,px})-lastx;dy=$signed({1'b0,py})-lasty;
    if(dx*dx+dy*dy>(5*256)*(5*256))$fatal(1,"route discontinuity %d,%d",dx,dy);
    if(outputs>1 && dx*old_dx+dy*old_dy<0)$fatal(1,"non-tangent graph join");
    old_dx=dx;old_dy=dy;
   end
   qx[outputs]=px;qy[outputs]=py;outputs=outputs+1;lastx=px;lasty=py;
  end
  if(rw && ra>=28160)$fatal(1,"route bitmap outside map");
 end
 task plan;begin
  outputs=0;@(negedge clk);start=1;@(negedge clk);start=0;
  wait(done);@(negedge clk);
 end endtask
 integer i;
 initial begin
  wx[7:0]=135;wy[7:0]=147;
  repeat(5)@(negedge clk);rst=1;
  @(negedge clk);snap_start=1;@(negedge clk);snap_start=0;
  wait(sd);@(negedge clk);
  if(!sa || xx<11 || xx>14)$fatal(1,"touch did not snap to left road centre %d,%d",xx,yy);
  $display("PASS road snap: off-centre finger projected to line %d,%d",xx,yy);
  plan;
  if(error || !count || count!=outputs || count>1000)$fatal(1,"graph planning error %d count %d samples %d",error,count,outputs);
  $display("PASS road planner: ordered start/waypoint/goal, %d nodes %dcm, %d cycles",count,length,cycles);
  fd=$fopen("sim/road_route.csv","w");
  for(i=0;i<count;i=i+1)begin
   index=i;telemetry_index=count-1-i;repeat(3)@(negedge clk);
   if(ex!==qx[i] || ey!==qy[i])$fatal(1,"execution RAM mismatch index %d",i);
   if(telemetry_x!==qx[count-1-i] || telemetry_y!==qy[count-1-i])$fatal(1,"independent telemetry RAM mismatch index %d",count-1-i);
   $fwrite(fd,"%d,%d,%d\n",ex,ey,lock);
  end
  $fclose(fd);
  // A complete cross-map forbidden board disconnects this goal. No loop or
  // incomplete route may be published on the failed run.
  wc=0;sx=12;sy=100;gx=12;gy=20;
  bv=1;bx0[8:0]=0;bx1[8:0]=351;by0[8:0]=160;by1[8:0]=160;
  plan;if(error!=2 || count!=0)$fatal(1,"forbidden barrier ignored %d count %d",error,count);
  $display("PASS road barrier: disconnected route rejected and execution invalidated");
  bv=0;@(negedge clk);start=1;@(negedge clk);start=0;
  repeat(100)@(negedge clk);cancel=1;@(negedge clk);cancel=0;
  if(busy)$fatal(1,"cancel did not abort planner");
  $display("PASS road cancel: planning can be cancelled immediately");$finish;
 end
endmodule
