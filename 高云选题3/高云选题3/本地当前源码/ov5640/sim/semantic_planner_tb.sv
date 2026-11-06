`timescale 1ns/1ps
module semantic_planner_tb;
 reg clk=0;always #5 clk=~clk;reg rst=0,start=0,cancel=0,snap_start=0;
 reg [7:0] sx=88,sy=13,gx=88,gy=147,qx=17,qy=17;
 reg signed [15:0] heading=1800;wire snap_done,snap_accept,snap_heading_valid,done,busy;
 wire [7:0] snapped_x,snapped_y;wire signed [15:0] snapped_heading;wire [2:0] error;
 wire [17:0] length;wire [11:0] count;
 nav_road_planner dut(.clk(clk),.rst_n(rst),.clear(1'b0),.cancel(cancel),.start(start),
 .start_heading_valid(1'b1),.start_heading(heading),.snap_start(snap_start),.snap_x(qx),.snap_y(qy),
 .snap_done(snap_done),.snap_accept(snap_accept),.snapped_x(snapped_x),.snapped_y(snapped_y),
 .snapped_heading_valid(snap_heading_valid),.snapped_heading(snapped_heading),
 .sx(sx),.sy(sy),.gx(gx),.gy(gy),.waypoint_count(4'd0),.waypoint_x(64'd0),.waypoint_y(64'd0),
 .barrier_valid(4'd0),.barrier_x0(36'd0),.barrier_y0(36'd0),.barrier_x1(36'd0),.barrier_y1(36'd0),
 .busy(busy),.done(done),.error(error),.length_cm(length),.execution_index(11'd0),.execution_count(count),.telemetry_index(11'd0));
 integer i,j,valid_starts;
 task plan(input integer expected);begin
  @(negedge clk);start=1;@(negedge clk);start=0;wait(done);@(negedge clk);
  if(error || length!=expected || count<2)$fatal(1,"A* cost=%d expected=%d error=%d",length,expected,error);
  $display("PASS A* RTL exact oracle cost=%d samples=%d expansions=%d",length,count,dut.expansions);
 end endtask
 initial begin
  repeat(5)@(negedge clk);rst=1;
  plan(389);gx=78;gy=13;plan(12);
  for(i=0;i<4;i=i+1)begin
   qx=i<2?17:158;qy=i==0 || i==3?17:142;
   @(negedge clk);snap_start=1;@(negedge clk);snap_start=0;wait(snap_done);@(negedge clk);
   if(!snap_accept || !snap_heading_valid)$fatal(1,"curved start has no valid tangent");
   sx=snapped_x;sy=snapped_y;heading=snapped_heading;gx=88;gy=147;
   @(negedge clk);start=1;@(negedge clk);start=0;
   wait(dut.state==7);@(negedge clk);valid_starts=0;
   for(j=0;j<1764;j=j+1)if(!dut.work[j][27] && dut.work[j][26:11]==0)valid_starts=valid_starts+1;
   if(valid_starts==0)$fatal(1,"snapped curve heading fails start filter: cell=%d,%d heading=%d",sx,sy,heading);
   $display("PASS curve start initialized legal tangent: cell=%d,%d heading=%d aliases=%d",sx,sy,heading,valid_starts);
   cancel=1;@(negedge clk);cancel=0;repeat(3)@(negedge clk);
  end
  $display("PASS semantic planner: exact A* optimum and all four corner-point tangent defaults");$finish;
 end
 initial begin #500000000;$fatal(1,"planner timeout");end
endmodule
