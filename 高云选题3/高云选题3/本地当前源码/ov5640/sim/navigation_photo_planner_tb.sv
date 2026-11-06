`timescale 1ns/1ps
// Recreate the user's ordered photograph route in the unchanged RTL planner.
module navigation_photo_planner_tb;
 reg clk=0;always #5 clk=~clk;reg rst=0,start=0;
 reg [10:0] index=0;wire done,busy;wire [2:0] error;wire [17:0] length;
 wire [11:0] count;wire [16:0] x,y;wire locked;
 nav_road_planner dut(.clk(clk),.rst_n(rst),.start(start),.clear(1'b0),.cancel(1'b0),
 .snap_start(1'b0),.snap_x(8'd0),.snap_y(8'd0),
 .sx(8'd163),.sy(8'd72),.gx(8'd123),.gy(8'd41),
 .waypoint_count(4'd2),.waypoint_x(64'h0000000000005a60),.waypoint_y(64'h000000000000450d),
 .barrier_valid(4'd0),.barrier_x0(36'd0),.barrier_y0(36'd0),.barrier_x1(36'd0),.barrier_y1(36'd0),
 .done(done),.busy(busy),.error(error),.length_cm(length),.execution_index(index),
 .execution_count(count),.execution_x_q8(x),.execution_y_q8(y),.execution_lock(locked));
 integer i,fd,cycles=0;
 always @(posedge clk)begin cycles=cycles+1;if(cycles>80000000)$fatal(1,"photo planner timeout");end
 initial begin
  repeat(5)@(negedge clk);rst=1;@(negedge clk);start=1;@(negedge clk);start=0;
  wait(done);@(negedge clk);if(error || length!=634 || count<150)$fatal(1,"photo route mismatch error=%0d length=%0d count=%0d",error,length,count);
  fd=$fopen("sim/navigation_robust_photo_rtl_route.csv","w");
  for(i=0;i<count;i=i+1)begin index=i;repeat(4)@(negedge clk);$fwrite(fd,"%0d,%0d,%0d\n",x,y,locked);end
  $fclose(fd);$display("PASS photograph RTL planner: ordered waypoints, actual route RAM count=%0d length=%0dcm cycles=%0d",count,length,cycles);$finish;
 end
endmodule
