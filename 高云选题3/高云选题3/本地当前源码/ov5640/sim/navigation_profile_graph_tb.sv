`timescale 1ns/1ps
module navigation_profile_graph_tb;
 reg clk=0;always #5 clk=~clk;reg rst=0;reg [11:0] count=0;reg [10:0] execution=1;
 reg [16:0] xs[0:2047],ys[0:2047],x,y;reg locks[0:2047],locked;wire [10:0] index;
 wire ready,busy,exit_valid;wire signed [15:0] tangent,exit_heading;wire signed [1:0] curve;
 always @(posedge clk)begin x<=xs[index];y<=ys[index];locked<=locks[index];end
 vehicle_route_profile dut(.clk(clk),.rst_n(rst),.route_valid(1'b1),.route_count(count),
 .route_index(index),.route_x_q8(x),.route_y_q8(y),.route_lock(locked),.selected_segment(4'd0),
 .execution_index(execution),.execution_tangent(tangent),.execution_curve(curve),
 .execution_exit_valid(exit_valid),.execution_exit_heading(exit_heading),.busy(busy),.ready(ready));
 integer fd,a,b,c,fields,i,expected,delta,previous=0,tiny=0;real dx,dy;
 reg [1023:0] filename;
 initial begin
  if(!$value$plusargs("ROUTE=%s",filename))filename="sim/navigation_robust_photo_route.csv";
  fd=$fopen(filename,"r");if(!fd)$fatal(1,"graph input missing");
  while(!$feof(fd))begin fields=$fscanf(fd,"%d,%d,%d\n",a,b,c);if(fields==3)begin xs[count]=a;ys[count]=b;locks[count]=c;count=count+1;end end
  $fclose(fd);repeat(5)@(negedge clk);rst=1;wait(ready);
  for(i=1;i<count;i=i+1)begin
   dx=($itor(xs[i])-$itor(xs[i-1]))*10/256;dy=($itor(ys[i-1])-$itor(ys[i]))*10/256;
   if(dx*dx+dy*dy<1)begin expected=previous;tiny=tiny+1;end
   else expected=$rtoi($atan2(dy,dx)*1800/3.141592654);
   execution=i;repeat(5)@(negedge clk);delta=tangent-expected;
   if(delta>1800)delta=delta-3600;if(delta < -1800)delta=delta+3600;
   if($abs(delta)>5)$fatal(1,"graph tangent incorrect i=%0d actual=%0d expected=%0d tiny=%0d",i,tangent,expected,tiny);
   if(dx*dx+dy*dy<1 && curve!=0)$fatal(1,"tiny graph join invented curve i=%0d curve=%0d",i,curve);
   previous=expected;
  end
  if(tiny==0)$fatal(1,"test did not exercise submillimetre primitive joins");
  $display("PASS graph tangent metadata: samples=%0d tinyJoins=%0d no false turn direction",count,tiny);$finish;
 end
 initial begin #10000000;$fatal(1,"graph profile timeout");end
endmodule
