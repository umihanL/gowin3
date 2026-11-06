`timescale 1ns/1ps
// Audit the production planner's displayed samples against its execution RAM.
module inertial_planner_audit_tb;
 reg clk=0;always #5 clk=~clk;
 reg rst=0,start=0;reg [7:0] sx,sy,gx,gy;
 reg [3:0] wc=0;reg [63:0] wx=0,wy=0;
 reg [10:0] index=0;wire done,busy,sv,lock;
 wire [2:0] error;wire [17:0] length;
 wire [11:0] count;wire [16:0] x,y,px,py;
 nav_road_planner dut(.clk(clk),.rst_n(rst),.start(start),.clear(1'b0),.cancel(1'b0),
 .snap_start(1'b0),.snap_x(8'd0),.snap_y(8'd0),
 .sx(sx),.sy(sy),.gx(gx),.gy(gy),.waypoint_count(wc),.waypoint_x(wx),.waypoint_y(wy),
 .barrier_valid(4'd0),.barrier_x0(36'd0),.barrier_y0(36'd0),.barrier_x1(36'd0),.barrier_y1(36'd0),
 .done(done),.busy(busy),.error(error),.length_cm(length),
 .sample_valid(sv),.sample_x_q8(px),.sample_y_q8(py),
 .execution_index(index),.execution_count(count),.execution_x_q8(x),.execution_y_q8(y),.execution_lock(lock));
 reg [16:0] shown_x[0:2047],shown_y[0:2047];
 integer i,fd,cycles=0,outputs=0,dx,dy,old_dx=0,old_dy=0;
 reg [1023:0] filename;
 always @(posedge clk)begin
  cycles=cycles+1;if(cycles>80000000)$fatal(1,"planner timeout");
  if(sv)begin
   if(outputs>0)begin
    dx=$signed({1'b0,px})-$signed({1'b0,shown_x[outputs-1]});
    dy=$signed({1'b0,py})-$signed({1'b0,shown_y[outputs-1]});
    if(dx*dx+dy*dy>1280*1280)$fatal(1,"path jump");
    if(outputs>1 && dx*old_dx+dy*old_dy<0)$fatal(1,"path reverses at join");
    old_dx=dx;old_dy=dy;
   end
   shown_x[outputs]=px;shown_y[outputs]=py;outputs=outputs+1;
  end
 end
 initial begin
  if(!$value$plusargs("SX=%d",sx))sx=163;
  if(!$value$plusargs("SY=%d",sy))sy=50;
  if(!$value$plusargs("GX=%d",gx))gx=90;
  if(!$value$plusargs("GY=%d",gy))gy=13;
  if($test$plusargs("PHOTO"))begin
   sx=163;sy=72;gx=123;gy=41;wc=2;wx=64'h5a60;wy=64'h450d;
  end
  if(!$value$plusargs("ROUTE=%s",filename))$fatal(1,"missing output route");
  repeat(5)@(negedge clk);rst=1;@(negedge clk);start=1;@(negedge clk);start=0;
  wait(done);@(negedge clk);
  if(error || count<2 || count!=outputs)$fatal(1,"planner error=%0d count=%0d shown=%0d",error,count,outputs);
  fd=$fopen(filename,"w");
  for(i=0;i<count;i=i+1)begin
   index=i;repeat(4)@(negedge clk);
   if(x!==shown_x[i] || y!==shown_y[i])$fatal(1,"display/execution mismatch index=%0d",i);
   $fwrite(fd,"%0d,%0d,%0d\n",x,y,lock);
  end
  $fclose(fd);
  $display("PASS planner: displayed and executable points identical, count=%0d length=%0dcm cycles=%0d",count,length,cycles);
  $finish;
 end
endmodule
