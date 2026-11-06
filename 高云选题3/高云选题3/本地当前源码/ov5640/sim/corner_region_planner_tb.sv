`timescale 1ns/1ps
module corner_region_planner_tb;
 reg clk=0;always #5 clk=~clk;
 reg rst=0,start=0,region_enabled=0;
 reg [7:0] sx=56,sy=13,gx=88,gy=147,rx0=0,ry0=80,rx1=88,ry1=159;
 reg signed [15:0] heading=-1430;
 reg [3:0] waypoint_count=0;reg [63:0] wx=0,wy=0;
 wire busy,done,sample_valid;wire [2:0] error;wire [17:0] length;wire [11:0] count;
 wire [16:0] sample_x,sample_y;
 nav_road_planner dut(.clk(clk),.rst_n(rst),.clear(1'b0),.cancel(1'b0),.start(start),
  .start_heading_valid(1'b1),.start_heading(heading),.region_preference_enabled(region_enabled),
  .region_x0(rx0),.region_y0(ry0),.region_x1(rx1),.region_y1(ry1),
  .snap_start(1'b0),.snap_x(8'd0),.snap_y(8'd0),.sx(sx),.sy(sy),.gx(gx),.gy(gy),
  .waypoint_count(waypoint_count),.waypoint_x(wx),.waypoint_y(wy),
  .barrier_valid(4'd0),.barrier_x0(36'd0),.barrier_y0(36'd0),.barrier_x1(36'd0),.barrier_y1(36'd0),
  .busy(busy),.done(done),.error(error),.length_cm(length),.execution_index(11'd0),.execution_count(count),
  .telemetry_index(11'd0),.sample_valid(sample_valid),.sample_x_q8(sample_x),.sample_y_q8(sample_y));
 integer value,expected_error=0,expected_cost=-1,expected_length=-1,expected_fallback=0;
 integer weighted_cost=0,samples=0,first_distance,cycles=0;
 always @(posedge clk)begin
  cycles=cycles+1;
  if(cycles>50000000)$fatal(1,"planner timeout state=%0d",dut.state);
  if(rst && dut.state==12 && !dut.backward && dut.best_goal_valid &&
      (dut.minimum==65535 || dut.min_priority>={1'b0,dut.best_goal_score}))
    weighted_cost=weighted_cost+dut.best_goal_distance;
  if(sample_valid)begin
   if(samples==0)begin
    first_distance=$abs($signed({1'b0,sample_x})-(sx*512+256))+
                   $abs($signed({1'b0,sample_y})-(sy*512+256));
    if(dut.start_fallback && first_distance>1536)$fatal(1,"fallback teleported start %0d q8",first_distance);
   end
   samples=samples+1;
  end
 end
 initial begin
  if($value$plusargs("sx=%d",value))sx=value;
  if($value$plusargs("sy=%d",value))sy=value;
  if($value$plusargs("gx=%d",value))gx=value;
  if($value$plusargs("gy=%d",value))gy=value;
  if($value$plusargs("heading=%d",value))heading=value;
  if($value$plusargs("region=%d",value))region_enabled=value;
  if($value$plusargs("rx0=%d",value))rx0=value;
  if($value$plusargs("ry0=%d",value))ry0=value;
  if($value$plusargs("rx1=%d",value))rx1=value;
  if($value$plusargs("ry1=%d",value))ry1=value;
  if($value$plusargs("wx=%d",value))begin wx[7:0]=value;waypoint_count=1;end
  if($value$plusargs("wy=%d",value))wy[7:0]=value;
  if($value$plusargs("error=%d",value))expected_error=value;
  if($value$plusargs("cost=%d",value))expected_cost=value;
  if($value$plusargs("length=%d",value))expected_length=value;
  if($value$plusargs("fallback=%d",value))expected_fallback=value;
  repeat(5)@(negedge clk);rst=1;
  @(negedge clk);start=1;@(negedge clk);start=0;wait(done);@(negedge clk);
  if(error!=expected_error)$fatal(1,"error=%0d expected=%0d",error,expected_error);
  if(dut.start_fallback!=expected_fallback)$fatal(1,"fallback=%0d expected=%0d",dut.start_fallback,expected_fallback);
  if(dut.requested_heading!=heading)$fatal(1,"declared real heading overwritten");
  if(!expected_error)begin
   if(count<2 || count!=samples)$fatal(1,"route not executable count=%0d samples=%0d",count,samples);
   if(expected_cost>=0 && weighted_cost!=expected_cost)$fatal(1,"weighted cost=%0d oracle=%0d",weighted_cost,expected_cost);
   if(expected_length>=0 && length!=expected_length)$fatal(1,"length=%0d oracle=%0d",length,expected_length);
  end
  if(expected_error==2 && count!=0)$fatal(1,"unreachable start emitted route");
  $display("PASS corner/region planner error=%0d fallback=%0d cost=%0d true_length_cm=%0d samples=%0d cycles=%0d",error,dut.start_fallback,weighted_cost,length,count,cycles);
  $finish;
 end
endmodule
