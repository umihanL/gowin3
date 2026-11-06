`timescale 1ns/1ps
// A complete four-stop region task is compiled before motion starts. Exercise
// the production directed road ROM and ordered suffix planner, no fake paths.
module continuous_waypoint_planner_tb;
 reg clk=0;always #5 clk=~clk;
 reg rst=0,start=0;
 reg [7:0] sx=88,sy=13,gx=96,gy=13;
 reg [7:0] rx0=0,ry0=0,rx1=87,ry1=79;
 reg [63:0] wx={32'd0,8'd52,8'd34,8'd12,8'd79};
 reg [63:0] wy={32'd0,8'd34,8'd60,8'd41,8'd13};
 wire busy,done,sample_valid,anchor_valid;
 wire [2:0] error,anchor_index;
 wire [3:0] sample_segment;
 wire [7:0] anchor_x,anchor_y;
 wire [16:0] sample_x,sample_y;
 wire [17:0] length;
 wire [11:0] count;
 nav_road_planner dut(.clk(clk),.rst_n(rst),.clear(1'b0),.cancel(1'b0),.start(start),
  .start_heading_valid(1'b1),.start_heading(16'sd1800),
  .region_preference_enabled(1'b1),.region_x0(rx0),.region_y0(ry0),.region_x1(rx1),.region_y1(ry1),
  .snap_start(1'b0),.snap_x(8'd0),.snap_y(8'd0),.sx(sx),.sy(sy),.gx(gx),.gy(gy),
  .waypoint_count(4'd4),.waypoint_x(wx),.waypoint_y(wy),
  .barrier_valid(4'd0),.barrier_x0(36'd0),.barrier_y0(36'd0),.barrier_x1(36'd0),.barrier_y1(36'd0),
  .busy(busy),.done(done),.error(error),.length_cm(length),
  .execution_index(11'd0),.execution_count(count),.telemetry_index(11'd0),
  .sample_valid(sample_valid),.sample_x_q8(sample_x),.sample_y_q8(sample_y),.sample_segment(sample_segment),
  .anchor_valid(anchor_valid),.anchor_index(anchor_index),.anchor_x(anchor_x),.anchor_y(anchor_y));
 integer cycles=0,samples=0,anchors=0,previous_segment=0,plans=0;
 integer last_x=0,last_y=0,anchor_distance,check_dx,check_dy;
 integer route_csv;
 integer region_case=0;
 reg [1023:0] route_file;
 reg [4:0] segments_seen=0;
 always @(posedge clk)begin
  cycles=cycles+1;
  if(cycles>200000000)$fatal(1,"continuous planner timeout state=%0d",dut.state);
  if(start)plans=plans+1;
  if(sample_valid)begin
   if(sample_segment>4 || sample_segment<previous_segment)$fatal(1,"route segment order lost");
   // Keep integer intermediates: Icarus propagates the unsigned context of
   // $abs(expr)+$abs(expr) back into expr even when it contains signed casts.
   check_dx=sample_x;check_dx=check_dx-sx*512-256;if(check_dx<0)check_dx=-check_dx;
   check_dy=sample_y;check_dy=check_dy-sy*512-256;if(check_dy<0)check_dy=-check_dy;
   if(samples==0 && check_dx+check_dy>1536)
    $fatal(1,"continuous route teleported its start: q8=%0d,%0d cell=%0d,%0d",sample_x,sample_y,sx,sy);
   previous_segment=sample_segment;segments_seen[sample_segment]=1;
   $fwrite(route_csv,"%0d,%0d,%0d,%0d\n",samples,(sample_x*32'd10)>>8,(sample_y*32'd10)>>8,sample_segment);
   samples=samples+1;last_x=sample_x;last_y=sample_y;
  end
  if(anchor_valid)begin
   if(anchor_index!=anchors)$fatal(1,"checkpoint order was lost");
   check_dx=anchor_x;check_dx=check_dx-wx[anchor_index*8+:8];if(check_dx<0)check_dx=-check_dx;
   check_dy=anchor_y;check_dy=check_dy-wy[anchor_index*8+:8];if(check_dy<0)check_dy=-check_dy;
   anchor_distance=check_dx+check_dy;
   // 20 cm corner touch tolerance plus one quantization cell.
   if(anchor_distance>11)$fatal(1,"checkpoint was not visited");
   anchors=anchors+1;
  end
 end
 initial begin
  if($value$plusargs("region=%d",region_case))begin end
  case(region_case)
   0:route_file="diagnostics/continuous_route_20261103/a_route.csv";
   1:begin
    wx={32'd0,8'd96,8'd163,8'd163,8'd123};wy={32'd0,8'd13,8'd70,8'd46,8'd33};
    gx=79;gy=13;rx0=88;rx1=175;
    route_file="diagnostics/continuous_route_20261103/b_route.csv";
   end
   2:begin
    wx={32'd0,8'd79,8'd52,8'd32,8'd12};wy={32'd0,8'd147,8'd131,8'd110,8'd88};
    gx=96;gy=147;ry0=80;ry1=159;
    route_file="diagnostics/continuous_route_20261103/c_route.csv";
   end
   3:begin
    wx={32'd0,8'd123,8'd145,8'd163,8'd96};wy={32'd0,8'd125,8'd96,8'd116,8'd147};
    gx=79;gy=147;rx0=88;rx1=175;ry0=80;ry1=159;
    route_file="diagnostics/continuous_route_20261103/d_route.csv";
   end
   default:$fatal(1,"unknown region case");
  endcase
  route_csv=$fopen(route_file,"w");
  if(route_csv==0)$fatal(1,"cannot write route export");
  $fwrite(route_csv,"index,x_mm,y_mm,segment\n");
  repeat(5)@(negedge clk);rst=1;
  @(negedge clk);start=1;@(negedge clk);start=0;
  wait(done);@(negedge clk);
  if(error || count<2 || count!=samples || count>2048)
   $fatal(1,"continuous route invalid error=%0d count=%0d samples=%0d",error,count,samples);
  if(plans!=1 || anchors!=4 || segments_seen!=5'b11111)
   $fatal(1,"task did not include all four stops and final exit in one PLAN");
  check_dx=last_x-gx*512-256;if(check_dx<0)check_dx=-check_dx;
  check_dy=last_y-gy*512-256;if(check_dy<0)check_dy=-check_dy;
  if(check_dx+check_dy>2048)
   $fatal(1,"continuous route did not reach its final exit");
  $display("PASS continuous task region=%0d: one PLAN, four ordered stops, final exit, %0d route samples, %0d cm, %0d cycles",region_case,count,length,cycles);
  $fclose(route_csv);
  $finish;
 end
endmodule
