`timescale 1ns/1ps
// Costs are supplied by an independent augmented-state host Dijkstra oracle.
`include "nav_centerline_config.vh"
module road_global_tb;
 reg clk=0;always #5 clk=~clk;
 reg rst=0,start=0;reg [7:0] sx,sy,gx,gy;reg [3:0] wc,bv;
 reg [63:0] wx,wy;reg [35:0] bx0,by0,bx1,by1;
 wire done,busy,sv,lock;wire [2:0] error;wire [17:0] length_cm;
 wire [16:0] px,py,ex,ey;wire [3:0] segment;wire [11:0] count;
 nav_road_planner dut(.clk(clk),.rst_n(rst),.clear(1'b0),.cancel(1'b0),.start(start),
  .snap_start(1'b0),.snap_x(8'd0),.snap_y(8'd0),.sx(sx),.sy(sy),.gx(gx),.gy(gy),
  .waypoint_count(wc),.waypoint_x(wx),.waypoint_y(wy),
  .barrier_valid(bv),.barrier_x0(bx0),.barrier_y0(by0),.barrier_x1(bx1),.barrier_y1(by1),
  .busy(busy),.done(done),.error(error),.length_cm(length_cm),
  .sample_valid(sv),.sample_x_q8(px),.sample_y_q8(py),.sample_segment(segment),
  .execution_index(11'd0),.execution_count(count),.execution_x_q8(ex),.execution_y_q8(ey),.execution_lock(lock));
 integer fd,fields,expected,case_id=0,cycles=0,outputs=0,route_fd;
 integer first_case,last_case;reg [1023:0] route_filename;
 integer last_x,last_y,dx,dy,last_dx,last_dy;
 always @(posedge clk)if(rst)begin
  cycles=cycles+1;
  if(cycles%10000000==0)begin
   $display("INFO global case=%0d cycles=%0d state=%0d stage=%0d",case_id,cycles,dut.state,dut.dp_stage);$fflush();
  end
  // Dense Dijkstra scales with nodes squared and remaining waypoint layers.
  // Keep a bounded watchdog when changing the generated road graph.
  if(cycles>(`ROAD_NODES*`ROAD_NODES*6*(wc+2)+1000000))$fatal(1,"global planning timeout case=%0d state=%0d",case_id,dut.state);
  if(sv)begin
   if(outputs)begin
    dx=$signed({1'b0,px})-last_x;dy=$signed({1'b0,py})-last_y;
    if(dx*dx+dy*dy>1280*1280)$fatal(1,"global path discontinuity");
    if(outputs>1 && dx*last_dx+dy*last_dy<0)$fatal(1,"global path reverses at waypoint");
    last_dx=dx;last_dy=dy;
   end
   last_x=px;last_y=py;outputs=outputs+1;
   $fwrite(route_fd,"%0d,%0d,%0d,%0d\n",case_id,px,py,segment);
  end
 end
 initial begin
  if(!$value$plusargs("CASE_FROM=%d",first_case))first_case=0;
  if(!$value$plusargs("CASE_TO=%d",last_case))last_case=100;
  if(!$value$plusargs("ROUTES=%s",route_filename))route_filename="sim/road_global_routes.csv";
  fd=$fopen("sim/road_global_cases.txt","r");route_fd=$fopen(route_filename,"w");
  if(!fd || !route_fd)$fatal(1,"global cases missing");
  repeat(5)@(negedge clk);rst=1;
  while(!$feof(fd))begin
   fields=$fscanf(fd,"%d %d %d %d %d %h %h %d %d %d %d %d %d\n",sx,sy,gx,gy,wc,wx,wy,expected,bv,bx0,by0,bx1,by1);
   if(fields!=13)$fatal(1,"global case parse returned %0d",fields);
   if(fields==13 && case_id>=first_case && case_id<=last_case)begin
    cycles=0;outputs=0;
    @(negedge clk);start=1;@(negedge clk);start=0;wait(done);@(negedge clk);
    if(error || length_cm!=expected || count!=outputs)
     $fatal(1,"global oracle mismatch case=%0d err=%0d cost=%0d expected=%0d count=%0d/%0d",case_id,error,length_cm,expected,count,outputs);
    $display("PASS global road case %0d: %0d waypoints, optimal ROM cost=%0dcm, %0d samples, %0d cycles",case_id,wc,length_cm,count,cycles);
    $fflush();
   end
   case_id=case_id+1;
  end
  $fclose(fd);$fclose(route_fd);$finish;
 end
endmodule
