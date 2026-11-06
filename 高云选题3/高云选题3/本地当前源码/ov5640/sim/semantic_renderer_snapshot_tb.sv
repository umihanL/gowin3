`timescale 1ns/1ps
// A renderer fixture only; this creates no hardware controls or demo commands.
module semantic_renderer_snapshot_tb;
 reg clk=0;always #5 clk=~clk;reg rst=0;reg [11:0] x=0,y=0;
 wire [15:0] pixel;wire [16:0] map_addr;wire [14:0] plan_addr;wire mb,pb;
 nav_map_rom maps(.clk(clk),.display_addr(map_addr),.display_blocked(mb),
 .plan_addr(15'd0),.display_plan_addr(plan_addr),.display_plan_blocked(pb),.metadata_addr(6'd0));
 nav_renderer dut(.clk(clk),.rst_n(rst),.data_req(1'b1),.x_addr(x),.y_addr(y),.video(16'h1234),
 .screen(4'd9),.tool(2'd0),.live_page(2'd0),.have_start(1'b1),.have_goal(1'b1),
 .sx(8'd88),.sy(8'd13),.gx(8'd88),.gy(8'd147),.waypoint_count(4'd0),
 .waypoint_x(64'd0),.waypoint_y(64'd0),.selected_point(4'd4),
 .map_blocked(mb),.plan_blocked(pb),.route_pixel(1'b0),.centre_pixel(1'b0),
 .barrier_pixel(1'b0),.barrier_pending(1'b0),.route_ready(1'b1),.planning(1'b0),
 .motion_status(4'd2),.motion_phase(4'd0),.pose_x_q8(24'sd453120),.pose_y_q8(24'sd750080),.pose_heading(16'sd1800),
 .start_heading_valid(1'b1),.start_heading(16'sd1800),.map_addr(map_addr),.plan_addr(plan_addr),.pixel(pixel));
 integer f,a,b;
 initial begin
  repeat(4)@(negedge clk);rst=1;repeat(800)@(negedge clk);
  f=$fopen("sim/semantic_map_layout.ppm","wb");$fwrite(f,"P6\n800 480\n255\n");
  for(b=0;b<480;b=b+1)for(a=0;a<800;a=a+1)begin
   @(negedge clk);x=a;y=b;repeat(3)@(negedge clk);#1;
   $fwrite(f,"%c%c%c",{pixel[4:0],pixel[4:2]},{pixel[10:5],pixel[10:9]},{pixel[15:11],pixel[15:13]});
  end
  $fclose(f);$display("PASS semantic map layout snapshot");$finish;
 end
endmodule
