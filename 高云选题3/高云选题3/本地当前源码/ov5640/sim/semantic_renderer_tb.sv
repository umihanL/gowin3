`timescale 1ns/1ps
module semantic_renderer_tb;
 reg clk=0;always #5 clk=~clk;reg rst=0;reg [11:0] x=0,y=0;reg [3:0] screen=9;
 reg signed [23:0] px=1000*256,py=2000*256;reg signed [15:0] heading=0;
 wire [15:0] pixel;wire [16:0] map_addr;wire [14:0] plan_addr;
 nav_renderer dut(.clk(clk),.rst_n(rst),.data_req(1'b1),.x_addr(x),.y_addr(y),.video(16'h1234),
 .screen(screen),.tool(2'd0),.live_page(2'd0),.have_start(1'b0),.have_goal(1'b0),
 .waypoint_count(4'd0),.waypoint_x(64'd0),.waypoint_y(64'd0),.selected_point(4'd0),
 .map_blocked(1'b0),.plan_blocked(1'b0),.route_pixel(1'b0),.centre_pixel(1'b0),
 .barrier_pixel(1'b0),.barrier_pending(1'b0),.route_ready(1'b1),.planning(1'b0),
 .motion_status(4'd2),.motion_phase(4'd0),.pose_x_q8(px),.pose_y_q8(py),.pose_heading(heading),
 .start_heading_valid(1'b1),.start_heading(16'sd0),.map_addr(map_addr),.plan_addr(plan_addr),.pixel(pixel));
 task sample(input integer a,b);begin @(negedge clk);x=a;y=b;repeat(4)@(negedge clk);end endtask
 initial begin
  repeat(4)@(negedge clk);rst=1;
  sample(132,232);if(dut.canvas!=16'h5403)$fatal(1,"right-facing actual pose arrow missing");
  px=1200*256;sample(152,232);if(dut.canvas!=16'h5403)$fatal(1,"arrow did not follow odometry");
  sample(132,232);if(dut.canvas==16'h5403)$fatal(1,"old pose arrow remained");
  heading=900;sample(144,224);if(dut.canvas!=16'h5403)$fatal(1,"up heading missing");
  sample(152,232);if(dut.canvas==16'h5403)$fatal(1,"old orientation remained");
  sample(100,400);if(dut.canvas==16'h1234)$fatal(1,"navigation still shows camera");
  if(map_addr!=(400-112)*352+100-24)$fatal(1,"running map ROM coordinates");
  sample(600,396);if(dut.canvas!=16'h295b)$fatal(1,"stop button missing");
  screen=6;sample(100,400);if(dut.canvas!=16'h1234)$fatal(1,"range mode camera changed");
  $display("PASS semantic renderer: map-only navigation, moving/oriented pose arrow, stop visible, other mode camera retained");$finish;
 end
endmodule
