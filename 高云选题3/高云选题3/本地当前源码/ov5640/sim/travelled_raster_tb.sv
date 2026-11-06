`timescale 1ns/1ps
module travelled_raster_tb;
 reg clk=0;always #5 clk=~clk;reg rst=0,reset_route=0,running=0,finished=0;
 reg signed [23:0] px=1000*256,py=2000*256;reg signed [15:0] heading=0;
 wire write;wire [14:0] addr;reg bitmap[0:28159];integer k,n,before_count,after_count,step;
 nav_travelled_raster dut(.clk(clk),.rst_n(rst),.reset_route(reset_route),.running(running),.finished(finished),
  .pose_x_q8(px),.pose_y_q8(py),.pose_heading(heading),.write(write),.addr(addr));
 always @(posedge clk)if(write)begin
  if(addr>=28160)$fatal(1,"out of display RAM");
  bitmap[addr]<=0;
 end
 task count_removed(output integer result);integer j;begin result=0;for(j=0;j<28160;j=j+1)if(!bitmap[j])result=result+1;end endtask
 initial begin
  for(k=0;k<28160;k=k+1)bitmap[k]=1;
  repeat(4)@(negedge clk);rst=1;repeat(100)@(negedge clk);
  count_removed(n);if(n!=0)$fatal(1,"idle cleared route");
  running=1;repeat(100)@(negedge clk);
  count_removed(before_count);repeat(2000)@(negedge clk);count_removed(after_count);
  if(before_count!=after_count)$fatal(1,"stationary car erased farther route over time");
  if(!bitmap[60*176+51] || bitmap[60*176+49])$fatal(1,"forward erased or rear not erased");
  for(step=1;step<=10;step=step+1)begin px=(1000+step*20)*256;repeat(60)@(negedge clk);end
  if(bitmap[60*176+55] || !bitmap[60*176+61])$fatal(1,"travelled line/forward preservation");
  running=0;px=2000*256;repeat(100)@(negedge clk);
  if(!bitmap[60*176+99])$fatal(1,"pause erased undisplayed movement");
  running=1;heading=900;repeat(100)@(negedge clk);
  if(!bitmap[59*176+100] || bitmap[61*176+100])$fatal(1,"upward heading erased front");
  running=0;finished=1;repeat(28200)@(negedge clk);
  count_removed(n);if(n!=28160)$fatal(1,"arrival failed to clear remaining line");
  finished=0;reset_route=1;repeat(4)@(negedge clk);reset_route=0;repeat(10)@(negedge clk);
  if(write)$fatal(1,"reset left erase active");
  $display("PASS travelled raster: stationary footprint stable, pause immutable, actual odometry clears rear only, arrival clears remainder");$finish;
 end
endmodule
