`timescale 1ns/1ps
module navigation_profile_corner_tb;
 reg clk=0;always #5 clk=~clk;reg rst=0,valid=0;
 reg [11:0] count;wire [10:0] index;reg [10:0] execution=1;
 reg [16:0] x,y;reg [16:0] xs[0:127],ys[0:127];
 wire ready,busy,exit_valid;wire signed [15:0] tangent,exit_heading;wire signed [1:0] curve;
 wire signed [1:0] upcoming;wire [7:0] entry_mm;
 wire entry_valid;wire signed [15:0] entry_x_mm,entry_y_mm;
 always @(posedge clk)begin x<=xs[index];y<=ys[index];end
 vehicle_route_profile dut(.clk(clk),.rst_n(rst),.route_valid(valid),.route_count(count),
 .route_index(index),.route_x_q8(x),.route_y_q8(y),.route_lock(index+1==count),.selected_segment(4'd0),
 .execution_index(execution),.execution_tangent(tangent),.execution_curve(curve),
 .execution_upcoming_turn(upcoming),.execution_entry_mm(entry_mm),
 .execution_entry_valid(entry_valid),.execution_entry_x_mm(entry_x_mm),.execution_entry_y_mm(entry_y_mm),
 .execution_exit_valid(exit_valid),.execution_exit_heading(exit_heading),.busy(busy),.ready(ready));
 integer i,k,shape,base,side,heading,expected,delta,positive_seen,negative_seen,j,entry_index;
 integer expected_entry_x,expected_entry_y;
 real px,py,theta,expected_distance,edx,edy;
 initial begin
 repeat(5)@(negedge clk);rst=1;
 for(shape=0;shape<3;shape=shape+1)begin
  valid=0;repeat(5)@(negedge clk);px=1700;py=1600;xs[0]=px*256/10;ys[0]=py*256/10;
  base=shape==1?1700:0;side=shape==2?-1:1;count=shape==0?43:40;
  for(i=1;i<count;i=i+1)begin
   if(shape==0)heading=i<8?0:i<=16?(i-7)*50:i<20?450:i<=28?450-(i-19)*50:0;
   else heading=base+(i<8?0:i<=25?side*(i-7)*50:side*900);
   theta=heading*3.141592654/1800;px=px+20*$cos(theta);py=py-20*$sin(theta);
   xs[i]=$rtoi(px*256/10);ys[i]=$rtoi(py*256/10);
  end
  valid=1;wait(ready);positive_seen=0;negative_seen=0;
  for(k=2;k<count;k=k+1)begin
   execution=k;repeat(5)@(negedge clk);
   if(k<8 || (shape==0 && k>=17 && k<20))begin
    entry_index=k<8?7:19;expected_distance=0;
    for(j=k+1;j<=entry_index;j=j+1)begin
     edx=($itor(xs[j])-$itor(xs[j-1]))*10/256;
     edy=($itor(ys[j])-$itor(ys[j-1]))*10/256;
     expected_distance=expected_distance+$sqrt(edx*edx+edy*edy);
    end
    if(!exit_valid || upcoming!=(k<8?side:-1) || $abs(entry_mm-expected_distance)>8)
     $fatal(1,"wrong entry preview shape=%0d k=%0d upcoming=%0d mm=%0d expected=%f",shape,k,upcoming,entry_mm,expected_distance);
    expected_entry_x=xs[entry_index]*10/256;expected_entry_y=3200-ys[entry_index]*10/256;
    if(!entry_valid || dut.execution_entry_index!=entry_index)
     $fatal(1,"entry index drift shape=%0d index=%0d entry=%0d expected=%0d",shape,k,dut.execution_entry_index,entry_index);
   end
   if(curve!=0)begin
    if(!entry_valid || dut.execution_entry_index!=k-1)
     $fatal(1,"curve entry index mismatch shape=%0d index=%0d entry=%0d expected=%0d",shape,k,dut.execution_entry_index,k-1);
    positive_seen=positive_seen+(curve==1);negative_seen=negative_seen+(curve==-1);
    expected=shape==0?(curve==1?450:0):base+side*900;
    if(expected>1800)expected=expected-3600;if(expected < -1800)expected=expected+3600;
    delta=exit_heading-expected;if(delta>1800)delta=delta-3600;if(delta < -1800)delta=delta+3600;
    if(!exit_valid || $abs(delta)>25)$fatal(1,"wrong planned corner exit shape=%0d index=%0d curve=%0d tangent=%0d exit=%0d expected=%0d",shape,k,curve,tangent,exit_heading,expected);
   end
  end
  if(shape==0 && (positive_seen==0 || negative_seen==0))$fatal(1,"S geometry did not exercise both lobes");
  if(shape!=0 && positive_seen+negative_seen<15)$fatal(1,"quarter curve metadata missing");
  $display("PASS actual profile shape=%0d: independently generated S/90deg geometry and wrapped outgoing heading",shape);
 end
 $finish;
 end
 initial begin #10000000;$fatal(1,"profile corner timeout");end
endmodule
