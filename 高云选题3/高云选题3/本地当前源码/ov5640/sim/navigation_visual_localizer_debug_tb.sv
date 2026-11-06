`timescale 1ns/1ps
module visual_localizer_tb;
 GSR GSR(.GSRI(1'b1));
 reg clk=0;always #5 clk=~clk;reg rst=0,frame=0;
 reg [16:0] xs[0:63],ys[0:63],rx,ry;
 wire [10:0] index;wire busy,valid;wire signed [7:0] dx,dy;
 reg signed [15:0] px=1000,py=1000;
 reg signed [16:0] cs=32768,sn=0;
 reg signed [8:0] e0=0,e1=0;reg [11:0] count=50;
 always @(posedge clk)begin rx<=xs[index];ry<=ys[index];end
 vehicle_visual_localizer dut(.clk(clk),.rst_n(rst),.enabled(1'b1),.frame_valid(frame),.fit_valid(1'b1),
 .forward0(10'd180),.forward1(10'd300),.error0(e0),.error1(e1),.route_count(count),.execution_index(11'd1),
 .route_x_q8(rx),.route_y_q8(ry),.pose_x(px),.pose_y(py),.cosine(cs),.sine(sn),
 .busy(busy),.read_index(index),.correction_valid(valid),.correction_x(dx),.correction_y(dy));
 integer i,k,n=0;real x,y,t,off,df0,df1;
 task observe;begin
  @(negedge clk);frame=1;@(negedge clk);frame=0;
  wait(!busy);@(negedge clk);$display("OBS t=%0t xy=%0d,%0d valid=%b d=%0d,%0d n=%0d",$time,px,py,valid,dx,dy,n);
 end endtask
 always @(negedge clk)if(valid)begin px=px+dx;py=py+dy;n=n+1;end
 initial begin
  for(i=0;i<50;i=i+1)begin xs[i]=(900+i*20)*256/10;ys[i]=2200*256/10;end
  repeat(5)@(negedge clk);rst=1;e0=30;e1=30;
  repeat(10)observe;
  if(n<8 || $abs(px-1000)>1 || $abs(py-1030)>3)$fatal(1,"straight is unobservable longitudinally; pose=%0d,%0d n=%0d",px,py,n);
  $display("PASS straight localizer: lateral converges, unobservable longitudinal position unchanged");
  // Parabolic curve gives two independent normal constraints. The actual
  // car is 60mm farther forward than the encoder/map origin, with zero yaw.
  rst=0;px=1000;py=1000;n=0;
  for(i=0;i<50;i=i+1)begin x=900+i*20;y=1000+(x-1000)*(x-1000)/1000;
   xs[i]=$rtoi(x*256/10);ys[i]=$rtoi((3200-y)*256/10);end
  repeat(5)@(negedge clk);rst=1;
  e0=-$rtoi(240.0*240/1000);e1=-$rtoi(360.0*360/1000);
  repeat(22)observe;
  if(n<10 || $abs(px-1060)>10 || $abs(py-1000)>10)$fatal(1,"curve did not recover forward offset pose=%0d,%0d updates=%0d",px,py,n);
  $display("PASS curve localizer: visual ground points recover along-road displacement without resetting encoder/heading");
  // Rotate the scene 90deg and mirror the curve; repeat for both signs of
  // longitudinal displacement. This checks coordinate/sign behavior externally.
  for(k=-1;k<=1;k=k+2)begin
   rst=0;px=1000;py=1000;n=0;cs=0;sn=32768;
   for(i=0;i<50;i=i+1)begin x=-100+i*20;y=x*x/1000;
    xs[i]=$rtoi((1000+y)*256/10);ys[i]=$rtoi((2200-x)*256/10);end
   repeat(5)@(negedge clk);rst=1;
   e0=$rtoi((180.0+k*60)*(180.0+k*60)/1000);
   e1=$rtoi((300.0+k*60)*(300.0+k*60)/1000);
   repeat(22)observe;
   if(n<10 || $abs(px-1000)>10 || $abs(py-(1000+k*60))>10)
    $fatal(1,"rotated mirrored curve displacement=%0d pose=%0d,%0d updates=%0d",k*60,px,py,n);
   $display("PASS rotated mirrored curve: signed longitudinal displacement %0dmm recovered",k*60);
  end
  $finish;
 end
 initial begin #10000000;$fatal(1,"localizer timeout");end
endmodule
