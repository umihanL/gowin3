`timescale 1ns/1ps
// Full 640x480 pinhole image -> production row scan -> production controller.
// A far curved road can be valid while the nearest boundaries are absent.
module navigation_image_start_tb;
 reg clk=0;always #5 clk=~clk;
 reg rst=0,fb=0,req=0,pv=0,tick=0;
 reg [11:0] x=0,y=0;reg [15:0] rgb=16'hffff;
 wire update,valid,junction,ready;wire [2:0] visibility;
 wire signed [8:0] lane,heading;wire signed [15:0] steer,align_yaw,align_offset;
 vision_video video(.clk(clk),.rst_n(rst),.frame_begin(fb),.data_req(req),.pixel_valid(pv),
  .x(x),.y(y),.rgb(rgb),.threshold(8'd96),.turn_hint(2'sd0),
  .road_update(update),.road_valid(valid),.road_junction(junction),
  .road_visibility(visibility),.road_near(lane),.road_heading(heading));
 vehicle_vision_fusion fusion(.clk(clk),.rst_n(rst),.tick(tick),.enabled(1'b1),
  .road_update(update),.road_valid(valid),.road_junction(junction),.road_visibility(visibility),
  .road_near(lane),.road_heading(heading),.terminal_approach(1'b0),
  .route_steer(16'sd0),.pose_heading(16'sd900),.travel_q8(32'sd0),.moving(1'b0),
  .planned_turn(2'sd0),.planned_exit_valid(1'b0),.planned_exit_heading(16'sd0),
  .route_guidance_valid(1'b1),.route_target_behind(1'b0),.turn_entry_ready(1'b0),
  .incoming_heading_error(16'sd0),.incoming_lateral_error(16'sd0),.mapped_entry_due(1'b0),
  .start_ready(ready),.steering(steer),.alignment_yaw(align_yaw),.alignment_offset(align_offset));
 task controls;integer k;begin
  for(k=0;k<16;k=k+1)begin @(negedge clk);tick=1;@(negedge clk);tick=0;end
 end endtask
 task frame(input integer offset,input integer skew,input integer far_only);
  integer xx,yy;real pitch,q,depth,forward,centre;begin
   pitch=$atan2(185.0,395.0);
   for(yy=0;yy<480;yy=yy+1)for(xx=0;xx<640;xx=xx+1)begin
    @(negedge clk);req=1;pv=1;x=80+xx;y=yy;
    q=(yy-239.5)/554.2562584;
    depth=185/($sin(pitch)+q*$cos(pitch));
    forward=185*($cos(pitch)-q*$sin(pitch))/($sin(pitch)+q*$cos(pitch))-15;
    centre=319.5+554.2562584*(2+offset+forward*$tan(skew*3.141592654/1800.0))/depth;
    rgb=depth>0 && xx>centre-554.2562584*127/depth && xx<centre+554.2562584*127/depth?16'h0000:16'hffff;
    if(far_only && forward<310)rgb=16'hffff;
   end
   @(negedge clk);req=0;pv=0;fb=1;@(negedge clk);fb=0;
   wait(update);@(negedge clk);controls;
   $display("IMAGE offset=%0d yaw=%0d farOnly=%0d lane=%0d heading=%0d visible=%b steer=%0d",offset,skew,far_only,lane,heading,visibility,steer);
  end
 endtask
 integer side;
 initial begin
  repeat(5)@(negedge clk);rst=1;
  for(side=-1;side<=1;side=side+2)begin
   frame(side*30,side*50,1);frame(side*30,side*50,1);
   if(!valid || visibility!=4 || !ready || $abs(steer)>1 || $abs(align_yaw)>1 || $abs(align_offset)>1)
    $fatal(1,"PIXEL_START_PRELOAD side=%0d valid=%0d visible=%b steer=%0d yaw=%0d",side,valid,visibility,steer,align_yaw);
   frame(side*30,0,0);frame(side*30,0,0);
   if(!valid || visibility!=7 || steer*side>=-2)
    $fatal(1,"PIXEL_NEAR_CORRECTION side=%0d steer=%0d",side,steer);
   $display("PASS full-frame navigation startup: far slope rejected, measured near lane corrects side=%0d",side);
  end
  $finish;
 end
 initial begin #50000000;$fatal(1,"image startup timeout");end
endmodule
