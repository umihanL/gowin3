`timescale 1ns/1ps
module vision_video_tb;
 reg clk=0;always #5 clk=~clk;
 reg rst=0,fb=0,req=0,pv=0;
 reg [11:0] x=0,y=0;reg [15:0] rgb=0;reg [7:0] threshold=96;
 wire fit_valid;wire [9:0] fit_forward0,fit_forward1;wire signed [8:0] fit_error0,fit_error1;
 wire [2:0] visibility;wire [15:0] preview;wire update,valid;wire signed [8:0] near_error,heading;
 reg fusion_tick=0,fusion_enabled=0;
 wire fusion_blocked;wire signed [15:0] fusion_steer;
 vehicle_vision_fusion fusion(.turn_entry_ready(1'b1),.mapped_entry_due(1'b0),.incoming_heading_error(16'sd0),.planned_exit_valid(1'b0),.planned_exit_heading(16'sd0),.clk(clk),.rst_n(rst),.tick(fusion_tick),.enabled(fusion_enabled),
 .road_update(update),.road_valid(valid),.road_junction(1'b0),.road_visibility(visibility),.terminal_approach(1'b0),
 .road_near(near_error),.road_heading(heading),.route_steer(16'sd28),.pose_heading(16'sd0),
 .travel_q8(32'sd0),.moving(1'b1),.planned_turn(2'sd1),.blocked(fusion_blocked),.steering(fusion_steer));
 task control;integer n;begin
   for(n=0;n<4;n=n+1)begin @(negedge clk);fusion_tick=1;@(negedge clk);fusion_tick=0;end
 end endtask
 bit incomplete=0,interrupt_scan=0;
 vision_video dut(.turn_hint(2'sd0),.clk(clk),.rst_n(rst),.frame_begin(fb),.data_req(req),.pixel_valid(pv),
 .x(x),.y(y),.rgb(rgb),.threshold(threshold),.preview(preview),.road_update(update),
 .road_fit_valid(fit_valid),.road_fit_forward0(fit_forward0),.road_fit_forward1(fit_forward1),
 .road_fit_error0(fit_error0),.road_fit_error1(fit_error1),
 .road_visibility(visibility),.road_valid(valid),.road_near(near_error),.road_heading(heading));
 task frame(input integer kind,input integer offset,input integer slope);
 integer xx,yy,u,v,center;real q,depth,forward,cu,theta,cam_pitch,half_width;begin
   cam_pitch=$atan2(185.0,395.0);theta=slope*3.141592653589793/1800.0;
   for(yy=0;yy<480;yy=yy+1)for(xx=0;xx<640;xx=xx+1)begin
     @(negedge clk);req=1;pv=!(incomplete && xx==639 && yy==479);x=80+xx;y=yy;
   u=yy*78/480;v=xx*104/640;center=52+offset+slope*(68-u)/28;
     rgb=(v>=center-10 && v<center+10)?16'h0000:16'hffff;
     if(kind==15)rgb=v<62?16'h0000:16'hffff;
     if(kind==16)rgb=v>42?16'h0000:16'hffff;
     if(kind==1)rgb=16'hffff;
     if(kind==2)rgb=16'h0000;
     if(kind==3)rgb=xx<320?(yy<240?16'h001f:16'h07e0):(yy<240?16'hf800:16'hffff);
     if(kind==4 && u<39)rgb=16'hffff;
     if((kind>=5 && kind<15) || kind>=17)begin
       // Independent pinhole ray/ground intersection, with the measured
       // height, lateral offset and 38cm nose-referenced ground target.
       q=(yy-239.5)/554.2562584;
       depth=185/($sin(cam_pitch)+q*$cos(cam_pitch));
       forward=185*($cos(cam_pitch)-q*$sin(cam_pitch))/($sin(cam_pitch)+q*$cos(cam_pitch))-15;
       cu=319.5+554.2562584*(2+offset+forward*$tan(theta))/depth;
       half_width=kind==8?143:kind>=7?127:70;
       rgb=depth>0 && xx>cu-554.2562584*half_width/depth && xx<cu+554.2562584*half_width/depth?16'h0000:16'hffff;
       if(kind==6 && u<39)rgb=16'hffff;
       if(kind==9 && (u==58 || u==44 || u==30))rgb=16'hffff;
       if(kind==11 && u>50)rgb=16'hffff;
       if(kind==12 && xx>=318 && xx<=326)rgb=16'hffff;
       if(kind==13 && rgb==16'hffff)rgb=16'h528a;
       if(kind==14)begin
         cu=319.5+554.2562584*(2+offset+0.0008*(forward-100)*(forward-100))/depth;
         rgb=depth>0 && xx>cu-554.2562584*127/depth && xx<cu+554.2562584*127/depth?16'h0000:16'hffff;
       end
       if(kind==17)begin
         // The bend is beyond 350mm; the incoming near lane is still straight.
         cu=319.5+554.2562584*(2+offset+(forward>350?0.002*(forward-350)*(forward-350):0))/depth;
         rgb=depth>0 && xx>cu-554.2562584*127/depth && xx<cu+554.2562584*127/depth?16'h0000:16'hffff;
       end
       if(kind==18 && forward<310)rgb=16'h0000;
       if(kind==10 && u>=40 && u<=48)rgb=v>3 && v<101?16'h0000:16'hffff;
     end
   end
   @(negedge clk);req=0;pv=0;fb=1;
   @(negedge clk);fb=0;
   if(interrupt_scan)begin
     wait(dut.scan_state==3 && dut.column==20 && dut.row==0);
     @(negedge clk);req=1;x=330;y=230;
     repeat(3)@(negedge clk);
     req=0;
   end
   wait(update);@(negedge clk);
  $display("IPM frame kind=%0d offset=%0d yaw=%0d valid=%b near=%0d heading=%0d",kind,offset,slope,valid,near_error,heading);
 end endtask
 task pixel(input integer u,input integer v,input bit binary,input [15:0] expected);
 begin
  @(negedge clk);req=1;x=(binary?396:28)+(v*60+30)/13;y=479-(u*60+30)/13;
  @(negedge clk);#1;if(preview!==expected)$fatal(1,"rotation/color/binary pixel %d %d got %h expected %h",u,v,preview,expected);
  req=0;
 end endtask
 initial begin
  repeat(5)@(negedge clk);rst=1;
  frame(7,0,0);if(!valid || near_error>4 || near_error < -4 || heading>5 || heading < -5)$fatal(1,"image centre %b %d %d",valid,near_error,heading);
  frame(7,5,0);if(!valid || near_error<3 || near_error>7 || heading>4 || heading < -4)$fatal(1,"5mm correction lost %b %d %d",valid,near_error,heading);
  frame(7,-5,0);if(!valid || near_error > -3 || near_error < -7 || heading>4 || heading < -4)$fatal(1,"-5mm correction lost %b %d %d",valid,near_error,heading);
  frame(7,30,0);if(!valid || near_error<24 || near_error>36 || heading>7 || heading < -7)$fatal(1,"30mm right-image offset %b %d %d",valid,near_error,heading);
  if(!fit_valid || fit_forward0<150 || fit_forward1<=fit_forward0+80 || fit_forward1>450 || $abs(fit_error0-30)>8 || $abs(fit_error1-30)>8)
    $fatal(1,"measured visual-map fit lost metric ground samples f=%0d,%0d e=%0d,%0d",fit_forward0,fit_forward1,fit_error0,fit_error1);
  frame(7,-30,0);if(!valid || near_error < -36 || near_error > -24 || heading>7 || heading < -7)$fatal(1,"30mm left-image offset %b %d %d",valid,near_error,heading);
  frame(7,60,0);if(!valid || near_error<48 || near_error>72)$fatal(1,"one-edge displacement lost correction %b %d",valid,near_error);
  frame(7,0,50);if(!valid || near_error<12 || heading<4)$fatal(1,"image yaw trend %b %d %d",valid,near_error,heading);
  frame(7,0,-50);if(!valid || near_error > -12 || heading > -4)$fatal(1,"negative image yaw trend %b %d %d",valid,near_error,heading);
  frame(6,0,-40);if(!valid || near_error>=0 || heading>=0)$fatal(1,"far loss fallback %b %d %d",valid,near_error,heading);
  frame(9,30,0);if(!valid || near_error<24 || near_error>36)$fatal(1,"one corrupted row per band defeated robust centre near=%d heading=%d valid=%b",near_error,heading,valid);
  frame(10,0,0);if(!valid || !dut.road_junction)$fatal(1,"visible middle road widening missed junction");
  // The restored connected-row extractor may retain a usable steering
  // centre on widened rows. Widening must remain tagged at the interface;
  // navigation_junction_fit tests that motion excludes such frames from
  // map registration, independently of the scan's internal row mask.
  if(!dut.road_visibility[2])$fatal(1,"junction frame lost complete-scene metadata");
  frame(7,-120,0);if(!valid || near_error>=0)$fatal(1,"left-clipped road reversed correction sign %b %d",valid,near_error);
  frame(8,0,0);if(!valid || near_error>4 || near_error < -4 || heading>5 || heading < -5)$fatal(1,"286mm road rejected/biased");
  frame(11,30,0);if(!valid || near_error<18 || near_error>45)$fatal(1,"visible upper lane lost when near road vanished %b %d",valid,near_error);
  frame(11,30,50);if(!valid || near_error<35 || near_error>61 || heading<4)$fatal(1,"upper-only IPM extrapolation %b %d %d",valid,near_error,heading);
  frame(12,0,0);if(!valid || near_error>6 || near_error < -6)$fatal(1,"thin painted marking split the road centre %b %d",valid,near_error);
  frame(13,0,0);frame(13,0,0);
  if(!valid || near_error>6 || near_error < -6)$fatal(1,"contrast adaptation failed on dim light floor %b %d",valid,near_error);
  frame(14,0,0);if(!valid || heading<=0)$fatal(1,"connected curved contour lost directional trend %b %d",valid,heading);
  $display("PASS dense contours: measured projection camera, near region missing, marking gaps, dim floor and curved road");
  $display("PASS metric IPM 254/286mm lanes: centred and +/-30mm, yaw and far loss");
  frame(17,0,0);
  if(!valid || $abs(near_error)>5 || $abs(heading)>5 || visibility!=7)
    $fatal(1,"distant curve pulled the still-straight near lane %b %d %d %b",valid,near_error,heading,visibility);
  frame(18,0,0);
  if(visibility!=4)$fatal(1,"distant boundary fragments hid near-field edge disappearance %b",visibility);
  $display("PASS bend proximity: far curve cannot pull near straight; far fragments cannot conceal near boundary loss");
  frame(15,0,0);if(!valid || visibility!=3'b110)$fatal(1,"left boundary loss did not preserve right boundary %b %b",valid,visibility);
  frame(16,0,0);if(!valid || visibility!=3'b101)$fatal(1,"right boundary loss did not preserve left boundary %b %b",valid,visibility);
  $display("PASS independent boundaries: clipped left/right edge retains opposite measured boundary and centre");
  frame(1,0,0);if(visibility!=3'b100)$fatal(1,"complete empty frame lost its visibility metadata");if(valid || fit_valid)$fatal(1,"blank road accepted");
  frame(2,0,0);if(valid)$fatal(1,"junction/full dark accepted");
  frame(3,0,0);
  // Native 4:3 landscape retains its upright camera orientation.
  pixel(12,12,0,16'h001f);pixel(86,12,0,16'hf800);
  pixel(12,64,0,16'h07e0);pixel(86,64,0,16'hffff);
  pixel(12,12,1,16'h0000);threshold=16;
  pixel(12,12,1,16'hffff);
  incomplete=1;frame(2,0,0);
  if(visibility!=0)$fatal(1,"incomplete frame presented geometric edge loss");
  if(valid || fit_valid)$fatal(1,"incomplete frame renewed road confidence");
  pixel(12,12,0,16'h001f);pixel(12,64,0,16'h07e0);
  incomplete=0;interrupt_scan=1;threshold=96;frame(7,30,0);
  if(!valid || near_error<24 || near_error>36 || heading>7 || heading < -7)
    $fatal(1,"scan interrupted by active display lost RAM read alignment");
  // Actual frame scan -> separate boundary metadata -> turn controller.
  // Left disappears first; keep the measured right edge until it also exits.
  fusion_enabled=1;
  frame(7,0,0);control;frame(7,0,0);control;
  frame(7,30,0);control;frame(7,30,0);control;frame(7,30,0);control;
  if(fusion_blocked || fusion_steer>=0)$fatal(1,"image-right road failed to steer the vehicle right %0d",fusion_steer);
  frame(7,-30,0);control;frame(7,-30,0);control;frame(7,-30,0);control;frame(7,-30,0);control;
  if(fusion_blocked || fusion_steer<=0)$fatal(1,"image-left road failed to steer the vehicle left %0d",fusion_steer);
  $display("PASS image-to-control sign: actual projected +/-30mm images steer toward the road in both directions");
  frame(7,0,0);control;frame(7,0,0);control;frame(7,0,0);control;
  frame(15,0,0);control;frame(15,0,0);control;
  if(fusion.blind_turn || fusion_blocked || fusion_steer>8 || fusion_steer < -8)
    $fatal(1,"actual single-edge frames caused early full-lock/stop");
  frame(1,0,0);control;
  if(fusion.blind_turn)$fatal(1,"one empty frame committed a bend");
  frame(1,0,0);control;
  if(!fusion.blind_turn || fusion_blocked)$fatal(1,"outer edge disappearance failed actual video-to-fusion commit");
  $display("PASS video-to-fusion sequence: surviving outer boundary prevents early lock; two complete empty frames commit left turn");
  $display("PASS vision video: coherent double buffer, full-frame sampling, upright portrait scaling, RGB222, live threshold, road centre/heading, blank/junction rejection, incomplete frame invalidation and scan preemption");$finish;
 end
 initial begin #180000000;$fatal(1,"vision video timeout");end
endmodule
