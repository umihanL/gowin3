`timescale 1ns/1ps
// Full native camera frames feed the production detector and motion directly.
// Encoders follow an independent straight-line plant; no controller forcing.
module navigation_no_zebra_limit_tb;
 reg clk=0;always #5 clk=~clk;
 reg rst=0,fb=0,req=0,pv=0;reg [11:0] x=0,y=0;reg [15:0] rgb=0;
 wire zu,zd;wire [15:0] preview;
 vision_video camera(.clk(clk),.rst_n(rst),.zebra_enable(1'b1),.frame_begin(fb),.data_req(req),.pixel_valid(pv),
  .x(x),.y(y),.rgb(rgb),.threshold(8'd96),.preview(preview),.zebra_update(zu),.zebra_detected(zd));
 reg [3:0] mode=0;reg [15:0] configured=200,front=1500;reg pause=0;
 reg signed [31:0] e1=0,e2=0;real travel=0;
 wire tick,drive;wire signed [15:0] speed,steer;wire [3:0] status;wire [10:0] index;
 reg [16:0] rx,ry;
 always @(posedge clk)begin rx<=(index==0?500:3500)*256/10;ry<=2700*256/10;end
 vehicle_motion #(.CLK_HZ(2000000),.START_DELAY_MS(0),.STALL_MS(3000)) motion(
  .clk(clk),.rst_n(rst),.mode(mode),.navigation_request(mode==9),.navigation_avoidance(1'b0),
  .route_valid(1'b1),.profile_ready(1'b1),.route_count(12'd2),.route_index(index),
  .route_x_q8(rx),.route_y_q8(ry),.route_lock(1'b1),.route_curve(2'sd0),
  .configured_speed(configured),.slowdown_cm(16'd65),.bypass_cm(16'd65),.brake_speed(16'd200),
  .encoder1(e1),.encoder2(e2),.imu_available(1'b1),.imu_update(1'b1),.imu_restart(1'b0),.imu_yaw(16'sd0),
  .range_frame(1'b1),.range_samples(4'hf),.range_observations(4'hf),.range_diagnostics(32'd0),
  .left_mm(16'd1500),.right_mm(16'd1500),.left_front_mm(front),.right_front_mm(front),
  .remote_link(1'b1),.remote_run(1'b1),.remote_speed(16'sd200),.remote_steer(16'sd7),.host_pause(pause),
  .zebra_update(zu),.zebra_detected(zd),.drive_enabled(drive),.speed_mm_s(speed),.steer_deg(steer),
  .status(status),.tick_10ms(tick));
 integer ticks=0,frames=0,old_index,old_x,old_y;
 always @(negedge clk)if(rst && tick)begin
  ticks=ticks+1;
  if(drive)travel=travel+speed*0.01;
  e1=$rtoi(travel*1560.0/(66.0*3.141592654));e2=e1;
  if(mode==9 && status==2 && steer!=0)$fatal(1,"camera frame changed pure route steering");
 end
 task wait_ticks(input integer n);integer last;begin last=ticks+n;wait(ticks>=last);@(negedge clk);end endtask
 function white_band(input integer row);begin
  white_band=(row>=34 && row<=35)||(row>=38 && row<=39)||(row>=42 && row<=43)||
             (row>=46 && row<=47)||(row>=60 && row<=62);
 end endfunction
 task frame(input integer pattern,complete);integer xx,yy,sx,sy,white;begin
  for(yy=0;yy<480;yy=yy+1)for(xx=0;xx<640;xx=xx+1)begin
   @(negedge clk);req=1;pv=complete || !(xx==310 && yy==220);x=80+xx;y=yy;
   sx=xx*104/640;sy=yy*78/480;
   white=pattern==1?white_band(sy):pattern==2?(sy>=60 && sy<=62):pattern==3?1:0;
   // Alternating isolated salt/pepper away from a single sample of each window.
   if(pattern==4)white=white_band(sy)^((sx==44 || sx==52 || sx==60) && sy%3==0);
   rgb=white?16'hffff:16'h0000;
  end
  @(negedge clk);pv=0;req=0;fb=1;@(negedge clk);fb=0;#1;
  if(zu!==complete[0])$fatal(1,"complete-frame qualifier wrong");
  if(complete)begin
   frames=frames+1;
   if(zd!==(pattern==1 || pattern==4))$fatal(1,"raw image zebra result wrong pattern=%0d detected=%0d",pattern,zd);
  end
  @(negedge clk);
 end endtask
 initial begin
  repeat(8)@(negedge clk);rst=1;@(negedge clk);fb=1;@(negedge clk);fb=0;wait_ticks(3);mode=9;wait_ticks(115);
  if(!drive || status!=2 || speed!=200)$fatal(1,"camera-free normal cruise failed");
  old_index=index;old_x=motion.target_x_mm;old_y=motion.target_y_mm;
  frame(1,1);wait_ticks(38);
  if(speed!=200 || !drive || status!=2)$fatal(1,"image zebra changed autonomous navigation speed speed=%0d",speed);
  if(index!=old_index || motion.target_x_mm!=old_x || motion.target_y_mm!=old_y)$fatal(1,"camera changed path target");
  frame(0,0);wait_ticks(8);if(speed!=200)$fatal(1,"incomplete frame altered cruise speed");
  frame(4,1);wait_ticks(5);if(speed!=200)$fatal(1,"noise altered cruise speed");
  frame(0,1);wait_ticks(75);if(speed!=200)$fatal(1,"clear image altered 200 target speed=%0d",speed);
  frame(2,1);frame(3,1);if(speed!=200)$fatal(1,"single marking/white image changed navigation speed");
  configured=80;wait_ticks(65);frame(1,1);wait_ticks(15);
  if(speed!=80)$fatal(1,"configured 80mm/s changed by zebra speed=%0d",speed);
  configured=0;wait_ticks(50);if(speed!=0)$fatal(1,"camera overrode zero speed target");
  configured=200;pause=1;wait_ticks(3);if(speed || drive)$fatal(1,"zebra overrode communication pause");
  pause=0;wait_ticks(115);if(speed!=200 || !drive)$fatal(1,"navigation pause/resume changed requested speed");
  // Other tasks retain their own controls even when zebra remains visible.
  mode=2;wait_ticks(3);if(speed!=200 || steer!=7)$fatal(1,"zebra altered remote control");
  mode=8;wait_ticks(115);if(speed!=200)$fatal(1,"zebra altered independent bypass task");
  mode=9;wait_ticks(115);if(speed!=200)$fatal(1,"visible crossing altered navigation re-entry speed");
  // 2026-11-01: map navigation deliberately ignores ordinary range stops.
  // Manual/communication pause remains authoritative even at 40 mm.
  front=40;wait_ticks(4);if(speed!=200 || !drive || status!=2)$fatal(1,"range stop leaked into map navigation");
  pause=1;wait_ticks(3);if(speed || drive)$fatal(1,"manual pause lost while ignoring range");
  $display("PASS native camera->detector->navigation: no zebra speed limiting at 200 or 80, incomplete frame/noise and ordinary range cannot change speed/steering, explicit pause retained; frames=%0d",frames);
  $finish;
 end
 initial begin #400000000;$fatal(1,"camera integration timeout");end
endmodule
