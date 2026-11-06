`timescale 1ns/1ps
// Real thumbnail RAM + UI integration. The camera image is a synthetic road.
module vision_ui_tb;
 reg clk=0;always #5 clk=~clk;
 reg rst=0,req=0,tv=0,td=0,fb=0,capture=0;
 reg [11:0] x=0,y=0,cx=0,cy=0;
 reg [15:0] tx=0,ty=0,rgb=0;
 wire [15:0] preview,pixel;wire [7:0] threshold;
 wire [3:0] mode;wire update,road_valid;wire signed [8:0] near_error,heading;
 wire [15:0] portrait_osd;
 attitude_overlay #(.AXIS_VECTOR(1)) osd(.clk(clk),.rst_n(rst),.frame_begin(fb),.data_req(req),
   .portrait_mode(1'b1),.x_addr(x),.y_addr(y),.background(preview),
   .online(1'b1),.calibrated(1'b1),.available(1'b1),.sensor_status(3'd2),.cal_state(2'd2),
   .error_reg(8'd0),.error_expected(8'd0),.error_actual(8'd0),.error_mask(8'd0),.error_step(8'd0),
   .roll(16'sd123),.pitch(-16'sd45),.yaw(16'sd678),.pixel(portrait_osd));
 vision_video camera(.turn_hint(2'sd0),.clk(clk),.rst_n(rst),.frame_begin(fb),.data_req(capture || req),
   .pixel_valid(capture),.x(capture?cx:x),.y(capture?cy:y),.rgb(rgb),.threshold(threshold),
   .preview(preview),.road_update(update),.road_valid(road_valid),.road_near(near_error),.road_heading(heading));
 navigation_ui ui(.clk(clk),.rst_n(rst),.data_req(req),.x_addr(x),.y_addr(y),
   .video(ui.binary_page?portrait_osd:16'h1234),.preview(preview),.binary_threshold(threshold),
   .touch_valid(tv),.touch_down(td),.touch_x(tx),.touch_y(ty),.pixel(pixel),.operation_mode(mode),
   .speed_edit(1'b0),.speed_increase(1'b0),.speed_decrease(1'b0),
   .left_mm(16'hffff),.left_front_mm(16'hffff),.right_front_mm(16'hffff),.right_mm(16'hffff),
   .front_average_mm(16'hffff),.tof_diagnostics(32'd0),.front_valid(1'b0),
   .motion_status(4'd0),.motion_phase(4'd0),.commanded_speed(16'sd0),.commanded_steer(16'sd0),
   .pose_heading(16'sd0),.target_heading(16'sd0),.pose_x_q8(24'sd0),.pose_y_q8(24'sd0),
   .target_x_mm(16'sd0),.target_y_mm(16'sd0),.target_distance_mm(16'd0),
   .profile_ready(1'b0),.profile_segments(4'd0),.profile_length_mm(16'd0),
   .profile_left_revs(16'd0),.profile_right_revs(16'd0),.profile_entry_heading(16'sd0),
   .profile_exit_heading(16'sd0),.profile_peak_steer(8'd0));
 task tap(input integer u,input integer v);begin
   @(negedge clk);tx=u;ty=v;tv=1;td=1;
   @(negedge clk);tv=0;repeat(6)@(negedge clk);
   tv=1;td=0;@(negedge clk);tv=0;repeat(6)@(negedge clk);
 end endtask
 task sample(input integer u,input integer v);begin
   @(negedge clk);x=u;y=v;req=1;
   repeat(3)@(negedge clk);#1;req=0;
 end endtask
 integer out,uu,vv,xx,yy,ru,rv,half_width;
 real ray_q,depth,centre,pitch;
 task snapshot(input [255:0] filename);begin
   out=$fopen(filename,"wb");$fwrite(out,"P6\n800 480\n255\n");
   for(vv=0;vv<480;vv=vv+1)for(uu=0;uu<800;uu=uu+1)begin
     sample(uu,vv);
     $fwrite(out,"%c%c%c",{pixel[4:0],pixel[4:2]},
       {pixel[10:5],pixel[10:9]},{pixel[15:11],pixel[15:13]});
   end
   $fclose(out);
 end endtask
 initial begin
   repeat(5)@(negedge clk);rst=1;
   // A complete native frame: rotation puts the road vertically in the preview.
   for(yy=0;yy<480;yy=yy+1)for(xx=0;xx<640;xx=xx+1)begin
     @(negedge clk);capture=1;cx=80+xx;cy=yy;
     // A 140mm physical strip, wide enough for the calibrated extractor.
     pitch=$atan2(175.0,413.0);ray_q=(yy-239.5)/554.2562584;
     depth=175/($sin(pitch)+ray_q*$cos(pitch));
     centre=319.5+554.2562584*5/depth;
     rgb=(depth>0 && xx>=centre-554.2562584*70/depth &&
          xx<=centre+554.2562584*70/depth)?16'h0000:16'hd6ba;
   end
   @(negedge clk);capture=0;fb=1;
   @(negedge clk);fb=0;wait(update);@(negedge clk);
   if(!road_valid)$fatal(1,"synthetic road extraction");
   tap(180,200);if(mode!=1 || ui.live_page!=0)$fatal(1,"live submenu");
   if($test$plusargs("snapshot"))snapshot("sim/menu_live.ppm");
   tap(180,300);repeat(1200)@(negedge clk);
   if(ui.live_page!=2 || threshold!=96 || ui.renderer.display_bcd[27][11:0]!=12'h096)
     $fatal(1,"binary view/default threshold/decimal value");
   sample(8+312,102+136);if(pixel!==16'h0000)$fatal(1,"upper road RGB alignment");
   sample(392+312,102+136);if(pixel!==16'h0000)$fatal(1,"binary dark road alignment");
   sample(392+312,102+259);if(pixel!==16'hffff)$fatal(1,"binary light ground alignment");
   if($test$plusargs("snapshot"))snapshot("sim/menu_binary.ppm");
   tap(782,431);repeat(1200)@(negedge clk);
   sample(392+312,102+136);if(pixel!==16'hffff)$fatal(1,"UI slider failed to change real pixels");
   tap(782,50);if(threshold!=255)$fatal(1,"portrait slider maximum unreachable: %d",threshold);
   tap(782,431);if(threshold!=0)$fatal(1,"portrait slider minimum unreachable");
   tap(32,448);tap(180,180);sample(200,430);
   if(pixel!==16'h1234)$fatal(1,"normal video unchanged");
   $display("PASS vision UI: real camera RAM, live submenu, synchronous preview alignment, decimal threshold, slider changes binary pixels, normal video");$finish;
 end
 initial begin #100000000;$fatal(1,"vision UI timeout");end
endmodule
