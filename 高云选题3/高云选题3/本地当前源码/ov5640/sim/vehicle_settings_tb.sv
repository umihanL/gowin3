`timescale 1ns/1ps
module vehicle_settings_tb;
 reg clk=0;always #5 clk=~clk;
 reg rst_n=0,tv=0,td=0,req=0;
 reg [15:0] tx=0,ty=0;reg [11:0] x=0,y=0;
 wire [10:0] center;wire [3:0] screen;wire [15:0] pixel,limit;
 wire remote,host,avoidance,run,edit,inc,dec;wire signed [15:0] ts,steer,rs,rsteer;
 reg sample=0;reg [15:0] keys=16'hffff;reg [7:0] lx=128,ry=128;
 wire rrun,link;reg zebra_enabled=0,zebra_change=1;wire zebra_toggle;
 always @(posedge clk)if(zebra_toggle)zebra_enabled<=!zebra_enabled;
 // Every editor/remote update must keep the legacy stop-task port identical.
 always @(negedge clk)if(rst_n && ui.brake_speed!==limit)
  $fatal(1,"legacy brake speed diverged from shared setting");
 navigation_ui ui(.host_point_select(1'b0),.host_point_start(1'b0),.host_point_cancel(1'b0),.host_point_id(4'd0),
 .host_route_command(1'b0),.host_route_opcode(8'd0),.host_route_arguments(32'd0),.zebra_enable(zebra_enabled),.zebra_can_change(zebra_change),.zebra_toggle(zebra_toggle),.clk(clk),.rst_n(rst_n),.data_req(req),.x_addr(x),.y_addr(y),.video(16'h1234),
  .touch_valid(tv),.touch_down(td),.touch_x(tx),.touch_y(ty),.pixel(pixel),.operation_mode(screen),
  .speed_edit(edit),.speed_increase(inc),.speed_decrease(dec),.control_remote(remote),.control_host(host),
  .navigation_avoidance(avoidance),.servo_center_tenths(center),
  .configured_speed(limit),.touch_run(run),.touch_speed(ts),.touch_steer(steer));
 ps2_test_control #(.CLK_HZ(20000),.RUNTIME_SPEED(1),.LEFT_STEER_DEG(36),.RIGHT_STEER_DEG(34)) pad(
  .clk(clk),.rst_n(rst_n && screen==2 && remote),.sample_valid(sample),.analog_valid(1'b1),
  .buttons_raw(keys),.left_x(lx),.right_y(ry),.speed_limit(limit),.speed_edit(edit),
  .speed_increase(inc),.speed_decrease(dec),.steer_deg(rsteer),.speed_mm_s(rs),
  .angle_locked(),.run_enabled(rrun),.manual_mode(),.link_ok(link));
 task tap(input integer a,b);
  begin @(negedge clk);tx=a;ty=b;tv=1;td=1;@(negedge clk);tv=0;
   repeat(8)@(negedge clk);tv=1;td=0;@(negedge clk);tv=0;repeat(8)@(negedge clk);end
 endtask
 task frame;
  begin @(negedge clk);sample=1;@(negedge clk);sample=0;repeat(20)@(negedge clk);end
 endtask
 task button(input integer bitno);
  begin keys[bitno]=0;frame;frame;frame;keys[bitno]=1;frame;frame;frame;end
 endtask
 task back;begin tap(32,32);end endtask
 task snapshot(input [511:0] filename);
  integer f,a,b;
  begin f=$fopen(filename,"wb");if(!f)$fatal(1,"snapshot file open failed");$fwrite(f,"P6\n800 480\n255\n");
   for(b=0;b<480;b=b+1)for(a=0;a<800;a=a+1)begin
    @(negedge clk);x=a;y=b;req=1;repeat(2)@(negedge clk);#1;
    $fwrite(f,"%c%c%c",{pixel[4:0],pixel[4:2]},{pixel[10:5],pixel[10:9]},{pixel[15:11],pixel[15:13]});
   end
   $fclose(f);req=0;
  end
 endtask
 initial begin
  repeat(8)@(negedge clk);rst_n=1;repeat(28200)@(negedge clk);
  if(remote || limit!=200 || run)$fatal(1,"default must be manual/stopped/200");
  if($test$plusargs("snapshot"))snapshot("sim/settings_home.ppm");
  if($test$plusargs("revision_snapshot"))snapshot("sim/revision_home.ppm");
  tap(700,40);if(screen!=10)$fatal(1,"settings home");
  if(avoidance)$fatal(1,"navigation avoidance must default off");
  tap(580,436);tap(740,400);tap(580,456);
  if(avoidance || screen!=10)$fatal(1,"avoidance hitbox boundary");
  tap(580,400);
  if(!avoidance || remote || host || limit!=200 || screen!=10)$fatal(1,"avoidance ON must change only the avoidance selection");
  if($test$plusargs("avoidance_snapshot"))snapshot("sim/inertial_zebra_menu_on.ppm");
  back;tap(700,40);if(!avoidance)$fatal(1,"navigation avoidance selection must persist across pages");
  tap(456,400);tap(580,432);tap(720,400);
  if(!avoidance || screen!=10)$fatal(1,"avoidance gaps/boundaries must not clear ON");
  tap(320,400);if(avoidance || screen!=10)$fatal(1,"avoidance OFF isolation");
  if($test$plusargs("avoidance_snapshot"))snapshot("sim/inertial_zebra_menu_off.ppm");
  $display("PASS navigation menu: default OFF, separate ON/OFF buttons, exact hitboxes, page persistence, source/speed settings independent");
  tap(200,180);if(screen!=11)$fatal(1,"mode selector");
  if($test$plusargs("snapshot"))snapshot("sim/settings_modes.ppm");
  back;tap(200,300);if(screen!=12)$fatal(1,"parameter menu");
  if($test$plusargs("speed_snapshot"))snapshot("sim/shared_speed_parameters.ppm");
  tap(200,150);if(screen!=15)$fatal(1,"speed settings submenu");
  tap(200,350);if(screen!=14)$fatal(1,"button adjustment");
  tap(650,280);if(limit!=250)$fatal(1,"+50");tap(490,280);if(limit!=255)$fatal(1,"+5");
  tap(310,280);if(limit!=250)$fatal(1,"-5");tap(150,280);if(limit!=200)$fatal(1,"-50");
  repeat(5)tap(150,280);if(limit!=0)$fatal(1,"lower saturation");
  repeat(22)tap(650,280);if(limit!=1000)$fatal(1,"upper saturation");
  back;tap(200,240);if(screen!=13)$fatal(1,"keypad");
  tap(360,176);tap(510,392);tap(510,392);tap(510,392); // 1000
  if(ui.keypad_value!=1000)$fatal(1,"four-digit entry");
  tap(360,176);if(!ui.keypad_error || ui.keypad_value!=1000)$fatal(1,"overflow accepted");
  tap(360,392);if(ui.keypad_value!=100)$fatal(1,"delete");
  tap(650,392);if(limit!=100)$fatal(1,"confirm");
  if($test$plusargs("snapshot"))snapshot("sim/settings_keypad.ppm");
  back;back;back;back; // home
  tap(580,180);if(screen!=2)$fatal(1,"driving page");
  keys[3]=0;frame;frame;frame;if(rrun || rs || limit!=100)$fatal(1,"manual must ignore PS2");keys=16'hffff;
  tap(648,248);if(!run || ts!=100)$fatal(1,"touch forward");
  tap(552,320);if(steer!=28)$fatal(1,"touch left");
  tap(736,320);if(steer!=-28)$fatal(1,"touch right");
  tap(648,384);if(ts!=-100)$fatal(1,"touch reverse");
  tap(552,384);if(run || ts || steer)$fatal(1,"touch stop");
  if($test$plusargs("snapshot"))snapshot("sim/settings_drive.ppm");
  back;if(run)$fatal(1,"page exit must stop");
  tap(700,40);tap(200,180);tap(200,300);if(!remote)$fatal(1,"select remote");
  back;back;tap(580,180);frame;frame;frame;
  if(rrun)$fatal(1,"remote entry armed");button(3);if(!rrun || rs!=100)$fatal(1,"remote START");
  button(13);if(!edit || rrun || rs)$fatal(1,"circle editing must disarm");
  button(11);if(limit!=150)$fatal(1,"R1 increment %d",limit);
  keys[11]=0;repeat(12)frame;if(limit!=200)$fatal(1,"held R1 repeated");keys[11]=1;frame;frame;frame;
  button(9);if(limit!=150)$fatal(1,"R2 decrement");
  button(13);if(edit || rrun)$fatal(1,"exit edit must stay disarmed");
  button(3);if(!rrun || rs!=150)$fatal(1,"new cruise target");
  lx=0;ry=0;frame;frame;if(rsteer!=-36 || rs!=150)$fatal(1,"left/full forward");
  lx=255;ry=255;frame;frame;if(rsteer!=34 || rs!=-150)$fatal(1,"right/reverse %d %d",rsteer,rs);
  repeat(4100)@(negedge clk);if(link || rrun || edit || rs)$fatal(1,"link loss");
  back;tap(700,40);tap(200,300);tap(200,150);tap(200,350);if(limit!=150)$fatal(1,"remote/touch value mismatch");
  back;back;tap(200,210);if(screen!=13 || !ui.edit_distance)$fatal(1,"distance keypad");
  tap(510,176);tap(510,392);tap(510,392);tap(650,392);
  if(ui.slowdown_cm!=200 || limit!=150)$fatal(1,"200cm or speed isolation");
  back;tap(200,210);tap(510,248);tap(510,392);tap(650,392);
  if(ui.slowdown_cm!=50)$fatal(1,"50cm setting");
  back;tap(200,210);tap(360,176);tap(650,392);
  if(!ui.keypad_error || ui.slowdown_cm!=50)$fatal(1,"unsafe short distance accepted");
  back;tap(200,210);tap(360,248);tap(360,176);tap(650,392);
  if(ui.slowdown_cm!=41)$fatal(1,"1cm resolution");
  back;tap(200,275);if(screen!=13 || ui.edit_parameter!=2)$fatal(1,"bypass parameter selector");
  tap(650,176);tap(510,248);tap(650,392);
  if(ui.bypass_cm!=35 || ui.slowdown_cm!=41 || limit!=150)$fatal(1,"35cm bypass setting/isolation");
  back;tap(200,275);tap(650,176);tap(360,248);tap(650,392);
  if(!ui.keypad_error || ui.bypass_cm!=35)$fatal(1,"bypass lower bound");
  back;tap(200,275);tap(360,176);tap(510,248);tap(510,392);tap(650,392);
  if(ui.bypass_cm!=150)$fatal(1,"bypass upper bound");
  if(ui.brake_speed!=150)$fatal(1,"stop task must share target speed");
  $display("PASS parameters: 35/150cm bypass bounds, invalid 34 rejected, one shared speed for manual/remote/avoidance");
  back;
  if($test$plusargs("servo_snapshot"))snapshot("sim/servo_center_parameters.ppm");
  tap(200,372);if(screen!=12)$fatal(1,"parameter gap opened editor");
  tap(200,405);if(screen!=12)$fatal(1,"removed fifth parameter row remains clickable");
  tap(200,340);if(screen!=14 || ui.edit_parameter!=4 || center!=900)$fatal(1,"fourth row servo center selector/default");
  tap(650,280);if(center!=910)$fatal(1,"center +1.0");
  tap(490,280);if(center!=911)$fatal(1,"center +0.1");
  tap(310,280);if(center!=910)$fatal(1,"center -0.1");
  tap(150,280);if(center!=900)$fatal(1,"center -1.0");
  @(negedge clk);tx=490;ty=280;tv=1;td=1;repeat(20)@(negedge clk);
  if(center!=901)$fatal(1,"held center button repeated");
  td=0;repeat(2)@(negedge clk);tv=0;
  repeat(60)tap(150,280);tap(310,280);if(center!=500)$fatal(1,"center lower saturation");
  tap(490,280);tap(150,280);if(center!=500)$fatal(1,"center coarse lower clamp");
  repeat(80)tap(650,280);tap(490,280);if(center!=1200)$fatal(1,"center upper saturation");
  tap(310,280);tap(650,280);if(center!=1200)$fatal(1,"center coarse upper clamp");
  repeat(70)tap(150,280);repeat(42)tap(650,280);repeat(3)tap(490,280);
  if(center!=923 || limit!=150 || ui.slowdown_cm!=41 || ui.bypass_cm!=150 || ui.brake_speed!=150 || !remote || host || avoidance)
   $fatal(1,"servo parameter isolation");
  repeat(800)@(negedge clk);
  if(ui.renderer.display_bcd[28]!=16'h0923 || ui.renderer.display_bcd[29]!=16'h0423 || ui.renderer.display_bcd[30]!=16'h1523)
   $fatal(1,"servo center/limits decimal display");
  if($test$plusargs("servo_snapshot"))snapshot("sim/servo_center_adjust.ppm");
  back;if(screen!=12 || center!=923)$fatal(1,"servo back/persistence");
  if($test$plusargs("servo_snapshot"))snapshot("sim/servo_center_parameters_set.ppm");
  tap(200,150);tap(200,350);
  if(screen!=14 || ui.edit_parameter!=0)$fatal(1,"speed editor retained servo selection");
  tap(490,280);if(limit!=155 || center!=923)$fatal(1,"speed/servo editor isolation");
  tap(310,280);back;back;
  tap(200,275);if(screen!=13 || ui.edit_parameter!=2 || center!=923)$fatal(1,"servo leaked into distance editor");
  $display("PASS servo center UI: fourth row, +/-0.1/1.0 degree, held debounce, 50.0/120.0 clamps, 92.3 center / 42.3..152.3 display, isolation and page persistence");
  back;back;tap(200,180);tap(200,400);
  if(screen!=11 || !host || remote)$fatal(1,"communication mode selection");
  if($test$plusargs("snapshot") || $test$plusargs("host_snapshot"))snapshot("sim/settings_host.ppm");
  back;back;tap(580,180);tap(648,248);tap(552,320);
  if(run || ts || steer)$fatal(1,"host mode must ignore touch motion");
  back;tap(700,40);tap(200,180);tap(200,180);
  if(host || remote)$fatal(1,"switch host back to manual");
  back;tap(580,400);
  if(screen!=10 || !avoidance)$fatal(1,"avoidance ON selection before reset");
  rst_n=0;repeat(4)@(negedge clk);rst_n=1;repeat(4)@(negedge clk);
  if(remote || avoidance || limit!=200 || run || ui.slowdown_cm!=65 || ui.bypass_cm!=65 || ui.brake_speed!=200 || center!=900)$fatal(1,"reset defaults");
  tap(200,340);if(screen!=3)$fatal(1,"real-time avoidance menu");
  if($test$plusargs("speed_snapshot"))snapshot("sim/shared_speed_avoidance.ppm");
  tap(776,40);tap(700,72);if(screen!=3)$fatal(1,"avoidance speed entry boundary");
  tap(700,40);if(screen!=15 || ui.edit_parameter!=0)$fatal(1,"avoidance shared speed editor");
  tap(200,350);tap(650,280);if(limit!=250 || ui.brake_speed!=250)$fatal(1,"avoidance edit does not share speed");
  back;back;back;back;tap(200,340);tap(200,265);
  if(screen!=7)$fatal(1,"automatic stop task entry");
  if($test$plusargs("speed_snapshot"))snapshot("sim/shared_speed_stop.ppm");
  tap(700,40);if(screen!=15 || run)$fatal(1,"editing active stop task must exit motion mode");
  back;back;back;tap(200,340);tap(200,370);
  if(screen!=8)$fatal(1,"detour task entry");
  if($test$plusargs("speed_snapshot"))snapshot("sim/shared_speed_detour.ppm");
  tap(700,40);if(screen!=15 || run)$fatal(1,"editing active detour task must exit motion mode");
  back;back;back;tap(580,180);tap(648,248);
  if(screen!=2 || !run || ts!=250)$fatal(1,"avoidance speed edit must map to manual driving");
  back;if(run)$fatal(1,"manual mode exit must stop after shared speed edit");
  $display("PASS shared speed entry: menu/stop/detour display one target; editing exits task; touch drive reuses edited target");
  tap(180,452);if(screen!=2 || !host || remote || run)$fatal(1,"extension1 entry must select stopped communication page");
  if($test$plusargs("snapshot"))snapshot("sim/extension_drive.ppm");
  if($test$plusargs("revision_snapshot"))snapshot("sim/revision_zebra_off.ppm");
  tap(650,150);if(!zebra_enabled)$fatal(1,"zebra UI on action");
  if($test$plusargs("revision_snapshot"))snapshot("sim/revision_zebra_on.ppm");
  zebra_change=0;tap(650,150);if(!zebra_enabled)$fatal(1,"zebra busy UI emitted toggle");
  zebra_change=1;tap(650,150);if(zebra_enabled)$fatal(1,"zebra UI off action");
  back;tap(580,452);if(screen!=2 || !host || run)$fatal(1,"extension2 entry");
  $display("PASS extension menu: two visible entries select communication driving and never start motors");
  $display("PASS settings: manual default/200, all menus, keypad/delete/confirm/limits, +/-5/50, touch driving, PS2 ignored in manual, circle/R1/R2 edge debounce, synchronized target, disarm/rearm, disconnect and reset");$finish;
 end
 initial begin #3000000000;$fatal(1,"settings timeout");end
endmodule
