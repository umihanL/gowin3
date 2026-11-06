`timescale 1ns/1ps
module zebra_switch_tb;
 reg clk=0;always #5 clk=~clk;
 reg rst=0,tick=0,scope=1,link=1,command=0;reg [7:0] op=0,seq=0;reg [31:0] args=0;
 reg signed [31:0] enc=0;reg legacy=0;wire enabled,can_change,toggle;
 wire [7:0] result,mode,phase,flags;wire run,brake;
 reg tv=0,td=0;reg [15:0] tx=0,ty=0;wire [3:0] screen;
 reg fb=0,complete=1,sample=0,dark=1;reg [6:0] x=0,y=0;
 wire detected,update;
 vision_zebra detector(.clk(clk),.rst_n(rst),.enable(enabled),.frame_begin(fb),.frame_complete(complete),
 .sample_valid(sample),.dark(dark),.sample_x(x),.sample_y(y),.centre(7'd52),.update(update),.detected(detected));
 nav_control ui(.clk(clk),.rst_n(rst),.edit_distance(1'b0),.host_selected(scope),.host_command(1'b0),
 .host_opcode(8'd0),.host_arguments(32'd0),.host_motion_status(4'd0),.zebra_can_change(can_change),.zebra_toggle(toggle),
 .touch_valid(tv),.touch_down(td),.touch_x(tx),.touch_y(ty),.screen(screen),.snap_done(1'b0),.snap_accept(1'b0),
 .planned_anchor_valid(1'b0),.refine_done(1'b0),.refine_error(3'd0),.refined_length_cm(18'd0));
 vehicle_extension controller(.clk(clk),.rst_n(rst),.tick(tick),.scope(scope),.drive_page(1'b1),.link_ok(link),
 .command_valid(command),.opcode(op),.sequence_id(seq),.arguments(args),.result(result),
 .encoder1(enc),.encoder2(enc),.actual1(16'sd0),.actual2(16'sd0),.front_valid(1'b1),.front_mm(16'd1000),
 .zebra_update(update),.zebra_detected(detected),.zebra_toggle(toggle),.zebra_enable(enabled),.zebra_can_change(can_change),
 .legacy_run(legacy),.legacy_speed(16'd100),.task_mode(mode),.phase(phase),.flags(flags),.run(run),.brake(brake));
 task ticks(input integer n,input integer delta=0);integer j;begin
  for(j=0;j<n;j=j+1)begin @(negedge clk);enc=enc+delta;tick=1;@(negedge clk);tick=0;repeat(2)@(negedge clk);end
 end endtask
 task cmd(input [7:0] opcode,input [31:0] arg,input [7:0] expected=0);begin
  @(negedge clk);op=opcode;args=arg;seq=seq+1;@(negedge clk);
  if(result!==expected)$fatal(1,"switch command %h result%0d expected%0d phase%0d",op,result,expected,phase);
  command=result==0;@(negedge clk);command=0;repeat(3)@(negedge clk);
 end endtask
 task tap(input integer xx,yy);begin
  @(negedge clk);tv=1;td=1;tx=xx;ty=yy;@(negedge clk);tv=0;repeat(5)@(negedge clk);
  tv=1;td=0;@(negedge clk);tv=0;repeat(5)@(negedge clk);
 end endtask
 task frame_edge;begin @(negedge clk);sample=0;fb=1;@(negedge clk);fb=0;#1;end endtask
 task pattern(input integer first,last);integer xx,yy;begin
  for(yy=first;yy<=last;yy=yy+1)for(xx=0;xx<104;xx=xx+1)begin
   @(negedge clk);sample=1;x=xx;y=yy;
   dark=!((yy>=34 && yy<=35)||(yy>=38 && yy<=39)||(yy>=42 && yy<=43)||(yy>=46 && yy<=47)||(yy>=60 && yy<=62));
  end
  @(negedge clk);sample=0;
 end endtask
 initial begin
  repeat(5)@(negedge clk);rst=1;repeat(30000)@(negedge clk);ticks(14);
  if(enabled || flags[4])$fatal(1,"zebra must default OFF");
  pattern(34,74);frame_edge;if(detected || update)$fatal(1,"disabled detector processed image");
  tap(180,452);if(screen!=2)$fatal(1,"extension entry");
  tap(650,150);if(!enabled || !flags[4])$fatal(1,"local UI switch not wired to controller");
  pattern(34,74);frame_edge;if(update || detected)$fatal(1,"partial first enabled frame published");
  pattern(34,74);frame_edge;if(!update || !detected)$fatal(1,"fresh complete frame not detected");
  tap(650,150);if(enabled || detected || update || detector.white_bands!=0 || detector.collecting)$fatal(1,"disable failed to clear real detector");
  cmd(8'h17,2,2);cmd(8'h17,1);frame_edge;pattern(34,44);cmd(8'h17,0);cmd(8'h17,1);
  pattern(45,74);frame_edge;if(update || detected)$fatal(1,"on/off resurrected partial frame");
  pattern(34,74);frame_edge;if(!detected)$fatal(1,"restart full frame missing");
  cmd(8'h12,2);ticks(14);cmd(8'h11,{16'd0,16'd60});ticks(5,1);
  cmd(8'h17,0,4);tap(650,150);if(!enabled)$fatal(1,"moving local touch disabled detector");
  cmd(8'h05,0);ticks(14);cmd(8'h12,3);ticks(14);legacy=1;repeat(3)@(negedge clk);ticks(5,1);
  frame_edge;pattern(34,74);frame_edge;repeat(3)@(negedge clk);ticks(14);
  if(phase!=6 || run)$fatal(1,"traffic crossing did not wait after stop");
  cmd(8'h17,0);if(phase!=6 || run || !brake || enabled)$fatal(1,"disable released WAIT_LIGHT");
  cmd(8'h13,2,3);if(run)$fatal(1,"green bypassed disabled detector");
  cmd(8'h17,1);if(phase!=6 || run)$fatal(1,"enable resumed waiting car");
  cmd(8'h13,2);ticks(5,1);if(!run)$fatal(1,"explicit green failed after enabled");
  frame_edge;pattern(34,74);frame_edge;repeat(4)@(negedge clk);ticks(5,1);
  if(!run || phase!=2)$fatal(1,"off/on while waiting re-armed the same crossing");
  $display("PASS physical UI/UART zebra switch: default OFF, actual detector reset, full fresh frame, moving change rejected, WAIT_LIGHT disable retains stop and requires explicit enabled green");$finish;
 end
 initial begin #20000000;$fatal(1,"switch timeout");end
endmodule
