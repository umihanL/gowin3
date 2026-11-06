`timescale 1ns/1ps
module vehicle_extension_tb;
 reg clk=0;always #5 clk=~clk;
 reg rst=0,tick=0,scope=1,link=1,valid=0,front_valid=1,zebra_update=0,zebra=0,legacy_run=0;
 reg [7:0] op=0,seq=0;reg [31:0] args=0;reg signed [31:0] e1=0,e2=0;
 reg signed [15:0] a1=0,a2=0;reg [15:0] front=1000;
 wire [7:0] result,mode,phase,id,flags,ec,et,es;wire [15:0] remaining;wire owned,run,brake,event_valid;wire zebra_enabled,zebra_can_change;reg zebra_toggle=0;
 wire signed [15:0] speed,steer;
 vehicle_extension #(.LEASE_TICKS(40),.STALL_TICKS(150)) dut(
 .clk(clk),.rst_n(rst),.tick(tick),.scope(scope),.drive_page(1'b1),.link_ok(link),.command_valid(valid),.opcode(op),.sequence_id(seq),.arguments(args),.result(result),
 .encoder1(e1),.encoder2(e2),.actual1(a1),.actual2(a2),.front_valid(front_valid),.front_mm(front),.zebra_update(zebra_update),.zebra_detected(zebra),.zebra_toggle(zebra_toggle),.zebra_enable(zebra_enabled),.zebra_can_change(zebra_can_change),
 .legacy_run(legacy_run),.legacy_speed(16'd100),.owned(owned),.run(run),.brake(brake),.speed(speed),.steer(steer),
 .task_mode(mode),.phase(phase),.task_id(id),.flags(flags),.remaining_mm(remaining),.event_valid(event_valid),.event_code(ec),.event_task(et),.event_status(es));
 integer done_events=0,fault_events=0,zebra_events=0;reg [7:0] last_task,last_reason;
 always @(negedge clk)if(event_valid)begin
   if(ec==8'hf0)done_events=done_events+1;
   if(ec==8'hf1)fault_events=fault_events+1;
   if(ec==8'hf3)zebra_events=zebra_events+1;
   last_task=et;last_reason=es;
 end
 task ticks(input integer n,input integer delta);integer i;begin
  for(i=0;i<n;i=i+1)begin
   @(negedge clk);e1=e1+delta;e2=e2+delta;tick=1;
   @(negedge clk);tick=0;repeat(2)@(negedge clk);
  end
 end endtask
 task cmd(input [7:0] command,s,input [31:0] arg,input [7:0] expected=0);begin
  @(negedge clk);op=command;seq=s;args=arg;
  @(negedge clk);if(result!==expected)$fatal(1,"command %h result %d expected %d phase%0d",command,result,expected,phase);
  valid=result==0;@(negedge clk);valid=0;repeat(2)@(negedge clk);
 end endtask
 integer i,old_events;
 initial begin
  repeat(4)@(negedge clk);rst=1;ticks(12,0);
  if(dut.MM_PER_COUNT_Q16<8700 || dut.MM_PER_COUNT_Q16>8720)$fatal(1,"wheel scale overflow");
  cmd(8'h10,1,{16'd100,16'd100},1);
  cmd(8'h12,2,2);ticks(12,0);
  cmd(8'h10,30,{16'd100,16'd100});
  for(i=0;i<20;i=i+1)begin
   @(negedge clk);e1=e1+1;e2=e2+2;tick=1;@(negedge clk);tick=0;repeat(2)@(negedge clk);
  end
  if(dut.progress_q16!=20*3*8710/2)$fatal(1,"odd wheel counts lost half count: %0d",dut.progress_q16);
  cmd(8'h05,31,0);ticks(14,0);
  cmd(8'h10,3,{16'd100,16'd100});ticks(80,3);
  if(!run || speed<=0 || remaining<60 || remaining>75)$fatal(1,"forward encoder distance %d",remaining);
  cmd(8'h10,3,{16'd100,16'd100});if(remaining>75)$fatal(1,"duplicate restarted");
  for(i=0;i<200 && phase==1;i=i+1)ticks(1,3);
  if(phase!=3 || !brake || done_events!=0)$fatal(1,"distance must brake before completion");
  ticks(14,0);if(phase!=4 || done_events!=1 || last_task!=3)$fatal(1,"measured stop complete");
  cmd(8'h10,3,{16'd100,16'd100});if(phase!=4)$fatal(1,"duplicate completed request restarted");
  cmd(8'h10,4,{16'd80,-16'sd100});ticks(20,-2);
  if(speed>=0 || remaining>=100)$fatal(1,"reverse not signed speed=%d remaining=%d phase=%d req=%d prog=%d dq=%d directed=%d",speed,remaining,phase,dut.requested_speed,dut.progress_q16,dut.delta_q16,dut.directed_delta);
  old_events=done_events;cmd(8'h05,5,0);ticks(14,0);
  if(phase!=0 || done_events!=old_events)$fatal(1,"STOP pretended completion");
  cmd(8'h11,6,{16'd0,16'd60});ticks(30,1);cmd(8'h11,6,{16'd0,16'd60});ticks(30,1);
  if(!run)$fatal(1,"velocity renewal failed");
  cmd(8'h11,7,{16'd0,16'd60},4);ticks(15,1);ticks(14,0);
  if(phase!=5 || fault_events!=1 || last_reason!=4)$fatal(1,"400ms lease not enforced");
  cmd(8'h11,6,{16'd0,16'd60},3);if(run)$fatal(1,"late lease restarted expired movement");
  cmd(8'h12,8,3);ticks(14,0);cmd(8'h17,80,1);seq=9;legacy_run=1;repeat(4)@(negedge clk);ticks(20,1);
  @(negedge clk);zebra=1;zebra_update=1;@(negedge clk);zebra_update=0;
  if(run || phase!=3)$fatal(1,"crossing not stopped immediately");
  if(zebra_events!=0)$fatal(1,"crossing emitted before stopped");
  ticks(14,0);if(phase!=6 || zebra_events!=1)$fatal(1,"crossing not published after stop");
  cmd(8'h13,10,1);ticks(20,0);if(run)$fatal(1,"red moved");
  cmd(8'h13,11,2);ticks(20,1);if(!run)$fatal(1,"green no resume");
  @(negedge clk);zebra_update=1;@(negedge clk);zebra_update=0;ticks(5,1);
  if(!run || zebra_events!=1)$fatal(1,"same crossing retriggered");
  link=0;repeat(3)@(negedge clk);ticks(14,0);if(run || phase!=5)$fatal(1,"linkloss must stop");
  link=1;ticks(15,0);if(run)$fatal(1,"link restore resumed");
  scope=0;repeat(3)@(negedge clk);if(run || mode!=0)$fatal(1,"page loss retained mode");
  $display("PASS extension encoder move/reverse, duplicate rejection, stop feedback, velocity lease, independent traffic task and disconnect");$finish;
 end
 initial begin #5000000;$fatal(1,"extension test timeout");end
endmodule
