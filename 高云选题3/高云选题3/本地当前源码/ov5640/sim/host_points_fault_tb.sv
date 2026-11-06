`timescale 1ns/1ps
module host_points_fault_tb;
 reg clk=0;always #5 clk=~clk;reg rst=0,rx=1,selected=1;
 reg [3:0] mode=5,point=0;reg has_start=0,planning=0,ready=0;reg [2:0] error=0;reg [3:0] status=0;reg [7:0] extension_phase=0;integer busy_case;
 wire select_p,start_p,cancel_p,run,session,link,tx;wire [3:0] id;wire [15:0] speed;wire [7:0] task_id;
 vehicle_host_link #(.CLK_HZ(1000000),.BAUD(100000),.LINK_TIMEOUT_MS(40)) dut(
 .clk(clk),.rst_n(rst),.rx(rx),.selected(selected),.drive_page(mode==2),.navigation_page(mode==9),.navigation_ready(ready),
 .planning_page(mode==5),.point_select_ready(has_start && !planning),.point_route_ready(ready),.point_planning(planning),
 .selected_point(point),.point_error(error),.point_select(select_p),.point_start(start_p),.point_cancel(cancel_p),.point_id(id),
 .motion_status(status),.motion_phase(4'd0),.event_valid(1'b0),.event_code(8'd0),.event_value(16'd0),
 .route_page(1'b0),.extension_result(8'd5),.traffic_start_blocked(1'b0),
 .extension_status({24'd0,extension_phase}),.extension_capabilities(32'd0),.extension_event_valid(1'b0),
 .extension_event_code(8'd0),.extension_event_task(8'd0),.extension_event_status(8'd0),
 .tx(tx),.run(run),.nav_session(session),.speed_limit(speed),.link_ok(link),.task_id(task_id));
 always @(posedge clk)begin
  if(select_p)begin point<=id;planning<=1;ready<=0;error<=0;end
  if(start_p)mode<=9;
  if(cancel_p)begin planning<=0;ready<=0;error<=3;end
 end
 wire valid,frame_error;wire [7:0] data;integer bytes=0;reg [7:0] rb[0:4095];
 host_uart_rx #(.CLK_HZ(1000000),.BAUD(100000)) mon(.clk(clk),.rst_n(rst),.rx(tx),.valid(valid),.data(data),.framing_error(frame_error));
 always @(negedge clk)if(valid)begin rb[bytes]=data;bytes=bytes+1;end
 task uart(input [7:0] b);integer j;begin
  @(negedge clk);rx=0;repeat(10)@(negedge clk);
  for(j=0;j<8;j=j+1)begin rx=b[j];repeat(10)@(negedge clk);end
  rx=1;repeat(13)@(negedge clk);
 end endtask
 integer seq=0,last;
 task command(input [7:0] op,a,b,c,d,input integer expected);integer k,start_bytes,t;reg found;begin
  seq=seq+1;start_bytes=bytes;uart(8'ha5);uart(8'h5a);uart(1);uart(seq);uart(op);uart(a);uart(b);uart(c);uart(d);uart(1^seq^op^a^b^c^d);
  t=0;found=0;while(!found && t<10000)begin
   @(negedge clk);t=t+1;for(k=start_bytes/10;k<bytes/10;k=k+1)
    if(rb[k*10+3]==seq && rb[k*10+4]==(op|8'h80))begin found=1;last=k;end
  end
  if(!found || rb[last*10+5]!=expected)$fatal(1,"fault ACK op=%h expected=%0d result=%0d",op,expected,rb[last*10+5]);
  repeat(40)@(negedge clk);
 end endtask
 initial begin
  repeat(5)@(negedge clk);rst=1;repeat(5)@(negedge clk);
  command(8,1,7,0,0,3);has_start=1;
  ready=1;
  for(busy_case=0;busy_case<4;busy_case=busy_case+1)begin
   case(busy_case)0:extension_phase=1;1:extension_phase=2;2:extension_phase=3;3:extension_phase=6;endcase
   command(3,1,7,0,0,4);command(8,1,7,0,0,4);
   command(6,0,0,0,0,0);command(9,0,0,0,0,0);
   command(8'h14,0,0,0,0,extension_phase);command(8'h15,0,0,0,0,0);
   if(run || session || planning || point!=0)$fatal(1,"busy extension was preempted phase=%0d",extension_phase);
  end
  extension_phase=0;ready=0;
  command(8,1,7,1,0,2);command(8,0,7,0,0,2);command(8,255,7,0,0,2);
  command(8,9,7,0,0,0);if(!planning || run || point!=9)$fatal(1,"plan command state");
  command(9,0,0,0,0,0);if(rb[last*10+7]!=1)$fatal(1,"plan query");
  command(8,1,8,0,0,4);command(3,1,7,0,0,3);command(5,0,0,0,0,0);
  if(planning || ready || error!=3 || run)$fatal(1,"STOP failed to cancel plan");
  command(9,0,0,0,0,0);if(rb[last*10+7]!=3 || rb[last*10+8]!=3)$fatal(1,"cancel query");
  command(8,2,8,0,0,0);planning=0;error=1;
  command(9,0,0,0,0,0);if(rb[last*10+7]!=3 || rb[last*10+8]!=1)$fatal(1,"failed route query");
  command(3,1,8,0,0,3);
  command(8,2,8,0,0,0);planning=0;error=0;ready=1;
  command(3,1,8,0,0,0);if(mode!=9 || !run || !session || !link)$fatal(1,"automatic handoff");
  command(3,1,8,0,0,4);command(8,3,9,0,0,1);
  repeat(42000)@(negedge clk);if(run || link || !session)$fatal(1,"watchdog state");
  command(1,0,0,0,0,0);if(run)$fatal(1,"heartbeat rearmed after timeout");
  command(7,0,0,0,0,0);if(!run)$fatal(1,"explicit resume after timeout");
  mode=4;repeat(10)@(negedge clk);if(run || session)$fatal(1,"page exit");
  mode=5;ready=1;repeat(10)@(negedge clk);command(3,1,8,0,0,0);
  selected=0;repeat(10)@(negedge clk);if(run || session)$fatal(1,"source switch");
  command(8,1,9,0,0,1);command(3,1,9,0,0,1);
  $display("PASS destination faults: extension phase 1/2/3/6 cannot be preempted by 03/08; 06/09/14/15 remain readable; bad ID/argument, missing start, planning busy, STOP cancellation, failed route, handoff, duplicate START, watchdog, explicit resume, page/source exit");$finish;
 end
 initial begin #5000000;$fatal(1,"fault bench timeout");end
endmodule
