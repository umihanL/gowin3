`timescale 1ns/1ps
module vehicle_host_link_tb;
 localparam CLK_HZ=1000000,BAUD=100000,BIT=10;
 reg clk=0;always #5 clk=~clk;reg rst=0,rx=1,selected=0,drive_page=1,nav_page=0,ready=1;
 reg [3:0] status=0,phase=0;reg event_valid=0;reg [7:0] event_code=8'he0;reg [15:0] event_value=16'h1234;
 wire tx,run,nav,link;wire [15:0] speed;wire [7:0] task_id;
 vehicle_host_link #(.CLK_HZ(CLK_HZ),.BAUD(BAUD),.LINK_TIMEOUT_MS(200)) dut(
  .clk(clk),.rst_n(rst),.rx(rx),.selected(selected),.drive_page(drive_page),
  .navigation_page(nav_page),.navigation_ready(ready),.motion_status(status),.motion_phase(phase),
  .event_valid(event_valid),.event_code(event_code),.event_value(event_value),
  .tx(tx),.run(run),.nav_session(nav),.speed_limit(speed),.link_ok(link),.task_id(task_id));
 wire received,error;wire [7:0] data;integer bytes=0;reg [7:0] received_bytes[0:2047];
 host_uart_rx #(.CLK_HZ(CLK_HZ),.BAUD(BAUD)) monitor(.clk(clk),.rst_n(rst),.rx(tx),
 .valid(received),.data(data),.framing_error(error));
 always @(negedge clk)if(received)begin received_bytes[bytes]=data;bytes=bytes+1;end
 task uart(input [7:0] b,input bit good_stop=1,input integer idle_cycles=3);integer j;begin
  @(negedge clk);rx=0;repeat(BIT)@(negedge clk);
  for(j=0;j<8;j=j+1)begin rx=b[j];repeat(BIT)@(negedge clk);end
  rx=good_stop;repeat(BIT)@(negedge clk);rx=1;repeat(idle_cycles)@(negedge clk);
 end endtask
 task command(input [7:0] seq,op,b0,b1,b2,b3,input bit good=1);begin
  uart(8'ha5);uart(8'h5a);uart(1);uart(seq);uart(op);
  uart(b0);uart(b1);uart(b2);uart(b3);uart(8'h01^seq^op^b0^b1^b2^b3^(good?0:8'h80));
 end endtask
 task await_frames(input integer wanted);integer timeout;begin
  timeout=0;while(bytes<wanted*10 && timeout<50000)begin @(negedge clk);timeout=timeout+1;end
  if(bytes<wanted*10)$fatal(1,"missing reply wanted=%0d bytes=%0d",wanted,bytes);
  repeat(30)@(negedge clk);
 end endtask
 task check_frame(input integer frame,input [7:0] op,b0);integer j;reg [7:0] sum;begin
  if(received_bytes[frame*10]!=8'ha5 || received_bytes[frame*10+1]!=8'h5a ||
     received_bytes[frame*10+4]!=op || received_bytes[frame*10+5]!=b0)
    $fatal(1,"bad reply frame=%0d op=%h/%h result=%h/%h",frame,received_bytes[frame*10+4],op,received_bytes[frame*10+5],b0);
  sum=0;for(j=2;j<=8;j=j+1)sum=sum^received_bytes[frame*10+j];
  if(sum!=received_bytes[frame*10+9])$fatal(1,"reply checksum");
 end endtask
 integer frames=0;
 initial begin
  repeat(10)@(negedge clk);rst=1;repeat(20)@(negedge clk);
  command(1,3,0,7,0,0);await_frames(1);check_frame(0,8'h83,1);
  if(run || nav)$fatal(1,"not selected command started car");
  selected=1;repeat(10)@(negedge clk);
  command(2,2,160,0,0,0);await_frames(2);check_frame(1,8'h82,0);if(speed!=160)$fatal(1,"speed command");
  command(3,3,0,7,0,0);await_frames(3);check_frame(2,8'h83,0);if(!run || nav || task_id!=7)$fatal(1,"drive START");
  command(4,4,80,0,0,0);await_frames(4);check_frame(3,8'h84,0);if(speed!=80)$fatal(1,"slowdown");
  command(5,2,99,0,0,0,0);repeat(1500)@(negedge clk);if(speed!=80 || bytes!=40)$fatal(1,"bad checksum mutated state");
  command(6,2,8'he9,3,0,0);await_frames(5);check_frame(4,8'h82,2);if(speed!=80)$fatal(1,"range accepted");
  command(7,8'h55,0,0,0,0);await_frames(6);check_frame(5,8'hd5,5);
  command(8,5,0,0,0,0);await_frames(7);check_frame(6,8'h85,0);if(run)$fatal(1,"STOP");
  uart(8'ha5);uart(8'h5a);repeat(22000)@(negedge clk);
  uart(8'ha5,0);repeat(100)@(negedge clk);
  command(9,3,0,8,0,0);await_frames(8);if(!run)$fatal(1,"parser failed to resync");
  repeat(202000)@(negedge clk);await_frames(9);check_frame(8,8'hf2,8);if(run || link)$fatal(1,"watchdog");
  command(10,1,0,0,0,0);await_frames(10);if(run || !link)$fatal(1,"heartbeat rearmed");
  selected=0;repeat(5)@(negedge clk);selected=1;drive_page=0;nav_page=1;repeat(10)@(negedge clk);
  ready=0;command(11,3,1,9,0,0);await_frames(11);check_frame(10,8'h83,3);if(nav)$fatal(1,"unready nav");
  ready=1;command(12,3,1,9,0,0);await_frames(12);if(!run || !nav)$fatal(1,"nav START");
  status=2;repeat(15)@(negedge clk);command(13,5,0,0,0,0);await_frames(13);if(run || !nav)$fatal(1,"pause discarded nav");
  command(14,7,0,0,0,0);await_frames(14);if(!run || !nav)$fatal(1,"nav resume");
  status=3;await_frames(15);check_frame(14,8'hf0,9);if(run || !nav)$fatal(1,"completion must disarm and retain displayed session");
  command(15,7,0,0,0,0);await_frames(16);check_frame(15,8'h87,3);if(run)$fatal(1,"completed nav restarted");
  event_valid=1;@(negedge clk);event_valid=0;await_frames(17);check_frame(16,8'he0,9);
  if(received_bytes[166]!=8'h34 || received_bytes[167]!=8'h12)$fatal(1,"extension event value");
  // Overlap commands: ordinary mutation is rejected, STOP still acts.
  selected=0;repeat(5)@(negedge clk);selected=1;drive_page=1;nav_page=0;status=0;repeat(5)@(negedge clk);
  command(16,3,0,10,0,0);await_frames(18);if(!run)$fatal(1,"re-arm drive");
  command(17,1,0,0,0,0);command(18,5,0,0,0,0);
  if(run)$fatal(1,"STOP during reply did not act immediately");
  await_frames(20);check_frame(19,8'h85,0);
  command(19,1,0,0,0,0);
  // Back-to-back bytes finish before the reply's final stop bit.
  uart(8'ha5,1,0);uart(8'h5a,1,0);uart(1,1,0);uart(20,1,0);uart(2,1,0);
  uart(99,1,0);uart(0,1,0);uart(0,1,0);uart(0,1,0);uart(1^20^2^99,1,0);await_frames(22);
  check_frame(21,8'h82,4);if(speed!=80)$fatal(1,"BUSY mutated speed");
  $display("PASS host UART: real RX/TX bits, checksum, scope, bounds, startup/slowdown/STOP, parser timeout/bad stop, watchdog, navigation pause/resume/completion and extension event");$finish;
 end
 initial begin #30000000;$fatal(1,"host test timeout bytes=%0d pos=%0d",bytes,dut.pos);end
endmodule
