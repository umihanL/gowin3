`timescale 1ns/1ps
// Exercise the real UART receiver/transmitter and production pump interlocks.
module vehicle_pump_tb;
 localparam CLK_HZ=100000, BAUD=10000, BIT=10;
 reg clk=0;always #5 clk=~clk;
 reg rst=0,rx=1,selected=0,drive_page=1;
 reg signed [15:0] actual1=0,actual2=0;
 wire tx,run,nav,link,pump_command,gate;
 wire [7:0] opcode,sequence_id,pump_result;
 wire [31:0] arguments,pump_status;
 wire scope=selected && drive_page;
 vehicle_host_link #(.CLK_HZ(CLK_HZ),.BAUD(BAUD),.LINK_TIMEOUT_MS(250)) host(
  .clk(clk),.rst_n(rst),.rx(rx),.selected(selected),.drive_page(drive_page),
  .navigation_page(1'b0),.navigation_ready(1'b1),.planning_page(1'b0),.route_page(1'b0),
  .start_heading_valid(1'b1),.motion_status(4'd0),.motion_phase(4'd0),
  .extension_result(8'd0),.extension_status(32'd0),.extension_capabilities(32'h003f0100),
  .pump_supported(1'b1),.pump_result(pump_result),.pump_status(pump_status),.pump_command_valid(pump_command),
  .extension_opcode(opcode),.extension_sequence(sequence_id),.extension_arguments(arguments),
  .event_valid(1'b0),.extension_event_valid(1'b0),.tx(tx),.run(run),.nav_session(nav),.link_ok(link));
 vehicle_pump #(.CLK_HZ(CLK_HZ)) pump(.clk(clk),.rst_n(rst),.scope(scope),.link_ok(link),
  .movement_requested(run),.actual1(actual1),.actual2(actual2),.command_valid(pump_command),
  .opcode(opcode),.sequence_id(sequence_id),.arguments(arguments),.result(pump_result),
  .status_word(pump_status),.gate_high(gate));
 wire valid,error;wire [7:0] data;
 integer bytes=0;reg [7:0] received[0:4095];
 host_uart_rx #(.CLK_HZ(CLK_HZ),.BAUD(BAUD)) monitor(.clk(clk),.rst_n(rst),.rx(tx),.valid(valid),.data(data),.framing_error(error));
 always @(negedge clk)if(valid)begin received[bytes]=data;bytes=bytes+1;end
 task uart(input [7:0] b,input integer idle=3);integer i;begin
  @(negedge clk);rx=0;repeat(BIT)@(negedge clk);
  for(i=0;i<8;i=i+1)begin rx=b[i];repeat(BIT)@(negedge clk);end
  rx=1;repeat(BIT+idle)@(negedge clk);
 end endtask
 task command(input [7:0] seq,op,input [31:0] args,input bit good=1,input integer idle=3);begin
  uart(8'ha5,idle);uart(8'h5a,idle);uart(1,idle);uart(seq,idle);uart(op,idle);
  uart(args[7:0],idle);uart(args[15:8],idle);uart(args[23:16],idle);uart(args[31:24],idle);
  uart(1^seq^op^args[7:0]^args[15:8]^args[23:16]^args[31:24]^(good?0:8'h80),idle);
 end endtask
 task wait_bytes(input integer wanted);integer n;begin
  n=0;while(bytes<wanted && n<50000)begin @(negedge clk);n=n+1;end
  if(bytes!=wanted)$fatal(1,"UART reply missing or unexpected wanted=%0d got=%0d",wanted,bytes);
  repeat(30)@(negedge clk);
 end endtask
 integer seq=0,frames=0;
 task checked(input [7:0] op,input [31:0] args,input [7:0] expected);integer j;reg [7:0] checksum;begin
  seq=seq+1;command(seq,op,args);frames=frames+1;wait_bytes(frames*10);
  if(received[(frames-1)*10+4]!=(op|8'h80) || received[(frames-1)*10+5]!=expected)
    $fatal(1,"bad ACK op=%h result=%h expected=%h",op,received[(frames-1)*10+5],expected);
  checksum=0;for(j=2;j<9;j=j+1)checksum=checksum^received[(frames-1)*10+j];
  if(checksum!=received[(frames-1)*10+9])$fatal(1,"reply checksum");
 end endtask
 task heartbeat;begin checked(1,0,0);end endtask
 task off;begin checked(8'h19,0,0);if(gate)$fatal(1,"OFF did not lower B13");end endtask
 task on(input [15:0] ms);begin heartbeat();checked(8'h18,{16'd0,ms},0);if(!gate)$fatal(1,"ON no output");end endtask
 reg [15:0] before_remaining;
 integer i;
 initial begin
  repeat(10)@(negedge clk);rst=1;repeat(10)@(negedge clk);
  if(gate || !pump_status[5] || pump_status[15:8]!=8)$fatal(1,"reset must be off/locked");
  off();checked(8'h18,100,1);
  selected=1;repeat(11000)@(negedge clk);heartbeat();
  checked(8'h18,0,2);checked(8'h18,1001,2);checked(8'h18,32'h10064,2);
  on(400);before_remaining=pump_status[31:16];
  checked(8'h18,400,4);
  if(!gate || pump_status[31:16]>=before_remaining)$fatal(1,"ON retry extended pulse");
  checked(3,0,4);if(run)$fatal(1,"motion started while pumping");
  // A status query snapshots the output register and does not renew the link.
  seq=seq+1;command(seq,8'h1a,0);frames=frames+1;wait_bytes(frames*10);
  if(!received[(frames-1)*10+5][0] || received[(frames-1)*10+6]!=0)$fatal(1,"pump status missing output");
  // OFF must act even while a preceding reply is still transmitting.
  seq=seq+1;command(seq,1,0);seq=seq+1;command(seq,8'h19,0,1,0);
  repeat(5)@(negedge clk);if(gate)$fatal(1,"busy OFF did not act");
  frames=frames+2;wait_bytes(frames*10);
  if(received[(frames-1)*10+4]!=8'h99 || received[(frames-1)*10+5]!=0)$fatal(1,"busy OFF ack");
  on(30);repeat(4000)@(negedge clk);
  if(gate || !pump_status[5] || pump_status[15:8]!=1)$fatal(1,"pulse timeout lock");
  heartbeat();checked(8'h18,100,3);off();on(100);
  actual1=20;repeat(3)@(negedge clk);
  if(gate || pump_status[15:8]!=4 || !pump_status[5])$fatal(1,"motion cutoff");
  actual1=0;repeat(11000)@(negedge clk);off();on(1000);
  // Repeated status reads do not keep the UART control lease alive.
  for(i=0;i<12;i=i+1)begin
   seq=seq+1;command(seq,8'h1a,0);frames=frames+1;wait_bytes(frames*10);
  end
  repeat(3000)@(negedge clk);
  if(gate || pump_status[15:8]!=2 || !pump_status[5])$fatal(1,"link loss cutoff");
  off();on(200);selected=0;repeat(5)@(negedge clk);
  if(gate || !pump_status[5] || pump_status[15:8]!=3)$fatal(1,"scope exit cutoff");
  off();if(pump_status[5])$fatal(1,"out-of-scope OFF did not clear lock");
  selected=1;repeat(5)@(negedge clk);on(200);checked(5,0,0);
  if(gate || !pump_status[5] || pump_status[15:8]!=5)$fatal(1,"STOP must cut pump");
  off();heartbeat();checked(3,0,0);if(!run)$fatal(1,"motion test setup");
  checked(8'h18,100,3);if(gate)$fatal(1,"ON while drive running");checked(5,0,0);
  repeat(11000)@(negedge clk);off();heartbeat();
  seq=seq+1;command(seq,8'h18,100,0);repeat(2000)@(negedge clk);
  if(gate || bytes!=frames*10)$fatal(1,"corrupt ON accepted");
  on(200);checked(8'h12,0,0);if(gate || !pump_status[5] || pump_status[15:8]!=6)$fatal(1,"mode request cutoff");
  $display("PASS B13 pump UART: default off, OFF any scope/busy, finite pulse, replay rejection, deadline lock, link loss, motion interlock, mode/STOP, status and checksum");$finish;
 end
 initial begin #10000000;$fatal(1,"pump test timeout");end
endmodule
