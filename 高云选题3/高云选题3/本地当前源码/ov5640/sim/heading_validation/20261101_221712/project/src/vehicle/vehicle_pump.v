// B13 active-high MOS control. This is FPGA output-state feedback, not a flow sensor.
// A finite one-shot cannot be extended by retries or heartbeats.
module vehicle_pump #(
 parameter CLK_HZ=33000000, MAX_ON_MS=1000, STILL_MS=100
)(
 input clk,rst_n,scope,link_ok,movement_requested,
 input signed [15:0] actual1,actual2,
 input command_valid,input [7:0] opcode,sequence_id,input [31:0] arguments,
 output reg [7:0] result,output [31:0] status_word,output reg gate_high
);
 localparam MS_CYCLES=CLK_HZ/1000;
 localparam MW=$clog2(MS_CYCLES+1);
 reg [MW-1:0] ms_counter;
 wire ms_tick=ms_counter==MS_CYCLES-1;
 wire stationary_sample=actual1>=-16'sd5 && actual1<=16'sd5 &&
                         actual2>=-16'sd5 && actual2<=16'sd5 && !movement_requested;
 reg [7:0] still_ms;
 wire wheels_still=stationary_sample && still_ms>=STILL_MS;
 reg lockout;
 wire ready=scope && link_ok && wheels_still && !gate_high && !lockout;
 reg [15:0] remaining_ms;
 reg [7:0] stop_reason;
 wire [7:0] status_flags={2'd0,lockout,ready,scope,link_ok,wheels_still,gate_high};
 assign status_word={remaining_ms,stop_reason,status_flags};
 always @*begin
  result=0;
  case(opcode)
   8'h18:if(!scope)result=1;
     else if(arguments[31:16]!=0 || arguments[15:0]==0 || arguments[15:0]>MAX_ON_MS)result=2;
     else if(gate_high)result=4;
     else if(!link_ok || !wheels_still || lockout)result=3;
   8'h19,8'h05:result=0;
   8'h1a:if(arguments!=0)result=2;
   default:result=5;
  endcase
 end
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin
   ms_counter<=0;still_ms<=0;gate_high<=0;remaining_ms<=0;stop_reason<=8;lockout<=1;
  end else begin
   if(ms_tick)ms_counter<=0;else ms_counter<=ms_counter+1'b1;
   if(!stationary_sample)still_ms<=0;
   else if(ms_tick && still_ms<STILL_MS)still_ms<=still_ms+1'b1;
   if(gate_high && ms_tick)begin
    if(remaining_ms<=1)begin gate_high<=0;remaining_ms<=0;stop_reason<=1;lockout<=1;end
    else remaining_ms<=remaining_ms-1'b1;
   end
   if(command_valid && result==0)begin
    if(opcode==8'h18)begin
     gate_high<=1;remaining_ms<=arguments[15:0];stop_reason<=0;
    end
    if(opcode==8'h19 || opcode==8'h05)begin
     gate_high<=0;remaining_ms<=0;stop_reason<=opcode==8'h05 ? 8'd5:8'd0;
     lockout<=opcode==8'h05;
    end
   end
   // Mode-setting and movement requests also cancel an existing burst. The
   // command path must not wait for the next measured wheel-speed update.
   if(command_valid && (opcode==8'h12 || opcode==8'h20 || opcode==8'h03 ||
      opcode==8'h07 || opcode==8'h10 || opcode==8'h11))begin
    gate_high<=0;remaining_ms<=0;stop_reason<=6;lockout<=1;
   end
   // Interlocks take priority over any simultaneous ON command.
   if(gate_high && !scope)begin gate_high<=0;remaining_ms<=0;stop_reason<=3;lockout<=1;end
   else if(gate_high && !link_ok)begin gate_high<=0;remaining_ms<=0;stop_reason<=2;lockout<=1;end
   else if(gate_high && !stationary_sample)begin gate_high<=0;remaining_ms<=0;stop_reason<=4;lockout<=1;end
   // Explicit OFF clears the latch even if a fault/page change arrives on the
   // same edge. A new ON still has to pass all current interlocks.
   if(command_valid && opcode==8'h19)begin
    gate_high<=0;remaining_ms<=0;stop_reason<=0;lockout<=0;
   end
  end
 end
endmodule
