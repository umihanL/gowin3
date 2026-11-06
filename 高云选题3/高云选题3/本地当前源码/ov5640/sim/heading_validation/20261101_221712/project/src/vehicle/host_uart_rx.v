// 8N1 receiver: two-flop input synchronizer, centre sampling, stop validation.
module host_uart_rx #(parameter CLK_HZ=33000000,BAUD=115200)(
 input clk,rst_n,rx,output reg valid,output reg [7:0] data,output reg framing_error
);
 localparam BIT_CYCLES=(CLK_HZ+BAUD/2)/BAUD;
 localparam W=$clog2(BIT_CYCLES+1);
 reg meta,line;reg [W-1:0] timer;reg [3:0] state;reg [7:0] shift;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin meta<=1;line<=1;end
  else begin meta<=rx;line<=meta;end
 end
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin state<=0;timer<=0;valid<=0;data<=0;shift<=0;framing_error<=0;end
  else begin
   valid<=0;framing_error<=0;
   if(state==0)begin
    if(!line)begin timer<=BIT_CYCLES/2-1;state<=1;end
   end else if(timer!=0)timer<=timer-1'b1;
   else if(state==1)begin
    if(line)state<=0;else begin timer<=BIT_CYCLES-1;state<=2;end
   end else if(state<=9)begin
    shift[state-2]<=line;timer<=BIT_CYCLES-1;state<=state+1'b1;
   end else if(state==10)begin
    if(line)begin data<=shift;valid<=1;state<=0;end
    else begin framing_error<=1;state<=11;end
   end else if(line)state<=0; // A held-low/break input cannot create commands.
  end
 end
endmodule
