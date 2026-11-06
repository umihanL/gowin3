// Serial constant multiply: full range, no new ROM or DSP.
// Publish the converted speed and Q4 count target atomically.
module motor_target_counts #(
    parameter integer COUNTS_PER_REV=1560,SAMPLE_MS=10,
    parameter integer WHEEL_DIAMETER_MM=66,MAX_SPEED_MM_S=1000
)(input clk,input signed [15:0] speed_mm_s,
    output reg signed [19:0] counts_q4=0,
    output reg signed [15:0] converted_speed=0);
    localparam [63:0] DEN=64'd1*WHEEL_DIAMETER_MM*314159*1000;
    localparam [31:0] SCALE_Q20=(64'd1048576*COUNTS_PER_REV*SAMPLE_MS*16*100000+DEN/2)/DEN;
    wire signed [15:0] bounded=speed_mm_s>MAX_SPEED_MM_S?MAX_SPEED_MM_S:
        speed_mm_s < -MAX_SPEED_MM_S?-MAX_SPEED_MM_S:speed_mm_s;
    reg [4:0] step=0;
    reg signed [15:0] saved=0;
    reg [15:0] multiplier=0;
    reg [47:0] acc=0,scale=0;
    wire [47:0] sum=acc+(multiplier[0]?scale:48'd0);
    wire [47:0] rounded=(sum+48'd524288)>>20;
    always @(posedge clk)begin
        if(step==0)begin
            saved<=bounded;multiplier<=bounded[15]?-bounded:bounded;
            acc<=0;scale<=SCALE_Q20;step<=1;
        end else begin
            acc<=sum;scale<=scale<<1;multiplier<=multiplier>>1;
            if(step==16)begin
                counts_q4<=saved[15]?-$signed(rounded[19:0]):$signed(rounded[19:0]);
                converted_speed<=saved;step<=0;
            end else step<=step+1'b1;
        end
    end
endmodule
