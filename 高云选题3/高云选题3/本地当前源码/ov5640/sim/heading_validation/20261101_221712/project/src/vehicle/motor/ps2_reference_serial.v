// Pin sequence follows supplied PS2_Cmd (STM32) or scanout (C51).
// STYLE 0: STM32 delay arguments interpreted as explicit FPGA microseconds.
// STYLE 1: C51 delay comment (2.5+n us), with 2 us CMD setup margin.
// C delay loops are uncalibrated: these are explicit comparison timings,
// not a claim about the original MCU program's measured clock frequency.
module ps2_reference_serial #(
    parameter integer CLK_HZ=50000000,
    parameter integer STYLE=0
)(
    input clk,rst_n,start,long_setup,
    input [71:0] tx_frame,
    input [4:0] tx_length,
    output reg [71:0] rx_frame,
    output reg done,busy,
    output reg [1:0] dat_levels,
    output reg [15:0] dat_edges,
    output reg ps2_cs_n,ps2_clk,ps2_mosi,
    input ps2_miso
);
    localparam integer CMD_CYCLES=(STYLE==0)?CLK_HZ/10000:CLK_HZ/500000;
    localparam integer LOW_CYCLES=(STYLE==0)?CLK_HZ/10000:CLK_HZ/80000;
    localparam integer HOLD_CYCLES=(STYLE==0)?CLK_HZ/1000000:(CLK_HZ/2000000)*11;
    localparam integer CS_LONG=CLK_HZ/10000;
    localparam integer CS_SHORT=(STYLE==0)?CLK_HZ/100000:CLK_HZ/500000;
    localparam integer BYTE_RELEASE=(STYLE==0)?1:CLK_HZ/1000000;
    localparam integer MAX_CYCLES=(STYLE==0)?CS_LONG:LOW_CYCLES;
    localparam integer TIMER_W=(MAX_CYCLES<2)?1:$clog2(MAX_CYCLES);
    localparam [2:0] IDLE=0,CS_WAIT=1,CMD_WAIT=2,CLK_LOW=3,CLK_HOLD=4,BYTE_WAIT=5;
    reg [2:0] state;
    reg [TIMER_W-1:0] timer;
    reg [71:0] tx_shift;
    reg [7:0] rx_byte;
    reg [2:0] bit_index;
    reg [4:0] byte_index,length;
    reg previous_miso;
    (* syn_preserve=1 *) reg miso_meta,miso_sync;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin miso_meta<=1; miso_sync<=1; end
        else begin miso_meta<=ps2_miso; miso_sync<=miso_meta; end
    end
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            state<=IDLE; timer<=0; tx_shift<=0; rx_byte<=0;
            bit_index<=0; byte_index<=0; length<=9;
            rx_frame<=0; done<=0; busy<=0;
            dat_levels<=0; dat_edges<=0; previous_miso<=1;
            ps2_cs_n<=1; ps2_clk<=1; ps2_mosi<=1;
        end else begin
            done<=0;
            if(busy) begin
                dat_levels[miso_sync]<=1'b1;
                if(miso_sync!=previous_miso && dat_edges!=16'hffff)
                    dat_edges<=dat_edges+1'b1;
                previous_miso<=miso_sync;
            end
            case(state)
                IDLE: if(start) begin
                    tx_shift<=tx_frame; length<=tx_length;
                    rx_frame<=0; rx_byte<=0; bit_index<=0; byte_index<=0;
                    dat_levels<=0; dat_edges<=0; previous_miso<=miso_sync;
                    ps2_cs_n<=0; ps2_clk<=1; busy<=1;
                    timer<=(long_setup && STYLE==0)?CS_LONG-1:CS_SHORT-1;
                    state<=CS_WAIT;
                end
                CS_WAIT,BYTE_WAIT: if(timer!=0) timer<=timer-1'b1;
                    else begin
                        ps2_mosi<=tx_shift[0]; timer<=CMD_CYCLES-1; state<=CMD_WAIT;
                    end
                CMD_WAIT: if(timer!=0) timer<=timer-1'b1;
                    else begin
                        ps2_clk<=0; timer<=LOW_CYCLES-1; state<=CLK_LOW;
                    end
                CLK_LOW: if(timer!=0) timer<=timer-1'b1;
                    else begin
                        // Sample before raising CLK. CMD stays unchanged
                        // throughout the following post-rise hold interval.
                        rx_byte[bit_index]<=miso_sync;
                        ps2_clk<=1; timer<=HOLD_CYCLES-1; state<=CLK_HOLD;
                        if(bit_index==7)
                            rx_frame[byte_index*8 +: 8]<={miso_sync,rx_byte[6:0]};
                    end
                CLK_HOLD: if(timer!=0) timer<=timer-1'b1;
                    else begin
                        tx_shift<={1'b0,tx_shift[71:1]};
                        if(bit_index!=7) begin
                            bit_index<=bit_index+1'b1;
                            ps2_mosi<=tx_shift[1];
                            timer<=CMD_CYCLES-1; state<=CMD_WAIT;
                        end else if(byte_index==length-1'b1) begin
                            // C51 scanout releases CMD; STM32 PS2_Cmd
                            // leaves the final command bit on the line.
                            if(STYLE==1) ps2_mosi<=1;
                            ps2_cs_n<=1; busy<=0; done<=1; state<=IDLE;
                        end else begin
                            byte_index<=byte_index+1'b1; bit_index<=0; rx_byte<=0;
                            if(STYLE==1) begin
                                ps2_mosi<=1; timer<=BYTE_RELEASE-1; state<=BYTE_WAIT;
                            end else begin
                                ps2_mosi<=tx_shift[1]; timer<=CMD_CYCLES-1; state<=CMD_WAIT;
                            end
                        end
                    end
                default: begin state<=IDLE; busy<=0; ps2_cs_n<=1; ps2_clk<=1; ps2_mosi<=1; end
            endcase
        end
    end
endmodule
