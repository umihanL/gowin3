`timescale 1ns/1ps
// Independent pin-level receiver: samples commands on falling SCLK and
// returns DAT with 200 ns propagation delay. Exercises initialization,
// digital/analog/pressure frame lengths, invalid headers and disconnection.
module ps2_receiver_model #(
    parameter integer MIN_PERIOD_NS=0,
    parameter integer MAX_PERIOD_NS=0
)(
    input cs_n, sclk, mosi, output reg miso=1,
    input connected, bad_header, bad_id,
    input [7:0] left_x,right_y,left_y,right_x,
    input [15:0] buttons,
    input [7:0] forced_mode,
    output reg [7:0] active_mode=8'h41,
    output integer transactions, output integer polls
);
    reg config_mode=0;
    reg [7:0] rx [0:20];
    reg [7:0] reply [0:20];
    integer bit_no, byte_no, i, count_bits, bytes;
    time previous_fall, mosi_changed;
    reg frame_period_ok;
    initial begin transactions=0; polls=0; mosi_changed=0; end
    always @(mosi) mosi_changed=$time;
    always @(negedge cs_n) begin
        if (sclk!==1) $fatal(1,"SCLK must idle high before select");
        bit_no=0; byte_no=0; count_bits=0; previous_fall=0;
        frame_period_ok=(MIN_PERIOD_NS==0 && MAX_PERIOD_NS==0);
        for(i=0;i<21;i=i+1) begin rx[i]=0; reply[i]=0; end
        reply[0]=8'hff;
        reply[1]=bad_id?8'h12:config_mode?8'hf3:
            (forced_mode!=0)?forced_mode:active_mode;
        reply[2]=bad_header?8'h00:8'h5a;
        reply[3]=buttons[7:0]; reply[4]=buttons[15:8];
        reply[5]=right_x; reply[6]=right_y; reply[7]=left_x; reply[8]=left_y;
        miso=1;
    end
    always @(negedge sclk) if (!cs_n) begin
        if ($time-mosi_changed<1000) $fatal(1,"Insufficient MOSI setup time");
        if (byte_no>20) $fatal(1,"Too many PS2 bytes");
        rx[byte_no][bit_no]=mosi;
        if (previous_fall && $time-previous_fall<3990)
            $fatal(1,"SCLK faster than 250 kHz");
        // A receiver variant can reject a transaction purely from measured
        // pin-level SCLK timing; it never reads the DUT profile selection.
        if (count_bits==1)
            frame_period_ok=(MIN_PERIOD_NS==0 || $time-previous_fall>=MIN_PERIOD_NS) &&
                (MAX_PERIOD_NS==0 || $time-previous_fall<=MAX_PERIOD_NS);
        previous_fall=$time;
        if (connected && frame_period_ok) miso<=#200 reply[byte_no][bit_no];
        else miso<=#200 1'b1;
        count_bits=count_bits+1;
        if(bit_no==7) begin bit_no=0; byte_no=byte_no+1; end
        else bit_no=bit_no+1;
    end
    always @(posedge cs_n) if(count_bits>0) begin
        if(sclk!==1 || count_bits%8!=0) $fatal(1,"CS released inside a byte");
        bytes=count_bits/8; transactions=transactions+1;
        if(rx[0]!==8'h01) $fatal(1,"Command address or LSB order incorrect");
        case(rx[1])
            8'h42: begin
                polls=polls+1;
                if(bytes!=5 && bytes!=9 && bytes!=21) $fatal(1,"Bad poll length");
                if(bytes==21 && reply[1]!=8'h79) $fatal(1,"Unexpected pressure poll");
                for(i=2;i<bytes;i=i+1)
                    if(rx[i]!==0) $fatal(1,"Unexpected rumble/command byte");
            end
            8'h43: begin
                if(bytes!=9) $fatal(1,"Bad config command length");
                if(rx[3]==1) begin
                    if(connected && frame_period_ok) config_mode=1;
                end
                else if(rx[3]==0) begin
                    if(connected && frame_period_ok) config_mode=0;
                    for(i=4;i<9;i=i+1) if(rx[i]!==8'h5a) $fatal(1,"Bad exit config bytes");
                end else $fatal(1,"Bad config flag");
            end
            8'h44: begin
                if((connected && frame_period_ok && !config_mode) || bytes!=9 || rx[3]!=1 || rx[4]!=8'hee)
                    $fatal(1,"Bad analog initialization sequence");
                if(connected && frame_period_ok) active_mode=8'h73;
            end
            default: $fatal(1,"Unknown PS2 command %h",rx[1]);
        endcase
        miso<=1;
    end
endmodule
