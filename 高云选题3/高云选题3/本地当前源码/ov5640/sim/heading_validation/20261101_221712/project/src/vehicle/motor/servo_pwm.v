// 180-degree positional servo. Change commands only at 20 ms frame boundaries.
module servo_pwm #(
    parameter integer CLK_HZ = 50000000,
    parameter integer FRAME_US = 20000,
    parameter integer MIN_PULSE_US = 500,
    parameter integer MAX_PULSE_US = 2500,
    parameter integer CENTER_DEG = 90,
    parameter integer MIN_ANGLE_DEG = 0,
    parameter integer MAX_ANGLE_DEG = 180
)(
    input clk, input rst_n,
    input [10:0] angle_tenths,
    output pwm_out
);
    localparam integer CLOCKS_PER_US = CLK_HZ / 1000000;
    localparam integer FRAME_CLOCKS = CLOCKS_PER_US * FRAME_US;
    localparam integer COUNTER_W = $clog2(FRAME_CLOCKS);
    localparam integer CENTER_CLOCKS = CLOCKS_PER_US *
        (MIN_PULSE_US + ((MAX_PULSE_US-MIN_PULSE_US)*CENTER_DEG + 90)/180);
    reg [COUNTER_W-1:0] frame_count, active_width, requested_width;
    reg [11:0] pulse_us;
    wire [10:0] bounded_angle = angle_tenths > MAX_ANGLE_DEG*10 ? MAX_ANGLE_DEG*10 :
        angle_tenths < MIN_ANGLE_DEG*10 ? MIN_ANGLE_DEG*10 : angle_tenths;
    reg [11:0] pulse_rom [0:2047] /* synthesis syn_romstyle="block_rom" */;
    integer i;
    function [COUNTER_W-1:0] pulse_clocks;
        input [11:0] value;
        integer bit_number;
        reg [COUNTER_W-1:0] extended_value;
        begin
            extended_value=value;
            pulse_clocks=0;
            for(bit_number=0;bit_number<32;bit_number=bit_number+1)
                if((CLOCKS_PER_US >> bit_number) & 1)
                    pulse_clocks=pulse_clocks+(extended_value << bit_number);
        end
    endfunction
    initial begin
        // Gowin's elaborator limits each loop to 2000 iterations.
        for(i=0;i<1024;i=i+1)
            pulse_rom[i] = MIN_PULSE_US +
                ((MAX_PULSE_US-MIN_PULSE_US)*i + 900)/1800;
        for(i=1024;i<2048;i=i+1)
            pulse_rom[i] = MIN_PULSE_US +
                ((MAX_PULSE_US-MIN_PULSE_US)*((i>1800)?1800:i) + 900)/1800;
    end
    always @(posedge clk) pulse_us <= pulse_rom[bounded_angle];
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            frame_count <= 0; active_width <= CENTER_CLOCKS;
            requested_width <= CENTER_CLOCKS;
        end else begin
            requested_width <= pulse_clocks(pulse_us);
            if(frame_count == FRAME_CLOCKS-1) begin
                frame_count <= 0;
                active_width <= requested_width;
            end else frame_count <= frame_count+1'b1;
        end
    end
    assign pwm_out = rst_n && (frame_count < active_width);
endmodule
