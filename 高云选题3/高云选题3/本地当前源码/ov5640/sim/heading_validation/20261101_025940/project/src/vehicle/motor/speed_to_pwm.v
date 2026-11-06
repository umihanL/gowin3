module speed_to_pwm #(
    parameter integer REVERSE_BLANK_CYCLES = 50000
)(
    input clk,
    input rst_n,
    input signed [11:0] speed,
    output reg [1:0] dir,
    output pwm_out
);
    localparam integer BLANK_WIDTH = (REVERSE_BLANK_CYCLES < 2) ? 1 :
                                     $clog2(REVERSE_BLANK_CYCLES + 1);
    reg [BLANK_WIDTH-1:0] blank_count;
    reg last_direction;
    reg [10:0] duty_cycle;
    wire raw_pwm;
    // Extend before negation: abs(-2048) must not wrap to zero.
    wire signed [12:0] extended_speed = {speed[11], speed};
    wire [12:0] magnitude = speed[11] ? -extended_speed : extended_speed;
    wire [10:0] limited_duty = (magnitude > 13'd2000) ? 11'd2000 : magnitude[10:0];

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            dir <= 2'b00;
            duty_cycle <= 11'd0;
            blank_count <= 0;
            last_direction <= 1'b0;
        end else if (speed == 0) begin
            dir <= 2'b00;
            duty_cycle <= 11'd0;
            if (blank_count != 0)
                blank_count <= blank_count - 1'b1;
        end else if (blank_count != 0) begin
            dir <= 2'b00;
            duty_cycle <= 11'd0;
            blank_count <= blank_count - 1'b1;
            if (blank_count == 1)
                last_direction <= speed[11];
        end else if ((speed[11] != last_direction) && (REVERSE_BLANK_CYCLES > 0)) begin
            dir <= 2'b00;
            duty_cycle <= 11'd0;
            blank_count <= REVERSE_BLANK_CYCLES;
        end else begin
            last_direction <= speed[11];
            dir <= speed[11] ? 2'b10 : 2'b01;
            duty_cycle <= limited_duty;
        end
    end

    assign pwm_out = rst_n && (dir != 2'b00) && raw_pwm;
    motor_PWM u_motor_pwm(
        .clk(clk), .rst_n(rst_n), .duty_cycle(duty_cycle), .pwm_out(raw_pwm)
    );
endmodule
