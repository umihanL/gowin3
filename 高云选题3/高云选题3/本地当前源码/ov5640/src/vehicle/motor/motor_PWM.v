// 50 MHz / 2000 = 25 kHz. A duty of 2000 stays high continuously.
module motor_PWM(
    input clk,
    input rst_n,
    input [10:0] duty_cycle,
    output reg pwm_out
);
    reg [10:0] cnt_pwm;
    wire [10:0] limited_duty = (duty_cycle > 11'd2000) ? 11'd2000 : duty_cycle;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cnt_pwm <= 11'd0;
            pwm_out <= 1'b0;
        end else begin
            cnt_pwm <= (cnt_pwm == 11'd1999) ? 11'd0 : cnt_pwm + 1'b1;
            pwm_out <= (cnt_pwm < limited_duty);
        end
    end
endmodule
