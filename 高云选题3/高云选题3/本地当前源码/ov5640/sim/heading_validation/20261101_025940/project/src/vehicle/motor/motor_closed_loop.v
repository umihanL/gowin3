// Real-time command input; no encoder-based startup cap or automatic drive hold.
module motor_closed_loop #(
    parameter integer CLK_HZ = 50000000,
    parameter integer SAMPLE_MS = 10,
    parameter integer COUNTS_PER_REV = 1560,
    parameter integer WHEEL_DIAMETER_MM = 66,
    parameter integer ENCODER_INVERT = 0,
    parameter integer ENCODER_FILTER_CYCLES = 250,
    parameter integer REVERSE_BLANK_CYCLES = 50000,
    parameter integer P_AVERAGE_LOG2 = 1,
    parameter integer SPEED_AVERAGE_LOG2 = 2,
    parameter [15:0] KP_Q8 = 16'd6144,
    parameter [15:0] KI_Q8 = 16'd128
)(
    input clk, input rst_n, input drive_enabled,input brake_request,
    input signed [15:0] speed_command, converted_speed,
    input signed [19:0] target_counts_q4,
    input encoder_A, encoder_B,
    output [1:0] dir, output pwm_out,
    output signed [15:0] actual_speed_mm_s,
    // Export the existing counter; do not instantiate a second decoder.
    output signed [31:0] encoder_position
);
    wire signed [31:0] position;
    assign encoder_position = position;
    wire signed [15:0] delta_count;
    wire sample_valid;
    wire signed [11:0] pwm_command;
    wire [1:0] drive_dir;
    wire drive_pwm;
    // Proportional short-brake PWM instead of full-brake bang/bang.
    // The speed PI keeps its state during controlled braking, and unwinds
    // through the ordinary negative speed error. No repeated cold starts.
    reg [10:0] brake_duty;
    reg [10:0] brake_counter;
    wire [15:0] abs_actual=actual_speed_mm_s[15]?-actual_speed_mm_s:actual_speed_mm_s;
    wire [15:0] abs_target=speed_command[15]?-speed_command:speed_command;
    wire [15:0] excess=abs_actual>abs_target+4?abs_actual-abs_target-4:16'd0;
    wire [10:0] desired_brake=(!drive_enabled || speed_command==0)?11'd2000:
        excess>=32?11'd2000:{excess[4:0],6'b0};
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin brake_duty<=0;brake_counter<=0;end
        else begin
            brake_counter<=brake_counter==1999?0:brake_counter+1'b1;
            if(!brake_request)brake_duty<=0;
            else if(!drive_enabled)brake_duty<=2000;
            else if(sample_valid)begin
                if(desired_brake>brake_duty+128)brake_duty<=brake_duty+128;
                else if(brake_duty>desired_brake+128)brake_duty<=brake_duty-128;
                else brake_duty<=desired_brake;
            end
        end
    end
    wire brake_output=rst_n && brake_request && brake_counter<brake_duty;
    wire braking=rst_n && brake_request && brake_duty!=0;
    wire motor_rst_n = rst_n && drive_enabled && (speed_command != 0);
    wire command_ready = converted_speed == speed_command;
    wire polarity_matches = pwm_command[11] == speed_command[15];
    wire output_enabled = motor_rst_n && command_ready && polarity_matches && !braking;
    // Capture feedback and target together. Give the PI arithmetic
    // four system clocks to settle before its one-cycle sample enable.
    // Inputs and all PI state then remain stable until the next 10 ms sample.
    (* syn_preserve=1 *) reg signed [19:0] pi_target_counts;
    (* syn_preserve=1 *) reg signed [15:0] pi_measured_counts;
    reg [3:0] pi_pending;
    always @(posedge clk or negedge motor_rst_n) begin
        if(!motor_rst_n) begin
            pi_target_counts<=0; pi_measured_counts<=0; pi_pending<=0;
        end else begin
            pi_pending<={pi_pending[2:0],1'b0};
            if(sample_valid && command_ready) begin
                pi_target_counts<=target_counts_q4;
                pi_measured_counts<=delta_count;
                pi_pending<=4'b0001;
            end
        end
    end
    motor_encoder #(.FILTER_CYCLES(ENCODER_FILTER_CYCLES),
        .INVERT_DIRECTION(ENCODER_INVERT)) u_encoder(
        .clk(clk),.rst_n(rst_n),.encoder_A(encoder_A),.encoder_B(encoder_B),.position(position));
    motor_speed #(.CLK_HZ(CLK_HZ),.SAMPLE_MS(SAMPLE_MS),.CALCULATE_RPM(0),
        .COUNTS_PER_REV(COUNTS_PER_REV),.WHEEL_DIAMETER_MM(WHEEL_DIAMETER_MM),
        .AVERAGE_LOG2(SPEED_AVERAGE_LOG2)) u_speed(
        .clk(clk),.rst_n(rst_n),.position(position),.delta_count(delta_count),
        .speed_mm_s(actual_speed_mm_s),.speed_rpm(),.sample_valid(sample_valid));
    motor_pi #(.KP_Q8(KP_Q8),.KI_Q8(KI_Q8),.STARTUP_KI_Q8(16'd512),
        .UNWIND_KI_Q8(16'd512),
        .SETTLE_SAMPLES((40+SAMPLE_MS-1)/SAMPLE_MS),
        .PROPORTIONAL_AVERAGE_LOG2(P_AVERAGE_LOG2)) u_pi(
        .clk(clk),.rst_n(motor_rst_n),.sample_valid(pi_pending[3]),
        .target_counts_q4(pi_target_counts),.measured_counts(pi_measured_counts),.pwm_command(pwm_command));
    speed_to_pwm #(.REVERSE_BLANK_CYCLES(REVERSE_BLANK_CYCLES)) u_drive(
        .clk(clk),.rst_n(motor_rst_n),.speed(pwm_command),.dir(drive_dir),.pwm_out(drive_pwm));
    // Brake phase: IN1=IN2=1/PWM=1. Coast phase: IN1=IN2=0/PWM=1.
    // PWM=0 would itself short-brake a TB6612 and defeat duty modulation.
    assign dir = brake_output ? 2'b11 : output_enabled ? drive_dir : 2'b00;
    assign pwm_out = braking || (output_enabled && drive_pwm);
endmodule
