module motor_pi #(
    parameter [15:0] KP_Q8 = 16'd2048,
    parameter [15:0] KI_Q8 = 16'd128,
    parameter [15:0] STARTUP_KI_Q8 = KI_Q8,
    parameter [15:0] UNWIND_KI_Q8 = KI_Q8,
    parameter integer SETTLE_SAMPLES = 4,
    parameter integer PROPORTIONAL_AVERAGE_LOG2 = 0
)(
    input clk,
    input rst_n,
    input sample_valid,
    input signed [19:0] target_counts_q4,
    input signed [15:0] measured_counts,
    output reg signed [11:0] pwm_command
);
    // Error has four fractional bits; gains have eight; accumulator has twelve.
    localparam signed [37:0] LIMIT_Q12 = 38'sd8192000; // 2000 << 12
    localparam integer SETTLE_LIMIT = (SETTLE_SAMPLES < 1) ? 1 : SETTLE_SAMPLES;
    localparam integer SETTLE_WIDTH = (SETTLE_LIMIT < 2) ? 1 : $clog2(SETTLE_LIMIT);
    localparam integer P_AVG_SHIFT = (PROPORTIONAL_AVERAGE_LOG2 < 0) ? 0 :
        ((PROPORTIONAL_AVERAGE_LOG2 > 4) ? 4 : PROPORTIONAL_AVERAGE_LOG2);
    localparam integer P_AVG_SAMPLES = 1 << P_AVG_SHIFT;
    localparam integer P_COUNT_WIDTH = 16 + P_AVG_SHIFT;
    reg signed [23:0] integral_q12;
    reg previous_target_negative;
    reg control_settled;
    reg signed [19:0] previous_target;
    wire signed [20:0] target_change = $signed(target_counts_q4)-$signed(previous_target);
    wire [20:0] target_step = target_change[20] ? -target_change : target_change;
    wire large_target_change = target_step > settle_band_q4;
    reg [SETTLE_WIDTH-1:0] settle_count;
    wire forward_feedback = target_counts_q4[19] ? (measured_counts < 0) : (measured_counts > 0);
    // Encoder movement is feedback, not permission to start or continue driving.
    // A persistent speed error may build PWM all the way to the hardware limit.
    wire signed [19:0] measured_q4 = {measured_counts, 4'b0000};
    wire signed [20:0] error_q4 = {target_counts_q4[19], target_counts_q4} -
                                {measured_q4[19], measured_q4};
    wire [20:0] target_magnitude_q4 = target_counts_q4[19] ?
        -{target_counts_q4[19], target_counts_q4} : {1'b0, target_counts_q4};
    wire [20:0] error_magnitude_q4 = error_q4[20] ? -error_q4 : error_q4;
    wire [20:0] settle_band_q4 = ((target_magnitude_q4 >> 3) < 21'd16) ?
        21'd16 : (target_magnitude_q4 >> 3);
    wire near_target = forward_feedback && (error_magnitude_q4 <= settle_band_q4);
    wire signed [P_COUNT_WIDTH-1:0] proportional_count_total;
    generate
        if (P_AVG_SHIFT == 0) begin : unfiltered_proportional
            assign proportional_count_total = measured_counts;
        end else begin : averaged_proportional
            reg signed [15:0] history [0:P_AVG_SAMPLES-1];
            reg signed [P_COUNT_WIDTH-1:0] count_sum;
            reg initialized;
            integer i;
            wire signed [P_COUNT_WIDTH-1:0] newest = {{P_AVG_SHIFT{measured_counts[15]}}, measured_counts};
            wire signed [P_COUNT_WIDTH-1:0] oldest = {{P_AVG_SHIFT{history[P_AVG_SAMPLES-1][15]}}, history[P_AVG_SAMPLES-1]};
            assign proportional_count_total = initialized ? (count_sum - oldest + newest) : (newest <<< P_AVG_SHIFT);
            always @(posedge clk or negedge rst_n) begin
                if (!rst_n) begin
                    initialized <= 0;
                    count_sum <= 0;
                    for (i = 0; i < P_AVG_SAMPLES; i = i + 1) history[i] <= 0;
                end else if (sample_valid) begin
                    if ((target_counts_q4 == 0) || (target_counts_q4[19] != previous_target_negative)) begin
                        initialized <= 0;
                        count_sum <= 0;
                        for (i = 0; i < P_AVG_SAMPLES; i = i + 1) history[i] <= 0;
                    end else begin
                        initialized <= 1;
                        count_sum <= proportional_count_total;
                        history[0] <= measured_counts;
                        for (i = 1; i < P_AVG_SAMPLES; i = i + 1)
                            history[i] <= initialized ? history[i-1] : measured_counts;
                    end
                end
            end
        end
    endgenerate

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            control_settled <= 0;
            settle_count <= 0;
            previous_target <= 0;
        end else if (sample_valid) begin
            previous_target <= target_counts_q4;
            if ((target_counts_q4 == 0) || (target_counts_q4[19] != previous_target_negative) || large_target_change) begin
                control_settled <= 0;
                settle_count <= 0;
            end else begin
                if (!control_settled) begin
                    if (near_target) begin
                        if (settle_count == SETTLE_LIMIT - 1)
                            control_settled <= 1;
                        else
                            settle_count <= settle_count + 1'b1;
                    end else settle_count <= 0;
                end
            end
        end
    end

    // Constant coefficients become only the selected shift/add terms.
    function signed [36:0] constant_gain;
        input signed [20:0] value;
        input [15:0] gain;
        reg signed [36:0] extended_value;
        integer i;
        begin
            extended_value = {{16{value[20]}}, value};
            constant_gain = 37'sd0;
            for (i = 0; i < 16; i = i + 1)
                if (gain[i])
                    constant_gain = constant_gain + (extended_value <<< i);
        end
    endfunction

    // Keep the ordinary gain near target; asymmetric gains on encoder
    // quantization alone would shift the average regulated speed.
    wire reducing_drive = (error_magnitude_q4 > settle_band_q4) &&
        (target_counts_q4[19] ? (error_q4 > 0) : (error_q4 < 0));
    wire signed [19:0] proportional_total_q4 =
        {{(20-P_COUNT_WIDTH){proportional_count_total[P_COUNT_WIDTH-1]}}, proportional_count_total} <<< (4-P_AVG_SHIFT);
    // Smooth small moving-speed corrections, but respond to raw stop/reverse
    // feedback and substantial overspeed without waiting for the average.
    wire signed [19:0] proportional_measured_q4 = (!forward_feedback || reducing_drive) ?
        measured_q4 : proportional_total_q4;
    wire signed [20:0] proportional_error_q4 = {target_counts_q4[19], target_counts_q4} -
        {proportional_measured_q4[19], proportional_measured_q4};
    wire signed [36:0] proportional_q12 = constant_gain(proportional_error_q4, KP_Q8);
    // Each gain is a compile-time constant, so all three paths use shifts/adds.
    // Build drive faster after reset, integrate gently during normal running,
    // and remove surplus integral faster when measured speed exceeds target.
    wire signed [36:0] normal_step_q12 = constant_gain(error_q4, KI_Q8);
    wire signed [36:0] startup_step_q12 = constant_gain(error_q4, STARTUP_KI_Q8);
    wire signed [36:0] unwind_step_q12 = constant_gain(error_q4, UNWIND_KI_Q8);
    wire signed [36:0] integral_step_q12 = reducing_drive ? unwind_step_q12 :
                                         ((control_settled && !large_target_change) ? normal_step_q12 : startup_step_q12);
    // Keep integrating a nonzero speed error even when no encoder edges arrive.
    wire signed [37:0] stored_integral = {{14{integral_q12[23]}}, integral_q12};
    wire signed [37:0] integral_candidate = stored_integral +
                                           {integral_step_q12[36], integral_step_q12};
    wire signed [37:0] limited_integral = (integral_candidate > LIMIT_Q12) ? LIMIT_Q12 :
                                        (integral_candidate < -LIMIT_Q12) ? -LIMIT_Q12 : integral_candidate;
    wire signed [37:0] output_before_integration = {proportional_q12[36], proportional_q12} +
                                                  stored_integral;
    wire signed [37:0] upper_limit = target_counts_q4[19] ? 38'sd0 : LIMIT_Q12;
    wire signed [37:0] lower_limit = target_counts_q4[19] ? -LIMIT_Q12 : 38'sd0;
    // Test the previous integral: a step crossing a limit must still reach that
    // limit, rather than freezing just short of zero or full duty.
    wire reject_integral = ((output_before_integration >= upper_limit) && (error_q4 > 0)) ||
                           ((output_before_integration <= lower_limit) && (error_q4 < 0));
    wire signed [37:0] accepted_integral = reject_integral ? stored_integral : limited_integral;
    wire signed [37:0] output_q12 = {proportional_q12[36], proportional_q12} + accepted_integral;
    wire signed [37:0] limited_output = (output_q12 > upper_limit) ? upper_limit :
                                       (output_q12 < lower_limit) ? lower_limit : output_q12;
    wire [37:0] output_magnitude = limited_output[37] ? -limited_output : limited_output;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            integral_q12 <= 0;
            pwm_command <= 0;
            previous_target_negative <= 0;
        end else if (sample_valid) begin
            previous_target_negative <= target_counts_q4[19];
            if ((target_counts_q4 == 0) || (target_counts_q4[19] != previous_target_negative)) begin
                integral_q12 <= 0;
                pwm_command <= 0;
            end else begin
                integral_q12 <= accepted_integral[23:0];
                // Divide by 4096, rounding toward zero for symmetric directions.
                if (limited_output[37])
                    pwm_command <= -output_magnitude[23:12];
                else
                    pwm_command <= output_magnitude[23:12];
            end
        end
    end
endmodule
