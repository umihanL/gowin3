module motor_speed #(
    parameter integer CLK_HZ = 50000000,
    parameter integer SAMPLE_MS = 10,
    parameter integer CALCULATE_RPM = 1,
    parameter integer COUNTS_PER_REV = 1560,
    parameter integer WHEEL_DIAMETER_MM = 66,
    parameter integer AVERAGE_LOG2 = 0
)(
    input clk,
    input rst_n,
    input signed [31:0] position,
    output reg signed [15:0] delta_count,
    output reg signed [15:0] speed_mm_s,
    output reg signed [15:0] speed_rpm,
    output reg sample_valid
);
    localparam integer SAMPLE_CYCLES = (CLK_HZ / 1000) * SAMPLE_MS;
    localparam integer TIMER_WIDTH = (SAMPLE_CYCLES < 2) ? 1 : $clog2(SAMPLE_CYCLES);
    localparam integer AVG_SHIFT = (AVERAGE_LOG2 < 0) ? 0 : ((AVERAGE_LOG2 > 4) ? 4 : AVERAGE_LOG2);
    localparam integer AVG_SAMPLES = 1 << AVG_SHIFT;
    localparam integer COUNT_WIDTH = 16 + AVG_SHIFT;
    // All divisions below are elaboration-time constants, not hardware dividers.
    localparam [63:0] MM_NUM = 64'd65536 * WHEEL_DIAMETER_MM * 314159 * 1000;
    localparam [63:0] MM_DEN = 64'd100000 * COUNTS_PER_REV * SAMPLE_MS;
    localparam [31:0] MM_SCALE_Q16 = (MM_NUM + MM_DEN / 2) / MM_DEN;
    localparam [63:0] RPM_DEN = 64'd1 * COUNTS_PER_REV * SAMPLE_MS;
    localparam [31:0] RPM_SCALE_Q16 = (64'd65536 * 60000 + RPM_DEN / 2) / RPM_DEN;
    localparam [1:0] IDLE = 0, MM_MULTIPLY = 1, RPM_LOAD = 2, RPM_MULTIPLY = 3;

    reg [TIMER_WIDTH-1:0] timer;
    reg signed [31:0] previous_position;
    wire signed [31:0] difference = position - previous_position;
    wire signed [15:0] limited_difference = (difference > 32'sd32767) ? 16'sd32767 :
                                           (difference < -32'sd32768) ? -16'sd32768 : difference[15:0];
    wire take_sample = (state == IDLE) && (timer == SAMPLE_CYCLES - 1);
    wire signed [COUNT_WIDTH-1:0] count_total;
    generate
        if (AVG_SHIFT == 0) begin : unfiltered_speed
            assign count_total = limited_difference;
        end else begin : averaged_speed
            reg signed [15:0] history [0:AVG_SAMPLES-1];
            reg signed [COUNT_WIDTH-1:0] count_sum;
            reg initialized;
            integer i;
            wire signed [COUNT_WIDTH-1:0] newest = {{AVG_SHIFT{limited_difference[15]}}, limited_difference};
            wire signed [COUNT_WIDTH-1:0] oldest = {{AVG_SHIFT{history[AVG_SAMPLES-1][15]}}, history[AVG_SAMPLES-1]};
            // Average counts before unit conversion, preserving fractional counts.
            // Seed from the first sample instead of diluting it with reset zeros.
            assign count_total = initialized ? (count_sum - oldest + newest) : (newest <<< AVG_SHIFT);
            always @(posedge clk or negedge rst_n) begin
                if (!rst_n) begin
                    count_sum <= 0;
                    initialized <= 0;
                    for (i = 0; i < AVG_SAMPLES; i = i + 1) history[i] <= 0;
                end else if (take_sample) begin
                    count_sum <= count_total;
                    initialized <= 1;
                    history[0] <= limited_difference;
                    for (i = 1; i < AVG_SAMPLES; i = i + 1)
                        history[i] <= initialized ? history[i-1] : limited_difference;
                end
            end
        end
    endgenerate
    wire [COUNT_WIDTH-1:0] absolute_total = count_total[COUNT_WIDTH-1] ? -count_total : count_total;
    reg [1:0] state;
    reg negative;
    reg [COUNT_WIDTH-1:0] magnitude, multiplier;
    reg [47:0] accumulator, shifted_scale;
    reg [$clog2(COUNT_WIDTH)-1:0] bit_index;
    wire [47:0] next_accumulator = accumulator + (multiplier[0] ? shifted_scale : 48'd0);

    function [15:0] rounded_speed;
        input [47:0] product;
        input sign_bit;
        reg [48:0] rounded;
        begin
            rounded = ({1'b0, product} + (49'd32768 << AVG_SHIFT)) >> (16 + AVG_SHIFT);
            if (sign_bit)
                rounded_speed = (rounded >= 32768) ? 16'h8000 : -rounded[15:0];
            else
                rounded_speed = (rounded > 32767) ? 16'h7fff : rounded[15:0];
        end
    endfunction

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            timer <= 0;
            previous_position <= 0;
            delta_count <= 0;
            speed_mm_s <= 0;
            speed_rpm <= 0;
            sample_valid <= 0;
            state <= IDLE;
            negative <= 0;
            magnitude <= 0;
            multiplier <= 0;
            accumulator <= 0;
            shifted_scale <= 0;
            bit_index <= 0;
        end else begin
            sample_valid <= 0;
            if (timer == SAMPLE_CYCLES - 1) timer <= 0;
            else timer <= timer + 1'b1;
            case (state)
                IDLE: if (timer == SAMPLE_CYCLES - 1) begin
                    // A 32-bit modulo subtraction also works across position wrap.
                    previous_position <= position;
                    delta_count <= limited_difference;
                    negative <= count_total[COUNT_WIDTH-1];
                    magnitude <= absolute_total;
                    multiplier <= absolute_total;
                    accumulator <= 0;
                    shifted_scale <= {16'd0, MM_SCALE_Q16};
                    bit_index <= 0;
                    state <= MM_MULTIPLY;
                end
                MM_MULTIPLY, RPM_MULTIPLY: begin
                    accumulator <= next_accumulator;
                    shifted_scale <= shifted_scale << 1;
                    multiplier <= multiplier >> 1;
                    bit_index <= bit_index + 1'b1;
                    if (bit_index == COUNT_WIDTH - 1) begin
                        if (state == MM_MULTIPLY) begin
                            speed_mm_s <= rounded_speed(next_accumulator, negative);
                            if (CALCULATE_RPM) state <= RPM_LOAD;
                            else begin
                                // Speed-only telemetry lets synthesis remove RPM conversion.
                                sample_valid <= 1'b1;
                                state <= IDLE;
                            end
                        end else begin
                            speed_rpm <= rounded_speed(next_accumulator, negative);
                            sample_valid <= 1'b1;
                            state <= IDLE;
                        end
                    end
                end
                RPM_LOAD: begin
                    multiplier <= magnitude;
                    accumulator <= 0;
                    shifted_scale <= {16'd0, RPM_SCALE_Q16};
                    bit_index <= 0;
                    state <= RPM_MULTIPLY;
                end
                default: state <= IDLE;
            endcase
        end
    end
endmodule
