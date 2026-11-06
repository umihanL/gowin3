// FireWater ASCII, original three inputs in degrees, one decimal place.
// EXTENDED_POSE adds Euler Roll/Pitch/Yaw and quaternion W/X/Y/Z (Q14 input,
// four decimal places). All ten fields belong to the same accepted sample.
// MOTOR_TELEMETRY appends target1/actual1/target2/actual2 in integer mm/s.
// It sends periodically even without IMU data; invalid angles become zero.
// Inputs are signed tenths of a degree, in the same clock domain as sample_valid.
// Take a complete fresh sample at most SEND_HZ times per second. Do not queue
// stale samples while transmitting; finish an active frame even if offline.
module imu_firewater_tx #(
    parameter integer CLK_HZ = 33000000,
    parameter integer BAUD = 115200,
    parameter integer SEND_HZ = 20,
    parameter integer EXTENDED_POSE = 0,
    parameter integer MOTOR_TELEMETRY = 0
)(
    input clk,
    input rst_n,
    input sample_valid,
    input available,
    input signed [15:0] roll,
    input signed [15:0] pitch,
    input signed [15:0] yaw,
    input signed [15:0] euler_roll, euler_pitch, euler_yaw,
    input signed [17:0] q_w, q_x, q_y, q_z,
    input signed [15:0] motor_target1,motor_actual1,motor_target2,motor_actual2,
    output tx,
    output busy
);
    localparam integer PERIOD = (CLK_HZ / SEND_HZ > 0) ? CLK_HZ / SEND_HZ : 1;
    localparam integer TIMER_BITS = (PERIOD > 1) ? $clog2(PERIOD) : 1;
    localparam [TIMER_BITS-1:0] PERIOD_LAST = PERIOD - 1;
    localparam integer MOTOR_FIRST = EXTENDED_POSE ? 10 : 3;
    localparam [3:0] LAST_CHANNEL = MOTOR_TELEMETRY ? MOTOR_FIRST+3 : (EXTENDED_POSE ? 9 : 2);
    localparam [3:0] IDLE = 0, PREFIX = 1, PREFIX_WAIT = 2, LOAD = 3,
                     CONVERT = 4, SIGN = 5, SIGN_WAIT = 6, DIGIT = 7,
                     DIGIT_WAIT = 8, DOT = 9, DOT_WAIT = 10,
                     SEPARATOR = 11, SEPARATOR_WAIT = 12,
                     LF_SEND = 13, LF_WAIT = 14;
    reg [3:0] state;
    reg [TIMER_BITS-1:0] period_count;
    reg signed [17:0] values [0:13];
    reg [3:0] channel;
    reg [2:0] prefix_index;
    reg [16:0] binary_value;
    reg negative;
    reg [19:0] bcd, adjusted_bcd;
    wire [19:0] next_bcd = {adjusted_bcd[18:0], binary_value[16]};
    reg [4:0] convert_index;
    reg [2:0] digit_index;
    reg [2:0] fraction_digits;
    reg [3:0] current_digit;
    reg digits_started;
    reg [7:0] data_byte;
    reg send_en;
    wire tx_done, uart_busy;
    wire accept_sample = (state == IDLE) && (MOTOR_TELEMETRY || (available && sample_valid)) &&
                         (period_count == PERIOD_LAST);
    wire quaternion_channel = EXTENDED_POSE && channel>=6 && channel<=9;
    wire motor_channel = MOTOR_TELEMETRY && channel>=MOTOR_FIRST;
    integer i,j;
    wire [17:0] magnitude = values[channel][17] ? -values[channel] : values[channel];
    wire [31:0] wide_magnitude = {14'd0,magnitude};
    // Round Q14 to units of 0.0001 using shifts/adds. No new multiplier/DSP.
    wire [31:0] quaternion_scaled = (wide_magnitude << 13) +
        (wide_magnitude << 10) + (wide_magnitude << 9) +
        (wide_magnitude << 8) + (wide_magnitude << 4) + 32'd8192;

    assign busy = (state != IDLE);

    // Seventeen-cycle binary-to-BCD conversion, without a divider or DSP.
    always @(*) begin
        adjusted_bcd = bcd;
        for (i = 0; i < 5; i = i + 1)
            if (bcd[i*4 +: 4] >= 5)
                adjusted_bcd[i*4 +: 4] = bcd[i*4 +: 4] + 4'd3;
        case (digit_index)
            3'd4: current_digit = bcd[19:16];
            3'd3: current_digit = bcd[15:12];
            3'd2: current_digit = bcd[11:8];
            3'd1: current_digit = bcd[7:4];
            default: current_digit = bcd[3:0];
        endcase
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= IDLE;
            // Let the existing motor target/feedback pipelines settle before
            // the first periodic frame after reset.
            period_count <= MOTOR_TELEMETRY ? 0 : PERIOD_LAST;
            for(j=0;j<14;j=j+1) values[j] <= 0;
            channel <= 0;
            prefix_index <= 0;
            binary_value <= 0;
            negative <= 0;
            bcd <= 0;
            convert_index <= 0;
            digit_index <= 0;
            fraction_digits <= 1;
            digits_started <= 0;
            data_byte <= 0;
            send_en <= 0;
        end else begin
            send_en <= 0;
            if (accept_sample)
                period_count <= 0;
            else if (period_count != PERIOD_LAST)
                period_count <= period_count + 1'b1;
            case (state)
                IDLE: if (accept_sample) begin
                    values[0] <= available ? {{2{roll[15]}},roll} : 18'sd0;
                    values[1] <= available ? {{2{pitch[15]}},pitch} : 18'sd0;
                    values[2] <= available ? {{2{yaw[15]}},yaw} : 18'sd0;
                    if(EXTENDED_POSE) begin
                        values[3] <= available ? {{2{euler_roll[15]}},euler_roll} : 18'sd0;
                        values[4] <= available ? {{2{euler_pitch[15]}},euler_pitch} : 18'sd0;
                        values[5] <= available ? {{2{euler_yaw[15]}},euler_yaw} : 18'sd0;
                        values[6] <= available ? q_w : 18'sd16384;
                        values[7] <= available ? q_x : 18'sd0;
                        values[8] <= available ? q_y : 18'sd0;
                        values[9] <= available ? q_z : 18'sd0;
                    end
                    if(MOTOR_TELEMETRY) begin
                        values[MOTOR_FIRST] <= {{2{motor_target1[15]}},motor_target1};
                        values[MOTOR_FIRST+1] <= {{2{motor_actual1[15]}},motor_actual1};
                        values[MOTOR_FIRST+2] <= {{2{motor_target2[15]}},motor_target2};
                        values[MOTOR_FIRST+3] <= {{2{motor_actual2[15]}},motor_actual2};
                    end
                    channel <= 0;
                    prefix_index <= 0;
                    state <= PREFIX;
                end
                PREFIX: if (!uart_busy) begin
                    case (prefix_index)
                        0: data_byte <= "s";
                        1: data_byte <= "a";
                        2: data_byte <= "m";
                        3: data_byte <= "p";
                        4: data_byte <= "l";
                        5: data_byte <= "e";
                        6: data_byte <= "s";
                        default: data_byte <= ":";
                    endcase
                    send_en <= 1;
                    state <= PREFIX_WAIT;
                end
                PREFIX_WAIT: if (tx_done) begin
                    if (prefix_index == 7)
                        state <= LOAD;
                    else begin
                        prefix_index <= prefix_index + 1'b1;
                        state <= PREFIX;
                    end
                end
                LOAD: begin
                    // Angles are tenths; quaternion fields use four decimals.
                    binary_value <= quaternion_channel ?
                                    quaternion_scaled[30:14] : magnitude[16:0];
                    negative <= values[channel][17];
                    bcd <= 0;
                    convert_index <= 0;
                    digit_index <= 4;
                    fraction_digits <= quaternion_channel ? 4 : (motor_channel ? 0 : 1);
                    digits_started <= 0;
                    state <= CONVERT;
                end
                CONVERT: begin
                    bcd <= next_bcd;
                    binary_value <= binary_value << 1;
                    convert_index <= convert_index + 1'b1;
                    if (convert_index == 16)
                        state <= SIGN;
                end
                SIGN: if (!negative) state <= DIGIT;
                      else if (!uart_busy) begin
                          data_byte <= "-";
                          send_en <= 1;
                          state <= SIGN_WAIT;
                      end
                SIGN_WAIT: if (tx_done) state <= DIGIT;
                DIGIT: if ((current_digit == 0) && !digits_started && (digit_index > fraction_digits))
                           digit_index <= digit_index - 1'b1;
                       else if (!uart_busy) begin
                           data_byte <= 8'd48 + current_digit;
                           send_en <= 1;
                           digits_started <= 1;
                           state <= DIGIT_WAIT;
                       end
                DIGIT_WAIT: if (tx_done) begin
                    if (digit_index == 0)
                        state <= SEPARATOR;
                    else if (digit_index == fraction_digits)
                        state <= DOT;
                    else begin
                        digit_index <= digit_index - 1'b1;
                        state <= DIGIT;
                    end
                end
                DOT: if (!uart_busy) begin
                    data_byte <= ".";
                    send_en <= 1;
                    state <= DOT_WAIT;
                end
                DOT_WAIT: if (tx_done) begin
                    digit_index <= fraction_digits - 1'b1;
                    state <= DIGIT;
                end
                SEPARATOR: if (!uart_busy) begin
                    data_byte <= (channel == LAST_CHANNEL) ? 8'h0d : 8'h2c;
                    send_en <= 1;
                    state <= SEPARATOR_WAIT;
                end
                SEPARATOR_WAIT: if (tx_done) begin
                    if (channel == LAST_CHANNEL)
                        state <= LF_SEND;
                    else begin
                        channel <= channel + 1'b1;
                        state <= LOAD;
                    end
                end
                LF_SEND: if (!uart_busy) begin
                    data_byte <= 8'h0a;
                    send_en <= 1;
                    state <= LF_WAIT;
                end
                LF_WAIT: if (tx_done) state <= IDLE;
                default: state <= IDLE;
            endcase
        end
    end

    uart_byte_tx #(.CLK_HZ(CLK_HZ), .BAUD(BAUD)) u_uart(
        .clk(clk), .rst_n(rst_n), .data_byte(data_byte), .send_en(send_en),
        .baud_set(3'd4), .uart_tx(tx), .tx_done(tx_done), .uart_state(uart_busy)
    );
endmodule
