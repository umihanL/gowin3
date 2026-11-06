// FireWater: samples:target1_mm_s,actual1_mm_s,target2_mm_s,actual2_mm_s\r\n
module vofa_firewater_tx #(
    parameter integer CLK_HZ = 50000000,
    parameter integer BAUD = 115200
)(
    input clk,
    input rst_n,
    input send_valid,
    input signed [15:0] ch0,
    input signed [15:0] ch1,
    input signed [15:0] ch2,
    input signed [15:0] ch3,
    output tx,
    output busy
);
    localparam [4:0] IDLE = 0, PREFIX = 1, PREFIX_WAIT = 2, LOAD = 3,
                     CONVERT = 4, SIGN = 5, SIGN_WAIT = 6, DIGIT = 7,
                     DIGIT_WAIT = 8, SEPARATOR = 9, SEPARATOR_WAIT = 10,
                     LF_SEND = 11, LF_WAIT = 12;
    reg [4:0] state;
    reg signed [15:0] values [0:3];
    reg [1:0] channel;
    reg [3:0] prefix_index;
    reg [15:0] binary_value;
    reg negative;
    reg [19:0] bcd;
    reg [19:0] adjusted_bcd;
    wire [19:0] next_bcd = {adjusted_bcd[18:0], binary_value[15]};
    reg [3:0] convert_index;
    reg [2:0] digit_index;
    reg [3:0] current_digit;
    reg digits_started;
    reg [7:0] data_byte;
    reg send_en;
    wire tx_done, uart_busy;
    integer i;

    assign busy = (state != IDLE);

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
            channel <= 0;
            prefix_index <= 0;
            values[0] <= 0;
            values[1] <= 0;
            values[2] <= 0;
            values[3] <= 0;
            binary_value <= 0;
            negative <= 0;
            bcd <= 0;
            convert_index <= 0;
            digit_index <= 0;
            digits_started <= 0;
            data_byte <= 0;
            send_en <= 0;
        end else begin
            send_en <= 0;
            case (state)
                IDLE: if (send_valid) begin
                    values[0] <= ch0;
                    values[1] <= ch1;
                    values[2] <= ch2;
                    values[3] <= ch3;
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
                    binary_value <= values[channel][15] ? -values[channel] : values[channel];
                    negative <= values[channel][15];
                    bcd <= 0;
                    convert_index <= 0;
                    digit_index <= 4;
                    digits_started <= 0;
                    state <= CONVERT;
                end
                CONVERT: begin
                    bcd <= next_bcd;
                    binary_value <= binary_value << 1;
                    convert_index <= convert_index + 1'b1;
                    if (convert_index == 15)
                        state <= SIGN;
                end
                SIGN: if (!negative) state <= DIGIT;
                      else if (!uart_busy) begin
                          data_byte <= 8'h2d;
                          send_en <= 1;
                          state <= SIGN_WAIT;
                      end
                SIGN_WAIT: if (tx_done) state <= DIGIT;
                DIGIT: if ((current_digit == 0) && !digits_started && (digit_index != 0))
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
                    else begin
                        digit_index <= digit_index - 1'b1;
                        state <= DIGIT;
                    end
                end
                SEPARATOR: if (!uart_busy) begin
                    data_byte <= (channel == 3) ? 8'h0d : 8'h2c;
                    send_en <= 1;
                    state <= SEPARATOR_WAIT;
                end
                SEPARATOR_WAIT: if (tx_done) begin
                    if (channel == 3)
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
