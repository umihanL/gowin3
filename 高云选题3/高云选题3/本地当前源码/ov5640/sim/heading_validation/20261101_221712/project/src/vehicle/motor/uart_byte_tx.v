module uart_byte_tx #(
    parameter integer CLK_HZ = 50000000,
    parameter integer BAUD = 0
)(
    input clk,
    input rst_n,
    input [7:0] data_byte,
    input send_en,
    input [2:0] baud_set,
    output reg uart_tx,
    output reg tx_done,
    output reg uart_state
);
    reg [15:0] bit_timer, bit_reload;
    reg [3:0] bit_index;
    reg [9:0] frame;
    wire [15:0] selected_reload;
    generate
        if (BAUD > 0) begin : fixed_baud
            localparam [15:0] RELOAD = (CLK_HZ + BAUD / 2) / BAUD - 1;
            assign selected_reload = RELOAD;
        end else begin : selectable_baud
            reg [15:0] reload;
            assign selected_reload = reload;
            always @(*) begin
            case (baud_set)
                3'd1: reload = (CLK_HZ + 19200 / 2) / 19200 - 1;
                3'd2: reload = (CLK_HZ + 38400 / 2) / 38400 - 1;
                3'd3: reload = (CLK_HZ + 57600 / 2) / 57600 - 1;
                3'd4: reload = (CLK_HZ + 115200 / 2) / 115200 - 1;
                default: reload = (CLK_HZ + 9600 / 2) / 9600 - 1;
            endcase
            end
        end
    endgenerate

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            uart_tx <= 1'b1;
            tx_done <= 1'b0;
            uart_state <= 1'b0;
            bit_timer <= 0;
            bit_reload <= 0;
            bit_index <= 0;
            frame <= 10'h3ff;
        end else begin
            tx_done <= 1'b0;
            if (!uart_state) begin
                uart_tx <= 1'b1;
                if (send_en) begin
                    frame <= {1'b1, data_byte, 1'b0};
                    uart_tx <= 1'b0;
                    uart_state <= 1'b1;
                    bit_index <= 0;
                    bit_reload <= selected_reload;
                    bit_timer <= selected_reload;
                end
            end else if (bit_timer != 0) begin
                bit_timer <= bit_timer - 1'b1;
            end else if (bit_index == 9) begin
                uart_tx <= 1'b1;
                uart_state <= 1'b0;
                tx_done <= 1'b1;
            end else begin
                frame <= {1'b1, frame[9:1]};
                uart_tx <= frame[1];
                bit_index <= bit_index + 1'b1;
                bit_timer <= bit_reload;
            end
        end
    end
endmodule
