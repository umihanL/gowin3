module motor_encoder #(
    parameter integer FILTER_CYCLES = 250,
    parameter integer INVERT_DIRECTION = 0
)(
    input clk,
    input rst_n,
    input encoder_A,
    input encoder_B,
    output reg signed [31:0] position
);
    localparam integer FILTER_LIMIT = (FILTER_CYCLES < 1) ? 1 : FILTER_CYCLES;
    localparam integer FILTER_WIDTH = (FILTER_LIMIT < 2) ? 1 : $clog2(FILTER_LIMIT);
    reg [1:0] sync_A, sync_B;
    reg [2:0] sync_ready;
    reg [1:0] candidate_state, prev_state;
    reg [FILTER_WIDTH-1:0] stable_count;
    reg initialized;
    wire [1:0] curr_state = {sync_A[1], sync_B[1]};

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sync_A <= 2'b00;
            sync_B <= 2'b00;
            sync_ready <= 3'b000;
        end else begin
            sync_A <= {sync_A[0], encoder_A};
            sync_B <= {sync_B[0], encoder_B};
            sync_ready <= {sync_ready[1:0], 1'b1};
        end
    end

    // Accept a quadrature state only after it stays stable for 5 us by default.
    // The first stable state initializes the decoder without a fictitious edge.
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            position <= 32'sd0;
            candidate_state <= 2'b00;
            prev_state <= 2'b00;
            stable_count <= 0;
            initialized <= 1'b0;
        end else if (!sync_ready[2]) begin
            candidate_state <= curr_state;
            stable_count <= 0;
        end else if (curr_state != candidate_state) begin
            candidate_state <= curr_state;
            stable_count <= 0;
        end else if (stable_count != FILTER_LIMIT - 1) begin
            stable_count <= stable_count + 1'b1;
        end else begin
            initialized <= 1'b1;
            prev_state <= candidate_state;
            if (initialized) begin
                case ({prev_state, candidate_state})
                    4'b0001, 4'b0111, 4'b1110, 4'b1000:
                        position <= INVERT_DIRECTION ? position - 32'sd1 : position + 32'sd1;
                    4'b0010, 4'b0100, 4'b1101, 4'b1011:
                        position <= INVERT_DIRECTION ? position + 32'sd1 : position - 32'sd1;
                    default: position <= position;
                endcase
            end
        end
    end
endmodule
