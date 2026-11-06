// Single-master, open-drain I2C with 16-bit register addresses.
// One/two byte transfers are MSB first. Commands are latched on start;
// done pulses after STOP, and ack_error covers NACK or bus timeout.
module vl53l1x_i2c_master #(
    parameter integer CLK_HZ = 50000000,
    parameter integer I2C_HZ = 100000,
    parameter integer TIMEOUT_MS = 10
)(
    input clk,
    input rst_n,
    input start,
    input [6:0] dev_addr,
    input [15:0] reg_addr,
    input reg_addr_8bit,
    input rw_read,
    input [1:0] write_len,
    input [1:0] read_len,
    input [15:0] write_data,
    output reg [15:0] read_data,
    output reg busy,
    output reg done,
    output reg ack_error,
    output reg [7:0] error_code,
    inout sda,
    inout scl
);
    localparam integer Q_RAW = (CLK_HZ + 4*I2C_HZ - 1) / (4*I2C_HZ);
    localparam integer QUARTER = (Q_RAW < 4) ? 4 : Q_RAW;
    localparam integer DIV_W = $clog2(2*QUARTER);
    localparam integer TIMEOUT_RAW = (CLK_HZ/1000)*TIMEOUT_MS;
    localparam integer TIMEOUT_CYCLES = (TIMEOUT_RAW < 1) ? 1 : TIMEOUT_RAW;
    localparam integer TIMEOUT_W = (TIMEOUT_CYCLES < 2) ? 1 : $clog2(TIMEOUT_CYCLES);

    localparam [4:0] IDLE = 0, BUS_WAIT = 1, START_SETUP = 2,
        START_HOLD = 3, BIT_FALL = 4, BIT_SETUP = 5, BIT_RISE = 6,
        BIT_HIGH_WAIT = 7, BIT_SAMPLE = 8, RESTART_FALL = 9,
        RESTART_RELEASE = 10, RESTART_RISE = 11, RESTART_HIGH_WAIT = 12,
        RESTART_START = 13, STOP_FALL = 14, STOP_SET = 15,
        STOP_RISE = 16, STOP_HIGH_WAIT = 17, STOP_RELEASE = 18, STOP_FREE = 19;
    localparam [1:0] TX_BIT = 0, SLAVE_ACK = 1, RX_BIT = 2, MASTER_ACK = 3;

    reg [4:0] state;
    reg [1:0] symbol;
    reg [DIV_W-1:0] div_count;
    reg [TIMEOUT_W-1:0] timeout_count;
    reg [6:0] address_reg;
    reg [15:0] register_reg, write_reg;
    reg read_reg, two_write, two_read, short_register;
    reg [2:0] byte_index, bit_index;
    reg read_index, read_address;
    reg [7:0] tx_byte, read_shift;
    reg sda_low, scl_low;
    reg [1:0] sda_sync, scl_sync;

    assign sda = sda_low ? 1'b0 : 1'bz;
    assign scl = scl_low ? 1'b0 : 1'bz;

    // SCL readback supports clock stretching. Sample SDA only after SCL
    // has actually risen and remained high for a quarter cycle.
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sda_sync <= 2'b11;
            scl_sync <= 2'b11;
        end else begin
            sda_sync <= {sda_sync[0], sda};
            scl_sync <= {scl_sync[0], scl};
        end
    end

    always @(*) begin
        if (read_address)
            tx_byte = {address_reg, 1'b1};
        else case (byte_index)
            3'd0: tx_byte = {address_reg, 1'b0};
            3'd1: tx_byte = register_reg[15:8];
            3'd2: tx_byte = register_reg[7:0];
            3'd3: tx_byte = two_write ? write_reg[15:8] : write_reg[7:0];
            default: tx_byte = write_reg[7:0];
        endcase
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= IDLE;
            symbol <= TX_BIT;
            div_count <= 0;
            timeout_count <= 0;
            address_reg <= 0;
            register_reg <= 0;
            write_reg <= 0;
            read_reg <= 0;
            two_write <= 0;
            two_read <= 0;
            short_register <= 0;
            byte_index <= 0;
            bit_index <= 7;
            read_index <= 0;
            read_address <= 0;
            read_shift <= 0;
            read_data <= 0;
            sda_low <= 0;
            scl_low <= 0;
            busy <= 0;
            done <= 0;
            ack_error <= 0;
            error_code <= 0;
        end else begin
            done <= 0;
            if (!busy) begin
                sda_low <= 0;
                scl_low <= 0;
                timeout_count <= 0;
                if (start) begin
                    address_reg <= dev_addr;
                    register_reg <= reg_addr;
                    write_reg <= write_data;
                    read_reg <= rw_read;
                    two_write <= (write_len == 2);
                    two_read <= (read_len == 2);
                    short_register <= reg_addr_8bit;
                    byte_index <= 0;
                    bit_index <= 7;
                    read_index <= 0;
                    read_address <= 0;
                    read_shift <= 0;
                    read_data <= 0;
                    symbol <= TX_BIT;
                    ack_error <= 0;
                    error_code <= 0;
                    busy <= 1;
                    div_count <= 0;
                    state <= BUS_WAIT;
                end
            end else if (timeout_count == TIMEOUT_CYCLES-1) begin
                // Release a stuck bus; the array controller can reset devices.
                sda_low <= 0;
                scl_low <= 0;
                ack_error <= 1;
                // Bus idle timeout distinguishes SDA, SCL and both stuck low.
                // 0A means SCL did not rise during a transaction/STOP.
                if (state == BUS_WAIT)
                    error_code <= !sda_sync[1] && !scl_sync[1] ? 8'h09 :
                                  !scl_sync[1] ? 8'h08 : 8'h07;
                else error_code <= 8'h0A;
                busy <= 0;
                done <= 1;
                state <= IDLE;
            end else begin
                timeout_count <= timeout_count + 1'b1;
                if (state == BUS_WAIT) begin
                    if (sda_sync[1] && scl_sync[1]) begin
                        div_count <= 2*QUARTER-1;
                        state <= START_SETUP;
                    end
                end else if (state == BIT_HIGH_WAIT ||
                             state == RESTART_HIGH_WAIT ||
                             state == STOP_HIGH_WAIT) begin
                    if (scl_sync[1]) begin
                        div_count <= (state == BIT_HIGH_WAIT) ?
                                     QUARTER-1 : 2*QUARTER-1;
                        if (state == BIT_HIGH_WAIT) state <= BIT_SAMPLE;
                        else if (state == RESTART_HIGH_WAIT) state <= RESTART_START;
                        else state <= STOP_RELEASE;
                    end
                end else if (div_count != 0)
                    div_count <= div_count - 1'b1;
                else begin
                    div_count <= QUARTER-1;
                    case (state)
                        START_SETUP: begin
                            sda_low <= 1;
                            div_count <= 2*QUARTER-1;
                            state <= START_HOLD;
                        end
                        START_HOLD: state <= BIT_FALL;
                        BIT_FALL: begin
                            scl_low <= 1;
                            state <= BIT_SETUP;
                        end
                        BIT_SETUP: begin
                            // Change SDA only after SCL has fallen.
                            if (symbol == TX_BIT) sda_low <= !tx_byte[bit_index];
                            else if (symbol == MASTER_ACK)
                                sda_low <= two_read && !read_index;
                            else sda_low <= 0;
                            state <= BIT_RISE;
                        end
                        BIT_RISE: begin
                            scl_low <= 0;
                            state <= BIT_HIGH_WAIT;
                        end
                        BIT_SAMPLE: begin
                            state <= BIT_FALL; // Another quarter of high hold.
                            case (symbol)
                                TX_BIT: begin
                                    if (bit_index == 0) symbol <= SLAVE_ACK;
                                    else bit_index <= bit_index - 1'b1;
                                end
                                SLAVE_ACK: begin
                                    bit_index <= 7;
                                    if (sda_sync[1]) begin
                                        ack_error <= 1;
                                        error_code <= read_address ? 8'h06 :
                                                      {5'b0, byte_index} + 8'd1;
                                        state <= STOP_FALL;
                                    end else if (read_address) begin
                                        symbol <= RX_BIT;
                                    end else if (read_reg && byte_index == 2) begin
                                        symbol <= TX_BIT;
                                        read_address <= 1;
                                        state <= RESTART_FALL;
                                    end else if (!read_reg &&
                                        byte_index == (two_write ? 4 : 3)) begin
                                        state <= STOP_FALL;
                                    end else begin
                                        // Optional one-byte register index for
                                        // the read-only VL53L0X ID probe.
                                        byte_index <= (short_register && byte_index == 0) ?
                                                      3'd2 : byte_index + 1'b1;
                                        symbol <= TX_BIT;
                                    end
                                end
                                RX_BIT: begin
                                    read_shift <= {read_shift[6:0], sda_sync[1]};
                                    if (bit_index == 0) begin
                                        if (two_read && !read_index)
                                            read_data[15:8] <= {read_shift[6:0], sda_sync[1]};
                                        else read_data[7:0] <= {read_shift[6:0], sda_sync[1]};
                                        symbol <= MASTER_ACK;
                                    end else bit_index <= bit_index - 1'b1;
                                end
                                MASTER_ACK: begin
                                    if (two_read && !read_index) begin
                                        read_index <= 1;
                                        bit_index <= 7;
                                        symbol <= RX_BIT;
                                    end else state <= STOP_FALL;
                                end
                            endcase
                        end
                        RESTART_FALL: begin
                            scl_low <= 1;
                            state <= RESTART_RELEASE;
                        end
                        RESTART_RELEASE: begin
                            sda_low <= 0;
                            state <= RESTART_RISE;
                        end
                        RESTART_RISE: begin
                            scl_low <= 0;
                            state <= RESTART_HIGH_WAIT;
                        end
                        RESTART_START: begin
                            sda_low <= 1;
                            div_count <= 2*QUARTER-1;
                            state <= START_HOLD;
                        end
                        STOP_FALL: begin
                            scl_low <= 1;
                            state <= STOP_SET;
                        end
                        STOP_SET: begin
                            sda_low <= 1;
                            state <= STOP_RISE;
                        end
                        STOP_RISE: begin
                            scl_low <= 0;
                            state <= STOP_HIGH_WAIT;
                        end
                        STOP_HIGH_WAIT: state <= STOP_RELEASE;
                        STOP_RELEASE: begin
                            sda_low <= 0;
                            div_count <= 2*QUARTER-1;
                            state <= STOP_FREE;
                        end
                        STOP_FREE: begin
                            busy <= 0;
                            done <= 1;
                            state <= IDLE;
                        end
                        default: begin
                            ack_error <= 1;
                            error_code <= 8'h0B;
                            sda_low <= 0;
                            scl_low <= 0;
                            busy <= 0;
                            done <= 1;
                            state <= IDLE;
                        end
                    endcase
                end
            end
        end
    end
endmodule
