// Four VL53L1X devices share SDA/SCL and have independent XSHUT pins.
// Sequential boot/address assignment, ULD first-ranging calibration, then
// autonomous long-mode measurements (50 ms budget, 60 ms interval by default).
// Register configuration derived from ST VL53L1X ULD; see ST_VL53L1X_LICENSE.txt.
module vl53l1x_array_controller #(
    parameter integer CLK_HZ = 50000000,
    parameter integer I2C_HZ = 100000,
    parameter integer ENABLE_WAIT_MS = 50,
    parameter integer RESET_WAIT_MS = 10,
    parameter integer POLL_WAIT_MS = 2,
    parameter integer READY_TIMEOUT_MS = 500,
    parameter integer SAMPLE_MS = 60,
    parameter integer INTERMEASUREMENT_MS = 60,
    parameter integer STALE_MS = 500,
    parameter integer RETRY_MS = 2000
)(
    input clk,
    input rst_n,
    output xs1,
    output xs2,
    output xs3,
    output xs4,
    inout i2c_sda,
    inout i2c_scl,
    output reg [15:0] distance0,
    output reg [15:0] distance1,
    output reg [15:0] distance2,
    output reg [15:0] distance3,
    output reg frame_valid,
    output reg [3:0] sample_valid,
    output reg [3:0] observation_valid,
    output reg [47:0] debug0,
    output reg [47:0] debug1,
    output reg [47:0] debug2,
    output reg [47:0] debug3
);
    localparam integer RESET_CYCLES = (CLK_HZ/1000)*RESET_WAIT_MS;
    localparam integer ENABLE_CYCLES = (CLK_HZ/1000)*ENABLE_WAIT_MS;
    localparam integer POLL_CYCLES = (CLK_HZ/1000)*POLL_WAIT_MS;
    localparam integer SAMPLE_CYCLES_RAW = (CLK_HZ/1000)*SAMPLE_MS;
    localparam integer SAMPLE_CYCLES = (SAMPLE_CYCLES_RAW < 1) ? 1 : SAMPLE_CYCLES_RAW;
    localparam integer SAMPLE_W = (SAMPLE_CYCLES < 2) ? 1 : $clog2(SAMPLE_CYCLES);
    localparam integer MAX_POLLS_RAW = READY_TIMEOUT_MS / ((POLL_WAIT_MS < 1) ? 1 : POLL_WAIT_MS);
    localparam integer MAX_POLLS = (MAX_POLLS_RAW < 1) ? 1 : MAX_POLLS_RAW;
    localparam integer POLL_W = (MAX_POLLS < 2) ? 1 : $clog2(MAX_POLLS);
    localparam integer STALE_FRAMES_RAW = (STALE_MS + SAMPLE_MS-1)/SAMPLE_MS;
    localparam integer STALE_FRAMES = (STALE_FRAMES_RAW < 1) ? 1 : STALE_FRAMES_RAW;
    localparam integer STALE_W = (STALE_FRAMES < 2) ? 1 : $clog2(STALE_FRAMES+1);
    localparam integer RETRY_FRAMES_RAW = (RETRY_MS + SAMPLE_MS-1)/SAMPLE_MS;
    localparam integer RETRY_FRAMES = (RETRY_FRAMES_RAW < 1) ? 1 : RETRY_FRAMES_RAW;
    localparam integer RETRY_W = (RETRY_FRAMES < 2) ? 1 : $clog2(RETRY_FRAMES+1);

    localparam [3:0] RESET_WAIT = 0, ENABLE_WAIT = 1, LAUNCH = 2,
        WAIT_DONE = 3, POLL_WAIT = 4, INIT_FAIL = 5, INIT_NEXT = 6,
        RUN_SELECT = 7, RUN_NEXT = 8, FRAME_WAIT = 9, RETRY_RESET_WAIT = 10;
    localparam [4:0] OP_BOOT = 0, OP_ADDRESS = 1, OP_ID = 2, OP_CONFIG = 3,
        OP_CAL_START = 4, OP_CAL_READY = 5, OP_CAL_CLEAR = 6, OP_CAL_STOP = 7,
        OP_VHV_BOUND = 8, OP_VHV_START = 9, OP_TIMING_A = 10, OP_TIMING_B = 11,
        OP_OSC = 12, OP_INTERVAL_HIGH = 13, OP_INTERVAL_LOW = 14,
        OP_START = 15, OP_READY = 16, OP_RANGE_STATUS = 17,
        OP_DISTANCE = 18, OP_CLEAR = 19, OP_PROBE_ID = 20, OP_LEGACY_ID = 21;

    reg [3:0] state;
    reg [4:0] operation;
    reg [31:0] delay_count;
    reg [SAMPLE_W-1:0] sample_count;
    reg [POLL_W-1:0] poll_count;
    reg [STALE_W-1:0] stale_count [0:3];
    reg [RETRY_W-1:0] retry_count [0:3];
    reg [1:0] sensor_index;
    reg [6:0] cfg_index;
    reg [3:0] xshut_reg, initialized;
    reg running, range_valid;
    reg [31:0] interval_value;
    reg [15:0] command_reg;
    reg [6:0] command_addr;
    reg command_read;
    reg command_short_register;
    reg [1:0] command_write_len, command_read_len;
    reg [15:0] command_write_data;
    reg i2c_start;
    wire [15:0] i2c_read_data;
    wire i2c_busy, i2c_done, i2c_ack_error;
    wire [7:0] i2c_error_code;

    assign xs1 = xshut_reg[0];
    assign xs2 = xshut_reg[1];
    assign xs3 = xshut_reg[2];
    assign xs4 = xshut_reg[3];

    function [6:0] sensor_address;
        input [1:0] index;
        begin
            sensor_address = 7'h2A + {5'b0, index};
        end
    endfunction

    // ST ULD 91-byte configuration: addresses 0x002D through 0x0087.
    // 0x002E/0x002F bit 0 set for the module's AVDD-level I/O.
    function [7:0] default_config;
        input [6:0] index;
        begin
            case (index)
                7'd0: default_config = 8'h00; // 0x002D
                7'd1: default_config = 8'h01; // 0x002E
                7'd2: default_config = 8'h01; // 0x002F
                7'd3: default_config = 8'h01; // 0x0030
                7'd4: default_config = 8'h02; // 0x0031
                7'd5: default_config = 8'h00; // 0x0032
                7'd6: default_config = 8'h02; // 0x0033
                7'd7: default_config = 8'h08; // 0x0034
                7'd8: default_config = 8'h00; // 0x0035
                7'd9: default_config = 8'h08; // 0x0036
                7'd10: default_config = 8'h10; // 0x0037
                7'd11: default_config = 8'h01; // 0x0038
                7'd12: default_config = 8'h01; // 0x0039
                7'd13: default_config = 8'h00; // 0x003A
                7'd14: default_config = 8'h00; // 0x003B
                7'd15: default_config = 8'h00; // 0x003C
                7'd16: default_config = 8'h00; // 0x003D
                7'd17: default_config = 8'hFF; // 0x003E
                7'd18: default_config = 8'h00; // 0x003F
                7'd19: default_config = 8'h0F; // 0x0040
                7'd20: default_config = 8'h00; // 0x0041
                7'd21: default_config = 8'h00; // 0x0042
                7'd22: default_config = 8'h00; // 0x0043
                7'd23: default_config = 8'h00; // 0x0044
                7'd24: default_config = 8'h00; // 0x0045
                7'd25: default_config = 8'h20; // 0x0046
                7'd26: default_config = 8'h0B; // 0x0047
                7'd27: default_config = 8'h00; // 0x0048
                7'd28: default_config = 8'h00; // 0x0049
                7'd29: default_config = 8'h02; // 0x004A
                7'd30: default_config = 8'h0A; // 0x004B
                7'd31: default_config = 8'h21; // 0x004C
                7'd32: default_config = 8'h00; // 0x004D
                7'd33: default_config = 8'h00; // 0x004E
                7'd34: default_config = 8'h05; // 0x004F
                7'd35: default_config = 8'h00; // 0x0050
                7'd36: default_config = 8'h00; // 0x0051
                7'd37: default_config = 8'h00; // 0x0052
                7'd38: default_config = 8'h00; // 0x0053
                7'd39: default_config = 8'hC8; // 0x0054
                7'd40: default_config = 8'h00; // 0x0055
                7'd41: default_config = 8'h00; // 0x0056
                7'd42: default_config = 8'h38; // 0x0057
                7'd43: default_config = 8'hFF; // 0x0058
                7'd44: default_config = 8'h01; // 0x0059
                7'd45: default_config = 8'h00; // 0x005A
                7'd46: default_config = 8'h08; // 0x005B
                7'd47: default_config = 8'h00; // 0x005C
                7'd48: default_config = 8'h00; // 0x005D
                7'd49: default_config = 8'h01; // 0x005E
                7'd50: default_config = 8'hCC; // 0x005F
                7'd51: default_config = 8'h0F; // 0x0060
                7'd52: default_config = 8'h01; // 0x0061
                7'd53: default_config = 8'hF1; // 0x0062
                7'd54: default_config = 8'h0D; // 0x0063
                7'd55: default_config = 8'h01; // 0x0064
                7'd56: default_config = 8'h68; // 0x0065
                7'd57: default_config = 8'h00; // 0x0066
                7'd58: default_config = 8'h80; // 0x0067
                7'd59: default_config = 8'h08; // 0x0068
                7'd60: default_config = 8'hB8; // 0x0069
                7'd61: default_config = 8'h00; // 0x006A
                7'd62: default_config = 8'h00; // 0x006B
                7'd63: default_config = 8'h00; // 0x006C
                7'd64: default_config = 8'h00; // 0x006D
                7'd65: default_config = 8'h0F; // 0x006E
                7'd66: default_config = 8'h89; // 0x006F
                7'd67: default_config = 8'h00; // 0x0070
                7'd68: default_config = 8'h00; // 0x0071
                7'd69: default_config = 8'h00; // 0x0072
                7'd70: default_config = 8'h00; // 0x0073
                7'd71: default_config = 8'h00; // 0x0074
                7'd72: default_config = 8'h00; // 0x0075
                7'd73: default_config = 8'h00; // 0x0076
                7'd74: default_config = 8'h01; // 0x0077
                7'd75: default_config = 8'h0F; // 0x0078
                7'd76: default_config = 8'h0D; // 0x0079
                7'd77: default_config = 8'h0E; // 0x007A
                7'd78: default_config = 8'h0E; // 0x007B
                7'd79: default_config = 8'h00; // 0x007C
                7'd80: default_config = 8'h00; // 0x007D
                7'd81: default_config = 8'h02; // 0x007E
                7'd82: default_config = 8'hC7; // 0x007F
                7'd83: default_config = 8'hFF; // 0x0080
                7'd84: default_config = 8'h9B; // 0x0081
                7'd85: default_config = 8'h00; // 0x0082
                7'd86: default_config = 8'h00; // 0x0083
                7'd87: default_config = 8'h00; // 0x0084
                7'd88: default_config = 8'h01; // 0x0085
                7'd89: default_config = 8'h00; // 0x0086
                7'd90: default_config = 8'h00; // 0x0087
                default: default_config = 8'h00;
            endcase
        end
    endfunction

    // 0xFFFF is -1 at the existing signed FireWater transmitter input.
    // This marks a missing sensor, invalid range, or communication failure.
    task store_distance;
        input [15:0] value;
        begin
            case (sensor_index)
                0: distance0 <= value;
                1: distance1 <= value;
                2: distance2 <= value;
                3: distance3 <= value;
            endcase
        end
    endtask

    // E (error), O (operation), R (register), V (readback) in hexadecimal.
    // Keep the failure until successful recovery, so diagnostics are visible
    // even while a sensor is being retried. No fabricated distance is sent.
    task store_debug;
        input [7:0] error;
        input [15:0] value;
        reg [47:0] record;
        begin
            record = {error, 3'b0, operation, command_reg, value};
            case (sensor_index)
                0: debug0 <= record;
                1: debug1 <= record;
                2: debug2 <= record;
                3: debug3 <= record;
            endcase
        end
    endtask

    vl53l1x_i2c_master #(.CLK_HZ(CLK_HZ), .I2C_HZ(I2C_HZ)) u_i2c(
        .clk(clk), .rst_n(rst_n), .start(i2c_start),
        .dev_addr(command_addr), .reg_addr(command_reg),
        .reg_addr_8bit(command_short_register),
        .rw_read(command_read), .write_len(command_write_len),
        .read_len(command_read_len), .write_data(command_write_data),
        .read_data(i2c_read_data), .busy(i2c_busy), .done(i2c_done),
        .ack_error(i2c_ack_error), .error_code(i2c_error_code),
        .sda(i2c_sda), .scl(i2c_scl)
    );

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= RESET_WAIT;
            operation <= OP_BOOT;
            delay_count <= 0;
            sample_count <= 0;
            poll_count <= 0;
            stale_count[0] <= 0;
            stale_count[1] <= 0;
            stale_count[2] <= 0;
            stale_count[3] <= 0;
            retry_count[0] <= 0;
            retry_count[1] <= 0;
            retry_count[2] <= 0;
            retry_count[3] <= 0;
            sensor_index <= 0;
            cfg_index <= 0;
            xshut_reg <= 0;
            initialized <= 0;
            running <= 0;
            range_valid <= 0;
            interval_value <= 0;
            command_reg <= 0;
            command_addr <= 7'h29;
            command_read <= 0;
            command_short_register <= 0;
            command_write_len <= 1;
            command_read_len <= 1;
            command_write_data <= 0;
            i2c_start <= 0;
            distance0 <= 16'hFFFF;
            distance1 <= 16'hFFFF;
            distance2 <= 16'hFFFF;
            distance3 <= 16'hFFFF;
            frame_valid <= 0;
            sample_valid <= 0;
            observation_valid <= 0;
            debug0 <= 48'h7F0000E50000;
            debug1 <= 48'h7F0000E50000;
            debug2 <= 48'h7F0000E50000;
            debug3 <= 48'h7F0000E50000;
        end else begin
            i2c_start <= 0;
            frame_valid <= 0;
            sample_valid <= 0;
            observation_valid <= 0;
            if (running && sample_count < SAMPLE_CYCLES-1)
                sample_count <= sample_count + 1'b1;

            case (state)
                RESET_WAIT: begin
                    xshut_reg <= 0;
                    if (RESET_CYCLES < 1 || delay_count >= RESET_CYCLES-1) begin
                        delay_count <= 0;
                        sensor_index <= 0;
                        xshut_reg[0] <= 1;
                        state <= ENABLE_WAIT;
                    end else delay_count <= delay_count + 1'b1;
                end
                ENABLE_WAIT: begin
                    if (ENABLE_CYCLES < 1 || delay_count >= ENABLE_CYCLES-1) begin
                        delay_count <= 0;
                        poll_count <= 0;
                        operation <= OP_PROBE_ID;
                        state <= LAUNCH;
                    end else delay_count <= delay_count + 1'b1;
                end
                LAUNCH: if (!i2c_busy) begin
                    command_addr <= sensor_address(sensor_index);
                    command_read <= 0;
                    command_short_register <= 0;
                    command_write_len <= 1;
                    command_read_len <= 1;
                    command_write_data <= 0;
                    case (operation)
                        OP_BOOT: begin
                            command_addr <= 7'h29;
                            command_reg <= 16'h00E5;
                            command_read <= 1;
                        end
                        OP_ADDRESS: begin
                            // Earlier sensors are awake at their new addresses;
                            // only this sensor is awake at the default 0x29.
                            command_addr <= 7'h29;
                            command_reg <= 16'h0001;
                            command_write_data <= {9'd0, sensor_address(sensor_index)};
                        end
                        OP_ID, OP_PROBE_ID: begin
                            if (operation == OP_PROBE_ID) command_addr <= 7'h29;
                            command_reg <= 16'h010F;
                            command_read <= 1;
                            command_read_len <= 2;
                        end
                        OP_LEGACY_ID: begin
                            // No payload writes: one-byte index 0xC0 followed
                            // by repeated START/read of model/module ID.
                            command_addr <= 7'h29;
                            command_reg <= 16'h00C0;
                            command_short_register <= 1;
                            command_read <= 1;
                            command_read_len <= 2;
                        end
                        OP_CONFIG: begin
                            command_reg <= 16'h002D + {9'd0, cfg_index};
                            command_write_data <= {8'd0, default_config(cfg_index)};
                        end
                        OP_CAL_START, OP_START: begin
                            command_reg <= 16'h0087;
                            command_write_data <= 16'h0040;
                        end
                        OP_CAL_READY, OP_READY: begin
                            command_reg <= 16'h0031;
                            command_read <= 1;
                        end
                        OP_CAL_CLEAR, OP_CLEAR: begin
                            command_reg <= 16'h0086;
                            command_write_data <= 16'h0001;
                        end
                        OP_CAL_STOP: command_reg <= 16'h0087;
                        OP_VHV_BOUND: begin
                            command_reg <= 16'h0008;
                            command_write_data <= 16'h0009;
                        end
                        OP_VHV_START: command_reg <= 16'h000B;
                        OP_TIMING_A: begin
                            command_reg <= 16'h005E;
                            command_write_len <= 2;
                            command_write_data <= 16'h00AD; // Long mode, 50 ms.
                        end
                        OP_TIMING_B: begin
                            command_reg <= 16'h0061;
                            command_write_len <= 2;
                            command_write_data <= 16'h00C6;
                        end
                        OP_OSC: begin
                            command_reg <= 16'h00DE;
                            command_read <= 1;
                            command_read_len <= 2;
                        end
                        OP_INTERVAL_HIGH: begin
                            command_reg <= 16'h006C;
                            command_write_len <= 2;
                            command_write_data <= interval_value[31:16];
                        end
                        OP_INTERVAL_LOW: begin
                            command_reg <= 16'h006E;
                            command_write_len <= 2;
                            command_write_data <= interval_value[15:0];
                        end
                        OP_RANGE_STATUS: begin
                            command_reg <= 16'h0089;
                            command_read <= 1;
                        end
                        OP_DISTANCE: begin
                            command_reg <= 16'h0096;
                            command_read <= 1;
                            command_read_len <= 2;
                        end
                    endcase
                    i2c_start <= 1;
                    state <= WAIT_DONE;
                end
                WAIT_DONE: if (i2c_done) begin
                    if (i2c_ack_error) begin
                        // Never advance initialization after a failed write.
                        if (operation != OP_LEGACY_ID)
                            store_debug(i2c_error_code, i2c_read_data);
                        if (operation >= OP_READY && operation <= OP_CLEAR) begin
                            initialized[sensor_index] <= 0;
                            xshut_reg[sensor_index] <= 0;
                            retry_count[sensor_index] <= 0;
                            store_distance(16'hFFFF);
                            state <= RUN_NEXT;
                        end else state <= INIT_FAIL;
                    end else begin
                        state <= LAUNCH;
                        case (operation)
                            OP_BOOT, OP_CAL_READY: begin
                                if (i2c_read_data[0]) begin
                                    poll_count <= 0;
                                    operation <= (operation == OP_BOOT) ? OP_ADDRESS : OP_CAL_CLEAR;
                                end else if (poll_count == MAX_POLLS-1) begin
                                    store_debug(operation == OP_BOOT ? 8'h0C : 8'h0D,
                                                i2c_read_data);
                                    state <= INIT_FAIL;
                                end else begin
                                    poll_count <= poll_count + 1'b1;
                                    delay_count <= 0;
                                    state <= POLL_WAIT;
                                end
                            end
                            OP_ADDRESS: operation <= OP_ID;
                            OP_ID, OP_PROBE_ID: begin
                                if (i2c_read_data == 16'hEACC) begin
                                    cfg_index <= 0;
                                    operation <= operation == OP_PROBE_ID ? OP_BOOT : OP_CONFIG;
                                end else begin
                                    store_debug(8'h0E, i2c_read_data);
                                    if (operation == OP_PROBE_ID) operation <= OP_LEGACY_ID;
                                    else state <= INIT_FAIL;
                                end
                            end
                            OP_LEGACY_ID: begin
                                if (i2c_read_data == 16'hEEAA)
                                    store_debug(8'h13, i2c_read_data);
                                else store_debug(8'h14, i2c_read_data);
                                state <= INIT_FAIL;
                            end
                            OP_CONFIG: begin
                                if (cfg_index == 90) begin
                                    poll_count <= 0;
                                    operation <= OP_CAL_START;
                                end else cfg_index <= cfg_index + 1'b1;
                            end
                            OP_CAL_START: operation <= OP_CAL_READY;
                            OP_CAL_CLEAR: operation <= OP_CAL_STOP;
                            OP_CAL_STOP: operation <= OP_VHV_BOUND;
                            OP_VHV_BOUND: operation <= OP_VHV_START;
                            OP_VHV_START: operation <= OP_TIMING_A;
                            OP_TIMING_A: operation <= OP_TIMING_B;
                            OP_TIMING_B: operation <= OP_OSC;
                            OP_OSC: begin
                                // ULD: PLL calibration * interval_ms * 1.075.
                                // 1101/1024 is 1.0751953125 (0.019% difference).
                                interval_value <= ({22'd0, i2c_read_data[9:0]} *
                                                   (INTERMEASUREMENT_MS * 1101)) >> 10;
                                if (i2c_read_data[9:0] == 0) begin
                                    store_debug(8'h0F, i2c_read_data);
                                    state <= INIT_FAIL;
                                end
                                else operation <= OP_INTERVAL_HIGH;
                            end
                            OP_INTERVAL_HIGH: operation <= OP_INTERVAL_LOW;
                            OP_INTERVAL_LOW: operation <= OP_START;
                            OP_START: begin
                                initialized[sensor_index] <= 1;
                                stale_count[sensor_index] <= 0;
                                retry_count[sensor_index] <= 0;
                                store_debug(8'h00, 16'h0040);
                                state <= INIT_NEXT;
                            end
                            OP_READY: begin
                                if (i2c_read_data[0]) begin
                                    stale_count[sensor_index] <= 0;
                                    operation <= OP_RANGE_STATUS;
                                end else begin
                                    if (stale_count[sensor_index] < STALE_FRAMES)
                                        stale_count[sensor_index] <= stale_count[sensor_index] + 1'b1;
                                    if (stale_count[sensor_index] >= STALE_FRAMES-1) begin
                                        store_distance(16'hFFFF);
                                        store_debug(8'h10, i2c_read_data);
                                        initialized[sensor_index] <= 0;
                                        xshut_reg[sensor_index] <= 0;
                                        retry_count[sensor_index] <= 0;
                                    end
                                    state <= RUN_NEXT;
                                end
                            end
                            OP_RANGE_STATUS: begin
                                // Raw status 9 = valid, 22 = valid without wrap check.
                                range_valid <= (i2c_read_data[4:0] == 9 ||
                                                i2c_read_data[4:0] == 22);
                                if (i2c_read_data[4:0] != 9 && i2c_read_data[4:0] != 22)
                                    store_debug(8'h11, i2c_read_data);
                                operation <= OP_DISTANCE;
                            end
                            OP_DISTANCE: begin
                                observation_valid[sensor_index] <= 1'b1;
                                if (range_valid && i2c_read_data <= 16'd4000) begin
                                    store_distance(i2c_read_data);
                                    store_debug(8'h00, i2c_read_data);
                                    sample_valid[sensor_index] <= 1'b1;
                                end else begin
                                    store_distance(16'hFFFF);
                                    if (range_valid) store_debug(8'h12, i2c_read_data);
                                end
                                operation <= OP_CLEAR;
                            end
                            OP_CLEAR: state <= RUN_NEXT;
                            default: state <= INIT_FAIL;
                        endcase
                    end
                end
                POLL_WAIT: begin
                    if (POLL_CYCLES < 1 || delay_count >= POLL_CYCLES-1) begin
                        delay_count <= 0;
                        state <= LAUNCH;
                    end else delay_count <= delay_count + 1'b1;
                end
                INIT_FAIL: begin
                    xshut_reg[sensor_index] <= 0;
                    initialized[sensor_index] <= 0;
                    retry_count[sensor_index] <= 0;
                    store_distance(16'hFFFF);
                    state <= INIT_NEXT;
                end
                INIT_NEXT: begin
                    if (running) state <= RUN_NEXT;
                    else if (sensor_index == 3) begin
                        sensor_index <= 0;
                        sample_count <= 0;
                        running <= 1;
                        state <= RUN_SELECT;
                    end else begin
                        sensor_index <= sensor_index + 1'b1;
                        xshut_reg[sensor_index + 2'd1] <= 1;
                        delay_count <= 0;
                        state <= ENABLE_WAIT;
                    end
                end
                RUN_SELECT: begin
                    if (initialized[sensor_index]) begin
                        operation <= OP_READY;
                        state <= LAUNCH;
                    end else if (retry_count[sensor_index] >= RETRY_FRAMES-1) begin
                        // Only reset/readdress the failed device. Other devices
                        // keep their assigned addresses and autonomous ranging.
                        retry_count[sensor_index] <= 0;
                        delay_count <= 0;
                        xshut_reg[sensor_index] <= 0;
                        state <= RETRY_RESET_WAIT;
                    end else begin
                        retry_count[sensor_index] <= retry_count[sensor_index] + 1'b1;
                        state <= RUN_NEXT;
                    end
                end
                RETRY_RESET_WAIT: begin
                    if (RESET_CYCLES < 1 || delay_count >= RESET_CYCLES-1) begin
                        delay_count <= 0;
                        xshut_reg[sensor_index] <= 1;
                        state <= ENABLE_WAIT;
                    end else delay_count <= delay_count + 1'b1;
                end
                RUN_NEXT: begin
                    if (sensor_index == 3) state <= FRAME_WAIT;
                    else begin
                        sensor_index <= sensor_index + 1'b1;
                        state <= RUN_SELECT;
                    end
                end
                FRAME_WAIT: begin
                    if (sample_count >= SAMPLE_CYCLES-1) begin
                        frame_valid <= 1;
                        sample_count <= 0;
                        sensor_index <= 0;
                        state <= RUN_SELECT;
                    end
                end
                default: begin
                    xshut_reg <= 0;
                    initialized <= 0;
                    running <= 0;
                    delay_count <= 0;
                    state <= RESET_WAIT;
                end
            endcase
        end
    end
endmodule
