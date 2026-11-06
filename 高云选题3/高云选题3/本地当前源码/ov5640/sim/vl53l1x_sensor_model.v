`timescale 1ns/1ps
// Behavioral register slave. Observes physical START/STOP and SCL edges;
// it does not use or force any internal master state.
module vl53l1x_sensor_model #(
    parameter integer SENSOR = 0,
    parameter [15:0] DISTANCE = 123,
    parameter integer LEGACY = 0
)(
    input rst_n,
    input xs,
    input present,
    input invalid_range,
    input never_ready,
    input nack_config,
    inout sda,
    input scl
);
    localparam RX_ADDR=0, RX_HIGH=1, RX_LOW=2, RX_DATA=3,
        ACK_SETUP=4, ACK_HIGH=5, ACK_RELEASE=6, TX=7,
        TX_RELEASE=8, MASTER_ACK=9, MASTER_ACK_END=10, IGNORE=11;
    integer state=IGNORE, next_state=IGNORE, bits=0;
    reg [6:0] address=7'h29;
    reg [15:0] pointer=0;
    reg [7:0] shift=0, outgoing=0, memory[0:255];
    reg selected=0, drive_low=0, reading=0, master_acked=0;
    reg ranging=0, calibrated=0;
    integer config_count=0, start_count=0, clear_count=0, stop_count=0;
    integer ready_reads=0, boot_reads=0, distance_reads=0, repeated_starts=0;
    integer i;
    assign sda = drive_low ? 1'b0 : 1'bz;

    function [7:0] read_register;
        input [15:0] reg_address;
        begin
            if (LEGACY) begin
                case (reg_address)
                    16'h00C0: read_register = 8'hEE;
                    16'h00C1: read_register = 8'hAA;
                    default: read_register = memory[reg_address[7:0]];
                endcase
            end else begin
            case (reg_address)
                16'h00E5: read_register = (boot_reads > 1) ? 1 : 0;
                16'h010F: read_register = 8'hEA;
                16'h0110: read_register = 8'hCC;
                16'h0031: read_register = ranging && !never_ready &&
                                         ((start_count > 1) || (ready_reads > 1));
                16'h0089: read_register = invalid_range ? 8'd4 : 8'd9;
                16'h0096: read_register = DISTANCE[15:8];
                16'h0097: read_register = DISTANCE[7:0];
                16'h00DE: read_register = 8'h01;
                16'h00DF: read_register = 8'h23; // PLL calibration = 291.
                default: read_register = memory[reg_address[7:0]];
            endcase
            end
        end
    endfunction

    always @(negedge rst_n or negedge xs) begin
        state = IGNORE;
        address = 7'h29;
        pointer = 0;
        drive_low = 0;
        selected = 0;
        reading = 0;
        ranging = 0;
        calibrated = 0;
        config_count = 0;
        start_count = 0;
        clear_count = 0;
        stop_count = 0;
        ready_reads = 0;
        boot_reads = 0;
        distance_reads = 0;
        repeated_starts = 0;
        for (i=0; i<256; i=i+1) memory[i] = 0;
    end

    always @(negedge sda) if (scl === 1'b1 && !drive_low && rst_n && xs) begin
        if (selected) repeated_starts = repeated_starts + 1;
        state = RX_ADDR;
        shift = 0;
        bits = 0;
        selected = 0;
        reading = 0;
    end
    always @(posedge sda) if (scl === 1'b1 && !drive_low) begin
        state = IGNORE;
        selected = 0;
    end

    always @(posedge scl) if (rst_n && xs && present) begin
        if (state == RX_ADDR || state == RX_HIGH || state == RX_LOW || state == RX_DATA) begin
            shift = {shift[6:0], sda};
            bits = bits + 1;
            if (bits == 8) begin
                bits = 0;
                case (state)
                    RX_ADDR: begin
                        selected = (shift[7:1] == address);
                        reading = shift[0];
                        next_state = reading ? TX : RX_HIGH;
                        if (selected && reading) begin
                            if (pointer == 16'h00E5) boot_reads = boot_reads + 1;
                            if (pointer == 16'h0031) ready_reads = ready_reads + 1;
                            if (pointer == 16'h0096) distance_reads = distance_reads + 1;
                        end
                    end
                    RX_HIGH: begin
                        if (LEGACY) begin
                            pointer = {8'b0,shift};
                            next_state = RX_DATA;
                        end else begin
                            pointer[15:8] = shift;
                            next_state = RX_LOW;
                        end
                    end
                    RX_LOW: begin
                        pointer[7:0] = shift;
                        next_state = RX_DATA;
                    end
                    RX_DATA: begin
                        if (selected) begin
                            if (nack_config && pointer == 16'h0047) selected = 0;
                            else begin
                                if (pointer < 256) memory[pointer] = shift;
                                if (!LEGACY && pointer >= 16'h002D && pointer <= 16'h0087)
                                    config_count = config_count + 1;
                                if (!LEGACY && pointer == 16'h0001) address = shift[6:0];
                                if (!LEGACY && pointer == 16'h0087 && shift == 8'h40) begin
                                    start_count = start_count + 1;
                                    ranging = 1;
                                    ready_reads = 0;
                                    if (start_count == 2) begin
                                        if (!calibrated || memory[8] != 9 || memory[11] != 0)
                                            $fatal(1, "Sensor %0d: missing first-ranging calibration", SENSOR);
                                        if ({memory['h5e],memory['h5f]} != 16'h00AD ||
                                            {memory['h61],memory['h62]} != 16'h00C6)
                                            $fatal(1, "Sensor %0d: timing budget mismatch", SENSOR);
                                        if ({memory['h6c],memory['h6d],memory['h6e],memory['h6f]} != 32'd18772)
                                            $fatal(1, "Sensor %0d: interval mismatch", SENSOR);
                                        if (memory['h2e] != 1 || memory['h2f] != 1)
                                            $fatal(1, "Sensor %0d: AVDD IO mode mismatch", SENSOR);
                                        if (memory['h54] != 'hc8 || memory['h6e] != 'h49 || memory['h77] != 1)
                                            $fatal(1, "Sensor %0d: configuration table offset", SENSOR);
                                    end
                                end
                                if (!LEGACY && pointer == 16'h0087 && shift == 0 && start_count > 0) begin
                                    ranging = 0;
                                    stop_count = stop_count + 1;
                                    calibrated = clear_count > 0;
                                end
                                if (!LEGACY && pointer == 16'h0086 && shift == 1) clear_count = clear_count + 1;
                            end
                        end
                        pointer = pointer + 1;
                        next_state = RX_DATA;
                    end
                endcase
                state = ACK_SETUP;
            end
        end else if (state == ACK_HIGH) state = ACK_RELEASE;
        else if (state == TX) begin
            if (bits == 0) state = TX_RELEASE;
            else bits = bits - 1;
        end else if (state == MASTER_ACK) begin
            master_acked = !sda;
            state = MASTER_ACK_END;
        end
    end

    always @(negedge scl) if (rst_n && xs && present) begin
        if (state == ACK_SETUP) begin
            drive_low = selected;
            state = ACK_HIGH;
        end else if (state == ACK_RELEASE) begin
            drive_low = 0;
            state = selected ? next_state : IGNORE;
            if (state == TX) begin
                outgoing = read_register(pointer);
                bits = 7;
                drive_low = !outgoing[7];
            end
        end else if (state == TX) drive_low = !outgoing[bits];
        else if (state == TX_RELEASE) begin
            drive_low = 0;
            state = MASTER_ACK;
        end else if (state == MASTER_ACK_END) begin
            if (master_acked) begin
                pointer = pointer + 1;
                outgoing = read_register(pointer);
                bits = 7;
                drive_low = !outgoing[7];
                state = TX;
            end else begin
                drive_low = 0;
                state = IGNORE;
            end
        end
    end
endmodule

