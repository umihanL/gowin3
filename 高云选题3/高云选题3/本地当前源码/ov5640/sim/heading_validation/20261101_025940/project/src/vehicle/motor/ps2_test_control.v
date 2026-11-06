// Serial-only Ackermann command model: no servo or motor outputs.
// START: stopped -> +MAX_SPEED cruise -> stopped.
// Moving RY during cruise takes over manually until the next START toggle.
// SELECT freezes the last issued steering command until a second press.
module ps2_test_control #(
    parameter integer CLK_HZ = 50000000,
    parameter integer LINK_TIMEOUT_MS = 200,
    parameter integer DEBOUNCE_FRAMES = 2,
    parameter integer DEADZONE = 10,
    parameter integer MAX_STEER_DEG = 20,
    parameter integer MAX_SPEED_MM_S = 200,
    parameter integer RUNTIME_SPEED = 0,
    parameter integer LEFT_STEER_DEG = MAX_STEER_DEG,
    parameter integer RIGHT_STEER_DEG = MAX_STEER_DEG
)(
    input clk, input rst_n, input sample_valid, input analog_valid,
    input [15:0] buttons_raw, input [7:0] left_x, right_y,
    input [15:0] speed_limit,
    output reg speed_edit,speed_increase,speed_decrease,
    output reg signed [15:0] steer_deg, speed_mm_s,
    output reg angle_locked, run_enabled, manual_mode, link_ok
);
    localparam integer TIMEOUT_CYCLES = (CLK_HZ / 1000) * LINK_TIMEOUT_MS;
    localparam integer TIMEOUT_W = (TIMEOUT_CYCLES < 2) ? 1 : $clog2(TIMEOUT_CYCLES);
    localparam integer DEBOUNCE_N = (DEBOUNCE_FRAMES < 1) ? 1 : DEBOUNCE_FRAMES;
    localparam integer DEBOUNCE_W = (DEBOUNCE_N < 2) ? 1 : $clog2(DEBOUNCE_N);
    localparam [7:0] LOW_EDGE = 128-DEADZONE, HIGH_EDGE = 128+DEADZONE;
    localparam [15:0] NEG_RANGE = 128-DEADZONE, POS_RANGE = 127-DEADZONE;
    reg [TIMEOUT_W-1:0] timeout_count;
    reg keys_initialized;
    reg select_stable, start_stable;
    reg [2:0] adjust_stable,adjust_count;
    wire [2:0] adjust_raw={!buttons_ready[9],!buttons_ready[11],!buttons_ready[13]};
    wire [2:0] adjust_press=adjust_raw & ~adjust_stable & adjust_count;
    reg [DEBOUNCE_W-1:0] select_count, start_count;
    reg sample_ready, analog_ready, right_moved;
    reg [15:0] buttons_ready;
    wire select_raw = !buttons_ready[0], start_raw = !buttons_ready[3];
    wire select_press = keys_initialized && select_raw && !select_stable &&
        (select_count == DEBOUNCE_N-1);
    wire start_press = keys_initialized && start_raw && !start_stable &&
        (start_count == DEBOUNCE_N-1);

    function signed [15:0] map_steering;
        input [7:0] axis;
        reg [7:0] magnitude;
        reg [15:0] product;
        begin
            map_steering = 0;
            if (axis < LOW_EDGE) begin
                magnitude = LOW_EDGE-axis; product = magnitude * LEFT_STEER_DEG;
                map_steering = -$signed(product / NEG_RANGE);
            end else if (axis > HIGH_EDGE) begin
                magnitude = axis-HIGH_EDGE; product = magnitude * RIGHT_STEER_DEG;
                map_steering = $signed(product / POS_RANGE);
            end
        end
    endfunction
    function signed [15:0] map_speed;
        input [7:0] axis;
        reg [7:0] magnitude;
        reg [15:0] product;
        begin
            map_speed = 0;
            if (axis < LOW_EDGE) begin
                magnitude = LOW_EDGE-axis; product = magnitude * (RUNTIME_SPEED?256:MAX_SPEED_MM_S);
                map_speed = $signed(product / NEG_RANGE);
            end else if (axis > HIGH_EDGE) begin
                magnitude = axis-HIGH_EDGE; product = magnitude * (RUNTIME_SPEED?256:MAX_SPEED_MM_S);
                map_speed = -$signed(product / POS_RANGE);
            end
        end
    endfunction
    // Precalculate all 256 axis values at synthesis time. Small synchronous
    // logic ROMs preserve BSRAM for the camera buffers and navigation map.
    reg [15:0] steering_rom [0:255] /* synthesis syn_romstyle="logic" */;
    reg [15:0] speed_rom [0:255] /* synthesis syn_romstyle="logic" */;
    reg signed [15:0] requested_steer, requested_speed;
    reg signed [15:0] scaled_speed;
    wire signed [15:0] cruise_speed=RUNTIME_SPEED?$signed(speed_limit):MAX_SPEED_MM_S;
    function signed [15:0] scale_command;
        input signed [15:0] ratio;input [15:0] limit;
        reg [8:0] mag;reg [25:0] total;integer bitno;
        begin
            mag=ratio<0?-ratio:ratio;total=0;
            for(bitno=0;bitno<9;bitno=bitno+1)
                if(mag[bitno])total=total+({10'd0,limit}<<bitno);
            scale_command=ratio<0?-$signed(total[23:8]):$signed(total[23:8]);
        end
    endfunction
    integer axis_index;
    initial begin
        for (axis_index=0; axis_index<256; axis_index=axis_index+1) begin
            steering_rom[axis_index] = map_steering(axis_index[7:0]);
            speed_rom[axis_index] = map_speed(axis_index[7:0]);
        end
    end
    always @(posedge clk) begin
        requested_steer <= steering_rom[left_x];
        requested_speed <= speed_rom[right_y];
        scaled_speed <= RUNTIME_SPEED?scale_command(requested_speed,speed_limit):requested_speed;
    end
    // Align button events and analog-valid with the synchronous ROM result.
    reg [2:0] sample_delay;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin sample_delay<=0;sample_ready<=0; analog_ready<=0; buttons_ready<=16'hffff; right_moved<=0; end
        else begin
            sample_delay<={sample_delay[1:0],sample_valid};sample_ready<=sample_delay[2];
            if(sample_valid)begin
                analog_ready<=analog_valid; buttons_ready<=buttons_raw;
                right_moved<=(right_y<LOW_EDGE)||(right_y>HIGH_EDGE);
            end
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            steer_deg <= 0; speed_mm_s <= 0; angle_locked <= 0;
            run_enabled <= 0; manual_mode <= 0; link_ok <= 0;
            timeout_count <= 0; keys_initialized <= 0;
            select_stable <= 0; start_stable <= 0; select_count <= 0; start_count <= 0;
            speed_edit<=0;speed_increase<=0;speed_decrease<=0;adjust_stable<=0;adjust_count<=0;
        end else begin
          speed_increase<=0;speed_decrease<=0;
          if (sample_ready) begin
            timeout_count <= 0; link_ok <= 1;
            if (!keys_initialized) begin
                // A held key at connection/reconnection does not toggle a command.
                keys_initialized <= 1;
                select_stable <= select_raw; start_stable <= start_raw;
                select_count <= 0; start_count <= 0;
                adjust_stable<=adjust_raw;adjust_count<=0;
            end else begin
                adjust_count<=(adjust_raw ^ adjust_stable);
                adjust_stable<=(adjust_stable & ~adjust_count) | (adjust_raw & adjust_count);
                if (select_raw == select_stable) select_count <= 0;
                else if (select_count == DEBOUNCE_N-1) begin
                    select_stable <= select_raw; select_count <= 0;
                end else select_count <= select_count + 1'b1;
                if (start_raw == start_stable) start_count <= 0;
                else if (start_count == DEBOUNCE_N-1) begin
                    start_stable <= start_raw; start_count <= 0;
                end else start_count <= start_count + 1'b1;
            end

            if (select_press) begin
                angle_locked <= !angle_locked;
                if (angle_locked && analog_ready) steer_deg <= requested_steer;
            end else if (!angle_locked && analog_ready) steer_deg <= requested_steer;

            if(RUNTIME_SPEED && (speed_edit || adjust_press[0]))begin
                // Editing always disarms; START is required after leaving it.
                run_enabled<=0;manual_mode<=0;speed_mm_s<=0;
                if(adjust_press[0])speed_edit<=!speed_edit;
                else if(adjust_press[1] && !adjust_press[2])speed_increase<=1;
                else if(adjust_press[2] && !adjust_press[1])speed_decrease<=1;
            end else if (start_press) begin
                run_enabled <= !run_enabled; manual_mode <= 0;
                speed_mm_s <= run_enabled ? 16'sd0 : cruise_speed;
            end else if (!run_enabled) speed_mm_s <= 0;
            else if (analog_ready && (manual_mode || right_moved)) begin
                manual_mode <= 1; speed_mm_s <= scaled_speed;
            end else if (!manual_mode) speed_mm_s <= cruise_speed;
            else speed_mm_s <= 0;
        end else if (timeout_count == TIMEOUT_CYCLES-1) begin
            link_ok <= 0; run_enabled <= 0; manual_mode <= 0; speed_mm_s <= 0;
            keys_initialized <= 0; select_count <= 0; start_count <= 0;
            speed_edit<=0;adjust_stable<=0;adjust_count<=0;
        end else timeout_count <= timeout_count + 1'b1;
        end
    end
endmodule
