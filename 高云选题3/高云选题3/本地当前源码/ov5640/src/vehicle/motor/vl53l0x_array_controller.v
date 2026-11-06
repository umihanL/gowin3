// Four VL53L0X devices, 8-bit register index, independent XSHUT and calibration.
// Init/tuning derived from Pololu and ST; see VL53L0X_LICENSE.txt.
module vl53l0x_array_controller #(
    parameter integer CLK_HZ=50000000, I2C_HZ=100000,
    parameter integer ENABLE_WAIT_MS=50, RESET_WAIT_MS=10, POLL_WAIT_MS=2,
    parameter integer READY_TIMEOUT_MS=500, SAMPLE_MS=60, STALE_MS=500,
    parameter integer RETRY_MS=2000
)(
    input clk, input rst_n,
    output xs1, output xs2, output xs3, output xs4,
    inout i2c_sda, inout i2c_scl,
    output reg [15:0] distance0, distance1, distance2, distance3,
    output reg frame_valid,
    // One-cycle per-device success pulses; a completed scan can contain no
    // new measurement on a not-ready device while distance still holds old data.
    output reg [3:0] sample_valid,
    output reg [3:0] observation_valid,
    output reg [47:0] debug0, debug1, debug2, debug3
);
    localparam integer SAMPLE_CYCLES=(CLK_HZ/1000)*SAMPLE_MS;
    localparam integer SAMPLE_W=(SAMPLE_CYCLES<2)?1:$clog2(SAMPLE_CYCLES);
    localparam integer MAX_POLLS=(READY_TIMEOUT_MS/POLL_WAIT_MS<1)?1:READY_TIMEOUT_MS/POLL_WAIT_MS;
    localparam integer POLL_W=(MAX_POLLS<2)?1:$clog2(MAX_POLLS);
    localparam integer STALE_FRAMES=(STALE_MS+SAMPLE_MS-1)/SAMPLE_MS;
    localparam integer STALE_W=(STALE_FRAMES<2)?1:$clog2(STALE_FRAMES+1);
    localparam integer RETRY_FRAMES=(RETRY_MS+SAMPLE_MS-1)/SAMPLE_MS;
    localparam integer RETRY_W=(RETRY_FRAMES<2)?1:$clog2(RETRY_FRAMES+1);
    localparam [3:0] RESET_WAIT=0, ENABLE_WAIT=1, INIT_EXEC=2, WAIT_BUS=3,
        POLL_WAIT=4, MAP_FILTER=5, INIT_NEXT=6, RUN_SELECT=7, RUN_NEXT=8,
        FRAME_WAIT=9, RETRY_RESET=10, RUN_EXEC=11;
    // Instruction [31:28]=action, [27:24]=diagnostic stage, [23:16]=register.
    localparam [3:0] WRITE=0, READ=1, OR_LAST=2, AND_LAST=3, ID=4, ADDRESS=5,
        STOP_READ=6, SPAD_INFO=7, MAP_READ=8, MAP_WRITE=9, POLL_SPAD=10,
        POLL_CAL=11, STOP_WRITE=12, FILTER=13, FINISH=14, WRITE16=15;
    reg [3:0] state;
    reg [7:0] pc;
    reg [1:0] channel;
    reg [3:0] shutdown, initialized;
    reg first_pass, init_command, range_valid;
    reg [31:0] delay_count;
    reg [SAMPLE_W-1:0] sample_count;
    reg [POLL_W-1:0] poll_count;
    reg [STALE_W-1:0] stale [0:3];
    reg [RETRY_W-1:0] retry [0:3];
    reg [7:0] stop_variable [0:3];
    reg [15:0] last_value;
    reg [47:0] spad_map;
    reg [6:0] spad_count;
    reg aperture;
    reg [5:0] filter_index, enabled_count;
    reg [3:0] run_step;
    reg [6:0] command_addr;
    reg [15:0] command_reg, command_data;
    reg command_read;
    reg [1:0] command_len;
    reg [7:0] command_stage;
    reg i2c_start;
    wire [15:0] read_data;
    wire busy, done, ack_error;
    wire [7:0] error_code;
    integer j;
    assign {xs4,xs3,xs2,xs1}=shutdown;

    function [31:0] init_instruction;
        input [7:0] index;
        begin
            case(index)
                8'd0: init_instruction = 32'h40C00000;
                8'd1: init_instruction = 32'h518A0000;
                8'd2: init_instruction = 32'h40C00000;
                8'd3: init_instruction = 32'h12890000;
                8'd4: init_instruction = 32'h22890001;
                8'd5: init_instruction = 32'h02880000;
                8'd6: init_instruction = 32'h03800001;
                8'd7: init_instruction = 32'h03FF0001;
                8'd8: init_instruction = 32'h03000000;
                8'd9: init_instruction = 32'h63910000;
                8'd10: init_instruction = 32'h03000001;
                8'd11: init_instruction = 32'h03FF0000;
                8'd12: init_instruction = 32'h03800000;
                8'd13: init_instruction = 32'h12600000;
                8'd14: init_instruction = 32'h22600012;
                8'd15: init_instruction = 32'hF2440020;
                8'd16: init_instruction = 32'h020100FF;
                8'd17: init_instruction = 32'h04800001;
                8'd18: init_instruction = 32'h04FF0001;
                8'd19: init_instruction = 32'h04000000;
                8'd20: init_instruction = 32'h04FF0006;
                8'd21: init_instruction = 32'h14830000;
                8'd22: init_instruction = 32'h24830004;
                8'd23: init_instruction = 32'h04FF0007;
                8'd24: init_instruction = 32'h04810001;
                8'd25: init_instruction = 32'h04800001;
                8'd26: init_instruction = 32'h0494006B;
                8'd27: init_instruction = 32'h04830000;
                8'd28: init_instruction = 32'hA4830000;
                8'd29: init_instruction = 32'h04830001;
                8'd30: init_instruction = 32'h74920000;
                8'd31: init_instruction = 32'h04810000;
                8'd32: init_instruction = 32'h04FF0006;
                8'd33: init_instruction = 32'h14830000;
                8'd34: init_instruction = 32'h348300FB;
                8'd35: init_instruction = 32'h04FF0001;
                8'd36: init_instruction = 32'h04000001;
                8'd37: init_instruction = 32'h04FF0000;
                8'd38: init_instruction = 32'h04800000;
                8'd39: init_instruction = 32'h85B00000;
                8'd40: init_instruction = 32'h85B20000;
                8'd41: init_instruction = 32'h85B40000;
                8'd42: init_instruction = 32'h05FF0001;
                8'd43: init_instruction = 32'h054F0000;
                8'd44: init_instruction = 32'h054E002C;
                8'd45: init_instruction = 32'h05FF0000;
                8'd46: init_instruction = 32'h05B600B4;
                8'd47: init_instruction = 32'hD5000000;
                8'd48: init_instruction = 32'h95B00000;
                8'd49: init_instruction = 32'h95B20000;
                8'd50: init_instruction = 32'h95B40000;
                8'd51: init_instruction = 32'h06FF0001;
                8'd52: init_instruction = 32'h06000000;
                8'd53: init_instruction = 32'h06FF0000;
                8'd54: init_instruction = 32'h06090000;
                8'd55: init_instruction = 32'h06100000;
                8'd56: init_instruction = 32'h06110000;
                8'd57: init_instruction = 32'h06240001;
                8'd58: init_instruction = 32'h062500FF;
                8'd59: init_instruction = 32'h06750000;
                8'd60: init_instruction = 32'h06FF0001;
                8'd61: init_instruction = 32'h064E002C;
                8'd62: init_instruction = 32'h06480000;
                8'd63: init_instruction = 32'h06300020;
                8'd64: init_instruction = 32'h06FF0000;
                8'd65: init_instruction = 32'h06300009;
                8'd66: init_instruction = 32'h06540000;
                8'd67: init_instruction = 32'h06310004;
                8'd68: init_instruction = 32'h06320003;
                8'd69: init_instruction = 32'h06400083;
                8'd70: init_instruction = 32'h06460025;
                8'd71: init_instruction = 32'h06600000;
                8'd72: init_instruction = 32'h06270000;
                8'd73: init_instruction = 32'h06500006;
                8'd74: init_instruction = 32'h06510000;
                8'd75: init_instruction = 32'h06520096;
                8'd76: init_instruction = 32'h06560008;
                8'd77: init_instruction = 32'h06570030;
                8'd78: init_instruction = 32'h06610000;
                8'd79: init_instruction = 32'h06620000;
                8'd80: init_instruction = 32'h06640000;
                8'd81: init_instruction = 32'h06650000;
                8'd82: init_instruction = 32'h066600A0;
                8'd83: init_instruction = 32'h06FF0001;
                8'd84: init_instruction = 32'h06220032;
                8'd85: init_instruction = 32'h06470014;
                8'd86: init_instruction = 32'h064900FF;
                8'd87: init_instruction = 32'h064A0000;
                8'd88: init_instruction = 32'h06FF0000;
                8'd89: init_instruction = 32'h067A000A;
                8'd90: init_instruction = 32'h067B0000;
                8'd91: init_instruction = 32'h06780021;
                8'd92: init_instruction = 32'h06FF0001;
                8'd93: init_instruction = 32'h06230034;
                8'd94: init_instruction = 32'h06420000;
                8'd95: init_instruction = 32'h064400FF;
                8'd96: init_instruction = 32'h06450026;
                8'd97: init_instruction = 32'h06460005;
                8'd98: init_instruction = 32'h06400040;
                8'd99: init_instruction = 32'h060E0006;
                8'd100: init_instruction = 32'h0620001A;
                8'd101: init_instruction = 32'h06430040;
                8'd102: init_instruction = 32'h06FF0000;
                8'd103: init_instruction = 32'h06340003;
                8'd104: init_instruction = 32'h06350044;
                8'd105: init_instruction = 32'h06FF0001;
                8'd106: init_instruction = 32'h06310004;
                8'd107: init_instruction = 32'h064B0009;
                8'd108: init_instruction = 32'h064C0005;
                8'd109: init_instruction = 32'h064D0004;
                8'd110: init_instruction = 32'h06FF0000;
                8'd111: init_instruction = 32'h06440000;
                8'd112: init_instruction = 32'h06450020;
                8'd113: init_instruction = 32'h06470008;
                8'd114: init_instruction = 32'h06480028;
                8'd115: init_instruction = 32'h06670000;
                8'd116: init_instruction = 32'h06700004;
                8'd117: init_instruction = 32'h06710001;
                8'd118: init_instruction = 32'h067200FE;
                8'd119: init_instruction = 32'h06760000;
                8'd120: init_instruction = 32'h06770000;
                8'd121: init_instruction = 32'h06FF0001;
                8'd122: init_instruction = 32'h060D0001;
                8'd123: init_instruction = 32'h06FF0000;
                8'd124: init_instruction = 32'h06800001;
                8'd125: init_instruction = 32'h060100F8;
                8'd126: init_instruction = 32'h06FF0001;
                8'd127: init_instruction = 32'h068E0001;
                8'd128: init_instruction = 32'h06000001;
                8'd129: init_instruction = 32'h06FF0000;
                8'd130: init_instruction = 32'h06800000;
                8'd131: init_instruction = 32'h070A0004;
                8'd132: init_instruction = 32'h17840000;
                8'd133: init_instruction = 32'h378400EF;
                8'd134: init_instruction = 32'h070B0001;
                8'd135: init_instruction = 32'h080100E8;
                8'd136: init_instruction = 32'hF87102FA;
                8'd137: init_instruction = 32'h09010001;
                8'd138: init_instruction = 32'h09000041;
                8'd139: init_instruction = 32'hB9130000;
                8'd140: init_instruction = 32'h090B0001;
                8'd141: init_instruction = 32'h09000000;
                8'd142: init_instruction = 32'h0A010002;
                8'd143: init_instruction = 32'h0A000001;
                8'd144: init_instruction = 32'hBA130000;
                8'd145: init_instruction = 32'h0A0B0001;
                8'd146: init_instruction = 32'h0A000000;
                8'd147: init_instruction = 32'h0B0100E8;
                8'd148: init_instruction = 32'h0B800001;
                8'd149: init_instruction = 32'h0BFF0001;
                8'd150: init_instruction = 32'h0B000000;
                8'd151: init_instruction = 32'hCB910000;
                8'd152: init_instruction = 32'h0B000001;
                8'd153: init_instruction = 32'h0BFF0000;
                8'd154: init_instruction = 32'h0B800000;
                8'd155: init_instruction = 32'h0B000002;
                8'd156: init_instruction = 32'hEB000000;
                default: init_instruction=32'hEB000000;
            endcase
        end
    endfunction
    wire [31:0] instruction=init_instruction(pc);
    wire [3:0] action=instruction[31:28];
    wire [3:0] stage=instruction[27:24];
    wire [7:0] register_index=instruction[23:16];
    wire [15:0] value=instruction[15:0];
    function [6:0] address_for;
        input [1:0] index;
        begin address_for=7'h2A+{5'b0,index}; end
    endfunction
    task distance;
        input [15:0] v;
        begin
            case(channel)
                0: distance0<=v; 1: distance1<=v;
                2: distance2<=v; 3: distance3<=v;
            endcase
        end
    endtask
    task diagnostic;
        input [7:0] e,o;
        input [15:0] r,v;
        begin
            case(channel)
                0: debug0<={e,o,r,v}; 1: debug1<={e,o,r,v};
                2: debug2<={e,o,r,v}; 3: debug3<={e,o,r,v};
            endcase
        end
    endtask
    task fail_channel;
        input [7:0] e;
        begin
            shutdown[channel]<=0; initialized[channel]<=0;
            retry[channel]<=RETRY_FRAMES; stale[channel]<=0;
            distance(16'hFFFF); diagnostic(e,command_stage,command_reg,read_data);
            state<=first_pass?INIT_NEXT:RUN_NEXT;
        end
    endtask
    vl53l0x_i2c_master #(.CLK_HZ(CLK_HZ),.I2C_HZ(I2C_HZ)) bus(
        .clk(clk),.rst_n(rst_n),.start(i2c_start),.dev_addr(command_addr),
        .reg_addr(command_reg),.reg_addr_8bit(1'b1),.rw_read(command_read),
        .write_len(command_len),.read_len(command_len),.write_data(command_data),
        .read_data(read_data),.busy(busy),.done(done),.ack_error(ack_error),
        .error_code(error_code),.sda(i2c_sda),.scl(i2c_scl));

    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            state<=RESET_WAIT; pc<=0; channel<=0; shutdown<=0; initialized<=0;
            first_pass<=1; init_command<=1; range_valid<=0; delay_count<=0;
            sample_count<=0; poll_count<=0; last_value<=0; spad_map<=0;
            spad_count<=0; aperture<=0; filter_index<=0; enabled_count<=0;
            run_step<=0; command_addr<=0; command_reg<=0; command_data<=0;
            command_read<=0; command_len<=1; command_stage<=0; i2c_start<=0;
            distance0<=16'hFFFF; distance1<=16'hFFFF;
            distance2<=16'hFFFF; distance3<=16'hFFFF; frame_valid<=0; sample_valid<=0;observation_valid<=0;
            debug0<=48'h7F0000C00000; debug1<=48'h7F0000C00000;
            debug2<=48'h7F0000C00000; debug3<=48'h7F0000C00000;
            for(j=0;j<4;j=j+1) begin stale[j]<=0; retry[j]<=0; stop_variable[j]<=0; end
        end else begin
            i2c_start<=0; frame_valid<=0; sample_valid<=0;observation_valid<=0;
            if(sample_count<SAMPLE_CYCLES-1) sample_count<=sample_count+1'b1;
            case(state)
                RESET_WAIT: begin
                    if(delay_count>=(CLK_HZ/1000)*RESET_WAIT_MS-1) begin
                        delay_count<=0; shutdown[0]<=1; state<=ENABLE_WAIT;
                    end else delay_count<=delay_count+1'b1;
                end
                RETRY_RESET: begin
                    if(delay_count>=(CLK_HZ/1000)*RESET_WAIT_MS-1) begin
                        delay_count<=0; shutdown[channel]<=1; state<=ENABLE_WAIT;
                    end else delay_count<=delay_count+1'b1;
                end
                ENABLE_WAIT: begin
                    if(delay_count>=(CLK_HZ/1000)*ENABLE_WAIT_MS-1) begin
                        delay_count<=0; pc<=0; poll_count<=0; state<=INIT_EXEC;
                    end else delay_count<=delay_count+1'b1;
                end
                INIT_EXEC: begin
                    if(action==FILTER) begin
                        filter_index<=0; enabled_count<=0; state<=MAP_FILTER;
                    end else if(action==FINISH) begin
                        initialized[channel]<=1; stale[channel]<=0;
                        diagnostic(0,8'h0B,16'h0000,16'h0002);
                        state<=first_pass?INIT_NEXT:RUN_NEXT;
                    end else if(!busy) begin
                        init_command<=1; command_reg<={8'b0,register_index};
                        command_stage<={4'b0,stage}; command_len<=1;
                        command_addr<=pc<=1?7'h29:address_for(channel);
                        command_read<=action==READ || action==ID || action==STOP_READ ||
                            action==SPAD_INFO || action==MAP_READ ||
                            action==POLL_SPAD || action==POLL_CAL;
                        case(action)
                            OR_LAST: command_data<=last_value|value;
                            AND_LAST: command_data<=last_value&value;
                            ADDRESS: command_data<={9'b0,address_for(channel)};
                            STOP_WRITE: command_data<={8'b0,stop_variable[channel]};
                            MAP_READ,MAP_WRITE: begin
                                command_len<=2;
                                case(register_index)
                                    8'hB0: command_data<={spad_map[7:0],spad_map[15:8]};
                                    8'hB2: command_data<={spad_map[23:16],spad_map[31:24]};
                                    default: command_data<={spad_map[39:32],spad_map[47:40]};
                                endcase
                            end
                            WRITE16: begin command_len<=2; command_data<=value; end
                            default: command_data<=value;
                        endcase
                        i2c_start<=1; state<=WAIT_BUS;
                    end
                end
                MAP_FILTER: begin
                    // RefGoodSpadMap bytes are little bit order, while I2C
                    // two-byte transfers are big endian. Preserve factory bits.
                    if((aperture && filter_index<12) || enabled_count>=spad_count)
                        spad_map[filter_index]<=0;
                    else if(spad_map[filter_index]) enabled_count<=enabled_count+1'b1;
                    if(filter_index==47) begin pc<=pc+1'b1; state<=INIT_EXEC; end
                    else filter_index<=filter_index+1'b1;
                end
                WAIT_BUS: if(done) begin
                    if(ack_error) fail_channel(error_code);
                    else if(init_command) begin
                        last_value<=read_data;
                        diagnostic(8'h7F,command_stage,command_reg,read_data);
                        if(action==ID && read_data[7:0]!=8'hEE) fail_channel(8'h0E);
                        else if(action==SPAD_INFO && (read_data[6:0]==0 || read_data[6:0]>48))
                            fail_channel(8'h0F);
                        else if((action==POLL_SPAD && read_data[7:0]==0) ||
                                (action==POLL_CAL && read_data[2:0]==0)) begin
                            if(poll_count==MAX_POLLS-1)
                                fail_channel(action==POLL_SPAD?8'h0C:8'h0D);
                            else begin
                                poll_count<=poll_count+1'b1; delay_count<=0; state<=POLL_WAIT;
                            end
                        end else begin
                            poll_count<=0; pc<=pc+1'b1; state<=INIT_EXEC;
                            if(action==STOP_READ) stop_variable[channel]<=read_data[7:0];
                            if(action==SPAD_INFO) begin
                                spad_count<=read_data[6:0]; aperture<=read_data[7];
                            end
                            if(action==MAP_READ) case(register_index)
                                8'hB0: spad_map[15:0]<={read_data[7:0],read_data[15:8]};
                                8'hB2: spad_map[31:16]<={read_data[7:0],read_data[15:8]};
                                default: spad_map[47:32]<={read_data[7:0],read_data[15:8]};
                            endcase
                        end
                    end else case(run_step)
                        0: begin
                            if(read_data[2:0]==0) begin
                                if(stale[channel]>=STALE_FRAMES-1) fail_channel(8'h10);
                                else begin stale[channel]<=stale[channel]+1'b1; state<=RUN_NEXT; end
                            end else begin stale[channel]<=0; run_step<=1; state<=RUN_EXEC; end
                        end
                        1: begin
                            // ST raw hardware status 11 means a completed range.
                            range_valid<=read_data[6:3]==4'd11;
                            if(read_data[6:3]!=4'd11) begin
                                distance(16'hFFFF); diagnostic(8'h11,8'h0D,16'h0014,read_data);
                            end
                            run_step<=2; state<=RUN_EXEC;
                        end
                        2: begin
                            observation_valid[channel]<=1;
                            if(range_valid) begin
                                if(read_data!=0 && read_data<=2000) begin
                                    distance(read_data); diagnostic(0,8'h0E,16'h001E,read_data);
                                    sample_valid[channel]<=1;
                                end else begin
                                    distance(16'hFFFF); diagnostic(8'h12,8'h0E,16'h001E,read_data);
                                end
                            end
                            run_step<=3; state<=RUN_EXEC;
                        end
                        default: state<=RUN_NEXT;
                    endcase
                end
                POLL_WAIT: begin
                    if(delay_count>=(CLK_HZ/1000)*POLL_WAIT_MS-1) begin
                        delay_count<=0; state<=INIT_EXEC;
                    end else delay_count<=delay_count+1'b1;
                end
                INIT_NEXT: begin
                    if(channel==3) begin first_pass<=0; channel<=0; state<=RUN_SELECT; end
                    else begin
                        channel<=channel+1'b1; shutdown[channel+1'b1]<=1;
                        delay_count<=0; state<=ENABLE_WAIT;
                    end
                end
                RUN_SELECT: begin
                    if(initialized[channel]) begin run_step<=0; state<=RUN_EXEC; end
                    else if(retry[channel]==0) begin
                        shutdown[channel]<=0; delay_count<=0; state<=RETRY_RESET;
                    end else begin retry[channel]<=retry[channel]-1'b1; state<=RUN_NEXT; end
                end
                RUN_EXEC: if(!busy) begin
                    init_command<=0; command_addr<=address_for(channel);
                    command_read<=run_step!=3; command_len<=run_step==2?2:1;
                    command_data<=1; command_stage<=8'h0C+{4'b0,run_step};
                    case(run_step)
                        0: command_reg<=16'h0013; 1: command_reg<=16'h0014;
                        2: command_reg<=16'h001E; default: command_reg<=16'h000B;
                    endcase
                    i2c_start<=1; state<=WAIT_BUS;
                end
                RUN_NEXT: begin
                    if(channel==3) begin frame_valid<=1; state<=FRAME_WAIT; end
                    else begin channel<=channel+1'b1; state<=RUN_SELECT; end
                end
                FRAME_WAIT: if(sample_count==SAMPLE_CYCLES-1) begin
                    sample_count<=0; channel<=0; state<=RUN_SELECT;
                end
                default: state<=RESET_WAIT;
            endcase
        end
    end
endmodule
