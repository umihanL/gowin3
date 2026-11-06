// STYLE 0 ports the supplied STM32 PS2_Init sequence once at startup:
// short poll x3, 43 enter, 44 analog/unlocked, 43 exit, nine-byte reads.
// STYLE 1 ports the supplied C51 loop: only nine-byte 01/42 reads.
// No speed scanning or repeated configuration, for a controlled comparison.
module ps2_reference_controller #(
    parameter integer CLK_HZ=50000000,
    parameter integer STYLE=0,
    parameter integer POWERUP_MS=500,
    parameter integer POLL_MS=(STYLE==0)?20:500,
    parameter integer CONFIG_GAP_US=100
)(
    input clk,rst_n,
    output ps2_cs_n,ps2_clk,ps2_mosi,input ps2_miso,
    output reg sample_valid,analog_valid,
    output reg [15:0] buttons_raw,
    output reg [7:0] left_x,right_y,mode_id,header,error_code,
    output reg [2:0] debug_profile,
    output reg [1:0] debug_levels,
    output reg [15:0] debug_edges,
    output reg [71:0] debug_rx
);
    localparam integer BOOT_CYCLES=(CLK_HZ/1000)*POWERUP_MS;
    localparam integer POLL_CYCLES=(CLK_HZ/1000)*POLL_MS;
    localparam integer CONFIG_CYCLES=(CLK_HZ/1000000)*CONFIG_GAP_US;
    localparam integer MAX_A=(BOOT_CYCLES>POLL_CYCLES)?BOOT_CYCLES:POLL_CYCLES;
    localparam integer MAX_CYCLES=(MAX_A>CONFIG_CYCLES)?MAX_A:CONFIG_CYCLES;
    localparam integer WAIT_W=(MAX_CYCLES<2)?1:$clog2(MAX_CYCLES);
    localparam [1:0] WAIT_DELAY=0,LAUNCH=1,WAIT_DONE=2;
    reg [1:0] state;
    reg [WAIT_W-1:0] wait_count;
    reg [2:0] step;
    reg start;
    reg [71:0] command;
    wire [4:0] command_length=(STYLE==0 && step<3)?5'd5:5'd9;
    wire [71:0] response;
    wire transfer_done;
    wire [1:0] dat_levels;
    wire [15:0] dat_edges;
    wire [7:0] response_id=response[15:8];
    wire id_ok=(response_id==8'h41)||(response_id==8'h73)||(response_id==8'h79);
    wire header_ok=response[23:16]==8'h5a;
    always @(*) begin
        command={56'h0,8'h42,8'h01};
        case(step)
            3: command={40'h0,8'h01,8'h00,8'h43,8'h01};
            4: command={32'h0,8'hee,8'h01,8'h00,8'h44,8'h01};
            5: command={8'h5a,8'h5a,8'h5a,8'h5a,8'h5a,8'h00,8'h00,8'h43,8'h01};
            default: begin end
        endcase
    end
    ps2_reference_serial #(.CLK_HZ(CLK_HZ),.STYLE(STYLE)) u_transport(
        .clk(clk),.rst_n(rst_n),.start(start),.long_setup(step<6),
        .tx_frame(command),.tx_length(command_length),.rx_frame(response),
        .done(transfer_done),.busy(),.dat_levels(dat_levels),.dat_edges(dat_edges),
        .ps2_cs_n(ps2_cs_n),.ps2_clk(ps2_clk),.ps2_mosi(ps2_mosi),.ps2_miso(ps2_miso));
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            state<=WAIT_DELAY; wait_count<=(BOOT_CYCLES>0)?BOOT_CYCLES-1:0;
            step<=(STYLE==0)?0:6; start<=0;
            sample_valid<=0; analog_valid<=0; buttons_raw<=16'hffff;
            left_x<=128; right_y<=128; mode_id<=0; header<=0; error_code<=8'h7f;
            // Fixed reference modes, distinct from old profiles 0..5.
            debug_profile<=(STYLE==0)?3'd6:3'd7;
            debug_levels<=0; debug_edges<=0; debug_rx<=0;
        end else begin
            start<=0; sample_valid<=0;
            if(wait_count!=0) wait_count<=wait_count-1'b1;
            case(state)
                WAIT_DELAY: if(wait_count==0) state<=LAUNCH;
                LAUNCH: begin start<=1; state<=WAIT_DONE; end
                WAIT_DONE: if(transfer_done) begin
                    state<=WAIT_DELAY;
                    if(step<6) begin
                        step<=step+1'b1;
                        wait_count<=(CONFIG_CYCLES>0)?CONFIG_CYCLES-1:0;
                    end else begin
                        wait_count<=(POLL_CYCLES>0)?POLL_CYCLES-1:0;
                        mode_id<=response_id; header<=response[23:16];
                        debug_levels<=dat_levels; debug_edges<=dat_edges; debug_rx<=response;
                        if(id_ok && header_ok) begin
                            sample_valid<=1; analog_valid<=response_id!=8'h41;
                            error_code<=0; buttons_raw<={response[39:32],response[31:24]};
                            if(response_id!=8'h41) begin
                                right_y<=response[55:48]; left_x<=response[63:56];
                            end else begin right_y<=128; left_x<=128; end
                        end else error_code<=id_ok?8'h02:8'h01;
                    end
                end
                default: state<=WAIT_DELAY;
            endcase
        end
    end
endmodule
