`timescale 1ns/1ps
module imu_ui_tb #(parameter AXIS_VECTOR=0);
    reg clk=0;always #5 clk=~clk;
    reg rst_n=0,fb=0,req=1,online=1,calibrated=1;
    reg [11:0] x=0,y=0;wire [15:0] pixel;
    reg available=1;reg [2:0] status=2;reg [1:0] cal_state=3;
    reg key_n=1,zero_n=1;
    wire board_rst,zero;
    board_reset #(.RELEASE_CYCLES(10)) br(.clk(clk),.key_n(key_n),.reset_n(board_rst));
    imu_zero_button #(.DEBOUNCE_CYCLES(8)) zb(.clk(clk),.rst_n(rst_n),.key_n(zero_n),.pressed(zero));
    attitude_overlay #(.AXIS_VECTOR(AXIS_VECTOR)) dut(.clk(clk),.rst_n(rst_n),.frame_begin(fb),.data_req(req),.x_addr(x),.y_addr(y),
        .background(16'h1234),.online(online),.calibrated(calibrated),
        .available(available),.sensor_status(status),.cal_state(cal_state),
        .error_reg(8'h5e),.error_expected(8'h53),.error_actual(8'h43),.error_mask(8'h38),.error_step(8'h08),
        .roll(-16'sd301),.pitch(16'sd900),.yaw(16'sd1799),.pixel(pixel));
    integer presses=0;
    always @(posedge clk)if(zero)presses=presses+1;
    task frame;
        begin @(negedge clk);fb=1;@(negedge clk);fb=0;repeat(15)@(negedge clk);end
    endtask
    task position;
        input integer col,row;
        begin x=556+col*12;y=10+row*16;#1;end
    endtask
    initial begin
        repeat(4)@(negedge clk);rst_n=1;frame;
        if(dut.bcd[0]!==16'h0301 || dut.bcd[1]!==16'h0900 || dut.bcd[2]!==16'h1799)$fatal(1,"decimal conversion");
        position(6,1);if(dut.character!="-")$fatal(1,"negative sign");
        position(0,1);if(dut.character!="R")$fatal(1,"Roll label");
        position(0,2);if(dut.character!=(AXIS_VECTOR ? "R" : "P"))$fatal(1,"Pitch/RotY label");
        position(0,3);if(dut.character!=(AXIS_VECTOR ? "R" : "Y"))$fatal(1,"Yaw/RotZ label");
        if(AXIS_VECTOR) begin
            position(3,1);if(dut.character!="X" || dut.bitmap==0)$fatal(1,"RotX glyph");
            position(3,2);if(dut.character!="Y")$fatal(1,"RotY label");
            position(3,3);if(dut.character!="Z")$fatal(1,"RotZ label");
        end
        x=100;y=400;#1;if(pixel!==16'h1234)$fatal(1,"overlay outside its box");
        online=0;status=5;frame;position(5,0);if(dut.character!="N")$fatal(1,"missing-data status");
        position(7,1);if(dut.character!="-")$fatal(1,"offline value must be hidden");
        online=1;status=2;calibrated=0;cal_state=1;frame;
        position(7,1);if(dut.character!="0")$fatal(1,"live angles hidden before zero");
        position(5,0);if(dut.character!="M")$fatal(1,"motion status");
        // Entire old rectangular background and its border must be transparent.
        x=552;y=8;#1;if(pixel!==16'h1234)$fatal(1,"old opaque border remains");
        position(4,1);if(pixel!==16'h1234)$fatal(1,"blank glyph must pass through camera");
        position(0,1);if(pixel!==16'h07ff)$fatal(1,"Roll must be yellow on BGR panel");
        position(0,2);if(pixel!==16'hffe0)$fatal(1,"Pitch must be cyan");
        position(0,3);if(pixel!==16'h041f)$fatal(1,"Yaw must be orange");
        online=0;status=4;frame;
        position(0,1);if(dut.character!="R")$fatal(1,"REG label missing");
        position(5,1);if(dut.character!="5")$fatal(1,"register high hex digit");
        position(6,1);if(dut.character!="E")$fatal(1,"register low hex digit");
        position(6,2);if(dut.character!="3")$fatal(1,"expected byte formatting");
        position(5,3);if(dut.character!="4")$fatal(1,"actual byte formatting");
        position(8,4);if(dut.character!="8")$fatal(1,"configuration step formatting");
        key_n=0;#1;if(board_rst!==0)$fatal(1,"S4 immediate reset");
        key_n=1;repeat(5)@(negedge clk);key_n=0;@(negedge clk);key_n=1;
        repeat(9)@(negedge clk);if(board_rst)$fatal(1,"bounce release too early");
        repeat(8)@(negedge clk);if(!board_rst)$fatal(1,"reset release");
        zero_n=0;repeat(3)@(negedge clk);zero_n=1;repeat(12)@(negedge clk);
        if(presses)$fatal(1,"zero bounce accepted");
        zero_n=0;repeat(30)@(negedge clk);if(presses!=1)$fatal(1,"zero hold must be one pulse");
        $display("PASS imu_ui: transparent background, colored labels, pre-zero live angles, error register/expected/actual/mask/step, decimal formatting, S4/S0");$finish;
    end
endmodule
