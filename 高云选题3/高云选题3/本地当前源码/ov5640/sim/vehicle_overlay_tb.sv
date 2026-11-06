`timescale 1ns/1ps
module vehicle_overlay_tb;
    reg clk=0;always #5 clk=~clk;
    reg rst_n=0,req=0;reg [11:0] x=0,y=0;reg [3:0] screen=6;
    reg [15:0] left_range=123,average_range=728;
    reg average_valid=1;
    reg signed [15:0] heading=450;
    wire [15:0] pixel;
    nav_renderer dut(.clk(clk),.rst_n(rst_n),.data_req(req),.x_addr(x),.y_addr(y),
        .control_remote(1'b0),.speed_edit(1'b0),.touch_run(1'b0),.configured_speed(16'd200),
        .slowdown_cm(16'd65),.edit_distance(1'b0),.tof_diagnostics(32'h10001101),
        .keypad_value(16'd0),.keypad_digits(3'd0),.keypad_error(1'b0),.video(16'h8410),.screen(screen),.tool(2'd0),.have_start(1'b0),.have_goal(1'b0),
        .sx(8'd0),.sy(8'd0),.gx(8'd0),.gy(8'd0),.waypoint_count(4'd0),
        .waypoint_x(64'd0),.waypoint_y(64'd0),.selected_x_cm(9'd0),.selected_y_cm(9'd0),
        .message(6'd0),.route_ready(1'b1),.planning(1'b0),.length_cm(18'd0),
        .map_addr(),.plan_addr(),.map_blocked(1'b0),.plan_blocked(1'b0),.route_pixel(1'b0),.pixel(pixel),
        .left_mm(left_range),.left_front_mm(16'd456),.right_front_mm(16'd1000),.right_mm(16'd1800),
        .front_average_mm(average_range),.front_valid(average_valid),.motion_status(4'd2),.motion_phase(4'd1),
        .commanded_speed(16'sd120),.commanded_steer(-16'sd18),.pose_heading(heading),.target_heading(16'sd520),
        .pose_x_q8(24'sd230400),.pose_y_q8(24'sd409600),.target_x_mm(16'sd930),.target_y_mm(16'sd1640),
        .target_distance_mm(16'd50),.profile_ready(1'b1),.profile_segments(4'd3),.profile_length_mm(16'd800),
        .profile_left_revs(16'd31),.profile_right_revs(16'd43),
        .profile_entry_heading(16'sd250),.profile_exit_heading(16'sd850),.profile_peak_steer(8'd24));
    integer out,xx,yy;
    task glyph;
        input integer xpos,ypos;input [7:0] character;
        begin
            @(negedge clk);x=xpos;y=ypos;req=1;
            repeat(2)@(negedge clk);
            if(dut.text_code!==dut.nav_ascii(character))
                $fatal(1,"telemetry glyph at %0d,%0d expected %c",xpos,ypos,character);
        end
    endtask
    task capture;
        input [255:0] filename;
        begin
            out=$fopen(filename,"wb");$fwrite(out,"P6\n800 480\n255\n");
            for(yy=0;yy<480;yy=yy+1)for(xx=0;xx<800;xx=xx+1)begin
                @(negedge clk);x=xx;y=yy;req=1;
                repeat(2)@(negedge clk);#1;
                $fwrite(out,"%c%c%c",{pixel[4:0],pixel[4:2]},
                    {pixel[10:5],pixel[10:9]},{pixel[15:11],pixel[15:13]});
            end
            $fclose(out);req=0;
        end
    endtask
    initial begin
        repeat(3)@(negedge clk);rst_n=1;
        if(dut.nav_ascii("S")==0 || dut.nav_ascii("s")==0 || dut.nav_ascii("v")==0)$fatal(1,"OSD units missing");
        repeat(500)@(negedge clk);
        if(dut.display_bcd[0]!==16'h0123 || dut.display_bcd[1]!==16'h0456 ||
           dut.display_bcd[4]!==16'h0728 || dut.display_bcd[6]!==16'h0018 ||
           !dut.display_negative[6] || dut.display_bcd[7]!==16'h0900 ||
           dut.display_bcd[8]!==16'h1600 || dut.display_bcd[9]!==16'h0450 ||
           dut.display_bcd[17]!==16'h0031 || dut.display_bcd[21]!==16'h0024)
            $fatal(1,"telemetry conversion: distance, signed steering, pose or wheel revolutions");
        // The visible average must be 072.8 cm, including its decimal glyph.
        glyph(328,120,"0");glyph(344,120,"7");glyph(360,120,"2");
        glyph(376,120,".");glyph(392,120,"8");
        glyph(624,120,"0");glyph(640,120,"1");
        glyph(624,164,"1");glyph(640,164,"1");
        glyph(624,208,"0");glyph(640,208,"0");
        glyph(624,252,"1");glyph(640,252,"0");
        left_range=16'hffff;average_valid=0;heading=-1799;
        repeat(500)@(negedge clk);
        if(!dut.display_invalid[0] || !dut.display_invalid[4] ||
           !dut.display_negative[9] || dut.display_bcd[9]!==16'h1799)
            $fatal(1,"telemetry refresh: invalid range or negative heading");
        glyph(328,120,"-");glyph(392,120,"-");
        left_range=0;average_range=9999;average_valid=1;
        repeat(500)@(negedge clk);
        if(dut.display_bcd[0]!==0 || dut.display_bcd[4]!==16'h9999 || dut.display_invalid[0])
            $fatal(1,"telemetry refresh: zero and maximum displayed range");
        left_range=123;average_range=728;heading=450;
        repeat(500)@(negedge clk);
        $display("PASS telemetry decimal display: live refresh, 072.8 cm raster, negative heading, zero and invalid ranges");
        capture("sim/menu_detect.ppm");screen=9;capture("sim/menu_navigation.ppm");
        $display("PASS vehicle overlay raster: distance and navigation screens with complete units/segment font");$finish;
    end
endmodule
