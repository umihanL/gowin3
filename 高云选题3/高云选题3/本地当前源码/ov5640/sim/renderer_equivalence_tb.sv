`timescale 1ns/1ps
module renderer_equivalence_tb;
reg clk=0;always #5 clk=~clk;reg rst_n=0,req=1;reg [11:0] x=0,y=0;reg [3:0] screen=0;reg [1:0] page=0;reg remote=0;reg [15:0] left_range=123,average_range=728;reg average_valid=1;reg signed [15:0] heading=450;wire [15:0] pixel,reference_pixel;
    nav_renderer dut(.clk(clk),.rst_n(rst_n),.data_req(req),.x_addr(x),.y_addr(y),
        .control_remote(remote),.live_page(page),.preview(16'habcd),.binary_threshold(8'd96),.speed_edit(1'b0),.touch_run(1'b0),.configured_speed(16'd200),
        .bypass_cm(16'd65),.brake_speed(16'd200),.edit_parameter(2'd0),.barrier_pixel(1'b0),.barrier_pending(1'b0),.slowdown_cm(16'd65),.edit_distance(1'b0),.tof_diagnostics(32'h10001101),
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
    nav_renderer_reference reference(.clk(clk),.rst_n(rst_n),.data_req(req),.x_addr(x),.y_addr(y),
        .control_remote(remote),.live_page(page),.preview(16'habcd),.binary_threshold(8'd96),.speed_edit(1'b0),.touch_run(1'b0),.configured_speed(16'd200),
        .bypass_cm(16'd65),.brake_speed(16'd200),.edit_parameter(2'd0),.barrier_pixel(1'b0),.barrier_pending(1'b0),.slowdown_cm(16'd65),.edit_distance(1'b0),.tof_diagnostics(32'h10001101),
        .keypad_value(16'd0),.keypad_digits(3'd0),.keypad_error(1'b0),.video(16'h8410),.screen(screen),.tool(2'd0),.have_start(1'b0),.have_goal(1'b0),
        .sx(8'd0),.sy(8'd0),.gx(8'd0),.gy(8'd0),.waypoint_count(4'd0),
        .waypoint_x(64'd0),.waypoint_y(64'd0),.selected_x_cm(9'd0),.selected_y_cm(9'd0),
        .message(6'd0),.route_ready(1'b1),.planning(1'b0),.length_cm(18'd0),
        .map_addr(),.plan_addr(),.map_blocked(1'b0),.plan_blocked(1'b0),.route_pixel(1'b0),.pixel(reference_pixel),
        .left_mm(left_range),.left_front_mm(16'd456),.right_front_mm(16'd1000),.right_mm(16'd1800),
        .front_average_mm(average_range),.front_valid(average_valid),.motion_status(4'd2),.motion_phase(4'd1),
        .commanded_speed(16'sd120),.commanded_steer(-16'sd18),.pose_heading(heading),.target_heading(16'sd520),
        .pose_x_q8(24'sd230400),.pose_y_q8(24'sd409600),.target_x_mm(16'sd930),.target_y_mm(16'sd1640),
        .target_distance_mm(16'd50),.profile_ready(1'b1),.profile_segments(4'd3),.profile_length_mm(16'd800),
        .profile_left_revs(16'd31),.profile_right_revs(16'd43),
        .profile_entry_heading(16'sd250),.profile_exit_heading(16'sd850),.profile_peak_steer(8'd24));

integer xx,yy,ss,pp,rr;
initial begin
 repeat(4)@(negedge clk);rst_n=1;repeat(600)@(negedge clk);
 for(ss=0;ss<16;ss=ss+1)begin
 screen=ss;
 for(rr=0;rr<(ss==2?2:1);rr=rr+1)begin remote=rr;
 for(pp=0;pp<(ss==1?3:1);pp=pp+1)begin page=pp;
 for(yy=0;yy<480;yy=yy+4)for(xx=0;xx<800;xx=xx+4)begin
 @(negedge clk);x=xx;y=yy;@(negedge clk);#1;
 if(dut.text_code!==reference.text_code || (dut.text_code!=0 && (dut.font_x!==reference.font_x || dut.font_y!==reference.font_y || dut.text_color!==reference.text_color)))
 $fatal(1,"renderer glyph differs screen=%0d page=%0d remote=%0d x=%0d y=%0d got %0d/%h expected %0d/%h",ss,pp,rr,xx,yy,dut.text_code,dut.text_color,reference.text_code,reference.text_color);
 end
 end end end
 $display("PASS renderer equivalence: every menu and preview, parallel glyph selection matches previous renderer");$finish;
end
endmodule
