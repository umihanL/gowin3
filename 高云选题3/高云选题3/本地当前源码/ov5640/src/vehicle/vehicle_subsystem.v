// Imported MOTOR hardware runs in the existing 33 MHz display/control domain.
// All time-based MOTOR parameters are adjusted; fixed 2000-count PWM is 16.5 kHz.
`include "src/vehicle/vehicle_config.vh"
module vehicle_subsystem #(parameter CLK_HZ=33000000,HOST_BAUD=115200)(
    input zebra_update,zebra_detected,
    input zebra_toggle,output zebra_enable,zebra_can_change,
    output host_point_select,host_point_start,host_point_cancel,output [3:0] host_point_id,
    input host_point_select_ready,host_point_route_ready,host_point_planning,
    input [3:0] selected_point,input [2:0] host_point_error,
    input clk,rst_n,input [3:0] mode,input navigation_request,route_valid,
    input start_heading_valid,input signed [15:0] start_heading,
    input map_start_valid,input [7:0] map_start_x,map_start_y,input [31:0] map_start_edit_generation,
    input control_remote,control_host,navigation_avoidance,input host_rx,output host_tx,output water_pump,
    input host_route_page,input [7:0] host_route_result,host_route_status,
    output host_route_command,output [7:0] host_route_opcode,output [31:0] host_route_arguments,
    input [15:0] configured_speed,slowdown_cm,bypass_cm,brake_speed,
    input [10:0] servo_center_tenths,
    input touch_run,input signed [15:0] touch_speed,touch_steer,
    output speed_edit,speed_increase,speed_decrease,
    input [11:0] route_count,output [10:0] route_index,
    input [16:0] route_x_q8,route_y_q8,input route_lock,
    input [31:0] map_signature,input [15:0] map_generation,input map_route_valid,map_planning,
    output [10:0] map_route_index,input [16:0] map_route_x_q8,map_route_y_q8,
    input [15:0] map_barrier_generation,input [3:0] map_barrier_valid,
    input [35:0] map_barrier_x0,map_barrier_y0,map_barrier_x1,map_barrier_y1,
    input imu_available,imu_update,imu_restart,imu_rezero,input signed [15:0] imu_yaw,
    input encoder_A,encoder_B,encoder2_A,encoder2_B,
    output [1:0] dir,dir2,output pwm_out,pwm_out2,servo_pwm_out,
    output ps2_cs_n,ps2_mosi,ps2_clk,input ps2_miso,
    inout tof_sda,tof_scl,output tof_xs1,tof_xs2,tof_xs3,tof_xs4,output tx,
    output [15:0] left_mm,left_front_mm,right_front_mm,right_mm,front_average_mm,
    output front_valid,output [31:0] tof_diagnostics,output [3:0] status,phase,
    output signed [15:0] commanded_speed,commanded_steer,actual_speed1,actual_speed2,
    // Export per-wheel targets after the existing rear differential for UART.
    output signed [15:0] target_speed1,target_speed2,
    output signed [23:0] pose_x_q8,pose_y_q8,
    output signed [15:0] pose_heading,target_x_mm,target_y_mm,target_heading,
    output [15:0] target_distance_mm,
    output signed [31:0] relative1,relative2,
    output profile_ready,output [3:0] profile_segments,
    output [15:0] profile_length_mm,profile_left_revs,profile_right_revs,
    output signed [15:0] profile_entry_heading,profile_exit_heading,
    output [7:0] profile_peak_steer
);
    wire [47:0] tof_debug0,tof_debug1,tof_debug2,tof_debug3;
    assign tof_diagnostics={tof_debug3[47:40],tof_debug2[47:40],tof_debug1[47:40],tof_debug0[47:40]};
    wire range_frame,sample_valid,analog_valid,remote_link,remote_run;
    wire [3:0] range_samples,range_observations;
    wire [15:0] buttons;
    wire [7:0] left_x,right_y;
    wire signed [15:0] remote_speed,remote_steer,remote_axis_steer;
    assign remote_steer=-remote_axis_steer;
    wire drive_enabled,tick,brake_enable;
    wire host_selected=control_host===1'b1;
    wire [3:0] motion_mode=host_selected && mode==9 && !host_nav_session?4'd0:mode;
    wire host_run,host_nav_session,host_link_ok;wire [15:0] host_limit;
    wire extension_owned,extension_run,extension_brake;
    wire signed [15:0] extension_speed,extension_steer;
    wire [7:0] extension_mode,extension_phase,extension_flags,extension_task;
    wire [15:0] extension_remaining;
    wire extension_command;wire [7:0] extension_opcode,extension_sequence,extension_result;
    wire [31:0] extension_arguments;
    wire pump_command;wire [7:0] pump_result;wire [31:0] pump_status;
    wire jog_command,jog_stop,jog_active,jog_run,jog_resume_blocked,jog_motion_ready;
    wire [7:0] jog_result;wire [31:0] jog_status;
    wire signed [15:0] jog_speed,jog_steer;
    reg signed [15:0] applied_steer;
    wire pump_scope=host_selected && (mode==2 || mode==5 || mode==9 || host_route_page===1'b1);
    wire extension_event;wire [7:0] extension_event_code,extension_event_task,extension_event_status;
    wire [7:0] status_phase=host_route_status>=7?host_route_status:extension_phase;
    wire [31:0] extension_status_word={extension_remaining,extension_flags,status_phase};
    wire host_brake=jog_run?1'b0:extension_owned?extension_brake:host_selected && (mode==2 || mode==9) &&
        (!host_run || !host_link_ok || host_limit==0);
    assign host_route_command=extension_command;
    assign host_route_opcode=extension_opcode;
    assign host_route_arguments=extension_arguments;
    reg [15:0] host_drive_speed;
    wire pose_sample_valid;
    wire inertial_healthy;
    wire signed [31:0] odometry_travel_q8;
    wire [10:0] motion_index,profile_index;
    reg signed [23:0] map_committed_x,map_committed_y;
    reg signed [15:0] map_committed_heading;
    reg map_pose_committed;
    // The odometer updates heading before its XY stage. Publish only a whole
    // completed encoder/IMU sample, never a mixture of two internal stages.
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin map_committed_x<=0;map_committed_y<=0;map_committed_heading<=0;map_pose_committed<=0;end
        else if(mode!=9 || !navigation_request || !route_valid)map_pose_committed<=0;
        else if(pose_sample_valid)begin
            map_committed_x<=pose_x_q8;map_committed_y<=pose_y_q8;map_committed_heading<=pose_heading;map_pose_committed<=1;
        end
    end
    vehicle_host_link #(.CLK_HZ(CLK_HZ),.BAUD(HOST_BAUD)) host(.clk(clk),.rst_n(rst_n),.rx(host_rx),.tx(host_tx),
        .selected(host_selected),.drive_page(mode==2),.navigation_page(mode==9 && navigation_request),
        .navigation_ready(route_valid && profile_ready),
        .planning_page(mode==5),.point_select_ready(host_point_select_ready),.start_heading_valid(start_heading_valid),
        .point_route_ready(host_point_route_ready && profile_ready),
        .map_start_valid(map_start_valid),.map_start_x(map_start_x),.map_start_y(map_start_y),.map_start_heading(start_heading),.map_start_edit_generation(map_start_edit_generation),
        .point_planning(host_point_planning || (route_valid && !profile_ready)),
        .selected_point(selected_point),.point_error(host_point_error),
        .point_select(host_point_select),.point_start(host_point_start),.point_cancel(host_point_cancel),.point_id(host_point_id),
        .motion_status(status),.motion_phase(phase),
        .map_telemetry_enabled(1'b1),.map_mode(mode),.map_imu_available(imu_available),
        .map_pose_valid(map_pose_committed && inertial_healthy && mode==9 && navigation_request && route_valid && (status==2 || status==3)),
        .map_pose_x_q8(map_committed_x),.map_pose_y_q8(map_committed_y),.map_heading(map_committed_heading),
        .map_signature(map_signature),.map_generation(map_generation),.map_route_valid(map_route_valid),.map_planning(map_planning),
        .map_route_count(route_count),.map_route_index(map_route_index),.map_route_x_q8(map_route_x_q8),.map_route_y_q8(map_route_y_q8),
        .map_navigation_target_index(motion_index),
        .map_barrier_generation(map_barrier_generation),.map_barrier_valid(map_barrier_valid),
        .map_barrier_x0(map_barrier_x0),.map_barrier_y0(map_barrier_y0),.map_barrier_x1(map_barrier_x1),.map_barrier_y1(map_barrier_y1),
        .event_valid(1'b0),.event_code(8'd0),.event_value(16'd0),
        .route_page(host_route_page),.extension_result(extension_opcode>=8'h20 && extension_opcode<=8'h26 ? host_route_result:extension_result),
        .traffic_start_blocked(extension_mode==3 && !zebra_enable),
        .extension_status(extension_status_word),.extension_capabilities({extension_mode,8'h7f,8'd2,8'd0}),
        .pump_supported(1'b1),.pump_result(pump_result),.pump_status(pump_status),.pump_command_valid(pump_command),
        .retained_jog_supported(1'b1),.retained_jog_resume_blocked(jog_resume_blocked),.retained_jog_active(jog_active || jog_command),
        .retained_jog_result(jog_result),.retained_jog_status(jog_status),
        .retained_jog_command_valid(jog_command),.retained_jog_stop(jog_stop),
        .extension_event_valid(extension_event),.extension_event_code(extension_event_code),
        .extension_event_task(extension_event_task),.extension_event_status(extension_event_status),
        .extension_command_valid(extension_command),.extension_opcode(extension_opcode),
        .extension_sequence(extension_sequence),.extension_arguments(extension_arguments),
        .run(host_run),.nav_session(host_nav_session),.speed_limit(host_limit),.link_ok(host_link_ok),.task_id());
    vehicle_pump #(.CLK_HZ(CLK_HZ)) pump(.clk(clk),.rst_n(rst_n),.scope(pump_scope),.link_ok(host_link_ok),
        .movement_requested(drive_enabled || host_run || extension_run || jog_active || jog_command),.actual1(actual_speed1),.actual2(actual_speed2),
        .command_valid(pump_command),.opcode(extension_opcode),.sequence_id(extension_sequence),.arguments(extension_arguments),
        .result(pump_result),.status_word(pump_status),.gate_high(water_pump));
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)host_drive_speed<=0;
        else if(!host_selected || !host_run || !host_link_ok || mode!=2)host_drive_speed<=0;
        else if(tick)begin
            if(host_drive_speed+2<host_limit)host_drive_speed<=host_drive_speed+2;
            else if(host_drive_speed>host_limit+2)host_drive_speed<=host_drive_speed-2;
            else host_drive_speed<=host_limit;
        end
    end
    wire signed [31:0] encoder_position1,encoder_position2;
    // A retained jog belongs to the paused map-navigation session. The
    // extension task_mode selects the separate manual distance/velocity
    // controller (drive_page==mode 2); dialogue/vision legitimately leave
    // that controller in mode 1. Do not reject an otherwise valid paused
    // navigation jog because the unrelated manual controller is not mode 2.
    vehicle_retained_jog jog(.clk(clk),.rst_n(rst_n),.tick(tick),
        .scope(host_selected && mode==9 && navigation_request && host_nav_session && !host_run &&
            jog_motion_ready),.link_ok(host_link_ok),
        .pose_valid(map_pose_committed && map_route_valid && !map_planning),
        .inertial_healthy(inertial_healthy),.travel_q8(odometry_travel_q8),.pump_low(pump_status[0]===1'b0),
        .generation(map_generation),.route_cursor(motion_index),
        .command_valid(jog_command),.stop(jog_stop),.sequence_id(extension_sequence),.arguments(extension_arguments),
        .encoder1(encoder_position1),.encoder2(encoder_position2),.actual1(actual_speed1),.actual2(actual_speed2),
        .heading(pose_heading),.applied_steer(applied_steer),.result(jog_result),.status_word(jog_status),
        .active(jog_active),.run(jog_run),.resume_blocked(jog_resume_blocked),.speed(jog_speed),.steer(jog_steer));
    vehicle_extension extension(.clk(clk),.rst_n(rst_n),.tick(tick),.scope(host_selected && (mode==2 || mode==5 || mode==9 || host_route_page===1'b1)),.link_ok(host_link_ok),
        .drive_page(mode==2),
        .zebra_toggle(zebra_toggle),.zebra_enable(zebra_enable),.zebra_can_change(zebra_can_change),
        .command_valid(extension_command),.opcode(extension_opcode),.sequence_id(extension_sequence),.arguments(extension_arguments),.result(extension_result),
        .encoder1(encoder_position1),.encoder2(encoder_position2),.actual1(actual_speed1),.actual2(actual_speed2),
        .front_valid(front_valid),.front_mm(left_front_mm<right_front_mm?left_front_mm:right_front_mm),.zebra_update(zebra_update),.zebra_detected(zebra_detected),
        .legacy_run(host_run),.legacy_speed(host_limit),.owned(extension_owned),.run(extension_run),.brake(extension_brake),
        .speed(extension_speed),.steer(extension_steer),.task_mode(extension_mode),.phase(extension_phase),.task_id(extension_task),
        .flags(extension_flags),.remaining_mm(extension_remaining),.event_valid(extension_event),.event_code(extension_event_code),
        .event_task(extension_event_task),.event_status(extension_event_status));
    wire profile_busy;
    wire signed [15:0] execution_tangent;
    wire signed [1:0] execution_curve;
    wire execution_exit_valid;wire signed [15:0] execution_exit_heading;
    wire signed [1:0] execution_upcoming_turn;wire [7:0] execution_entry_mm;
    wire execution_entry_valid;wire signed [15:0] execution_entry_x_mm,execution_entry_y_mm;
    wire [10:0] execution_entry_index;
    assign route_index=profile_busy?profile_index:motion_index;
    vehicle_route_profile profile(.clk(clk),.rst_n(rst_n),.route_valid(route_valid),.route_count(route_count),
        .execution_index(motion_index),.execution_tangent(execution_tangent),.execution_curve(execution_curve),
        .execution_exit_valid(execution_exit_valid),.execution_exit_heading(execution_exit_heading),
        .execution_upcoming_turn(execution_upcoming_turn),.execution_entry_mm(execution_entry_mm),
        .execution_entry_valid(execution_entry_valid),.execution_entry_x_mm(execution_entry_x_mm),
        .execution_entry_y_mm(execution_entry_y_mm),.execution_entry_index(execution_entry_index),
        .route_index(profile_index),.route_x_q8(route_x_q8),.route_y_q8(route_y_q8),.route_lock(route_lock),
        .selected_segment(phase),.busy(profile_busy),.ready(profile_ready),.segment_count(profile_segments),
        .length_mm(profile_length_mm),.left_revs_tenths(profile_left_revs),.right_revs_tenths(profile_right_revs),
        .entry_heading(profile_entry_heading),.exit_heading(profile_exit_heading),.peak_steer(profile_peak_steer),
        .left_counts(),.right_counts());
    // Re-enter remote mode disarmed; old START state cannot carry across pages.
    wire remote_reset_n=rst_n && mode==2 && control_remote;
    ps2_reference_controller #(.CLK_HZ(CLK_HZ)) receiver(.clk(clk),.rst_n(rst_n),
        .ps2_cs_n(ps2_cs_n),.ps2_clk(ps2_clk),.ps2_mosi(ps2_mosi),.ps2_miso(ps2_miso),
        .sample_valid(sample_valid),.analog_valid(analog_valid),.buttons_raw(buttons),
        .left_x(left_x),.right_y(right_y),.mode_id(),.header(),.error_code(),
        .debug_profile(),.debug_levels(),.debug_edges(),.debug_rx());
    ps2_test_control #(.CLK_HZ(CLK_HZ),.RUNTIME_SPEED(1),
        .LEFT_STEER_DEG(`VEHICLE_LEFT_STEER_DEG),.RIGHT_STEER_DEG(`VEHICLE_RIGHT_STEER_DEG)) remote(.clk(clk),.rst_n(remote_reset_n),
        .speed_limit(configured_speed),.speed_edit(speed_edit),.speed_increase(speed_increase),.speed_decrease(speed_decrease),
        .sample_valid(sample_valid && analog_valid),.analog_valid(analog_valid),
        .buttons_raw(buttons),.left_x(left_x),.right_y(right_y),.steer_deg(remote_axis_steer),
        .speed_mm_s(remote_speed),.link_ok(remote_link),.run_enabled(remote_run),.angle_locked(),.manual_mode());
    vl53l1x_array_controller #(.CLK_HZ(CLK_HZ)) ranging(.clk(clk),.rst_n(rst_n),
        .xs1(tof_xs1),.xs2(tof_xs2),.xs3(tof_xs3),.xs4(tof_xs4),
        .i2c_sda(tof_sda),.i2c_scl(tof_scl),.distance0(left_mm),.distance1(left_front_mm),
        .distance2(right_front_mm),.distance3(right_mm),.frame_valid(range_frame),.sample_valid(range_samples),
        .observation_valid(range_observations),
        .debug0(tof_debug0),.debug1(tof_debug1),.debug2(tof_debug2),.debug3(tof_debug3));
    vehicle_motion #(.CLK_HZ(CLK_HZ)) motion(.route_curve(execution_curve),
        .route_start_heading_valid(start_heading_valid),.route_start_heading(start_heading),
        .zebra_update(zebra_update),.zebra_detected(zebra_detected),.clk(clk),.rst_n(rst_n),.navigation_avoidance(navigation_avoidance),.mode(motion_mode),
        // Shared screen speed belongs to manual/remote and standalone obstacle
        // tasks. Navigation retains its own cruise/host command authority.
        .configured_speed(mode==9?(host_selected?host_limit:16'd200):configured_speed),.slowdown_cm(slowdown_cm),.profile_ready(profile_ready),
        .bypass_cm(bypass_cm),.brake_speed(brake_speed),.range_observations(range_observations),
        .navigation_request(navigation_request && (!host_selected || host_nav_session)),.route_valid(route_valid),.route_count(route_count),
        .route_index(motion_index),.route_x_q8(route_x_q8),.route_y_q8(route_y_q8),.route_lock(route_lock),
        .encoder1(encoder_position1),.encoder2(encoder_position2),
        .imu_available(imu_available),.imu_update(imu_update),.imu_restart(imu_restart),.imu_rezero(imu_rezero),.imu_yaw(imu_yaw),
        .range_frame(range_frame),.range_samples(range_samples),.range_diagnostics(tof_diagnostics),
        .left_mm(left_mm),.left_front_mm(left_front_mm),
        .right_front_mm(right_front_mm),.right_mm(right_mm),
        .remote_link(host_selected?host_link_ok:control_remote?remote_link:1'b1),
        .remote_run(extension_owned?extension_run:host_selected?host_run:control_remote?remote_run:touch_run),
        .remote_speed(extension_owned?extension_speed:host_selected?$signed(host_drive_speed):control_remote?remote_speed:touch_speed),
        .remote_steer(extension_owned?extension_steer:host_selected?16'sd0:control_remote?remote_steer:touch_steer),
        .host_pause(host_selected && (!host_run || !host_link_ok)),
        .retained_jog_active(jog_active),.retained_jog_run(jog_run),.retained_jog_speed(jog_speed),.retained_jog_steer(jog_steer),
        .retained_jog_ready(jog_motion_ready),
        .drive_enabled(drive_enabled),.brake_enable(brake_enable),.speed_mm_s(commanded_speed),.steer_deg(commanded_steer),
        .status(status),.phase(phase),.front_average_mm(front_average_mm),.front_valid(front_valid),
        .pose_x_q8(pose_x_q8),.pose_y_q8(pose_y_q8),.pose_heading(pose_heading),
        .encoder_relative1(relative1),.encoder_relative2(relative2),
        .target_x_mm(target_x_mm),.target_y_mm(target_y_mm),.target_heading(target_heading),
        .target_distance_mm(target_distance_mm),.tick_10ms(tick),.pose_sample_valid(pose_sample_valid),
        .inertial_healthy(inertial_healthy),.travel_q8(odometry_travel_q8));
    // Electronic rear differential. Outer wheel is the requested speed, inner
    // wheel is scaled by (R-track/2)/(R+track/2), L=142 mm, track=168 mm.
    // Left is positive. Direction-specific measured steering maps reach 40/150.
    wire signed [15:0] bounded_steer=commanded_steer>`VEHICLE_LEFT_STEER_DEG?`VEHICLE_LEFT_STEER_DEG:
        commanded_steer < -`VEHICLE_RIGHT_STEER_DEG?-`VEHICLE_RIGHT_STEER_DEG:commanded_steer;
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)applied_steer<=0;
        else if(tick)begin
            if(applied_steer<bounded_steer)applied_steer<=applied_steer+1'b1;
            else if(applied_steer>bounded_steer)applied_steer<=applied_steer-1'b1;
        end
    end
    reg [5:0] steer_magnitude;
    reg servo_negative,servo_negative_delay;
    reg [8:0] inner_ratio_rom[0:63] /* synthesis syn_romstyle="logic" */;
    reg [8:0] inner_ratio;
    reg signed [25:0] inner_product;
    wire [25:0] inner_magnitude=inner_product[25]?-inner_product:inner_product;
    wire [25:0] inner_rounded=(inner_magnitude+26'd128)>>8;
    reg signed [15:0] inner_speed,speed1,speed2;
    assign target_speed1=speed1;
    assign target_speed2=speed2;
    reg [7:0] servo_offset_rom[0:127] /* synthesis syn_romstyle="logic" */;
    reg [7:0] servo_offset;
    reg [10:0] servo_angle_tenths;
    wire [10:0] bounded_center=servo_center_tenths<`VEHICLE_SERVO_CENTER_MIN_TENTHS?
        `VEHICLE_SERVO_CENTER_MIN_TENTHS:servo_center_tenths>`VEHICLE_SERVO_CENTER_MAX_TENTHS?
        `VEHICLE_SERVO_CENTER_MAX_TENTHS:servo_center_tenths;
    wire [10:0] servo_min_tenths=bounded_center-`VEHICLE_SERVO_RIGHT_OFFSET_TENTHS;
    wire [10:0] servo_max_tenths=bounded_center+`VEHICLE_SERVO_LEFT_OFFSET_TENTHS;
    wire [11:0] servo_offset_tenths={4'd0,servo_offset}*12'd10;
    wire signed [12:0] mapped_servo=servo_negative_delay ?
        $signed({2'b0,bounded_center})-$signed({1'b0,servo_offset_tenths}):
        $signed({2'b0,bounded_center})+$signed({1'b0,servo_offset_tenths});
    initial $readmemh("src/vehicle/rear_ratio.mem",inner_ratio_rom);
    initial $readmemh("src/vehicle/servo_offset.mem",servo_offset_rom);
    always @(posedge clk or negedge rst_n)begin
      if(!rst_n)begin
        steer_magnitude<=0;servo_negative<=0;servo_negative_delay<=0;
        servo_offset<=0;servo_angle_tenths<=`VEHICLE_SERVO_CENTER_DEG*10;inner_ratio<=256;inner_product<=0;inner_speed<=0;speed1<=0;speed2<=0;
      end else begin
        steer_magnitude<=applied_steer<0?-applied_steer:applied_steer;
        servo_negative<=`VEHICLE_SERVO_SIGN<0 ? applied_steer>0 : applied_steer<0;
        servo_negative_delay<=servo_negative;
        servo_offset<=servo_offset_rom[{servo_negative,steer_magnitude}];
        servo_angle_tenths<=mapped_servo<$signed({2'b0,servo_min_tenths})?servo_min_tenths:
            mapped_servo>$signed({2'b0,servo_max_tenths})?servo_max_tenths:mapped_servo[10:0];
        inner_ratio<=inner_ratio_rom[steer_magnitude];
        inner_product<=commanded_speed*$signed({1'b0,inner_ratio});
        inner_speed<=inner_product[25]?-$signed(inner_rounded[15:0]):$signed(inner_rounded[15:0]);
        speed1<=(`VEHICLE_MOTOR1_IS_LEFT ? applied_steer>0 : applied_steer<0)?inner_speed:commanded_speed;
        speed2<=(`VEHICLE_MOTOR1_IS_LEFT ? applied_steer<0 : applied_steer>0)?inner_speed:commanded_speed;
      end
    end
    servo_pwm #(.CLK_HZ(CLK_HZ)) servo(
        .clk(clk),.rst_n(rst_n),.angle_tenths(servo_angle_tenths),.pwm_out(servo_pwm_out));
    wire signed [19:0] counts1,counts2;
    wire signed [15:0] converted1,converted2;
    motor_target_counts #(.COUNTS_PER_REV(`VEHICLE_COUNTS_PER_REV),.WHEEL_DIAMETER_MM(`VEHICLE_WHEEL_DIAMETER_MM))
        target1(.clk(clk),.speed_mm_s(speed1),.counts_q4(counts1),.converted_speed(converted1));
    motor_target_counts #(.COUNTS_PER_REV(`VEHICLE_COUNTS_PER_REV),.WHEEL_DIAMETER_MM(`VEHICLE_WHEEL_DIAMETER_MM))
        target2(.clk(clk),.speed_mm_s(speed2),.counts_q4(counts2),.converted_speed(converted2));
    motor_closed_loop #(.CLK_HZ(CLK_HZ),.ENCODER_INVERT(`VEHICLE_ENCODER1_INVERT),
        .COUNTS_PER_REV(`VEHICLE_COUNTS_PER_REV),.WHEEL_DIAMETER_MM(`VEHICLE_WHEEL_DIAMETER_MM),
        .ENCODER_FILTER_CYCLES(CLK_HZ/200000),.REVERSE_BLANK_CYCLES(CLK_HZ/1000)) motor1(
        .clk(clk),.rst_n(rst_n),.drive_enabled(drive_enabled),.brake_request(brake_enable || host_brake),.speed_command(speed1),
        .converted_speed(converted1),.target_counts_q4(counts1),.encoder_A(encoder_A),.encoder_B(encoder_B),
        .dir(dir),.pwm_out(pwm_out),.actual_speed_mm_s(actual_speed1),.encoder_position(encoder_position1));
    // M2 only: swap the two bridge commands in logic, preserving stop 00.
    wire [1:0] motor2_dir;
    assign dir2={motor2_dir[0],motor2_dir[1]};
    motor_closed_loop #(.CLK_HZ(CLK_HZ),.ENCODER_INVERT(`VEHICLE_ENCODER2_INVERT),
        .COUNTS_PER_REV(`VEHICLE_COUNTS_PER_REV),.WHEEL_DIAMETER_MM(`VEHICLE_WHEEL_DIAMETER_MM),
        .ENCODER_FILTER_CYCLES(CLK_HZ/200000),.REVERSE_BLANK_CYCLES(CLK_HZ/1000)) motor2(
        .clk(clk),.rst_n(rst_n),.drive_enabled(drive_enabled),.brake_request(brake_enable || host_brake),.speed_command(speed2),
        .converted_speed(converted2),.target_counts_q4(counts2),.encoder_A(encoder2_A),.encoder_B(encoder2_B),
        .dir(motor2_dir),.pwm_out(pwm_out2),.actual_speed_mm_s(actual_speed2),.encoder_position(encoder_position2));
    reg [2:0] telemetry_count;
    reg send_valid;wire tx_busy;
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin telemetry_count<=0;send_valid<=0;end
        else begin
            send_valid<=0;
            if(tick)begin
                if(telemetry_count==4)begin
                    telemetry_count<=0;if(!tx_busy)send_valid<=1;
                end else telemetry_count<=telemetry_count+1'b1;
            end
        end
    end
    vofa_firewater_tx #(.CLK_HZ(CLK_HZ)) telemetry(.clk(clk),.rst_n(rst_n),.send_valid(send_valid),
        .ch0(speed1),.ch1(actual_speed1),.ch2(speed2),.ch3(actual_speed2),.tx(tx),.busy(tx_busy));
endmodule
