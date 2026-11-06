module ov5640_ddr3_tft(
    //System clock reset
    input       clk50m        , //系统时钟输入，50MHz
    input       reset_n       , //复位信号输入

    input imu_zero_n, // S0: recalibrate orientation
    output IMU_SCLK, IMU_MOSI, IMU_CS_N,
    input IMU_MISO,

    // Integrated MOTOR connector signals (original board pin tables).
    input encoder_A,encoder_B,encoder2_A,encoder2_B,
    output [1:0] dir,dir2,output pwm_out,pwm_out2,servo_pwm_out,
    output ps2_cs_n,ps2_mosi,ps2_clk,input ps2_miso,
    inout tof_sda,tof_scl,output tof_xs1,tof_xs2,tof_xs3,tof_xs4,output tx,
    output host_tx,input host_rx, // Independent host UART, E14/E13.

    //camera interface
    output      camera_sclk   ,
    inout       camera_sdat   ,
    input       camera_vsync  ,
    input       camera_href   ,
    input       camera_pclk   ,
    output      camera_xclk   ,
    input  [7:0]camera_data   ,
    output      camera_rst_n  ,
    output      camera_pwdn   ,
    //led
    output [3:0]led           ,

    //TFT Interface               
    output [15:0]   TFT_rgb       , //TFT数据输出
    output          TFT_hs        , //TFT行同步信号
    output          TFT_vs        , //TFT场同步信号
    output          TFT_clk       , //TFT像素时钟
    output          TFT_de        , //TFT数据使能
    output          TFT_pwm       , //TFT背光控制

    // GT911 capacitive touch interface
    inout TP_SCL,
    inout TP_SDA,
    inout TP_INT,
    output TP_RST_N,

    //ddr
    output [13:0] O_ddr_addr        ,
    output [2:0] O_ddr_ba           ,
    output O_ddr_cs_n               ,
    output O_ddr_ras_n              ,
    output O_ddr_cas_n              ,
    output O_ddr_we_n               ,
    output O_ddr_clk                ,
    output O_ddr_clk_n              ,
    output O_ddr_cke                ,
    output O_ddr_odt                ,
    output O_ddr_reset_n            ,
    output [1:0] O_ddr_dqm          ,
    inout [15:0] IO_ddr_dq          ,
    inout [1:0] IO_ddr_dqs          ,
    inout [1:0] IO_ddr_dqs_n        

);
    wire board_rst_n;
    board_reset u_board_reset(.clk(clk50m),.key_n(reset_n),.reset_n(board_rst_n));

    //Set IMAGE Size  
    // Preserve the sensor's 4:3 field of view. Native panel is landscape.
    parameter IMAGE_WIDTH  = 640;
    parameter IMAGE_HEIGHT = 480;

    wire loc_clk24m, loc_clk33m, loc_clk50m, loc_clk400m;
    wire pll_locked, pll_lock, pll_stop;
    wire sys_rst_n, disp_rst_n, capture_rst_n;
    wire camera_init_done, init_calib_complete;
    wire fifo_wrreq;
    wire [15:0] fifo_wrdata;
    wire clk_disp = loc_clk33m;
    wire pclk_bufg_o = camera_pclk;

    reset_sync u_sys_reset (
        .clk(loc_clk50m), .arst_n(board_rst_n & pll_locked), .rst_n(sys_rst_n)
    );
    reset_sync u_disp_reset (
        .clk(clk_disp), .arst_n(board_rst_n & pll_locked), .rst_n(disp_rst_n)
    );
    reset_sync u_capture_reset (
        .clk(pclk_bufg_o),
        .arst_n(sys_rst_n & camera_init_done & init_calib_complete),
        .rst_n(capture_rst_n)
    );

    // Active-high board LEDs (D0..D3): light means the stage is ready.
    // [3] system PLL, [2] camera SCCB, [1] DDR calibration, [0] DDR PLL.
    assign led = {pll_locked, camera_init_done, init_calib_complete, pll_lock};

    Gowin_PLL Gowin_PLL(
        .clkin(clk50m), //input  clkin
        .clkout0(loc_clk50m), //output  clkout0
        .clkout1(), //output  clkout1
        .clkout2(loc_clk33m), //output  clkout2
        .clkout3(loc_clk24m), //output  clkout3
        .lock(pll_locked), //output  lock
        .mdclk(clk50m), //input  mdclk
        .reset(~board_rst_n) //input  reset
);


    assign camera_xclk = loc_clk24m;
    //摄像头
    camera_init
    #(
        .SYS_CLOCK      ( 50_000_000   ),//系统时钟采用50MHz
        .SCL_CLOCK      ( 400_000      ),//SCL总线时钟采用400kHz
        .CAMERA_TYPE    ( "ov5640"     ),//"ov5640" or "ov7725"
        .IMAGE_TYPE     ( 0            ),// 0: RGB; 1: JPEG
        .IMAGE_WIDTH    ( IMAGE_WIDTH ),// 图片宽度
        .IMAGE_HEIGHT   ( IMAGE_HEIGHT ),// 图片高度
        .IMAGE_FLIP_EN  ( 0            ),// 0: 不翻转，1: 上下翻转
        .IMAGE_MIRROR_EN( 0            ) // 0: 不镜像，1: 左右镜像
    )camera_init
    (
        .Clk         (loc_clk50m       ),
        .Rst_n       (sys_rst_n        ),
        .Init_Done   (camera_init_done ),
        .camera_rst_n(camera_rst_n     ),
        .camera_pwdn (camera_pwdn      ),
        .i2c_sclk    (camera_sclk      ),
        .i2c_sdat    (camera_sdat      )
    );
    
    wire wr_fifo_clr, camera_line_active;

    DVP_Capture DVP_Capture(
        .Rst_n      (capture_rst_n    ),//input
        .PCLK       (pclk_bufg_o      ),//input
        .Vsync      (camera_vsync     ),//input
        .Href       (camera_href      ),//input
        .Data       (camera_data      ),//input     [7:0]

        .ImageState (wr_fifo_clr                 ),//output reg
        .DataValid  (fifo_wrreq       ),//output
        .DataPixel  (fifo_wrdata      ),//output    [15:0]
        .DataHs     (camera_line_active),//output
        .DataVs     (                 ),//output
        .Xaddr      (                 ),//output    [11:0],start is 1
        .Yaddr      (                 )   //output    [11:0],start is 1
    );
    
    wire mdrp_inc;     
    wire [1:0] mdrp_op;     
    wire [7:0] mdrp_wdata;     
    wire [7:0] mdrp_rdata;     
    reg pll_stop_meta, pll_stop_r, pll_stop_rr;
    reg wr;

    reg  pll_lock_r;
    reg  pll_lock_rr;

    always @(posedge clk50m or negedge board_rst_n) begin
        if (!board_rst_n) begin
            pll_lock_r <= 1'b0;
            pll_lock_rr <= 1'b0;
            pll_stop_meta <= 1'b0;
            pll_stop_r <= 1'b0;
            pll_stop_rr <= 1'b0;
        end else begin
            pll_lock_r <= pll_lock;
            pll_lock_rr <= pll_lock_r;
            pll_stop_meta <= pll_stop;
            pll_stop_r <= pll_stop_meta;
            pll_stop_rr <= pll_stop_r;
        end
    end

    pll_mDRP_intf u_pll_mDRP_intf(
        .clk(clk50m),
        .rst_n(board_rst_n),
        .pll_lock(pll_lock_rr),
        .wr(wr),
        .mdrp_inc(mdrp_inc),
        .mdrp_op(mdrp_op),
        .mdrp_wdata(mdrp_wdata),
        .mdrp_rdata(mdrp_rdata)
    );    

    always@(posedge clk50m or negedge board_rst_n)
    if(!board_rst_n)
        wr <= 1'b0;
    else if(!pll_lock_rr)
        wr <= 1'b0;
    else if(~pll_stop_r & pll_stop_rr)begin
        wr <= 1'b1;
    end
    else if(pll_stop_r & (~pll_stop_rr))begin
        wr <= 1'b1;
    end
    else begin
        wr <= 1'b0;
    end

    ddr_pll ddr_pll(
        .clkin(clk50m), //input  clkin
        .clkout0(), //output  clkout0
        .clkout1(), //output  clkout1
        .clkout2(loc_clk400m), //output  clkout2
        .lock(pll_lock), //output  lock
        .mdopc(mdrp_op), //input [1:0] mdopc
        .mdainc(mdrp_inc), //input  mdainc
        .mdwdi(mdrp_wdata), //input [7:0] mdwdi
        .mdrdo(mdrp_rdata), //output [7:0] mdrdo
        .pll_init_bypass(1'b0), //input  pll_init_bypass
        .mdclk(clk50m), //input  mdclk
        .reset(~board_rst_n) //input  reset
);

    wire rdfifo_rden, rdfifo_empty;
    wire [15:0] rdfifo_dout, display_pixel, camera_display_pixel, attitude_pixel;
    wire Frame_Begin, display_req;
    wire [11:0] display_x, display_y;
    wire frame_available, capture_overflow, display_underflow;
    wire touch_valid, touch_down, touch_online;
    wire [15:0] touch_x, touch_y;
    gt911_touch u_touch (
        .clk(clk_disp), .rst_n(disp_rst_n),
        .tp_scl(TP_SCL), .tp_sda(TP_SDA), .tp_int(TP_INT), .tp_rst_n(TP_RST_N),
        .sample_valid(touch_valid), .pressed(touch_down),
        .x(touch_x), .y(touch_y), .online(touch_online)
    );
    portrait_display u_portrait_display (
        .clk(clk_disp), .rst_n(disp_rst_n), .frame_begin(Frame_Begin),
        .data_req(display_req), .x_addr(display_x), .y_addr(display_y),
        .frame_available(frame_available), .fifo_empty(rdfifo_empty), .fifo_data(rdfifo_dout),
        .fifo_read(rdfifo_rden), .pixel_data(camera_display_pixel),
        .underflow(display_underflow)
    );

    wire [7:0] binary_threshold;wire binary_page;
    wire [15:0] binary_preview;
    wire zebra_update,zebra_detected,zebra_enable,zebra_toggle,zebra_can_change;
    vision_video u_vision_video(.clk(clk_disp),.rst_n(disp_rst_n),.frame_begin(Frame_Begin),
        .data_req(display_req),.pixel_valid(rdfifo_rden),.x(display_x),.y(display_y),
        .rgb(camera_display_pixel),.threshold(binary_threshold),.preview(binary_preview),
        .zebra_enable(zebra_enable),.zebra_update(zebra_update),.zebra_detected(zebra_detected));
    wire imu_online, imu_sample, imu_calibrated, imu_angle_valid, imu_rezero, imu_available,imu_fusion_reset;
    wire [7:0] imu_error_reg,imu_error_expected,imu_error_actual,imu_error_mask,imu_error_step;
    wire [2:0] imu_status;
    wire [1:0] imu_cal_state;
    wire [15:0] imu_qx,imu_qy,imu_qz;
    wire signed [15:0] imu_gx,imu_gy,imu_gz,imu_ax,imu_ay,imu_az;
    wire signed [15:0] imu_roll,imu_pitch,imu_yaw;
    wire signed [17:0] imu_relative_w,imu_relative_x,imu_relative_y,imu_relative_z;
    wire signed [15:0] imu_rot_x,imu_rot_y,imu_rot_z;
    wire signed [15:0] motor_target1,motor_target2,motor_actual1,motor_actual2;
    wire imu_rotation_valid,imu_rotation_available;
    imu_zero_button u_imu_button(.clk(clk_disp),.rst_n(disp_rst_n),
        .key_n(imu_zero_n),.pressed(imu_rezero));
    lsm6dsv16x_spi u_imu_spi(.clk(clk_disp),.rst_n(disp_rst_n),
        .miso(IMU_MISO),.sclk(IMU_SCLK),.mosi(IMU_MOSI),.cs_n(IMU_CS_N),
        .online(imu_online),.sample_valid(imu_sample),.fusion_reset(imu_fusion_reset),.status(imu_status),
        .error_reg(imu_error_reg),.error_expected(imu_error_expected),.error_actual(imu_error_actual),
        .error_mask(imu_error_mask),.error_step(imu_error_step),
        .qx_half(imu_qx),.qy_half(imu_qy),.qz_half(imu_qz),
        .gx(imu_gx),.gy(imu_gy),.gz(imu_gz),.ax(imu_ax),.ay(imu_ay),.az(imu_az));
    imu_attitude u_attitude(.clk(clk_disp),.rst_n(disp_rst_n),
        .online(imu_online),.recalibrate(imu_rezero),.sample_valid(imu_sample),.fusion_reset(imu_fusion_reset),
        .qx_half(imu_qx),.qy_half(imu_qy),.qz_half(imu_qz),
        .gx(imu_gx),.gy(imu_gy),.gz(imu_gz),.ax(imu_ax),.ay(imu_ay),.az(imu_az),
        .calibrated(imu_calibrated),.valid(imu_angle_valid),.available(imu_available),.cal_state(imu_cal_state),
        .roll(imu_roll),.pitch(imu_pitch),.yaw(imu_yaw),
        .relative_w(imu_relative_w),.relative_x(imu_relative_x),
        .relative_y(imu_relative_y),.relative_z(imu_relative_z));
    imu_rotation_vector u_imu_rotation(
        .clk(clk_disp),.rst_n(disp_rst_n),.online(imu_online & imu_available & !imu_rezero),
        .sample_valid(imu_angle_valid),.q_w(imu_relative_w),.q_x(imu_relative_x),
        .q_y(imu_relative_y),.q_z(imu_relative_z),
        .valid(imu_rotation_valid),.available(imu_rotation_available),
        .rot_x(imu_rot_x),.rot_y(imu_rot_y),.rot_z(imu_rot_z));
    // Exactly seven channels: Roll/Pitch/Yaw, M1 target/actual,
    // M2 target/actual (angles in degrees, wheel speeds in mm/s).
    // Continue periodic motor telemetry when IMU is offline; angle fields are 0.
    imu_firewater_tx #(.CLK_HZ(33000000),.BAUD(115200),.SEND_HZ(20),.EXTENDED_POSE(0),.MOTOR_TELEMETRY(1)) u_imu_uart(
        .clk(clk_disp),.rst_n(disp_rst_n),.sample_valid(imu_angle_valid),
        .available(imu_online & imu_available),
        .roll(imu_roll),.pitch(imu_pitch),.yaw(imu_yaw),
        .euler_roll(16'sd0),.euler_pitch(16'sd0),.euler_yaw(16'sd0),
        .q_w(18'sd16384),.q_x(18'sd0),.q_y(18'sd0),.q_z(18'sd0),
        .motor_target1(motor_target1),.motor_actual1(motor_actual1),
        .motor_target2(motor_target2),.motor_actual2(motor_actual2),
        .tx(tx),.busy());
    // Isolate the native counters/FIFO from the OSD and map address arithmetic.
    // These registers add display latency only; FIFO consumption is unchanged.
    reg [11:0] ui_x,ui_y;
    reg ui_req,ui_frame;
    reg [15:0] ui_camera,ui_preview;
    always @(posedge clk_disp or negedge disp_rst_n)begin
        if(!disp_rst_n)begin ui_x<=0;ui_y<=0;ui_req<=0;ui_frame<=0;ui_camera<=0;ui_preview<=0;end
        else begin
            ui_x<=display_x;ui_y<=display_y;ui_req<=display_req;ui_frame<=Frame_Begin;
            ui_camera<=camera_display_pixel;ui_preview<=binary_preview;
        end
    end
    attitude_overlay #(.AXIS_VECTOR(1)) u_attitude_overlay(.clk(clk_disp),.rst_n(disp_rst_n),
        .frame_begin(ui_frame),.data_req(ui_req),.x_addr(ui_x),.y_addr(ui_y),
        .portrait_mode(binary_page),.background(binary_page?binary_preview:ui_camera),.online(imu_online),.calibrated(imu_calibrated),
        .available(imu_rotation_available),.sensor_status(imu_status),.cal_state(imu_cal_state),
        .error_reg(imu_error_reg),.error_expected(imu_error_expected),.error_actual(imu_error_actual),
        .error_mask(imu_error_mask),.error_step(imu_error_step),
        .roll(imu_rot_x),.pitch(imu_rot_y),.yaw(imu_rot_z),.pixel(attitude_pixel));

    // Navigation and vehicle share one clock; the video FIFO timing is unchanged.
    wire [3:0] operation_mode,path_segment;
    wire host_route_command,host_route_page;wire [7:0] host_route_opcode,host_route_result,host_route_status;
    wire [31:0] host_route_arguments;
    wire navigation_request,route_valid,path_valid;
    wire [7:0] path_x,path_y;
    wire [10:0] execution_index;wire [11:0] execution_count;
    wire [16:0] execution_x,execution_y;wire execution_lock;
    wire [31:0] host_map_signature;wire [15:0] host_map_generation,host_barrier_generation;
    wire host_map_route_valid,host_map_planning;wire [10:0] host_map_route_index;
    wire [16:0] host_map_route_x,host_map_route_y;wire [3:0] host_barrier_valid;
    wire [35:0] host_barrier_x0,host_barrier_y0,host_barrier_x1,host_barrier_y1;
    wire [15:0] range_left,range_lf,range_rf,range_right,range_average,target_distance;
    wire range_valid;wire [3:0] motion_status,motion_phase;
    wire signed [15:0] vehicle_speed,vehicle_steer,pose_heading,target_heading,target_x,target_y;
    wire signed [23:0] pose_x,pose_y;
    wire profile_ready;wire [3:0] profile_segments;
    wire [15:0] profile_length,profile_left_revs,profile_right_revs;
    wire signed [15:0] profile_entry,profile_exit;wire [7:0] profile_peak;
    wire control_remote,control_host,navigation_avoidance,touch_run,speed_edit,speed_increase,speed_decrease;
    wire [31:0] tof_diagnostics;
    wire [15:0] configured_speed,slowdown_cm,bypass_cm,brake_speed;wire signed [15:0] touch_speed,touch_steer;
    wire [10:0] servo_center_tenths;
    wire host_point_select,host_point_start,host_point_cancel,host_point_select_ready,host_point_route_ready,host_point_planning;
    wire [3:0] host_point_id,selected_point;wire [2:0] host_point_error;
    wire start_heading_valid;wire signed [15:0] start_heading;
    vehicle_subsystem u_vehicle(.zebra_update(zebra_update),.zebra_detected(zebra_detected),.clk(clk_disp),.rst_n(disp_rst_n),.mode(operation_mode),
        .start_heading_valid(start_heading_valid),.start_heading(start_heading),
        .zebra_toggle(zebra_toggle),.zebra_enable(zebra_enable),.zebra_can_change(zebra_can_change),
        .host_route_command(host_route_command),.host_route_opcode(host_route_opcode),.host_route_arguments(host_route_arguments),
        .host_route_page(host_route_page),.host_route_result(host_route_result),.host_route_status(host_route_status),
        .servo_center_tenths(servo_center_tenths),
        .host_point_select(host_point_select),.host_point_start(host_point_start),.host_point_cancel(host_point_cancel),
        .host_point_id(host_point_id),.host_point_select_ready(host_point_select_ready),
        .host_point_route_ready(host_point_route_ready),.host_point_planning(host_point_planning),
        .selected_point(selected_point),.host_point_error(host_point_error),
        .bypass_cm(bypass_cm),.brake_speed(brake_speed),
        .control_remote(control_remote),.control_host(control_host),.navigation_avoidance(navigation_avoidance),.host_rx(host_rx),.host_tx(host_tx),
        .configured_speed(configured_speed),.slowdown_cm(slowdown_cm),
        .touch_run(touch_run),.touch_speed(touch_speed),.touch_steer(touch_steer),
        .speed_edit(speed_edit),.speed_increase(speed_increase),.speed_decrease(speed_decrease),
        .navigation_request(navigation_request),.route_valid(route_valid),.route_count(execution_count),
        .route_index(execution_index),.route_x_q8(execution_x),.route_y_q8(execution_y),.route_lock(execution_lock),
        .map_signature(host_map_signature),.map_generation(host_map_generation),.map_route_valid(host_map_route_valid),.map_planning(host_map_planning),
        .map_route_index(host_map_route_index),.map_route_x_q8(host_map_route_x),.map_route_y_q8(host_map_route_y),
        .map_barrier_generation(host_barrier_generation),.map_barrier_valid(host_barrier_valid),
        .map_barrier_x0(host_barrier_x0),.map_barrier_y0(host_barrier_y0),.map_barrier_x1(host_barrier_x1),.map_barrier_y1(host_barrier_y1),
        .imu_available(imu_available && imu_calibrated),.imu_update(imu_angle_valid && imu_calibrated),.imu_restart(imu_fusion_reset),.imu_yaw(imu_yaw),
        .encoder_A(encoder_A),.encoder_B(encoder_B),.encoder2_A(encoder2_A),.encoder2_B(encoder2_B),
        .dir(dir),.dir2(dir2),.pwm_out(pwm_out),.pwm_out2(pwm_out2),.servo_pwm_out(servo_pwm_out),
        .ps2_cs_n(ps2_cs_n),.ps2_mosi(ps2_mosi),.ps2_clk(ps2_clk),.ps2_miso(ps2_miso),
        .tof_sda(tof_sda),.tof_scl(tof_scl),.tof_xs1(tof_xs1),.tof_xs2(tof_xs2),
        .tof_xs3(tof_xs3),.tof_xs4(tof_xs4),.tx(),
        .left_mm(range_left),.left_front_mm(range_lf),.right_front_mm(range_rf),
        .right_mm(range_right),.front_average_mm(range_average),.tof_diagnostics(tof_diagnostics),.front_valid(range_valid),
        .status(motion_status),.phase(motion_phase),.commanded_speed(vehicle_speed),.commanded_steer(vehicle_steer),
        .target_speed1(motor_target1),.target_speed2(motor_target2),
        .actual_speed1(motor_actual1),.actual_speed2(motor_actual2),.pose_x_q8(pose_x),.pose_y_q8(pose_y),.pose_heading(pose_heading),
        .target_x_mm(target_x),.target_y_mm(target_y),.target_heading(target_heading),
        .target_distance_mm(target_distance),.relative1(),.relative2(),
        .profile_ready(profile_ready),.profile_segments(profile_segments),.profile_length_mm(profile_length),
        .profile_left_revs(profile_left_revs),.profile_right_revs(profile_right_revs),
        .profile_entry_heading(profile_entry),.profile_exit_heading(profile_exit),.profile_peak_steer(profile_peak));
    navigation_ui u_navigation_ui(.binary_page(binary_page),.preview(ui_preview),.binary_threshold(binary_threshold),.clk(clk_disp),.rst_n(disp_rst_n),
        .imu_available(imu_available && imu_calibrated),.imu_restart(imu_fusion_reset),.imu_yaw(imu_yaw),
        .start_heading_valid(start_heading_valid),.start_heading(start_heading),
        .zebra_toggle(zebra_toggle),.zebra_enable(zebra_enable),.zebra_can_change(zebra_can_change),
        .host_route_command(host_route_command),.host_route_opcode(host_route_opcode),.host_route_arguments(host_route_arguments),
        .host_route_page(host_route_page),.host_route_result(host_route_result),.host_route_status(host_route_status),
        .servo_center_tenths(servo_center_tenths),
        .host_point_select(host_point_select),.host_point_start(host_point_start),.host_point_cancel(host_point_cancel),
        .host_point_id(host_point_id),.host_point_select_ready(host_point_select_ready),
        .host_point_route_ready(host_point_route_ready),.host_point_planning(host_point_planning),
        .selected_point(selected_point),.host_point_error(host_point_error),
        .bypass_cm(bypass_cm),.brake_speed(brake_speed),
        .control_remote(control_remote),.control_host(control_host),.navigation_avoidance(navigation_avoidance),.configured_speed(configured_speed),.slowdown_cm(slowdown_cm),
        .touch_run(touch_run),.touch_speed(touch_speed),.touch_steer(touch_steer),
        .speed_edit(speed_edit),.speed_increase(speed_increase),.speed_decrease(speed_decrease),
        .data_req(ui_req),.x_addr(ui_x),.y_addr(ui_y),.video(attitude_pixel),
        .touch_valid(touch_valid),.touch_down(touch_down),.touch_x(touch_x),.touch_y(touch_y),
        .pixel(display_pixel),.operation_mode(operation_mode),.navigation_request(navigation_request),
        .route_valid(route_valid),.path_valid(path_valid),.path_x(path_x),.path_y(path_y),.path_segment(path_segment),
        .path_x_q8(),.path_y_q8(),.execution_index(execution_index),.execution_count(execution_count),
        .execution_x_q8(execution_x),.execution_y_q8(execution_y),.execution_lock(execution_lock),
        .map_signature(host_map_signature),.map_generation(host_map_generation),.map_route_valid(host_map_route_valid),.map_planning(host_map_planning),
        .map_route_index(host_map_route_index),.map_route_x_q8(host_map_route_x),.map_route_y_q8(host_map_route_y),
        .map_barrier_generation(host_barrier_generation),.map_barrier_valid(host_barrier_valid),
        .map_barrier_x0(host_barrier_x0),.map_barrier_y0(host_barrier_y0),.map_barrier_x1(host_barrier_x1),.map_barrier_y1(host_barrier_y1),
        .left_mm(range_left),.left_front_mm(range_lf),.right_front_mm(range_rf),.right_mm(range_right),
        .front_average_mm(range_average),.tof_diagnostics(tof_diagnostics),.front_valid(range_valid),.motion_status(motion_status),.motion_phase(motion_phase),
        .commanded_speed(vehicle_speed),.commanded_steer(vehicle_steer),
        .pose_x_q8(pose_x),.pose_y_q8(pose_y),.pose_heading(pose_heading),
        .target_x_mm(target_x),.target_y_mm(target_y),.target_heading(target_heading),.target_distance_mm(target_distance),
        .profile_ready(profile_ready),.profile_segments(profile_segments),.profile_length_mm(profile_length),
        .profile_left_revs(profile_left_revs),.profile_right_revs(profile_right_revs),
        .profile_entry_heading(profile_entry),.profile_exit_heading(profile_exit),.profile_peak_steer(profile_peak));

    ddr3_ctrl_2port ddr3_ctrl_2port(
        .clk(loc_clk50m)                 ,      //50M时钟信号
        .pll_stop(pll_stop), //output pll_stop
        .pll_lock(pll_lock)            ,
        .clk_200m(loc_clk400m)            ,      //DDR3参考时钟信号
        .sys_rst_n(sys_rst_n)           ,      //外部复位信号
        .init_calib_complete(init_calib_complete) ,    //DDR初始化完成信号

        .rd_load(Frame_Begin), .wr_load(wr_fifo_clr),
        .wr_clk(pclk_bufg_o), .cam_rst_n(capture_rst_n),
        .wfifo_wren(fifo_wrreq), .wfifo_din(fifo_wrdata), .camera_line_active(camera_line_active),
        .rd_clk(clk_disp), .rfifo_rden(rdfifo_rden),
        .rdfifo_empty(rdfifo_empty), .rfifo_dout(rdfifo_dout),
        .frame_available(frame_available), .capture_overflow(capture_overflow),

        //DDR3   
        .ddr3_dq(IO_ddr_dq)             ,   //DDR3 数据
        .ddr3_dqs_n(IO_ddr_dqs_n)          ,   //DDR3 dqs负
        .ddr3_dqs_p(IO_ddr_dqs)          ,   //DDR3 dqs正  
        .ddr3_addr(O_ddr_addr)           ,   //DDR3 地址   
        .ddr3_ba(O_ddr_ba)             ,   //DDR3 banck 选择
        .ddr3_ras_n(O_ddr_ras_n)          ,   //DDR3 行选择
        .ddr3_cas_n(O_ddr_cas_n)          ,   //DDR3 列选择
        .ddr3_we_n(O_ddr_we_n)           ,   //DDR3 读写选择
        .ddr3_reset_n(O_ddr_reset_n)        ,   //DDR3 复位
        .ddr3_ck_p(O_ddr_clk)          ,   //DDR3 时钟正
        .ddr3_ck_n(O_ddr_clk_n)           ,   //DDR3 时钟负
        .ddr3_cke(O_ddr_cke)            ,   //DDR3 时钟使能
        .ddr3_cs_n(O_ddr_cs_n)           ,   //DDR3 片选
        .ddr3_dm(O_ddr_dqm)             ,   //DDR3_dm
        .ddr3_odt(O_ddr_odt)                //DDR3_odt   
    );
    
    //TFT显示
    disp_driver #(.DATA_LATENCY(3)) disp_driver(
		.ClkDisp(clk_disp),
		.Rst_n(disp_rst_n),
		.Data(display_pixel),
		.DataReq(display_req),
		.H_Addr(display_x),
		.V_Addr(display_y),
		.Disp_HS(TFT_hs),
		.Disp_VS(TFT_vs),
		.Disp_Red(TFT_rgb[4:0]),
		.Disp_Green(TFT_rgb[10:5]),
		.Disp_Blue(TFT_rgb[15:11]),
        .Frame_Begin(Frame_Begin),
		.Disp_DE(TFT_de),
		.Disp_PCLK(TFT_clk)
	);
    assign TFT_pwm = 1'b1;
	

endmodule
