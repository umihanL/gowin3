// Menu motion arbitration, two-second arming, route replay and obstacle detour.
// Mode 2 remote; 6 ranging only; 7 gradual stop; 8 detour; 9 planned route.
// Runtime status: 0 idle,1 arming,2 driving,3 complete,4 sensor fault,
// 5 blocked,6 tracking fault,7 missing route,8 encoder fault,9 stalled.
`include "src/vehicle/vehicle_config.vh"
module vehicle_motion #(
    parameter CLK_HZ=33000000,START_DELAY_MS=2000,
    parameter CRUISE_MM_S=200,LOOKAHEAD_MM=60,
    parameter STOP_MM=160,SLOW_MM=650,EMERGENCY_MM=65,
    parameter DETOUR_TRIGGER_MM=650,SIDE_CLEAR_MM=480,
    parameter DETOUR_OFFSET_MM=400,PASS_EXTRA_MM=350,AFTER_MM=400,
    parameter RANGE_TIMEOUT_MS=800,IMU_TIMEOUT_MS=10000,
    parameter STALL_MS=1500,MAX_BYPASS_MM=3000,
    parameter SEARCH_MM=400,SEARCH_SPEED_MM_S=80,SIDE_MEMORY_MS=5000,
    parameter REQUIRE_VISION=1
)(
    input road_update,road_valid,road_junction,input signed [8:0] road_near,road_heading,
    input [2:0] road_visibility,
    input road_fit_valid,input [9:0] road_fit_forward0,road_fit_forward1,
    input signed [8:0] road_fit_error0,road_fit_error1,
    input zebra_update,zebra_detected,
    output signed [1:0] vision_turn_hint,output [2:0] vision_state,
    input clk,rst_n,input [3:0] mode,input navigation_request,route_valid,profile_ready,
    input [15:0] configured_speed,slowdown_cm,bypass_cm,brake_speed,
    input [11:0] route_count,
    output reg [10:0] route_index,
    output [10:0] route_read_index,
    input [16:0] route_x_q8,route_y_q8,input route_lock,
    input signed [15:0] route_tangent,
    input signed [1:0] route_curve,
    input route_exit_valid,input signed [15:0] route_exit_heading,
    input signed [1:0] route_upcoming_turn,input [7:0] route_entry_mm,
    input route_entry_valid,input signed [15:0] route_entry_x_mm,route_entry_y_mm,
    input route_entry_index_valid,input [10:0] route_entry_index,
    input signed [31:0] encoder1,encoder2,
    input imu_available,imu_update,imu_restart,input signed [15:0] imu_yaw,
    input range_frame,input [3:0] range_samples,range_observations,input [31:0] range_diagnostics,
    input [15:0] left_mm,left_front_mm,right_front_mm,right_mm,
    input remote_link,remote_run,input signed [15:0] remote_speed,remote_steer,
    input host_pause,
    output reg drive_enabled,output brake_enable,output reg signed [15:0] speed_mm_s,steer_deg,
    output reg [3:0] status,output reg [3:0] phase,
    output [15:0] front_average_mm,output front_valid,
    output signed [23:0] pose_x_q8,pose_y_q8,
    output signed [15:0] pose_heading,
    output signed [31:0] encoder_relative1,encoder_relative2,
    output reg signed [15:0] target_x_mm,target_y_mm,
    output signed [15:0] target_heading,
    output reg [15:0] target_distance_mm,
    output tick_10ms
);
    localparam TIMER_CYCLES=CLK_HZ/100;
    localparam TIMER_W=(TIMER_CYCLES<2)?1:$clog2(TIMER_CYCLES);
    localparam ARM_TICKS=(START_DELAY_MS+9)/10;
    localparam OFF=0,READ_FIRST=1,FIRST=2,READ_SECOND=3,SECOND=4,
        ALIGN=5,ARM=6,RUN=7,READ_TARGET=8,TARGET=9,FINISHED=10,REBASE_WAIT=11,WAIT_PROFILE=12,
        HEADING_REBASE_WAIT=13,HEADING_TRACK_WAIT=14;
    reg [3:0] state,last_mode;
    reg [2:0] read_wait;
    reg [TIMER_W-1:0] timer;
    reg [15:0] arm_count,range_age,imu_age,stall_count,stall_count2;
    reg [3:0] pending_mode;
    reg signed [15:0] initial_heading;
    reg heading_aligned,visual_start_aligned;
    wire visual_start_ready,mapped_entry_accepted;
    reg [3:0] visual_steer_settle;
    reg signed [23:0] initial_x,initial_y;
    reg signed [23:0] first_dx_q8,first_dy_q8;
    reg rebase,tracker_start;
    wire pose_updated,encoder_fault;
    wire signed [31:0] travel_q8;
    wire signed [16:0] cosine,sine;
    wire localizer_busy,visual_correction;
    wire [10:0] visual_read_index;
    wire signed [7:0] visual_dx,visual_dy;
    wire signed [23:0] raw_pose_x,raw_pose_y,raw_rear_x,raw_rear_y;
    reg signed [23:0] visual_offset_x,visual_offset_y;
    assign pose_x_q8=raw_pose_x+visual_offset_x;
    assign pose_y_q8=raw_pose_y+visual_offset_y;
    assign route_read_index=localizer_busy?visual_read_index:route_index;
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin visual_offset_x<=0;visual_offset_y<=0;end
        else if(!auto_mode || pending_mode!=9 || rebase)begin visual_offset_x<=0;visual_offset_y<=0;end
        else if(visual_correction)begin
            visual_offset_x<=visual_offset_x+($signed(visual_dx)*24'sd256);
            visual_offset_y<=visual_offset_y+($signed(visual_dy)*24'sd256);
        end
    end
    vehicle_visual_localizer localizer(.clk(clk),.rst_n(rst_n),
        .enabled(pending_mode==9 && REQUIRE_VISION && profile_ready &&
                 (state==RUN || state==READ_TARGET || state==TARGET) &&
                 !bypass_active && !nav_resume && host_pause!==1'b1),
        .frame_valid(state==RUN && road_update && road_valid),.fit_valid(road_fit_valid===1'b1),
        .forward0(road_fit_forward0),.forward1(road_fit_forward1),
        .error0(road_fit_error0),.error1(road_fit_error1),
        .route_count(route_count),.execution_index(route_index),
        .route_x_q8(route_x_q8),.route_y_q8(route_y_q8),
        .pose_x(pose_x_mm),.pose_y(pose_y_mm),.cosine(cosine),.sine(sine),
        .busy(localizer_busy),.read_index(visual_read_index),
        .correction_valid(visual_correction),.correction_x(visual_dx),.correction_y(visual_dy));
    wire tracker_done;
    reg tracker_inflight;
    reg signed [15:0] calculation_target_x,calculation_target_y;
    wire current_tracker_done=tracker_done && calculation_target_x==target_x_mm && calculation_target_y==target_y_mm;
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin tracker_inflight<=0;calculation_target_x<=0;calculation_target_y<=0;end
        else if(tracker_done)tracker_inflight<=0;
        else if(tracker_start && !tracker_inflight)begin
            tracker_inflight<=1;calculation_target_x<=target_x_mm;calculation_target_y<=target_y_mm;
        end
    end
    wire [15:0] distance_mm;
    wire signed [15:0] tracker_steer,bearing,forward_mm,lateral_mm;
    reg signed [31:0] previous_encoder1,previous_encoder2;
    reg [3:0] stopped_ticks;
    reg braking;
    reg [7:0] reverse_count;
    reg [15:0] latched_slow_mm;
    reg [15:0] latched_bypass_mm,latched_brake_speed;
    reg nav_resume,bypass_start,bypass_choose_left,bypass_single_front;
    reg [5:0] straight_ticks;
    reg zebra_latched;reg [7:0] zebra_clear_ticks;
    reg signed [31:0] zebra_last_seen_travel;
    // Camera-to-tail distance follows the current measured mount.
    wire zebra_tail_clear=travel_q8-zebra_last_seen_travel>=
        (`VEHICLE_BODY_LENGTH_MM-`VEHICLE_CAMERA_FROM_NOSE_MM)*256;
    wire bypass_active,bypass_done,bypass_blocked,passing_clear,left_detour;
    wire [3:0] bypass_phase;
    wire signed [15:0] bypass_steer,bypass_x,bypass_y;
    reg [15:0] stop_cap;
    reg [7:0] clear_count;
    reg signed [31:0] phase_origin,bypass_origin;
    reg blind_brake;
    reg [3:0] sample_pending;
    reg signed [15:0] local_target_x,local_target_y;
    reg signed [15:0] requested_speed;
    reg [15:0] nearest_front;
    reg signed [15:0] lateral_target_error;
    reg [15:0] final_distance;
    reg signed [15:0] previous_target_x,previous_target_y;
    wire signed [16:0] segment_x=$signed(target_x_mm)-$signed(previous_target_x);
    wire signed [16:0] segment_y=$signed(target_y_mm)-$signed(previous_target_y);
    wire signed [16:0] goal_x=$signed(target_x_mm)-$signed(pose_x_mm);
    wire signed [16:0] goal_y=$signed(target_y_mm)-$signed(pose_y_mm);
    wire [16:0] segment_norm=(segment_x<0?-segment_x:segment_x)+(segment_y<0?-segment_y:segment_y);
    reg signed [33:0] goal_dot_x,goal_dot_y,goal_cross_x,goal_cross_y;
    // Freeze the ENTRY of a same-direction bend. Recomputing this plane
    // from each advancing look-ahead target moves the gate away forever.
    reg signed [15:0] bend_entry_x,bend_entry_y;
    reg signed [16:0] bend_entry_dx,bend_entry_dy;
    reg signed [1:0] entry_direction;
    reg signed [31:0] entry_travel_q8;
    reg signed [15:0] entry_exit_heading;
    reg entry_exit_valid,entry_event_sent,entry_uses_distance,entry_position_valid;
    wire has_upcoming=route_upcoming_turn===2'b01 || route_upcoming_turn===2'b11;
    wire signed [1:0] announced_curve=has_upcoming?route_upcoming_turn:route_curve;
    wire signed [16:0] next_entry_exit_raw=$signed(route_exit_heading)-$signed(entry_exit_heading);
    wire signed [15:0] next_entry_exit_change=next_entry_exit_raw>1800?next_entry_exit_raw-3600:
        next_entry_exit_raw < -1800?next_entry_exit_raw+3600:next_entry_exit_raw;
    wire distinct_bend=announced_curve!=entry_direction ||
        (route_exit_valid && entry_exit_valid &&
         (next_entry_exit_change>100 || next_entry_exit_change < -100));
    wire signed [31:0] entry_ahead_mm=$signed(forward_mm)+$signed({1'b0,route_entry_mm});
    reg signed [15:0] incoming_heading,last_straight_heading;
    reg [4:0] entry_straight_ticks;
    wire signed [16:0] entry_gap_x=$signed(bend_entry_x)-$signed(pose_x_mm);
    wire signed [16:0] entry_gap_y=$signed(bend_entry_y)-$signed(pose_y_mm);
    reg signed [33:0] entry_dot_x,entry_dot_y;
    reg signed [33:0] incoming_side_x,incoming_side_y;
    wire signed [15:0] incoming_side=($signed(incoming_side_y)-$signed(incoming_side_x))>>>15;
    wire signed [34:0] entry_dot=$signed(entry_dot_x)+$signed(entry_dot_y);
    wire [16:0] entry_norm=(bend_entry_dx<0?-bend_entry_dx:bend_entry_dx)+
        (bend_entry_dy<0?-bend_entry_dy:bend_entry_dy);
    reg turn_entry_ready;
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin
            turn_entry_ready<=0;entry_direction<=0;entry_straight_ticks<=0;
            entry_exit_heading<=0;entry_exit_valid<=0;entry_event_sent<=0;entry_uses_distance<=0;entry_position_valid<=0;
            incoming_heading<=0;last_straight_heading<=0;
            entry_travel_q8<=0;
            bend_entry_x<=0;bend_entry_y<=0;bend_entry_dx<=0;bend_entry_dy<=0;
        end else if(!auto_mode || !profile_ready)begin
            turn_entry_ready<=0;entry_direction<=0;entry_straight_ticks<=0;
            entry_exit_valid<=0;entry_event_sent<=0;entry_position_valid<=0;
        end else if(tick_10ms && state==RUN)begin
            if(mapped_entry_accepted)entry_event_sent<=1;
            if(announced_curve!=0)begin
                entry_straight_ticks<=0;
                if(distinct_bend)begin
                    entry_direction<=announced_curve;turn_entry_ready<=0;entry_event_sent<=0;
                    incoming_heading<=entry_direction!=0 && entry_exit_valid?entry_exit_heading:
                        has_upcoming && route_curve==0?route_tangent:last_straight_heading;
                    entry_exit_heading<=route_exit_valid?route_exit_heading:route_tangent;
                    entry_exit_valid<=route_exit_valid;
                    entry_uses_distance<=route_entry_index_valid!==1'b1 &&
                        route_entry_valid!==1'b1 && has_upcoming && route_curve==0;
                    entry_travel_q8<=travel_q8+(entry_ahead_mm<<<8);
                    // An index is only an address, not proof the car has
                    // arrived: route targets advance up to LOOKAHEAD_MM early.
                    // Acquire the point through the existing motion RAM read,
                    // then freeze its physical plane across target advances.
                    entry_position_valid<=(route_entry_index_valid!==1'b1) || route_index>=route_entry_index;
                    bend_entry_x<=(route_entry_index_valid===1'b1)?
                        (route_index==route_entry_index?target_x_mm:previous_target_x):
                        (route_entry_valid===1'b1)?route_entry_x_mm:previous_target_x;
                    bend_entry_y<=(route_entry_index_valid===1'b1)?
                        (route_index==route_entry_index?target_y_mm:previous_target_y):
                        (route_entry_valid===1'b1)?route_entry_y_mm:previous_target_y;
                    bend_entry_dx<=segment_norm!=0?segment_x:($signed(cosine)>>>8);
                    bend_entry_dy<=segment_norm!=0?segment_y:($signed(sine)>>>8);
                end else if(!entry_position_valid && route_entry_index_valid===1'b1 &&
                    route_index>=route_entry_index)begin
                    bend_entry_x<=route_index==route_entry_index?target_x_mm:previous_target_x;
                    bend_entry_y<=route_index==route_entry_index?target_y_mm:previous_target_y;
                    entry_position_valid<=1;
                end else if(entry_position_valid && (entry_uses_distance?
                    (travel_q8>=entry_travel_q8):
                    (entry_dot<=0)))begin
                    turn_entry_ready<=1;
                end
            end else begin
                last_straight_heading<=route_tangent;
                if(entry_straight_ticks<20)entry_straight_ticks<=entry_straight_ticks+1'b1;
                else begin entry_direction<=0;turn_entry_ready<=1;entry_exit_valid<=0;entry_event_sent<=0;end
            end
        end
    end
    wire signed [16:0] incoming_raw=$signed(entry_direction==0?route_tangent:incoming_heading)-$signed(pose_heading);
    wire signed [15:0] incoming_error=incoming_raw>1800?incoming_raw-3600:
        incoming_raw < -1800?incoming_raw+3600:incoming_raw;
    // Profile ROM changes between control ticks. Mask the old bend's ready
    // bit immediately when a new direction arrives; otherwise fusion can
    // consume that stale ready bit on the very tick that motion clears it.
    wire entry_ready_current=turn_entry_ready &&
        (announced_curve==0 || !distinct_bend);
    wire signed [34:0] goal_dot=$signed(goal_dot_x)+$signed(goal_dot_y);
    wire signed [34:0] goal_cross=$signed(goal_cross_x)-$signed(goal_cross_y);
    wire signed [34:0] arrival_margin=$signed({1'b0,segment_norm})*18;
    wire signed [34:0] arrival_corridor=$signed({1'b0,segment_norm})*`VEHICLE_BODY_WIDTH_MM;
    wire goal_plane_reached=segment_norm!=0 && goal_dot<=arrival_margin;
    wire goal_in_corridor=goal_cross<=arrival_corridor && goal_cross>=-arrival_corridor;
    wire terminal_approach=route_index+1>=route_count && final_distance<160 && !nav_resume;
    always @(posedge clk)begin
        goal_dot_x<=goal_x*segment_x;goal_dot_y<=goal_y*segment_y;
        goal_cross_x<=goal_x*segment_y;goal_cross_y<=goal_y*segment_x;
        entry_dot_x<=entry_gap_x*bend_entry_dx;entry_dot_y<=entry_gap_y*bend_entry_dy;
        incoming_side_x<=entry_gap_x*sine;incoming_side_y<=entry_gap_y*cosine;
    end
    reg signed [31:0] brake_previous_travel;
    reg [15:0] measured_forward_speed;
    reg signed [15:0] last_good_yaw;
    reg imu_pending,imu_epoch_pending;
    reg [4:0] wheel_still_ticks;
    wire signed [23:0] rear_x_q8,rear_y_q8;
    assign rear_x_q8=raw_rear_x+visual_offset_x;
    assign rear_y_q8=raw_rear_y+visual_offset_y;
    wire signed [31:0] brake_delta_q8=travel_q8-brake_previous_travel;
    wire auto_mode=(mode==7 || mode==8 || (mode==9 && navigation_request));
    wire enabled_mode=mode==2 || auto_mode;
    wire front_pair=left_front_mm!=16'hffff && right_front_mm!=16'hffff &&
        left_front_mm!=0 && right_front_mm!=0;
    wire [16:0] front_sum={1'b0,left_front_mm}+{1'b0,right_front_mm};
    assign tick_10ms=timer==TIMER_CYCLES-1;
    assign front_valid=front_pair && range_age<RANGE_TIMEOUT_MS/10;
    assign front_average_mm=front_valid?front_sum[16:1]:16'hffff;
    assign target_heading=bearing;
    wire imu_ready=imu_age<IMU_TIMEOUT_MS/10;
    // Echo dropout does not erase a committed manoeuvre or a known obstacle.
    wire sensors_ready=imu_ready && (front_known || front_no_echo);
    // Differential wheel yaw bridges a bounded IMU gap, rather than freezing yaw.
    // Bridge short IMU packet gaps with encoder odometry, but retain a long
    // timeout so a genuinely dead sensor cannot leave navigation blind forever.
    wire navigation_sensors=imu_ready;
    wire close_left=range_diagnostics[7:0]==0 && age_left<RANGE_TIMEOUT_MS/10 && left_mm>0 && left_mm<100;
    wire close_right=range_diagnostics[31:24]==0 && age_right<RANGE_TIMEOUT_MS/10 && right_mm>0 && right_mm<100;
    wire signed [15:0] route_candidate=tracker_steer<8 && tracker_steer > -8 && close_left!=close_right?
        (close_left?-16'sd3:16'sd3):tracker_steer;
    // If the current target has fallen behind during a tight corner, pure
    // pursuit can legitimately point to the opposite side of the car. Keep
    // the active profile edge's turn sign until the next target is acquired;
    // this prevents a planned left corner from commanding a right turn after
    // an overshoot while leaving normal outgoing-lane centring untouched.
    wire signed [15:0] route_steer=route_curve==2'sd1 && forward_mm<0 && route_candidate<0?16'sd0:
        route_curve==-2'sd1 && forward_mm<0 && route_candidate>0?16'sd0:route_candidate;
    wire signed [15:0] pose_x_mm=pose_x_q8>>>8,pose_y_mm=pose_y_q8>>>8;
    wire signed [15:0] rear_x_mm=rear_x_q8>>>8,rear_y_mm=rear_y_q8>>>8;
    wire signed [15:0] vision_steer;wire vision_active,vision_blocked,vision_recovery;
    wire [15:0] vision_speed_limit;
    wire signed [15:0] visual_alignment_yaw,visual_alignment_offset;
    // Trusted map fallback is measured against the active segment, never
    // against a point's lateral offset in the rotating vehicle frame.
    wire signed [34:0] recovery_corridor=$signed({1'b0,segment_norm})*100;
    // Register this geometry predicate before the visual mux/slew arithmetic.
    // Its former target->norm->DSP->compare->steering path exceeded 33MHz.
    // One system clock is negligible beside the 10ms control period.
    reg route_guidance_valid;
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)route_guidance_valid<=0;
        else route_guidance_valid<=profile_ready && segment_norm!=0 &&
            goal_cross<=recovery_corridor && goal_cross>=-recovery_corridor &&
            distance_mm<=450 && forward_mm>=-80;
    end
    wire signed [32:0] startup_side_x=$signed(visual_alignment_offset)*$signed(sine);
    wire signed [32:0] startup_side_y=$signed(visual_alignment_offset)*$signed(cosine);
    wire signed [16:0] startup_heading_raw=$signed(initial_heading)+$signed(visual_alignment_yaw);
    // On a short incoming straight, both IPM fit bands can already lie in
    // the upcoming curve. Its slope is road curvature, not initial body yaw.
    wire startup_curve_preview=has_upcoming && route_curve==0 && route_entry_mm<240 &&
        road_near<16 && road_near > -16;
    wire signed [15:0] startup_heading=startup_heading_raw>1800?startup_heading_raw-3600:
        startup_heading_raw < -1800?startup_heading_raw+3600:startup_heading_raw;
    wire visual_navigation=REQUIRE_VISION && pending_mode==9 && auto_mode && state!=OFF && state!=FINISHED &&
        !bypass_active && !bypass_start && !nav_resume;
    wire signed [16:0] planned_delta_raw=$signed(route_tangent)-$signed(pose_heading);
    wire signed [16:0] planned_delta=planned_delta_raw>1800?planned_delta_raw-3600:
        planned_delta_raw < -1800?planned_delta_raw+3600:planned_delta_raw;
    reg signed [1:0] recent_curve;reg [4:0] curve_age;
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin recent_curve<=0;curve_age<=31;end
        else if(!profile_ready || !auto_mode)begin recent_curve<=0;curve_age<=31;end
        else if(route_curve!=0)begin recent_curve<=route_curve;curve_age<=0;end
        else if(tick_10ms && curve_age<31)curve_age<=curve_age+1'b1;
    end
    // The route supplies bend direction before yaw has changed. Requiring a
    // 12-degree tangent error hid the hint at the start of a tight bend.
    // A nearby route curve plus matching pursuit steering can own a turn;
    // a direction hint by itself cannot override a usable straight lane.
    assign vision_turn_hint=profile_ready && has_upcoming?route_upcoming_turn:
        profile_ready && curve_age<20?recent_curve:2'sd0;
    vehicle_vision_fusion vision_guidance(.clk(clk),.rst_n(rst_n),.tick(tick_10ms && host_pause!==1'b1),
        .road_visibility(road_visibility),.terminal_approach(terminal_approach),
        .enabled(visual_navigation),.moving(drive_enabled),.pose_heading(pose_heading),.travel_q8(travel_q8),
        .blocked(vision_blocked),.speed_limit(vision_speed_limit),.guidance_state(vision_state),
        .recovery_active(vision_recovery),
        .alignment_yaw(visual_alignment_yaw),.alignment_offset(visual_alignment_offset),
        .start_ready(visual_start_ready),
        .planned_turn(vision_turn_hint),
        .planned_exit_valid(route_exit_valid),.planned_exit_heading(route_exit_heading),
        .route_guidance_valid(route_guidance_valid),.route_target_behind(forward_mm<0),
        .turn_entry_ready(entry_ready_current),
        .incoming_heading_error(incoming_error),
        .incoming_lateral_error(incoming_side),
        .mapped_entry_due(entry_direction!=0 && entry_ready_current && !entry_event_sent),
        .mapped_entry_accepted(mapped_entry_accepted),
        .road_update(road_update),.road_valid(road_valid),.road_junction(road_junction),.road_near(road_near),.road_heading(road_heading),
        .route_steer(route_steer),.steering(vision_steer),.active(vision_active));
    wire signed [15:0] navigation_steer=vision_steer;
    wire lane_guidance_progress=vision_active || vision_recovery || vision_state==6;
    wire committed_corner=visual_navigation && vision_recovery;
    // TB6612 (MOTOR/src/ACKERMANN_CAR.md): automatic braking is explicit;
    // reset/menu/manual/remote never acquire this request.
    assign brake_enable=auto_mode && ((pending_mode==7 && state==RUN && braking) ||
        (state==FINISHED && status!=7 && status!=0));
    wire [15:0] side_distance=left_detour?right_mm:left_mm;
    wire [7:0] side_diagnostic=left_detour?range_diagnostics[31:24]:range_diagnostics[7:0];
    wire side_valid=side_diagnostic==0 && side_distance>0 && side_distance<=4000 &&
        (left_detour?age_right:age_left)<RANGE_TIMEOUT_MS/10;
    wire side_clear=side_valid && side_distance>SIDE_CLEAR_MM;
    wire [3:0] sample_event=range_samples | sample_pending;
    wire inner_sample=left_detour?sample_event[3]:sample_event[0];
    wire memory_clear=mode!=last_mode || !auto_mode || bypass_done;
    // Before rebase the previous run's odometer is irrelevant to new readings.
    wire signed [31:0] memory_travel=(state==ARM || state==REBASE_WAIT || state==OFF)?32'sd0:travel_q8;
    wire [15:0] remembered_left,remembered_lf,remembered_rf,remembered_right;
    wire known_left,known_lf,known_rf,known_right;
    wire [15:0] age_left,age_lf,age_rf,age_right;
    vehicle_range_memory ml(.clk(clk),.rst_n(rst_n),.clear(memory_clear),.tick(tick_10ms),
        .frame(range_samples[0]),.diagnostic(range_diagnostics[7:0]),.measured_mm(left_mm),.travel_q8(32'sd0),
        .remaining_mm(remembered_left),.known(known_left),.age_ticks(age_left));
    vehicle_range_memory mlf(.clk(clk),.rst_n(rst_n),.clear(memory_clear),.tick(tick_10ms),
        .frame(range_samples[1]),.diagnostic(range_diagnostics[15:8]),.measured_mm(left_front_mm),.travel_q8(memory_travel),
        .remaining_mm(remembered_lf),.known(known_lf),.age_ticks(age_lf));
    vehicle_range_memory mrf(.clk(clk),.rst_n(rst_n),.clear(memory_clear),.tick(tick_10ms),
        .frame(range_samples[2]),.diagnostic(range_diagnostics[23:16]),.measured_mm(right_front_mm),.travel_q8(memory_travel),
        .remaining_mm(remembered_rf),.known(known_rf),.age_ticks(age_rf));
    vehicle_range_memory mr(.clk(clk),.rst_n(rst_n),.clear(memory_clear),.tick(tick_10ms),
        .frame(range_samples[3]),.diagnostic(range_diagnostics[31:24]),.measured_mm(right_mm),.travel_q8(32'sd0),
        .remaining_mm(remembered_right),.known(known_right),.age_ticks(age_right));
    wire front_known=known_lf || known_rf;
    wire [15:0] remembered_front=remembered_lf<remembered_rf?remembered_lf:remembered_rf;
    wire live_lf=range_diagnostics[15:8]==0 && left_front_mm>0 && left_front_mm<=4000 &&
        age_lf<RANGE_TIMEOUT_MS/10;
    wire live_rf=range_diagnostics[23:16]==0 && right_front_mm>0 && right_front_mm<=4000 &&
        age_rf<RANGE_TIMEOUT_MS/10;
    wire [15:0] live_front=live_lf && live_rf?(left_front_mm<right_front_mm?left_front_mm:right_front_mm):
        live_lf?left_front_mm:live_rf?right_front_mm:16'hffff;
    wire front_no_echo=range_age<RANGE_TIMEOUT_MS/10 &&
        (range_diagnostics[15:8]==8'h11 || range_diagnostics[15:8]==8'h12 ||
         range_diagnostics[23:16]==8'h11 || range_diagnostics[23:16]==8'h12);
    wire [15:0] search_remaining=memory_travel<0?SEARCH_MM+STOP_MM:
        memory_travel>=(SEARCH_MM+STOP_MM)*256?16'd0:SEARCH_MM+STOP_MM-(memory_travel>>>8);
    wire committed_detour=bypass_active || nav_resume || (pending_mode==8 && phase==4);
    reg [2:0] far_lf,far_rf;reg [15:0] far_age_lf,far_age_rf;
    reg front_obstacle_latched;
    wire navigation_far=pending_mode==9 && !front_obstacle_latched && !live_lf && !live_rf &&
        ((far_lf>=3 && far_age_lf<RANGE_TIMEOUT_MS/10) || (far_rf>=3 && far_age_rf<RANGE_TIMEOUT_MS/10));
    wire [15:0] control_distance=committed_detour?live_front:
        navigation_far?16'd2000:front_known?remembered_front:
        pending_mode==7?16'd2000:search_remaining;
    wire [15:0] motion_limit=pending_mode!=7 && !front_known && !committed_detour && !navigation_far &&
        distance_limit>SEARCH_SPEED_MM_S?SEARCH_SPEED_MM_S:distance_limit;
    // Stop mode uses its own 80..400 mm/s setting. Bypass/route cruise stays
    // bounded at 200 mm/s; the distance envelope adapts to each speed/zone.
    // A front-point curve needs time for the real servo and visual loop.
    // This limits navigation only; the existing acceleration ramp smooths
    // transitions, and manual/remote/brake/bypass speed settings are unchanged.
    wire [15:0] map_ceiling=pending_mode==9 && !bypass_active && !nav_resume?
        ((tracker_steer>24 || tracker_steer < -24)?16'd110:
         (tracker_steer>12 || tracker_steer < -12)?16'd150:CRUISE_MM_S):CRUISE_MM_S;
    wire [15:0] automatic_ceiling=visual_navigation && vision_speed_limit<map_ceiling?vision_speed_limit:map_ceiling;
    wire [15:0] cruise_base_limit=pending_mode==7?latched_brake_speed:
        configured_speed<automatic_ceiling?configured_speed:automatic_ceiling;
    wire [15:0] cruise_limit=zebra_latched && cruise_base_limit>100?16'd100:cruise_base_limit;
    wire [15:0] range_speed;
    wire [15:0] distance_limit=range_speed<8?16'd0:range_speed;
    // Reserve 200 ms of measured travel for scan/driver latency. The remaining
    // v^2-distance envelope adapts deceleration to both initial speed and zone.
    wire [15:0] brake_velocity=measured_forward_speed>$unsigned(speed_mm_s)?
        measured_forward_speed:$unsigned(speed_mm_s);
    // 205=128+64+8+4+1; use LUT adders instead of scarce DSP columns.
    function [15:0] latency_reserve;
        input [15:0] velocity;
        reg [31:0] v,scaled;
        begin
            v={16'd0,velocity};
            scaled=((v<<7)+(v<<6))+((v<<3)+(v<<2))+v;
            latency_reserve=scaled>>10;
        end
    endfunction
    wire [15:0] brake_reserve=latency_reserve(brake_velocity);
    wire [15:0] brake_distance=nearest_front>brake_reserve?nearest_front-brake_reserve:16'd0;
    wire [15:0] initial_reserve=latency_reserve(latched_brake_speed);
    wire [15:0] brake_zone=latched_slow_mm>16'd125+initial_reserve?
        latched_slow_mm-initial_reserve:16'd125;
    wire [15:0] limiter_distance=pending_mode==7?brake_distance:nearest_front;
    vehicle_range_limit #(.QUADRATIC(1)) limiter(.clk(clk),.rst_n(rst_n),.distance_mm(limiter_distance),
        .slow_mm(pending_mode==7?brake_zone:latched_slow_mm),.stop_mm(pending_mode==7?16'd85:16'd160),.cruise(cruise_limit),.speed_limit(range_speed));
    // A completed invalid-range observation is usable as weak far-space
    // evidence; a NACK, E09 bus fault or uninitialized channel is not.
    reg [2:0] far_left,far_right;reg [15:0] far_age_left,far_age_right;
    wire [15:0] right_space=far_right>=3 && far_age_right<RANGE_TIMEOUT_MS/10?16'd2000:
        known_right && age_right<SIDE_MEMORY_MS/10?remembered_right:16'd0;
    wire [15:0] left_space=far_left>=3 && far_age_left<RANGE_TIMEOUT_MS/10?16'd2000:
        known_left && age_left<SIDE_MEMORY_MS/10?remembered_left:16'd0;
    wire right_open=right_space>=250,left_open=left_space>=250;
    wire only_right_obstacle=live_rf && right_front_mm<=latched_bypass_mm &&
        (!live_lf || left_front_mm>latched_bypass_mm+100);
    wire only_left_obstacle=live_lf && left_front_mm<=latched_bypass_mm &&
        (!live_rf || right_front_mm>latched_bypass_mm+100);
    wire select_left=only_right_obstacle && left_open?1'b1:
        only_left_obstacle && right_open?1'b0:left_open && (!right_open || left_space>right_space);
    // Choose once at the actual obstacle trigger. Fresh near readings may
    // revoke approach clearance, but an active detour never reselects sides.
    wire chosen_left=select_left;
    vehicle_bypass bypass(.clk(clk),.rst_n(rst_n),.tick(tick_10ms && host_pause!==1'b1),.start(bypass_start),
        .cancel(mode!=last_mode || !auto_mode),.choose_left(bypass_choose_left),
        .single_front_entry(bypass_single_front),
        .px(rear_x_mm),.py(rear_y_mm),.heading(pose_heading),.cosine(cosine),.sine(sine),
        .travel_q8(travel_q8),.lf(left_front_mm),.rf(right_front_mm),.left_mm(left_mm),.right_mm(right_mm),
        .diagnostics(range_diagnostics),.observations(range_observations),
        .active(bypass_active),.done(bypass_done),.blocked(bypass_blocked),.phase(bypass_phase),
        .steering(bypass_steer),.local_x(bypass_x),.local_y(bypass_y),
        .tail_confirmed(passing_clear),.left_turn(left_detour));
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin far_left<=0;far_right<=0;far_age_left<=65535;far_age_right<=65535;end
        else if(memory_clear)begin far_left<=0;far_right<=0;far_age_left<=65535;far_age_right<=65535;end
        else begin
            if(tick_10ms)begin
                if(far_age_left!=65535)far_age_left<=far_age_left+1'b1;
                if(far_age_right!=65535)far_age_right<=far_age_right+1'b1;
            end
            if(range_observations[0])begin
                far_age_left<=0;
                if(range_diagnostics[7:0]==8'h11 || range_diagnostics[7:0]==8'h12)begin if(far_left<7)far_left<=far_left+1'b1;end
                else far_left<=0;
            end
            if(range_observations[3])begin
                far_age_right<=0;
                if(range_diagnostics[31:24]==8'h11 || range_diagnostics[31:24]==8'h12)begin if(far_right<7)far_right<=far_right+1'b1;end
                else far_right<=0;
            end
            if(range_diagnostics[7:0]!=0 && range_diagnostics[7:0]!=8'h11 && range_diagnostics[7:0]!=8'h12)far_left<=0;
            if(range_diagnostics[31:24]!=0 && range_diagnostics[31:24]!=8'h11 && range_diagnostics[31:24]!=8'h12)far_right<=0;
        end
    end
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin far_lf<=0;far_rf<=0;far_age_lf<=65535;far_age_rf<=65535;front_obstacle_latched<=0;end
        else if(memory_clear)begin far_lf<=0;far_rf<=0;far_age_lf<=65535;far_age_rf<=65535;front_obstacle_latched<=0;end
        else begin
            if(tick_10ms)begin
                if(far_age_lf!=65535)far_age_lf<=far_age_lf+1'b1;
                if(far_age_rf!=65535)far_age_rf<=far_age_rf+1'b1;
            end
            if(range_observations[1])begin
                far_age_lf<=0;
                if(range_diagnostics[15:8]==8'h11 || range_diagnostics[15:8]==8'h12)begin if(far_lf<7)far_lf<=far_lf+1'b1;end
                else far_lf<=0;
            end
            if(range_observations[2])begin
                far_age_rf<=0;
                if(range_diagnostics[23:16]==8'h11 || range_diagnostics[23:16]==8'h12)begin if(far_rf<7)far_rf<=far_rf+1'b1;end
                else far_rf<=0;
            end
            if((range_samples[1] && range_diagnostics[15:8]==0 && left_front_mm<=latched_bypass_mm) ||
               (range_samples[2] && range_diagnostics[23:16]==0 && right_front_mm<=latched_bypass_mm))front_obstacle_latched<=1;
            if(range_diagnostics[15:8]!=0 && range_diagnostics[15:8]!=8'h11 && range_diagnostics[15:8]!=8'h12)far_lf<=0;
            if(range_diagnostics[23:16]!=0 && range_diagnostics[23:16]!=8'h11 && range_diagnostics[23:16]!=8'h12)far_rf<=0;
        end
    end
    vehicle_odometry #(.FRONT_REFERENCE(1)) odometry(.clk(clk),.rst_n(rst_n),.tick(tick_10ms),.rebase(rebase),
        .initial_x_q8(initial_x),.initial_y_q8(initial_y),.initial_heading(initial_heading),
        .imu_yaw(last_good_yaw),.imu_fresh(imu_pending),.imu_restarted(imu_epoch_pending),.encoder1(encoder1),.encoder2(encoder2),
        .rear_x_q8(raw_rear_x),.rear_y_q8(raw_rear_y),
        .x_q8(raw_pose_x),.y_q8(raw_pose_y),.heading(pose_heading),.travel_q8(travel_q8),
        .relative1(encoder_relative1),.relative2(encoder_relative2),.updated(pose_updated),
        .encoder_fault(encoder_fault),.cos_heading(cosine),.sin_heading(sine));
    vehicle_tracker #(.CONTROL_OFFSET_MM(`VEHICLE_NOSE_FROM_REAR_MM)) tracker(.clk(clk),.rst_n(rst_n),.start(tracker_start),
        .precise_bearing(state==ALIGN || state==ARM),.bearing_dx_q8(first_dx_q8),.bearing_dy_q8(first_dy_q8),
        .target_x(target_x_mm),.target_y(target_y_mm),.pose_x(pose_x_mm),.pose_y(pose_y_mm),
        .cosine(cosine),.sine(sine),.done(tracker_done),.distance_mm(distance_mm),
        .steering_deg(tracker_steer),.bearing_tenths(bearing),.forward_mm(forward_mm),.lateral_mm(lateral_mm));
    // Relative count zeroing does not disturb the running motor speed estimator.
    // User coordinates locate the NOSE. Odometry subtracts its rotating rear
    // offset internally; bypass retains rear-axle bicycle coordinates.
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin
            timer<=0;state<=OFF;last_mode<=0;pending_mode<=0;read_wait<=0;
            arm_count<=0;range_age<=65535;imu_age<=65535;stall_count<=0;stall_count2<=0;
            previous_encoder1<=0;previous_encoder2<=0;stopped_ticks<=0;
            drive_enabled<=0;speed_mm_s<=0;steer_deg<=0;status<=0;phase<=0;route_index<=0;
            initial_heading<=0;heading_aligned<=0;visual_start_aligned<=0;visual_steer_settle<=0;initial_x<=0;initial_y<=0;first_dx_q8<=0;first_dy_q8<=0;rebase<=0;tracker_start<=0;
            target_x_mm<=0;target_y_mm<=0;target_distance_mm<=0;
            clear_count<=0;phase_origin<=0;bypass_origin<=0;blind_brake<=0;sample_pending<=0;
            latched_bypass_mm<=650;latched_brake_speed<=200;nav_resume<=0;bypass_start<=0;bypass_choose_left<=0;bypass_single_front<=0;straight_ticks<=0;
            zebra_latched<=0;zebra_clear_ticks<=0;zebra_last_seen_travel<=0;
            local_target_x<=0;local_target_y<=0;requested_speed<=0;
            braking<=0;reverse_count<=0;latched_slow_mm<=650;stop_cap<=CRUISE_MM_S;
            nearest_front<=0;lateral_target_error<=0;final_distance<=65535;
            previous_target_x<=0;previous_target_y<=0;
            brake_previous_travel<=0;measured_forward_speed<=0;last_good_yaw<=0;imu_pending<=0;wheel_still_ticks<=0;imu_epoch_pending<=0;
        end else begin
            rebase<=0;tracker_start<=0;bypass_start<=0;last_mode<=mode;
            if(tick_10ms)sample_pending<=0;
            else sample_pending<=sample_pending | range_samples;
            if(tick_10ms)begin
                if(!visual_navigation || !vision_active || !visual_start_ready)visual_steer_settle<=0;
                else if(state==ARM)begin
                    if($signed(vision_steer)-$signed(steer_deg)>1 ||
                       $signed(vision_steer)-$signed(steer_deg) < -1)visual_steer_settle<=0;
                    else if(visual_steer_settle<12)visual_steer_settle<=visual_steer_settle+1'b1;
                end
                imu_pending<=0;
                if(imu_pending)imu_epoch_pending<=0;
                timer<=0;
                if(range_age!=65535)range_age<=range_age+1'b1;
                if(imu_age!=65535)imu_age<=imu_age+1'b1;
                nearest_front<=control_distance;
                brake_previous_travel<=travel_q8;
                measured_forward_speed<=brake_delta_q8<=0?16'd0:
                    brake_delta_q8>2560?16'd1000:(brake_delta_q8*100)>>8;
                if(speed_mm_s>=60 && encoder1==previous_encoder1)begin
                    if(stall_count!=65535)stall_count<=stall_count+1'b1;
                end else stall_count<=0;
                if(speed_mm_s>=60 && encoder2==previous_encoder2)begin
                    if(stall_count2!=65535)stall_count2<=stall_count2+1'b1;
                end else stall_count2<=0;
                if(drive_enabled && speed_mm_s>=60 &&
                   ($signed(encoder1-previous_encoder1)<-2 || $signed(encoder2-previous_encoder2)<-2))begin
                    if(reverse_count!=255)reverse_count<=reverse_count+1'b1;
                end else reverse_count<=0;
                previous_encoder1<=encoder1;previous_encoder2<=encoder2;
                if(encoder1==previous_encoder1 && encoder2==previous_encoder2)begin
                    if(wheel_still_ticks<31)wheel_still_ticks<=wheel_still_ticks+1'b1;
                end else wheel_still_ticks<=0;
            end else timer<=timer+1'b1;
            if(range_frame)range_age<=0;
            if(imu_update && imu_available)begin imu_age<=0;last_good_yaw<=imu_yaw;imu_pending<=1;end
            if(imu_restart)imu_epoch_pending<=1;
            if(mode!=last_mode || !auto_mode)begin zebra_latched<=0;zebra_clear_ticks<=0;end
            else begin
                if(zebra_update && zebra_detected)begin
                    zebra_latched<=1;zebra_clear_ticks<=0;zebra_last_seen_travel<=travel_q8;
                end
                else if(zebra_latched && zebra_update)begin
                    if(zebra_clear_ticks>=5 && zebra_tail_clear)begin zebra_latched<=0;zebra_clear_ticks<=0;end
                    else if(zebra_clear_ticks<255)zebra_clear_ticks<=zebra_clear_ticks+1'b1;
                end
            end
            if(mode!=last_mode || !enabled_mode)begin
                // A finished run cannot restart until the user leaves/re-enters.
                drive_enabled<=0;speed_mm_s<=0;steer_deg<=0;requested_speed<=0;
                state<=OFF;status<=0;phase<=0;arm_count<=0;route_index<=0;stall_count<=0;stall_count2<=0;nav_resume<=0;straight_ticks<=0;
                    if(mode!=last_mode && auto_mode)begin
                    pending_mode<=mode;status<=1;braking<=0;stop_cap<=400;blind_brake<=0;
                    latched_bypass_mm<=bypass_cm<35?350:bypass_cm>150?1500:bypass_cm*16'd10;
                    visual_start_aligned<=0;
                    latched_brake_speed<=brake_speed<80?80:brake_speed>400?400:brake_speed;
                    latched_slow_mm<=slowdown_cm<25?16'd250:slowdown_cm>200?16'd2000:
                        (slowdown_cm<<3)+(slowdown_cm<<1);
                    if(mode==9)begin
                        if(route_valid && route_count>=2)begin state<=WAIT_PROFILE;read_wait<=0;end
                        else begin state<=FINISHED;status<=7;end
                    end else begin
                        initial_x<=0;initial_y<=0;initial_heading<=0;heading_aligned<=0;state<=ARM;
                    end
                end
            end else if(mode==2)begin
                drive_enabled<=remote_link && remote_run;
                speed_mm_s<=remote_link && remote_run?remote_speed:16'sd0;
                steer_deg<=remote_link?remote_steer:16'sd0;
                status<=remote_link?(remote_run?2:0):4;
            end else if(host_pause===1'b1 && pending_mode==9 && state!=FINISHED)begin
                // Communication pause keeps the route, pose and committed
                // corner. Resuming must not rebase to the original start.
                drive_enabled<=0;speed_mm_s<=0;requested_speed<=0;
            end else case(state)
                WAIT_PROFILE:begin
                    drive_enabled<=0;speed_mm_s<=0;
                    if(!route_valid)begin state<=FINISHED;status<=7;end
                    else if(profile_ready)begin state<=READ_FIRST;read_wait<=0;end
                    else if(tick_10ms)begin
                        if(arm_count>=1000)begin state<=FINISHED;status<=7;end
                        else arm_count<=arm_count+1'b1;
                    end
                end
                READ_FIRST:if(read_wait==3)begin state<=FIRST;end else read_wait<=read_wait+1'b1;
                FIRST:begin
                    initial_x<=route_x_q8*24'sd10;
                    initial_y<=24'sd819200-route_y_q8*24'sd10;
                    route_index<=1;read_wait<=0;state<=READ_SECOND;
                end
                READ_SECOND:if(read_wait==3)state<=SECOND;else read_wait<=read_wait+1'b1;
                SECOND:begin
                    first_dx_q8<=route_x_q8*24'sd10-initial_x;
                    first_dy_q8<=24'sd819200-route_y_q8*24'sd10-initial_y;
                    target_x_mm<=(route_x_q8*32'd10)>>8;
                    target_y_mm<=16'd3200-((route_y_q8*32'd10)>>8);
                    previous_target_x<=initial_x>>>8;previous_target_y<=initial_y>>>8;
                    rebase<=1;initial_heading<=0;heading_aligned<=0;state<=ALIGN;read_wait<=0;
                end
                ALIGN:begin
                    if(pose_updated)begin tracker_start<=1;state<=ARM;arm_count<=0;end
                end
                ARM:begin
                    drive_enabled<=0;speed_mm_s<=0;
                    // Apply the live correction while the wheels are stopped,
                    // through the unchanged servo slew limiter. Do not launch
                    // straight and only then begin applying acquired vision.
                    if(tick_10ms && pending_mode==9 && REQUIRE_VISION)
                        steer_deg<=visual_start_ready && vision_active?vision_steer:16'sd0;
                    if(current_tracker_done && pending_mode==9 && !heading_aligned)begin
                        initial_heading<=bearing;heading_aligned<=1;rebase<=1;
                        state<=HEADING_REBASE_WAIT;arm_count<=0;
                    end
                    if(tick_10ms)begin
                        if(arm_count>=ARM_TICKS)begin
                            if(!(pending_mode==9?imu_ready:sensors_ready))begin state<=FINISHED;status<=4;end
                            else if(pending_mode==9 && REQUIRE_VISION &&
                                    (!vision_active || !visual_start_ready))begin
                                // Remain stationary until current road observations
                                // can correct a skewed placement. S0/IMU cannot
                                // reveal the initial road-relative yaw on its own.
                                state<=ARM;
                            end else if(pending_mode==9 && REQUIRE_VISION && !visual_start_aligned)begin
                                initial_heading<=startup_curve_preview?initial_heading:startup_heading;
                                visual_start_aligned<=1;rebase<=1;
                                if(!startup_curve_preview)begin
                                    initial_x<=initial_x-(startup_side_x>>>7);
                                    initial_y<=initial_y+(startup_side_y>>>7);
                                end
                                state<=HEADING_REBASE_WAIT;
                            end else if(pending_mode==9 && REQUIRE_VISION && visual_steer_settle<12)begin
                                state<=ARM;
                            end else begin
                                rebase<=1;state<=REBASE_WAIT;read_wait<=0;phase<=0;stall_count<=0;stall_count2<=0;
                                phase_origin<=0;clear_count<=0;

                            end
                        end else arm_count<=arm_count+1'b1;
                    end
                end
                REBASE_WAIT:begin
                    // The odometer consumes rebase on the following clock.
                    // Do not evaluate its previous run's fault before that.
                    drive_enabled<=0;speed_mm_s<=0;
                    if(pose_updated)begin state<=RUN;status<=2;end
                end
                HEADING_REBASE_WAIT:begin
                    drive_enabled<=0;speed_mm_s<=0;
                    // The first route tangent is the initial world heading.
                    // Establish it in odometry before the countdown releases
                    // the car, including vertical and diagonal straight starts.
                    if(pose_updated)begin tracker_start<=1;state<=HEADING_TRACK_WAIT;end
                end
                HEADING_TRACK_WAIT:begin
                    drive_enabled<=0;speed_mm_s<=0;
                    if(current_tracker_done)begin state<=ARM;arm_count<=visual_start_aligned?ARM_TICKS:0;end
                end
                READ_TARGET:if(localizer_busy)read_wait<=0;
                    else if(read_wait==3)state<=TARGET;else read_wait<=read_wait+1'b1;
                TARGET:begin
                    final_distance<=65535;
                    target_x_mm<=(route_x_q8*32'd10)>>8;
                    target_y_mm<=16'd3200-((route_y_q8*32'd10)>>8);
                    tracker_start<=1;state<=RUN;
                end
                RUN:begin
                if(tick_10ms)begin
                        drive_enabled<=1;
                        if(speed_mm_s<requested_speed)begin
                            speed_mm_s<=speed_mm_s+2>requested_speed?requested_speed:speed_mm_s+2;
                        end else if(speed_mm_s>requested_speed)begin
                            // Envelope sets the adaptive deceleration; this cap must
                            // not clip the 400 mm/s / 25 cm case to the old 1200 mm/s2.
                            // Preserve hard high-speed braking, but use a fine
                            // 10 ms ramp for the 80 mm/s long-zone setting.
                            // A 40 mm/s step is half of that command and makes
                            // each stop-envelope update feel like a lurch.
                            speed_mm_s<=speed_mm_s-(pending_mode==7?
                                (latched_brake_speed<=120?2:40):2)<requested_speed?requested_speed:
                                speed_mm_s-(pending_mode==7?(latched_brake_speed<=120?2:40):2);
                        end
                        if(pending_mode==7 || ((pending_mode==8 || pending_mode==9) && phase==5))begin
                            // Once braking starts the allowed speed cannot grow again
                            // when an individual ToF sample fluctuates farther away.
                            if(front_known && nearest_front<=latched_slow_mm)braking<=1;
                            if(braking && distance_limit<stop_cap)stop_cap<=distance_limit;
                            requested_speed<=braking && stop_cap<motion_limit?stop_cap:motion_limit;
                            target_x_mm<=pose_x_mm+200;target_y_mm<=0;tracker_start<=1;
                            if((motion_limit==0 || stop_cap==0) && speed_mm_s==0 && wheel_still_ticks>=10)begin
                                state<=FINISHED;status<=!front_known?4:pending_mode==7?3:5;
                            end
                        end
                        if((pending_mode==8 || pending_mode==9) && !bypass_active && !nav_resume && phase!=4 && phase!=5)begin
                            if(steer_deg<8 && steer_deg > -8)begin
                                if(straight_ticks<63)straight_ticks<=straight_ticks+1'b1;
                            end else straight_ticks<=0;
                            if(pending_mode==8)begin
                                requested_speed<=motion_limit;
                                target_x_mm<=pose_x_mm+200;target_y_mm<=0;tracker_start<=1;
                            end
                            if(front_known && nearest_front<=latched_bypass_mm &&
                               (pending_mode==8 || straight_ticks>=15))begin
                                if(right_open || left_open)begin
                                    bypass_start<=1;bypass_choose_left<=chosen_left;
                                    bypass_single_front<=chosen_left?only_right_obstacle:only_left_obstacle;
                                    phase<=1;requested_speed<=80;
                                end else begin phase<=5;braking<=1;requested_speed<=0;end
                            end
                        end
                        if(pending_mode==8 && phase==4)begin
                            target_x_mm<=pose_x_mm+200;target_y_mm<=0;tracker_start<=1;
                            requested_speed<=travel_q8-phase_origin>(AFTER_MM-50)*256?0:100;
                            if(travel_q8-phase_origin>(AFTER_MM-50)*256 && speed_mm_s==0)begin state<=FINISHED;status<=3;end
                        end
                        if(bypass_active)begin
                            phase<=bypass_phase;steer_deg<=bypass_steer;
                            // A committed manoeuvre follows its one latched box;
                            // the opposite side cannot reselect or stop it.
                            requested_speed<=bypass_blocked?0:100;
                            if(bypass_blocked && speed_mm_s==0)begin state<=FINISHED;status<=5;end
                        end
                        if(pending_mode==9 && !bypass_active && !bypass_start && phase!=5)begin
                            requested_speed<=nav_resume?80:motion_limit;
                            if(route_index+1>=route_count && final_distance<100 && final_distance<distance_limit)
                                requested_speed<=final_distance;
                            if(steer_deg>25 || steer_deg < -25)begin
                                if(distance_limit>100)requested_speed<=100;
                            end
                            tracker_start<=1;
                        end
                    end
                    if(bypass_done)begin
                        phase<=pending_mode==8?4:0;phase_origin<=travel_q8;steer_deg<=0;
                        if(pending_mode==9)begin nav_resume<=1;state<=READ_TARGET;read_wait<=0;end
                    end
                    if(current_tracker_done && !bypass_active && !bypass_start && (pending_mode==7 || pending_mode==8 || pending_mode==9))begin
                        target_distance_mm<=distance_mm;steer_deg<=pending_mode==9?navigation_steer:tracker_steer;
                        if(pending_mode==9)begin
                            final_distance<=distance_mm;
                            if(nav_resume && forward_mm>=0 && distance_mm<120)nav_resume<=0;
                            if(route_index+1<route_count)begin
                                // Intermediate anchors accept a 6 cm passage window;
                                // never turn back to chase finger-placement error.
                                if((nav_resume && (forward_mm<0 || distance_mm<LOOKAHEAD_MM)) ||
                                   distance_mm<LOOKAHEAD_MM ||
                                   (committed_corner && goal_plane_reached && goal_in_corridor) ||
                                   (lane_guidance_progress && forward_mm<=10 &&
                                    lateral_mm<`VEHICLE_BODY_WIDTH_MM && lateral_mm > -`VEHICLE_BODY_WIDTH_MM) ||
                                   (route_lock && forward_mm<=10 && lateral_mm<60 && lateral_mm > -60))begin
                                    // Route progress remains in route_index; phase is reserved for bypass.
                                    route_index<=route_index+1'b1;state<=READ_TARGET;read_wait<=0;
                                    previous_target_x<=target_x_mm;previous_target_y<=target_y_mm;
                                end
                            end else begin
                                if(distance_mm<100 && distance_limit>distance_mm)requested_speed<=distance_mm;
                                // Finish at the terminal segment's arrival plane,
                                // independent of the car's current yaw. A small
                                // lateral error must not turn the goal into a
                                // point behind the car that it circles to chase.
                                if(distance_mm<=18 || goal_plane_reached)begin
                                    speed_mm_s<=0;requested_speed<=0;steer_deg<=0;drive_enabled<=0;state<=FINISHED;
                                    status<=goal_in_corridor || distance_mm<=18?3:6;
                                end
                                // No forward sample remains after a bypass. A
                                // passed goal cannot be chased indefinitely.
                                if(nav_resume && forward_mm<0)begin
                                    nav_resume<=0;speed_mm_s<=0;drive_enabled<=0;state<=FINISHED;
                                    status<=distance_mm<=60?3:6;
                                end
                            end
                            // Sampled map anchors can pass behind the nose in
                            // a legitimate committed bend. Its bounded IMU
                            // state owns deviation until the exit lane returns.
                            if(!visual_navigation && !nav_resume && !committed_corner && (lateral_mm>350 || lateral_mm < -350 ||
                               ((!lane_guidance_progress || lateral_mm>=`VEHICLE_BODY_WIDTH_MM || lateral_mm<=-`VEHICLE_BODY_WIDTH_MM) &&
                                forward_mm < -40 && distance_mm>40)))begin
                                state<=FINISHED;status<=6;drive_enabled<=0;speed_mm_s<=0;
                            end
                        end
                    end
                end
                FINISHED:begin drive_enabled<=0;speed_mm_s<=0;requested_speed<=0;steer_deg<=0;end
                default:begin drive_enabled<=0;speed_mm_s<=0;end
            endcase
            if(auto_mode && (state==RUN || state==READ_TARGET || state==TARGET))begin
                if(visual_navigation && vision_blocked)begin state<=FINISHED;status<=6;drive_enabled<=0;speed_mm_s<=0;end
                else if(!navigation_sensors)begin state<=FINISHED;status<=4;drive_enabled<=0;speed_mm_s<=0;end
                // Once the initial obstacle has committed a bypass, the front
                // beams may keep seeing that same obstacle while the car turns
                // around it. Side-edge tracking and the bypass bounds own that
                // manoeuvre; re-running the approach emergency stop here cuts
                // it off at the obstacle's side. Sensor, encoder and stall
                // watchdogs below remain active.
                else if(!bypass_active && !bypass_start && live_front<=EMERGENCY_MM)begin state<=FINISHED;status<=5;drive_enabled<=0;speed_mm_s<=0;end
                else if(encoder_fault || reverse_count>=10)begin state<=FINISHED;status<=8;drive_enabled<=0;speed_mm_s<=0;end
                else if(stall_count>=STALL_MS/10 || stall_count2>=STALL_MS/10)begin state<=FINISHED;status<=9;drive_enabled<=0;speed_mm_s<=0;end
                else if(pending_mode==9 && !route_valid)begin state<=FINISHED;status<=7;drive_enabled<=0;speed_mm_s<=0;end
            end
        end
    end
endmodule


// A lost echo is neither zero distance nor infinite clearance. Retain the last
// successful sample and spend its distance using measured wheel travel. Using
// path length (rather than just forward X) is conservative during a turn.
// No multiplier, divider, DSP or block RAM is needed by these four memories.
module vehicle_range_memory(input clk,rst_n,clear,tick,frame,
    input [7:0] diagnostic,input [15:0] measured_mm,
    input signed [31:0] travel_q8,
    output [15:0] remaining_mm,output reg known,output reg [15:0] age_ticks);
    reg [15:0] last_mm;
    reg signed [31:0] origin;
    wire signed [31:0] used_q8=travel_q8-origin;
    wire [31:0] used_mm=used_q8<=0?32'd0:$unsigned(used_q8)>>8;
    assign remaining_mm=!known?16'hffff:used_mm>=last_mm?16'd0:last_mm-used_mm;
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin last_mm<=0;origin<=0;known<=0;age_ticks<=65535;end
        else if(clear)begin last_mm<=0;origin<=0;known<=0;age_ticks<=65535;end
        else begin
            if(tick && age_ticks!=65535)age_ticks<=age_ticks+1'b1;
            // VL53L1X long-mode samples remain valid beyond the old 2m cap.
            if(frame && diagnostic==0 && measured_mm>0 && measured_mm<=4000)begin
                last_mm<=measured_mm;origin<=travel_q8;known<=1;age_ticks<=0;
            end
        end
    end
endmodule


// Shared shift/add multiply and restoring divide. No run-time combinational
// divider, DSP or BSRAM. Refreshes in <2 us at 33 MHz; control samples at 10 ms.
module vehicle_range_limit #(parameter QUADRATIC=0)(input clk,rst_n,input [15:0] distance_mm,slow_mm,stop_mm,cruise,
    output reg [15:0] speed_limit);
    reg [2:0] state;reg [5:0] bit_index;
    reg [15:0] remaining_distance,root;reg [33:0] root_remainder;
    wire [33:0] root_trial={root_remainder[31:0],quotient[31:30]};
    wire [33:0] root_test={16'd0,root,2'b01};
    reg [31:0] product,addend,quotient;
    reg [15:0] multiplier,denominator;
    reg [16:0] remainder;
    wire [16:0] trial={remainder[15:0],quotient[31]};
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin state<=0;bit_index<=0;product<=0;addend<=0;quotient<=0;
            multiplier<=0;denominator<=1;remainder<=0;speed_limit<=0;remaining_distance<=0;root<=0;root_remainder<=0;end
        else case(state)
            0:begin
                if(distance_mm<=stop_mm || slow_mm<=stop_mm)speed_limit<=0;
                else if(distance_mm>=slow_mm)speed_limit<=cruise;
                else begin product<=0;addend<=QUADRATIC?cruise:distance_mm-stop_mm;multiplier<=cruise;
                    remaining_distance<=distance_mm-stop_mm;
                    denominator<=slow_mm-stop_mm;bit_index<=0;state<=1;end
            end
            1:begin
                product<=product+(multiplier[0]?addend:32'd0);
                multiplier<=multiplier>>1;addend<=addend<<1;
                if(bit_index==15)begin
                    if(QUADRATIC)begin addend<=product+(multiplier[0]?addend:32'd0);
                        multiplier<=remaining_distance;product<=0;state<=3;end
                    else begin quotient<=product+(multiplier[0]?addend:32'd0);remainder<=0;state<=2;end
                    bit_index<=0;end
                else bit_index<=bit_index+1'b1;
            end
            2:begin
                if(trial>={1'b0,denominator})begin remainder<=trial-denominator;
                    quotient<={quotient[30:0],1'b1};end
                else begin remainder<=trial;quotient<={quotient[30:0],1'b0};end
                if(bit_index==31)begin
                    if(QUADRATIC)begin root<=0;root_remainder<=0;bit_index<=0;state<=4;end
                    else begin speed_limit<={quotient[14:0],trial>={1'b0,denominator}};state<=0;end
                end else bit_index<=bit_index+1'b1;
            end
            3:begin
                product<=product+(multiplier[0]?addend:32'd0);
                multiplier<=multiplier>>1;addend<=addend<<1;
                if(bit_index==15)begin quotient<=product+(multiplier[0]?addend:32'd0);remainder<=0;bit_index<=0;state<=2;end
                else bit_index<=bit_index+1'b1;
            end
            4:begin
                quotient<=quotient<<2;
                root_remainder<=root_trial>=root_test?root_trial-root_test:root_trial;
                root<={root[14:0],root_trial>=root_test};
                if(bit_index==15)begin speed_limit<={root[14:0],root_trial>=root_test};state<=0;end
                else bit_index<=bit_index+1'b1;
            end
            default:state<=0;
        endcase
    end
endmodule
