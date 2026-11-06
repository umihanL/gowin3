module navigation_ui(
    input clk,rst_n,data_req,
    input imu_available,imu_restart,imu_rezero,input signed [15:0] imu_yaw,
    output start_heading_valid,output signed [15:0] start_heading,
    output map_start_valid,output [7:0] map_start_x,map_start_y,
    output [31:0] map_start_edit_generation,
    input zebra_enable,zebra_can_change,output zebra_toggle,
    input host_route_command,input [7:0] host_route_opcode,input [31:0] host_route_arguments,
    output host_route_page,output [7:0] host_route_result,host_route_status,
    input [11:0] x_addr,y_addr, input [15:0] video,preview,
    output [7:0] binary_threshold,output binary_page,

    input host_point_select,host_point_start,host_point_cancel,input [3:0] host_point_id,
    output host_point_select_ready,host_point_route_ready,host_point_planning,
    output [3:0] selected_point,output [2:0] host_point_error,
    input touch_valid,touch_down,input [15:0] touch_x,touch_y,
    input speed_edit,speed_increase,speed_decrease,
    output control_remote,control_host,navigation_avoidance,output [15:0] configured_speed,slowdown_cm,bypass_cm,brake_speed,
    output [10:0] servo_center_tenths,
    output touch_run,output signed [15:0] touch_speed,touch_steer,
    output [15:0] pixel,
    output [3:0] operation_mode,
    output navigation_request,route_valid,
    output path_valid,output [7:0] path_x,path_y,output [3:0] path_segment,
    output [16:0] path_x_q8,path_y_q8,
    input [10:0] execution_index,output [11:0] execution_count,
    output [16:0] execution_x_q8,execution_y_q8,output execution_lock,
    output [31:0] map_signature,output reg [15:0] map_generation,
    output map_route_valid,map_planning,input [10:0] map_route_index,
    output [16:0] map_route_x_q8,map_route_y_q8,
    output reg [15:0] map_barrier_generation,output [3:0] map_barrier_valid,
    output [35:0] map_barrier_x0,map_barrier_y0,map_barrier_x1,map_barrier_y1,
    input [15:0] left_mm,left_front_mm,right_front_mm,right_mm,front_average_mm,
    input front_valid,input [31:0] tof_diagnostics,input [3:0] motion_status,motion_phase,
    input signed [15:0] commanded_speed,commanded_steer,pose_heading,target_heading,
    input signed [23:0] pose_x_q8,pose_y_q8,
    input signed [15:0] target_x_mm,target_y_mm,
    input [15:0] target_distance_mm,
    input profile_ready,input [3:0] profile_segments,
    input [15:0] profile_length_mm,profile_left_revs,profile_right_revs,
    input signed [15:0] profile_entry_heading,profile_exit_heading,
    input [7:0] profile_peak_steer
);
    `include "nav_metadata.vh"
    `include "nav_host_map_id.vh"
    assign map_signature=HOST_MAP_SIGNATURE;
    wire [1:0] tool,live_page;
    assign binary_page=operation_mode==1 && live_page==2;
    wire have_start,have_goal,planning;
    assign host_point_planning=planning;
    wire [7:0] sx,sy,gx,gy;
    assign map_start_valid=have_start && start_heading_valid;
    assign map_start_x=sx;assign map_start_y=sy;
    wire [3:0] count;
    wire [63:0] wx,wy;
    wire [8:0] selected_x,selected_y;
    wire [5:0] message;
    wire [17:0] length_cm;
    wire edit_distance;wire [2:0] edit_parameter;wire [15:0] keypad_value;wire [2:0] keypad_digits;wire keypad_error;
    wire [3:0] barrier_valid;wire [35:0] barrier_x0,barrier_y0,barrier_x1,barrier_y1;
    wire barrier_pending;wire [8:0] barrier_anchor_x,barrier_anchor_y;
    assign map_barrier_valid=barrier_valid;
    assign map_barrier_x0=barrier_x0;assign map_barrier_y0=barrier_y0;
    assign map_barrier_x1=barrier_x1;assign map_barrier_y1=barrier_y1;
    vehicle_settings settings(.clk(clk),.rst_n(rst_n),.screen(operation_mode),
        .touch_valid(touch_valid),.touch_down(touch_down),.touch_x(touch_x),.touch_y(touch_y),
        .remote_increase(speed_increase),.remote_decrease(speed_decrease),
        .control_remote(control_remote),.control_host(control_host),.navigation_avoidance(navigation_avoidance),.configured_speed(configured_speed),
        .slowdown_cm(slowdown_cm),.bypass_cm(bypass_cm),.brake_speed(brake_speed),
        .edit_distance(edit_distance),.edit_parameter(edit_parameter),
        .servo_center_tenths(servo_center_tenths),
        .keypad_value(keypad_value),.keypad_digits(keypad_digits),.keypad_error(keypad_error),
        .touch_run(touch_run),.touch_speed(touch_speed),.touch_steer(touch_steer));
    wire [14:0] search_addr,display_plan_addr,route_addr,trace_addr,refine_plan_addr;
    wire [16:0] display_map_addr;
    wire [15:0] search_xy,refine_plan_xy;wire [17:0] refine_raw_xy;
    wire [8:0] display_cx=x_addr>=24 && x_addr<376 && y_addr>=112 && y_addr<432?x_addr-24:0;
    wire [8:0] display_cy=x_addr>=24 && x_addr<376 && y_addr>=112 && y_addr<432?y_addr-112:0;
    wire [15:0] active_plan_xy=search_xy;
    wire blocked,display_blocked,display_plan_blocked,route_write,route_bit;
    wire rom_blocked,rom_display_blocked,rom_display_plan_blocked;
    wire forbidden_plan,forbidden_raw,forbidden_display_plan;
    wire [14:0] active_plan_addr=search_addr;
    wire [16:0] active_raw_addr=display_map_addr;
    assign blocked=rom_blocked | forbidden_plan;
    assign display_blocked=rom_display_blocked | forbidden_raw;
    assign display_plan_blocked=rom_display_plan_blocked | forbidden_display_plan;
    nav_barrier_overlay #(.DIRECT_COORDINATES(1)) barriers(.clk(clk),.valid(barrier_valid),
        .plan_coordinates({active_plan_xy[15:8],1'b1,active_plan_xy[7:0],1'b1}),
        .raw_coordinates({display_cy,display_cx}),
        .display_coordinates({display_cy[8:1],1'b1,display_cx[8:1],1'b1}),
        .x0(barrier_x0),.y0(barrier_y0),.x1(barrier_x1),.y1(barrier_y1),
        .plan_addr(active_plan_addr),.raw_addr(active_raw_addr),.display_plan_addr(display_plan_addr),
        .plan_hit(forbidden_plan),.raw_hit(forbidden_raw),.display_plan_hit(forbidden_display_plan));
    wire [15:0] expanded;
    wire [7:0] map_directions;
    wire map_central;
    wire trace_valid,trace_lock,refine_clear,refine_cancel,refine_start,refine_busy,refine_done;
    reg previous_route_valid,previous_planning;
    reg [147:0] previous_barriers;
    wire [147:0] current_barriers={barrier_valid,barrier_x0,barrier_y0,barrier_x1,barrier_y1};
    wire barriers_changed=current_barriers!=previous_barriers;
    wire map_changing=refine_clear || refine_cancel || refine_start || barriers_changed ||
                      route_valid!=previous_route_valid || planning!=previous_planning;
    assign map_route_valid=route_valid && !planning && !refine_busy && !map_changing;
    assign map_planning=planning || refine_busy;
    // Generation invalidates every downloaded point at edit/start/cancel/
    // publication transitions. 0 and FFFF are never valid generations.
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin
            map_generation<=1;map_barrier_generation<=1;
            previous_route_valid<=0;previous_planning<=0;previous_barriers<=0;
        end else begin
            previous_route_valid<=route_valid;previous_planning<=planning;previous_barriers<=current_barriers;
            if(map_changing)map_generation<=map_generation>=16'hfffe ? 16'd1:map_generation+1'b1;
            if(barriers_changed)map_barrier_generation<=map_barrier_generation>=16'hfffe ? 16'd1:map_barrier_generation+1'b1;
        end
    end
    wire [7:0] trace_x,trace_y;
    wire [3:0] trace_segment;
    wire [2:0] refine_error;
    wire [17:0] refined_length;
    wire [16:0] refine_raw_addr;
    wire refine_raw_request;
    wire refine_raw_grant=1'b0;
    wire refine_route_write,refine_route_bit;
    wire [14:0] refine_route_addr;
    assign path_x=path_x_q8[16:9];
    assign path_y=path_y_q8[16:9];
    wire snap_start,snap_done,snap_accept,snapped_heading_valid;wire signed [15:0] snapped_heading;
    wire [7:0] snap_request_x,snap_request_y,snapped_x,snapped_y;
    wire planned_anchor_valid;wire [2:0] planned_anchor_index;wire [7:0] planned_anchor_x,planned_anchor_y;
    wire region_preference_enabled;wire [7:0] region_x0,region_y0,region_x1,region_y1;
    nav_control control(.host_selected(control_host),.start_edit_generation(map_start_edit_generation),
        .imu_available(imu_available),.imu_restart(imu_restart),.imu_rezero(imu_rezero),.imu_yaw(imu_yaw),
        .start_heading_valid(start_heading_valid),.start_heading(start_heading),
        .region_preference_enabled(region_preference_enabled),.region_x0(region_x0),.region_y0(region_y0),.region_x1(region_x1),.region_y1(region_y1),
        .zebra_can_change(zebra_can_change),.zebra_toggle(zebra_toggle),
        .host_command(host_route_command),.host_opcode(host_route_opcode),.host_arguments(host_route_arguments),
        .host_motion_status(motion_status),.host_route_page(host_route_page),.host_result(host_route_result),.host_route_status(host_route_status),.host_point_select(host_point_select),
        .host_point_start(host_point_start),.host_point_cancel(host_point_cancel),.host_point_id(host_point_id),
        .host_point_select_ready(host_point_select_ready),.host_point_route_ready(host_point_route_ready),
        .selected_point(selected_point),.host_point_error(host_point_error),.snap_start(snap_start),.snap_request_x(snap_request_x),.snap_request_y(snap_request_y),
        .planned_anchor_valid(planned_anchor_valid),.planned_anchor_index(planned_anchor_index),
        .planned_anchor_x(planned_anchor_x),.planned_anchor_y(planned_anchor_y),
        .snap_done(snap_done),.snap_accept(snap_accept),.snapped_x(snapped_x),.snapped_y(snapped_y),
        .snapped_heading_valid(snapped_heading_valid),.snapped_heading(snapped_heading),.clk(clk),.rst_n(rst_n),.edit_distance(edit_distance),
        .touch_valid(touch_valid),.touch_down(touch_down),.touch_x(touch_x),.touch_y(touch_y),
        .screen(operation_mode),.live_page(live_page),.binary_threshold(binary_threshold),.tool(tool),.have_start(have_start),.have_goal(have_goal),
        .sx(sx),.sy(sy),.gx(gx),.gy(gy),.waypoint_count(count),.waypoint_x(wx),.waypoint_y(wy),
        .selected_x_cm(selected_x),.selected_y_cm(selected_y),.message(message),
        .route_ready(route_valid),.navigation_active(navigation_request),.planning(planning),
        .barrier_valid(barrier_valid),.barrier_x0(barrier_x0),.barrier_y0(barrier_y0),
        .barrier_x1(barrier_x1),.barrier_y1(barrier_y1),.barrier_pending(barrier_pending),
        .barrier_anchor_x(barrier_anchor_x),.barrier_anchor_y(barrier_anchor_y),
        .map_addr(search_addr),.map_xy(search_xy),.map_blocked(blocked),.route_write(route_write),
        .map_directions(map_directions),
        .map_central(map_central),
        .route_addr(route_addr),.route_bit(route_bit),.length_cm(length_cm),
        .astar_trace_addr(trace_addr),.astar_trace_valid(trace_valid),
        .astar_trace_x(trace_x),.astar_trace_y(trace_y),.segment(trace_segment),.expanded(expanded),
        .refine_clear(refine_clear),.refine_cancel(refine_cancel),.trace_lock(trace_lock),
        .refine_start(refine_start),.refine_done(refine_done),.refine_error(refine_error),
        .refined_length_cm(refined_length));
    assign refine_plan_addr=0;assign refine_plan_xy=0;assign refine_raw_addr=0;
    assign refine_raw_xy=0;assign refine_raw_request=0;
    nav_road_planner road_planner(.clk(clk),.rst_n(rst_n),.clear(refine_clear),.cancel(refine_cancel),
        .start_heading_valid(start_heading_valid),.start_heading(start_heading),
        .region_preference_enabled(region_preference_enabled),.region_x0(region_x0),.region_y0(region_y0),.region_x1(region_x1),.region_y1(region_y1),
        .anchor_valid(planned_anchor_valid),.anchor_index(planned_anchor_index),
        .anchor_x(planned_anchor_x),.anchor_y(planned_anchor_y),
        .snap_start(snap_start),.snap_x(snap_request_x),.snap_y(snap_request_y),
        .snap_done(snap_done),.snap_accept(snap_accept),.snapped_x(snapped_x),.snapped_y(snapped_y),
        .snapped_heading_valid(snapped_heading_valid),.snapped_heading(snapped_heading),
        .sx(sx),.sy(sy),.gx(gx),.gy(gy),.waypoint_count(count),.waypoint_x(wx),.waypoint_y(wy),
        .barrier_valid(barrier_valid),.barrier_x0(barrier_x0),.barrier_y0(barrier_y0),
        .barrier_x1(barrier_x1),.barrier_y1(barrier_y1),
        .start(refine_start),.busy(refine_busy),.done(refine_done),.error(refine_error),
        .route_write(refine_route_write),.route_bit(refine_route_bit),.route_addr(refine_route_addr),
        .sample_valid(path_valid),.sample_x_q8(path_x_q8),.sample_y_q8(path_y_q8),
        .sample_segment(path_segment),.length_cm(refined_length),
        .execution_index(execution_index),.execution_count(execution_count),
        .telemetry_index(map_route_index),.telemetry_x_q8(map_route_x_q8),.telemetry_y_q8(map_route_y_q8),
        .execution_x_q8(execution_x_q8),.execution_y_q8(execution_y_q8),.execution_lock(execution_lock));
    wire [31:0] metadata;
    reg [6:0] metadata_index;
    reg [31:0] metadata_sum;
    reg metadata_valid;
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin metadata_index<=0;metadata_sum<=0;metadata_valid<=0;end
        else if(metadata_index<65)begin
            metadata_index<=metadata_index+1'b1;
            if(metadata_index!=0)metadata_sum<=metadata_sum^metadata;
            if(metadata_index==64)metadata_valid<=((metadata_sum^metadata)==MAP_SIGNATURE);
        end
    end
    // Raw ROM is shared during horizontal/vertical blanking; active pixels
    // always have priority, preserving the one-cycle rendering pipeline.
    nav_map_rom maps(.clk(clk),.display_addr(active_raw_addr),.display_blocked(rom_display_blocked),
        .plan_addr(active_plan_addr),.plan_blocked(rom_blocked),
        .plan_directions(map_directions),
        .plan_central(map_central),
        .display_plan_addr(display_plan_addr),.display_plan_blocked(rom_display_plan_blocked),
        .metadata_addr(metadata_index[5:0]),.metadata(metadata));
    reg [15:0] centre_map[0:1759];reg [15:0] centre_word;reg [3:0] centre_bit;
    initial begin
        `include "nav_centerline_bitmap.vh"
    end
    always @(posedge clk)begin centre_word<=centre_map[display_plan_addr[14:4]];centre_bit<=display_plan_addr[3:0];end
    reg route[0:28159];
    reg route_pixel;
    wire travelled_write;wire [14:0] travelled_addr;
    nav_travelled_raster travelled(.clk(clk),.rst_n(rst_n),
        .reset_route(refine_clear || refine_start || route_write || refine_route_write),
        .running(operation_mode==9 && motion_status==2),.finished(operation_mode==9 && motion_status==3),
        .pose_x_q8(pose_x_q8),.pose_y_q8(pose_y_q8),.pose_heading(pose_heading),
        .write(travelled_write),.addr(travelled_addr));
    always @(posedge clk)begin
        if(refine_route_write)route[refine_route_addr]<=refine_route_bit;
        else if(route_write)route[route_addr]<=route_bit;
        // Only this display raster is erased. Planner samples / UART route
        // downloads / pause-resume progress remain untouched.
        else if(travelled_write)route[travelled_addr]<=0;
        route_pixel<=route[display_plan_addr];
    end
    nav_renderer renderer(.selected_point(selected_point),.clk(clk),.rst_n(rst_n),.data_req(data_req),.x_addr(x_addr),.y_addr(y_addr),
        .zebra_enable(zebra_enable),.zebra_can_change(zebra_can_change),
        .control_remote(control_remote),.control_host(control_host),.navigation_avoidance(navigation_avoidance),.configured_speed(configured_speed),
        .slowdown_cm(slowdown_cm),.edit_distance(edit_distance),.speed_edit(speed_edit),
        .bypass_cm(bypass_cm),.brake_speed(brake_speed),.edit_parameter(edit_parameter),
        .servo_center_tenths(servo_center_tenths),
        .barrier_pixel(forbidden_raw),.barrier_pending(barrier_pending),
        .barrier_anchor_x(barrier_anchor_x),.barrier_anchor_y(barrier_anchor_y),
        .keypad_value(keypad_value),.keypad_digits(keypad_digits),.keypad_error(keypad_error),.touch_run(touch_run),
        .video(video),.preview(preview),.live_page(live_page),.binary_threshold(binary_threshold),.screen(operation_mode),.tool(tool),.have_start(have_start),.have_goal(have_goal),
        .sx(sx),.sy(sy),.gx(gx),.gy(gy),.waypoint_count(count),.waypoint_x(wx),.waypoint_y(wy),
        .start_heading_valid(start_heading_valid),.start_heading(start_heading),
        .selected_x_cm(selected_x),.selected_y_cm(selected_y),.message(metadata_valid?message:6'd34),
        .route_ready(route_valid),.planning(planning),.length_cm(length_cm),
        .map_addr(display_map_addr),.plan_addr(display_plan_addr),.map_blocked(display_blocked),
        .centre_pixel(centre_word[centre_bit]),.plan_blocked(display_plan_blocked),.route_pixel(route_pixel),.pixel(pixel),
        .left_mm(left_mm),.left_front_mm(left_front_mm),.right_front_mm(right_front_mm),
        .right_mm(right_mm),.front_average_mm(front_average_mm),.front_valid(front_valid),
        .tof_diagnostics(tof_diagnostics),.motion_status(motion_status),.motion_phase(motion_phase),
        .commanded_speed(commanded_speed),.commanded_steer(commanded_steer),
        .pose_x_q8(pose_x_q8),.pose_y_q8(pose_y_q8),.pose_heading(pose_heading),
        .target_x_mm(target_x_mm),.target_y_mm(target_y_mm),.target_heading(target_heading),
        .target_distance_mm(target_distance_mm),
        .profile_ready(profile_ready),.profile_segments(profile_segments),.profile_length_mm(profile_length_mm),
        .profile_left_revs(profile_left_revs),.profile_right_revs(profile_right_revs),
        .profile_entry_heading(profile_entry_heading),.profile_exit_heading(profile_exit_heading),.profile_peak_steer(profile_peak_steer));
endmodule

// Remove the driven footprint from the display bitmap using measured odometry,
// never the look-ahead route index (which can be ahead of the actual car).
// A 4 cm rear half-disc tolerates 2 cm map raster quantization. A future
// intersection remains in the execution route even if its visible cell was driven.
module nav_travelled_raster(
    input clk,rst_n,reset_route,running,finished,
    input signed [23:0] pose_x_q8,pose_y_q8,input signed [15:0] pose_heading,
    output reg write,output reg [14:0] addr
);
    reg [2:0] ox,oy;reg [7:0] cx,cy;reg signed [1:0] fx,fy;
    reg active,finish_done;reg [14:0] finish_index;
    wire pose_valid=pose_x_q8>=0 && pose_x_q8<24'sd901120 &&
                    pose_y_q8>0 && pose_y_q8<=24'sd819200;
    wire [7:0] cell_x=pose_x_q8[19:8]/20;
    wire [7:0] cell_y=(13'd3200-{1'b0,pose_y_q8[19:8]})/20;
    wire signed [9:0] dx=$signed({1'b0,ox})-10'sd2,dy=$signed({1'b0,oy})-10'sd2;
    wire signed [10:0] xx=$signed({1'b0,cx})+dx,yy=$signed({1'b0,cy})+dy;
    wire signed [12:0] forward=dx*fx-dy*fy;
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin
            write<=0;addr<=0;ox<=0;oy<=0;cx<=0;cy<=0;fx<=1;fy<=0;
            active<=0;finish_done<=0;finish_index<=0;
        end else begin
            write<=0;
            if(reset_route)begin active<=0;finish_done<=0;finish_index<=0;end
            else if(finished && !finish_done)begin
                active<=0;write<=1;addr<=finish_index;
                if(finish_index==28159)finish_done<=1;else finish_index<=finish_index+1'b1;
            end else if(!running || !pose_valid)active<=0;
            else if(!active)begin
                cx<=cell_x;cy<=cell_y;ox<=0;oy<=0;active<=1;
                fx<=pose_heading>=-675 && pose_heading<675 ? 2'sd1:
                    pose_heading>=1125 || pose_heading < -1125 ? -2'sd1:2'sd0;
                fy<=pose_heading>=225 && pose_heading<1575 ? 2'sd1:
                    pose_heading>=-1575 && pose_heading < -225 ? -2'sd1:2'sd0;
            end else begin
                if(xx>=0 && xx<176 && yy>=0 && yy<160 && forward<=0 && dx*dx+dy*dy<=4)begin
                    write<=1;addr<=yy*15'd176+xx;
                end
                if(ox==4)begin ox<=0;if(oy==4)active<=0;else oy<=oy+1'b1;end
                else ox<=ox+1'b1;
            end
        end
    end
endmodule
