module nav_control(
    input clk,rst_n,input edit_distance,
    input zebra_can_change,output reg zebra_toggle,
    input host_selected,host_command,input [7:0] host_opcode,input [31:0] host_arguments,
    input imu_available,imu_restart,input signed [15:0] imu_yaw,
    output start_heading_valid,output signed [15:0] start_heading,
    output reg [31:0] start_edit_generation,
    input [3:0] host_motion_status,output reg host_route_page,
    output reg [7:0] host_result,output [7:0] host_route_status,
    input host_point_select,host_point_start,host_point_cancel, input [3:0] host_point_id,
    output host_point_select_ready,host_point_route_ready,
    output reg [3:0] selected_point,output reg [2:0] host_point_error,
    output reg snap_start,output [7:0] snap_request_x,snap_request_y,
    input snap_done,snap_accept,input [7:0] snapped_x,snapped_y,
    input planned_anchor_valid,input [2:0] planned_anchor_index,
    input [7:0] planned_anchor_x,planned_anchor_y,
    input touch_valid,touch_down, input [15:0] touch_x,touch_y,
    output reg [1:0] live_page, output reg [7:0] binary_threshold,
    output reg [3:0] screen, output reg [1:0] tool,
    output reg have_start,have_goal, output reg [7:0] sx,sy,gx,gy,
    output reg [3:0] waypoint_count,
    output [63:0] waypoint_x,waypoint_y,
    output reg [8:0] selected_x_cm,selected_y_cm,
    output reg [5:0] message,
    output reg route_ready,navigation_active,
    output reg planning,
    output reg [3:0] barrier_valid,
    output reg [35:0] barrier_x0,barrier_y0,barrier_x1,barrier_y1,
    output reg barrier_pending,output reg [8:0] barrier_anchor_x,barrier_anchor_y,
    output [14:0] map_addr, input map_blocked,output [15:0] map_xy,
    input [7:0] map_directions,
    input map_central,
    output reg route_write,route_bit, output reg [14:0] route_addr,
    output reg [17:0] length_cm,
    output [14:0] astar_trace_addr, output astar_trace_valid,
    output [7:0] astar_trace_x,astar_trace_y,
    output reg [3:0] segment,
    output [15:0] expanded,
    output refine_clear,refine_cancel,trace_lock,
    output reg refine_start,
    input refine_done, input [2:0] refine_error,
    input [17:0] refined_length_cm
);
    localparam HOME=0,LIVE=1,REMOTE=2,AVOID=3,NAV=4,MAP=5,
               DETECT=6,STOP=7,DODGE=8,NAV_RUN=9,SETTINGS=10,MODES=11,PARAMETERS=12,KEYPAD=13,ADJUST=14,SPEED_MENU=15;
    localparam WIPE=0,IDLE=1,PROBE_WAIT=2,PROBE_TEST=3,
               LAUNCH=4,RUN=5,SNAP_SETUP=6,SNAP_NEXT=7,REFINE=8;
    reg [3:0] state;
    `include "nav_destinations.vh"
    wire [15:0] destination=nav_destination(host_point_id);
    assign host_point_select_ready=host_selected && screen==MAP && have_start && start_heading_valid && state==IDLE && !planning && !navigation_active;
    assign host_point_route_ready=host_selected && screen==MAP && have_goal && route_ready && start_heading_valid && state==IDLE && !planning;
    reg host_plan_error;
    reg heading_reference_valid;reg signed [15:0] heading_reference,yaw_reference;
    wire signed [17:0] current_map_heading=$signed(heading_reference)+$signed(imu_yaw)-$signed(yaw_reference);
    assign start_heading=current_map_heading>1800?current_map_heading-3600:current_map_heading < -1800?current_map_heading+3600:current_map_heading;
    assign start_heading_valid=heading_reference_valid && imu_available===1'b1;
    assign host_route_status=!host_route_page?8'd0:planning?8'd7:host_plan_error?8'd9:
        navigation_active && host_motion_status==3?8'd11:
        navigation_active && host_motion_status>=4?8'd12:
        navigation_active && host_motion_status==2?8'd10:route_ready?8'd8:8'd0;
    always @*begin
      host_result=0;
      if(host_selected!==1'b1)host_result=1;
      else if(host_opcode==8'h25)begin
          // Physical stop/pump interlocks are checked by vehicle_host_link.
          if(host_arguments!=0)host_result=2;
          else if(planning || state!=IDLE)host_result=4;
      end
      else if(host_motion_status==1 || host_motion_status==2 || planning || state!=IDLE)host_result=4;
      else case(host_opcode)
       8'h20,8'h21,8'h22:begin
         if(host_arguments[7:0]>175 || host_arguments[15:8]>159 || host_arguments[31:16]!=0)host_result=2;
         else if(host_opcode!=8'h20 && !host_route_page)host_result=3;
         else if(host_opcode==8'h21 && waypoint_count>=8)host_result=2;
       end
       8'h23:if(host_arguments!=0)host_result=2;else if(!host_route_page || !have_start || !have_goal)host_result=3;else if(!start_heading_valid)host_result=7;
       8'h24:if(host_arguments[31:16]!=0 || $signed(host_arguments[15:0]) < -1800 || $signed(host_arguments[15:0])>1800)host_result=2;
          else if(!have_start || screen!=MAP)host_result=3;else if(imu_available!==1'b1)host_result=7;
       default:host_result=5;
      endcase
    end
    reg [7:0] wx[0:7],wy[0:7];
    genvar k;
    generate for(k=0;k<8;k=k+1)begin:pack_points
        assign waypoint_x[k*8+:8]=wx[k];
        assign waypoint_y[k*8+:8]=wy[k];
    end endgenerate
    reg held,go_after_wipe;
    reg [14:0] wipe_index,probe_id;
    reg [7:0] probe_x,probe_y;
    reg [7:0] click_x,click_y,snap_x,snap_y;
    reg signed [4:0] offset_x,offset_y;
    reg scanning,snap_found,snap_attempted;
    reg align_checked,align_probe;
    reg [7:0] unaligned_x,unaligned_y;
    reg anchor_valid;reg [7:0] anchor_x,anchor_y;
    wire [8:0] align_dx=probe_x>anchor_x?probe_x-anchor_x:anchor_x-probe_x;
    wire [8:0] align_dy=probe_y>anchor_y?probe_y-anchor_y:anchor_y-probe_y;
    // Project slight touch offsets onto the preceding straight segment.
    // A long axis (>20 cm) and <=8 cm cross error distinguish it from a corner.
    always @*begin
        anchor_valid=0;anchor_x=0;anchor_y=0;
        if(tool==0)begin
            anchor_valid=waypoint_count!=0 || have_goal;
            anchor_x=waypoint_count!=0?wx[0]:gx;anchor_y=waypoint_count!=0?wy[0]:gy;
        end else begin
            anchor_valid=waypoint_count!=0 || have_start;
            anchor_x=waypoint_count!=0?wx[waypoint_count-1'b1]:sx;
            anchor_y=waypoint_count!=0?wy[waypoint_count-1'b1]:sy;
        end
    end
    reg [3:0] snap_distance;
    wire [4:0] offset_distance=(offset_x<0?-offset_x:offset_x)+
                               (offset_y<0?-offset_y:offset_y);
    wire signed [9:0] candidate_x=$signed({2'b0,click_x})+offset_x;
    wire signed [9:0] candidate_y=$signed({2'b0,click_y})+offset_y;
    reg astar_start,astar_cancel;
    reg [1:0] barrier_slot;
    wire [8:0] barrier_click_x=u-24,barrier_click_y=v-112;
    wire [8:0] barrier_dx=barrier_click_x>barrier_anchor_x?barrier_click_x-barrier_anchor_x:barrier_anchor_x-barrier_click_x;
    wire [8:0] barrier_dy=barrier_click_y>barrier_anchor_y?barrier_click_y-barrier_anchor_y:barrier_anchor_y-barrier_click_y;
    reg [7:0] from_x,from_y,to_x,to_y;
    wire astar_busy,astar_done;
    wire [2:0] astar_error;
    wire [13:0] astar_cost;
    wire [14:0] astar_map_addr;
    wire [15:0] astar_map_xy;
    wire [15:0] u=touch_x,v=touch_y;
    wire tap=touch_valid && touch_down && !held && touch_x<800 && touch_y<480;
    assign map_addr=0;
    assign map_xy=0;
    assign refine_clear=state==WIPE && wipe_index==0;
    assign refine_cancel=astar_cancel;
    assign trace_lock=0;
    assign astar_trace_valid=0;assign astar_trace_addr=0;
    assign astar_trace_x=0;assign astar_trace_y=0;assign expanded=0;
    assign snap_request_x=click_x;assign snap_request_y=click_y;
    integer i;
    always @* begin
        route_write=0;route_addr=0;route_bit=0;
        if(state==WIPE) begin route_write=1;route_addr=wipe_index;end
        else if(planning && astar_trace_valid) begin
            route_write=1;route_addr=astar_trace_addr;route_bit=1;
        end
    end
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            screen<=HOME;live_page<=0;binary_threshold<=96;tool<=0;have_start<=0;have_goal<=0;sx<=0;sy<=0;gx<=0;gy<=0;
            host_route_page<=0;host_plan_error<=0;selected_point<=0;host_point_error<=0;
            heading_reference_valid<=0;heading_reference<=0;yaw_reference<=0;start_edit_generation<=0;
            zebra_toggle<=0;
            waypoint_count<=0;selected_x_cm<=0;selected_y_cm<=0;message<=24;
            route_ready<=0;navigation_active<=0;planning<=0;state<=WIPE;
            held<=0;go_after_wipe<=0;wipe_index<=0;probe_id<=0;probe_x<=0;probe_y<=0;
            click_x<=0;click_y<=0;snap_x<=0;snap_y<=0;offset_x<=0;offset_y<=0;
            scanning<=0;snap_found<=0;snap_attempted<=0;snap_distance<=15;
            align_checked<=0;align_probe<=0;unaligned_x<=0;unaligned_y<=0;
            astar_start<=0;astar_cancel<=0;from_x<=0;from_y<=0;to_x<=0;to_y<=0;
            length_cm<=0;segment<=0;refine_start<=0;snap_start<=0;
            barrier_valid<=0;barrier_x0<=0;barrier_y0<=0;barrier_x1<=0;barrier_y1<=0;
            barrier_pending<=0;barrier_anchor_x<=0;barrier_anchor_y<=0;barrier_slot<=0;
            for(i=0;i<8;i=i+1)begin wx[i]<=0;wy[i]<=0;end
        end else begin
            astar_start<=0;astar_cancel<=0;refine_start<=0;snap_start<=0;
            zebra_toggle<=0;
            if(imu_restart===1'b1)heading_reference_valid<=0;
            if(planning && planned_anchor_valid===1'b1 && planned_anchor_index<waypoint_count)begin
                wx[planned_anchor_index]<=planned_anchor_x;wy[planned_anchor_index]<=planned_anchor_y;
            end
            if(touch_valid) held<=touch_down;
            // Continuous slider drag, 256 positions, four-pixel endpoint tolerance.
            if(screen==LIVE && live_page==2 && touch_valid && touch_down && touch_x>=750 && touch_x<800 && touch_y>=50 && touch_y<435)
                binary_threshold<=touch_y>=431?0:touch_y<=52?255:((16'd431-touch_y)*16'd171)>>8;
            case(state)
                WIPE:if(wipe_index==28159) begin
                    if(go_after_wipe) begin
                        from_x<=sx;from_y<=sy;to_x<=waypoint_count!=0?wx[0]:gx;
                        to_y<=waypoint_count!=0?wy[0]:gy;segment<=0;state<=LAUNCH;
                    end else state<=IDLE;
                end else wipe_index<=wipe_index+1'b1;
                PROBE_WAIT:begin snap_start<=1;state<=PROBE_TEST;end
                PROBE_TEST:if(snap_done)begin
                    state<=IDLE;
                    if(!snap_accept)message<=22;
                    else if(tool==2 && waypoint_count==8)message<=23;
                    else begin
                        selected_point<=0;host_point_error<=0;route_ready<=0;navigation_active<=0;message<=29;
                        selected_x_cm<={snapped_x,1'b1};selected_y_cm<=9'd319-{snapped_y,1'b0};
                        case(tool)
                            0:begin
                                sx<=snapped_x;sy<=snapped_y;have_start<=1;
                                // A local start is explicitly facing right on the map.
                                // Repeated taps rotate this declaration counterclockwise.
                                heading_reference<=0;yaw_reference<=imu_yaw;heading_reference_valid<=1;
                                start_edit_generation<=start_edit_generation==32'hffffffff ? 32'd1 : start_edit_generation+1'b1;
                            end
                            1:begin gx<=snapped_x;gy<=snapped_y;have_goal<=1;end
                            2:begin wx[waypoint_count[2:0]]<=snapped_x;wy[waypoint_count[2:0]]<=snapped_y;
                                waypoint_count<=waypoint_count+1'b1;end
                        endcase
                    end
                end
                LAUNCH:begin refine_start<=1;message<=18;state<=REFINE;end
                REFINE:if(refine_done)begin
                    planning<=0;state<=IDLE;host_point_error<=refine_error;
                    if(refine_error==0)begin
                        route_ready<=1;message<=refine_error==4?38:refine_error==5?39:19;length_cm<=refined_length_cm;
                        // PLAN publishes a ready route only. Movement always
                        // requires the separate START command / local run tap.
                    end else begin
                        route_ready<=0;message<=refine_error==1?40:20;
                        if(host_route_page)host_plan_error<=1;
                    end
                end
                default:;
            endcase
            if(host_command===1'b1 && host_result==0)case(host_opcode)
                8'h20:begin
                    selected_point<=0;host_point_error<=0;heading_reference_valid<=0;
                    host_route_page<=1;host_plan_error<=0;screen<=MAP;navigation_active<=0;route_ready<=0;
                    have_start<=1;have_goal<=0;sx<=host_arguments[7:0];sy<=host_arguments[15:8];waypoint_count<=0;
                    state<=WIPE;wipe_index<=0;go_after_wipe<=0;length_cm<=0;message<=29;
                end
                8'h21:begin
                    wx[waypoint_count[2:0]]<=host_arguments[7:0];wy[waypoint_count[2:0]]<=host_arguments[15:8];
                    waypoint_count<=waypoint_count+1'b1;route_ready<=0;host_plan_error<=0;
                end
                8'h22:begin gx<=host_arguments[7:0];gy<=host_arguments[15:8];have_goal<=1;route_ready<=0;host_plan_error<=0;end
                8'h24:begin
                    heading_reference<=$signed(host_arguments[15:0]);yaw_reference<=imu_yaw;heading_reference_valid<=1;
                    route_ready<=0;host_plan_error<=0;selected_point<=0;host_point_error<=0;message<=29;
                end
                8'h25:begin
                    navigation_active<=0;route_ready<=0;planning<=0;screen<=MAP;host_route_page<=1;
                    selected_point<=0;host_point_error<=0;host_plan_error<=0;have_goal<=0;waypoint_count<=0;
                    state<=WIPE;wipe_index<=0;go_after_wipe<=0;length_cm<=0;message<=29;
                end
                8'h23:begin
                    planning<=1;navigation_active<=0;route_ready<=0;host_plan_error<=0;message<=18;length_cm<=0;
                    state<=WIPE;wipe_index<=0;go_after_wipe<=1;
                end
                default:;
            endcase
            // A mode switch is accepted by the host controller only after its
            // active job is stopped. Return from remote navigation to driving.
            if(host_command===1'b1 && host_selected===1'b1 && host_opcode==8'h12)begin
                screen<=REMOTE;host_route_page<=0;host_plan_error<=0;navigation_active<=0;
                if(planning)begin planning<=0;route_ready<=0;astar_cancel<=1;state<=IDLE;end
            end
            if(host_command===1'b1 && host_opcode==8'h05 && host_route_page && planning)begin
                planning<=0;navigation_active<=0;route_ready<=0;host_plan_error<=1;astar_cancel<=1;state<=IDLE;
            end
            if(host_selected===1'b0)begin
                host_route_page<=0;host_plan_error<=0;
                if(host_route_page)begin planning<=0;navigation_active<=0;route_ready<=0;astar_cancel<=1;state<=IDLE;end
            end
            if(host_point_select===1'b1 && host_point_select_ready && host_point_id>=1 && host_point_id<=9)begin
                host_route_page<=0;host_plan_error<=0;
                gx<=destination[7:0];gy<=destination[15:8];have_goal<=1;
                selected_point<=host_point_id;host_point_error<=0;waypoint_count<=0;barrier_pending<=0;
                selected_x_cm<={destination[7:0],1'b1};selected_y_cm<=9'd319-{destination[15:8],1'b0};
                planning<=1;route_ready<=0;navigation_active<=0;message<=18;length_cm<=0;
                state<=WIPE;wipe_index<=0;go_after_wipe<=1;
            end else if(host_point_start===1'b1 && host_point_route_ready)begin
                screen<=NAV_RUN;navigation_active<=1;message<=26;
            end else if(host_point_cancel===1'b1 && host_selected && screen==MAP && planning)begin
                planning<=0;route_ready<=0;astar_cancel<=1;state<=IDLE;go_after_wipe<=0;message<=29;host_point_error<=3;
            end else if(tap) begin
                if(screen==REMOTE && host_selected===1'b1 && zebra_can_change===1'b1 &&
                   u>=520 && u<776 && v>=112 && v<200)zebra_toggle<=1;
                host_route_page<=0;host_plan_error<=0;
                if(((screen==LIVE && live_page==2)?(u<72 && v>=408):(u<72 && v<72)) && screen!=HOME) begin
                    if(screen==LIVE && live_page!=0)live_page<=0;
                    else if(screen==MODES || screen==PARAMETERS)screen<=SETTINGS;
                    else if(screen==SPEED_MENU)screen<=PARAMETERS;
                    else if(screen==KEYPAD || screen==ADJUST)screen<=edit_distance?PARAMETERS:SPEED_MENU;
                    else if(screen==MAP || screen==NAV_RUN) screen<=NAV;
                    else if(screen>=DETECT && screen<=DODGE) screen<=AVOID;
                    else screen<=HOME;
                    if(planning)begin planning<=0;route_ready<=0;astar_cancel<=1;state<=IDLE;message<=29;end
                    navigation_active<=0;
                end else if(screen==HOME) begin
                    if(v>=432 && v<476 && u>=48 && u<752)screen<=REMOTE;
                    else if(u>=624 && u<776 && v>=16 && v<72)screen<=SETTINGS;
                    else if(u>=48 && u<384 && v>=120 && v<248)begin screen<=LIVE;live_page<=0;end
                    else if(u>=416 && u<752 && v>=120 && v<248)screen<=REMOTE;
                    else if(u>=48 && u<384 && v>=280 && v<408)screen<=AVOID;
                    else if(u>=416 && u<752 && v>=280 && v<408)screen<=NAV;
                end else if(screen==LIVE && live_page==0 && u>=80 && u<720)begin
                    if(v>=144 && v<224)live_page<=1;
                    else if(v>=264 && v<344)live_page<=2;
                end else if(screen==SETTINGS && u>=80 && u<720)begin
                    if(v>=144 && v<224)screen<=MODES;
                    else if(v>=264 && v<344)screen<=PARAMETERS;
                end else if(screen==PARAMETERS && u>=80 && u<720)begin
                    if(v>=120 && v<176)screen<=SPEED_MENU;
                    else if((v>=184 && v<240)||(v>=248 && v<304)||(v>=312 && v<368))screen<=KEYPAD;
                    else if(v>=376 && v<432)screen<=ADJUST;
                end else if(screen==SPEED_MENU && u>=80 && u<720)begin
                    if(v>=200 && v<280)screen<=KEYPAD;
                    else if(v>=312 && v<392)screen<=ADJUST;
                end else if(screen==AVOID && u>=80 && u<720) begin
                    if(v>=128 && v<200)screen<=DETECT;
                    else if(v>=232 && v<304)screen<=STOP;
                    else if(v>=336 && v<408)screen<=DODGE;
                    message<=25;
                end else if(screen==NAV && u>=80 && u<720) begin
                    if(v>=128 && v<200) screen<=MAP;
                    else if(v>=232 && v<304)begin
                        if(route_ready)begin screen<=NAV_RUN;navigation_active<=1;if(message!=38 && message!=39)message<=26;end
                        else message<=24;
                    end
                end else if(screen==MAP)begin
                    if(u>=392 && u<512 && v>=304 && v<352)begin
                        have_start<=0;have_goal<=0;waypoint_count<=0;selected_point<=0;host_point_error<=0;route_ready<=0;navigation_active<=0;heading_reference_valid<=0;
                        planning<=0;astar_cancel<=1;message<=24;length_cm<=0;
                        barrier_pending<=0;
                        state<=WIPE;wipe_index<=0;go_after_wipe<=0;
                    end else if(!planning && state==IDLE)begin
                        if(u>=536 && u<784 && v>=384 && v<432)begin
                            route_ready<=0;navigation_active<=0;barrier_pending<=0;
                            if(u<656)begin tool<=3;message<=17;end
                            else begin barrier_valid<=0;barrier_slot<=0;message<=29;end
                            state<=WIPE;wipe_index<=0;go_after_wipe<=0;
                        end else if(u>=392 && u<512)begin
                            if(v>=112 && v<160)tool<=0;
                            else if(v>=176 && v<224)tool<=1;
                            else if(v>=240 && v<288)tool<=2;
                            else if(v>=368 && v<416)begin
                                if(!have_start || !have_goal) message<=21;
                                else begin
                                    selected_point<=0;host_point_error<=0;planning<=1;route_ready<=0;message<=18;length_cm<=0;
                                    state<=WIPE;wipe_index<=0;go_after_wipe<=1;
                                end
                            end
                        end else if(u>=24 && u<376 && v>=112 && v<432 && tool==3)begin
                            selected_point<=0;host_point_error<=0;route_ready<=0;navigation_active<=0;message<=29;
                            if(!barrier_pending)begin
                                barrier_pending<=1;barrier_anchor_x<=barrier_click_x;barrier_anchor_y<=barrier_click_y;
                            end else if(barrier_dx>=4 || barrier_dy>=4)begin
                                barrier_valid[barrier_slot]<=1;barrier_pending<=0;barrier_slot<=barrier_slot+1'b1;
                                barrier_x0[barrier_slot*9+:9]<=barrier_dx>=barrier_dy && barrier_click_x<barrier_anchor_x?barrier_click_x:barrier_anchor_x;
                                barrier_x1[barrier_slot*9+:9]<=barrier_dx>=barrier_dy && barrier_click_x>barrier_anchor_x?barrier_click_x:barrier_anchor_x;
                                barrier_y0[barrier_slot*9+:9]<=barrier_dy>barrier_dx && barrier_click_y<barrier_anchor_y?barrier_click_y:barrier_anchor_y;
                                barrier_y1[barrier_slot*9+:9]<=barrier_dy>barrier_dx && barrier_click_y>barrier_anchor_y?barrier_click_y:barrier_anchor_y;
                                state<=WIPE;wipe_index<=0;go_after_wipe<=0;
                            end
                        end else if(u>=24 && u<376 && v>=112 && v<432 && tool==0 && have_start &&
                                    u+8>=25+{sx,1'b0} && u<=33+{sx,1'b0} &&
                                    v+8>=113+{sy,1'b0} && v<=121+{sy,1'b0})begin
                            // Hit the existing arrow, not a new map location. Debounced
                            // tap means a held finger never repeats the rotation.
                            heading_reference<=heading_reference_valid ?
                                (start_heading>900 ? start_heading-2700:start_heading+900):16'sd0;
                            yaw_reference<=imu_yaw;heading_reference_valid<=1;
                            start_edit_generation<=start_edit_generation==32'hffffffff ? 32'd1 : start_edit_generation+1'b1;
                            route_ready<=0;navigation_active<=0;selected_point<=0;
                            host_plan_error<=0;host_point_error<=0;message<=29;
                        end else if(u>=24 && u<376 && v>=112 && v<432)begin
                            probe_x<=(u-24)>>1;probe_y<=(v-112)>>1;
                            click_x<=(u-24)>>1;click_y<=(v-112)>>1;
                            scanning<=0;snap_attempted<=0;align_checked<=0;align_probe<=0;
                            probe_id<=((v-112)>>1)*176+((u-24)>>1);
                            state<=PROBE_WAIT;
                        end
                    end
                end else if(((screen==NAV_RUN && u>=408) || ((screen==STOP || screen==DODGE) && u>=80)) && u<720 && v>=368 && v<424)begin
                    navigation_active<=0;screen<=screen==NAV_RUN?NAV:AVOID;
                end
            end
        end
    end
endmodule
