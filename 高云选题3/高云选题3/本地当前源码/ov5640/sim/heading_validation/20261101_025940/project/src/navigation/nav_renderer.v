// Map read stage followed by a synchronous, row-addressed font ROM stage.
`include "src/vehicle/vehicle_config.vh"
module nav_renderer(
    input clk,rst_n,data_req,
    input zebra_enable,zebra_can_change,
    input [11:0] x_addr,y_addr, input [15:0] video,
    input [3:0] screen, input [1:0] tool,live_page,
    input [7:0] binary_threshold,input [15:0] preview,
    input control_remote,control_host,navigation_avoidance,speed_edit,touch_run,input [15:0] configured_speed,keypad_value,
    input [15:0] slowdown_cm,input edit_distance,input [31:0] tof_diagnostics,
    input [15:0] bypass_cm,brake_speed,input [2:0] edit_parameter,
    input [10:0] servo_center_tenths,
    input barrier_pixel,barrier_pending,input [8:0] barrier_anchor_x,barrier_anchor_y,
    input [2:0] keypad_digits,input keypad_error,
    input [3:0] selected_point,
    input have_start,have_goal, input [7:0] sx,sy,gx,gy,
    input [3:0] waypoint_count, input [63:0] waypoint_x,waypoint_y,
    input [8:0] selected_x_cm,selected_y_cm,
    input [5:0] message, input route_ready,planning,
    input [17:0] length_cm,
    output [16:0] map_addr, output [14:0] plan_addr,
    input map_blocked,plan_blocked,route_pixel,centre_pixel,
    output reg [15:0] pixel,
    input [15:0] left_mm,left_front_mm,right_front_mm,right_mm,front_average_mm,
    input front_valid,input [3:0] motion_status,motion_phase,
    input signed [15:0] commanded_speed,commanded_steer,pose_heading,target_heading,
    input signed [23:0] pose_x_q8,pose_y_q8,
    input signed [15:0] target_x_mm,target_y_mm,
    input [15:0] target_distance_mm,
    input profile_ready,input [3:0] profile_segments,
    input [15:0] profile_length_mm,profile_left_revs,profile_right_revs,
    input signed [15:0] profile_entry_heading,profile_exit_heading,
    input [7:0] profile_peak_steer
);
    `include "nav_font.vh"
    `include "nav_destinations.vh"
    localparam BG=16'hf79e,INK=16'h3986,MUTED=16'h7b8e,WHITE=16'hffff,
               GREEN=16'h5403,RED=16'h295b,BLUE=16'hdc61,PALE=16'hd6ba;
    wire [11:0] native_u=x_addr,native_v=y_addr;
    wire in_map=native_u>=24 && native_u<376 && native_v>=112 && native_v<432;
    assign map_addr=in_map?(native_v-12'd112)*17'd352+(native_u-12'd24):17'd0;
    assign plan_addr=in_map?((native_v-12'd112)>>1)*15'd176+((native_u-12'd24)>>1):15'd0;
    reg [11:0] u,v;
    reg valid;
    reg [15:0] camera;
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin u<=0;v<=0;valid<=0;camera<=0;end
        else begin
            u<=screen==1 && live_page==2?12'd479-native_v:native_u;
            v<=screen==1 && live_page==2?native_u:native_v;
            valid<=data_req;camera<=video;
        end
    end
    reg [7:0] text_code;
    reg [3:0] font_x,font_y;
    reg [15:0] text_color;
    reg [15:0] canvas;
    reg [15:0] font_rom[0:4095];
    reg [15:0] font_row,canvas_d,color_d;
    reg [3:0] font_x_d;
    reg valid_d,ink_d;
    initial $readmemh("src/navigation/nav_font_rows.mem",font_rom);
    always @(posedge clk)font_row<=font_rom[{text_code,font_y}];
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin canvas_d<=0;color_d<=0;font_x_d<=0;valid_d<=0;ink_d<=0;end
        else begin
            canvas_d<=canvas;color_d<=text_color;font_x_d<=font_x;
            valid_d<=valid;ink_d<=text_code!=0;
        end
    end
    always @* pixel=!valid_d?16'd0:ink_d && font_row[15-font_x_d]?color_d:canvas_d;
    integer i,px,py,ex,ey,ddx,ddy,position;
    reg [7:0] point_x,point_y;
    reg [5:0] item_label;
    reg [15:0] item_color;
    reg [7:0] ascii;
    reg [5:0] runtime_label;

    task label_at;
        input [6:0] id; input integer left,top;
        input big; input [15:0] color;
        integer rx,ry;
        begin
            rx=(u-left)>>(big?1:0);ry=(v-top)>>(big?1:0);
            if(u>=left && v>=top && rx<nav_label_length(id)*16 && ry<16)begin
                text_code=text_code | nav_label(id,rx/16);font_x=font_x | (rx%16);font_y=font_y | ry;text_color=text_color | color;
            end
        end
    endtask

    // Shared sequential double-dabble converter: refresh all telemetry in
    // about 12 us without division or multiplication on the pixel path.
    reg [15:0] display_bcd[0:30];
    reg [30:0] display_negative,display_invalid;
    wire [10:0] servo_min_tenths=servo_center_tenths-`VEHICLE_SERVO_RIGHT_OFFSET_TENTHS;
    wire [10:0] servo_max_tenths=servo_center_tenths+`VEHICLE_SERVO_LEFT_OFFSET_TENTHS;
    reg [4:0] format_index,format_step;
    reg [15:0] format_binary,format_bcd;
    reg format_negative,format_invalid;
    reg [15:0] source_value,source_magnitude,bcd_adjusted;
    reg source_signed,source_invalid;
    integer digit_index,reset_index;
    always @* begin
        source_value=0;source_signed=0;source_invalid=0;
        case(format_index)
            0:source_value=left_mm;
            1:source_value=left_front_mm;
            2:source_value=right_front_mm;
            3:source_value=right_mm;
            4:begin source_value=front_average_mm;source_invalid=!front_valid;end
            5:begin source_value=commanded_speed;source_signed=1;end
            6:begin source_value=commanded_steer;source_signed=1;end
            7:begin source_value=pose_x_q8>>8;source_invalid=pose_x_q8<0;end
            8:begin source_value=pose_y_q8>>8;source_invalid=pose_y_q8<0;end
            9:begin source_value=pose_heading;source_signed=1;end
            10:source_value=target_x_mm;
            11:source_value=target_y_mm;
            12:begin source_value=target_heading;source_signed=1;end
            13:source_value=target_distance_mm;
            14:source_value={12'd0,motion_phase}+16'd1;
            15:source_value={12'd0,profile_segments};
            16:source_value=profile_length_mm;
            17:source_value=profile_left_revs;
            18:source_value=profile_right_revs;
            19:begin source_value=profile_entry_heading;source_signed=1;end
            20:begin source_value=profile_exit_heading;source_signed=1;end
            21:source_value={8'd0,profile_peak_steer};
            22:source_value=configured_speed;
            23:source_value=keypad_value;
            24:source_value=slowdown_cm;
            25:source_value=bypass_cm;
            26:source_value=brake_speed;
            27:source_value={8'd0,binary_threshold};
            28:source_value={5'd0,servo_center_tenths};
            29:source_value={5'd0,servo_min_tenths};
            30:source_value={5'd0,servo_max_tenths};
        endcase
        source_magnitude=source_signed && source_value[15] ? -source_value:source_value;
        bcd_adjusted=format_bcd;
        for(digit_index=0;digit_index<4;digit_index=digit_index+1)
            if(format_bcd[digit_index*4+:4]>=5)
                bcd_adjusted[digit_index*4+:4]=format_bcd[digit_index*4+:4]+4'd3;
    end
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin
            format_index<=0;format_step<=0;format_binary<=0;format_bcd<=0;
            format_negative<=0;format_invalid<=0;display_negative<=0;display_invalid<=0;
            for(reset_index=0;reset_index<31;reset_index=reset_index+1)display_bcd[reset_index]<=0;
        end else if(format_step==0)begin
            format_binary<=source_magnitude>9999?16'd9999:source_magnitude;
            format_negative<=source_signed && source_value[15];
            format_invalid<=source_invalid || (!source_signed && source_value==16'hffff);
            format_bcd<=0;format_step<=1;
        end else if(format_step<=16)begin
            format_binary<={format_binary[14:0],1'b0};
            format_bcd<={bcd_adjusted[14:0],format_binary[15]};
            format_step<=format_step+1'b1;
        end else begin
            display_bcd[format_index]<=format_bcd;
            display_negative[format_index]<=format_negative;
            display_invalid[format_index]<=format_invalid;
            format_index<=format_index==30?5'd0:format_index+1'b1;format_step<=0;
        end
    end
    task bcd3;
        input [11:0] bcd;input integer left,top;input [15:0] color;
        reg [3:0] digit;
        begin
            digit=0;
            if(u>=left && u<left+48 && v>=top && v<top+16)begin
                case((u-left)/16)
                    0:digit=bcd[11:8];1:digit=bcd[7:4];2:digit=bcd[3:0];
                endcase
                text_code=text_code | nav_ascii(8'd48+digit);font_x=font_x | ((u-left)%16);font_y=font_y | (v-top);text_color=text_color | color;
            end
        end
    endtask
    task angle_at;
        input [4:0] id;input integer left,top;input [15:0] color;
        begin
            bcd3(display_bcd[id][15:4],left,top,color);
            char_at(".",left+48,top,color);
            char_at(8'd48+display_bcd[id][3:0],left+64,top,color);
            label_at(112,left+88,top,0,MUTED);
        end
    endtask
    // Range bank stores mm or tenths of a revolution as four decimal digits.
    task range_at;
        input [4:0] id;input integer left,top;input [15:0] color;
        begin
            if(!display_invalid[id])bcd3(display_bcd[id][15:4],left,top,color);
            char_at(".",left+48,top,color);
            char_at(display_invalid[id]?"-":8'd48+display_bcd[id][3:0],left+64,top,color);
            if(display_invalid[id])begin
                char_at("-",left,top,color);char_at("-",left+16,top,color);char_at("-",left+32,top,color);
            end
            char_at("c",left+88,top,MUTED);char_at("m",left+104,top,MUTED);
        end
    endtask
    task signed_at;
        input [4:0] id;input integer left,top;input [15:0] color;
        begin
            char_at(display_negative[id]?"-":"+",left,top,color);
            bcd3((id==9 || id==12 || id==19 || id==20)?
                display_bcd[id][15:4]:display_bcd[id][11:0],left+16,top,color);
        end
    endtask
    task char_at;
        input [7:0] character;input integer left,top;input [15:0] color;
        begin
            if(u>=left && u<left+16 && v>=top && v<top+16)begin
                text_code=text_code | nav_ascii(character);font_x=font_x | (u-left);font_y=font_y | (v-top);text_color=text_color | color;
            end
        end
    endtask
    task speed_at;
        input [4:0] id;input integer left,top;input [15:0] color;
        begin
            char_at(8'd48+display_bcd[id][15:12],left,top,color);
            bcd3(display_bcd[id][11:0],left+16,top,color);
            char_at("m",left+80,top,MUTED);char_at("m",left+96,top,MUTED);
            char_at("/",left+112,top,MUTED);char_at("s",left+128,top,MUTED);
        end
    endtask
    task number3;
        input [9:0] value;input integer left,top;input [15:0] color;
        reg [7:0] digit;
        begin
            digit=0;
            if(u>=left && u<left+48 && v>=top && v<top+16)begin
                case((u-left)/16)
                    0:digit=value/100;
                    1:digit=(value/10)%10;
                    2:digit=value%10;
                endcase
                text_code=text_code | nav_ascii(8'd48+digit);font_x=font_x | ((u-left)%16);font_y=font_y | (v-top);text_color=text_color | color;
            end
        end
    endtask
    task hex_at;
        input [3:0] value;input integer left,top;
        begin char_at(value<10?8'd48+value:8'd65+value-10,left,top,MUTED);end
    endtask
    task coordinates;
        input [7:0] xx,yy;input enabled;input integer left,top;input [15:0] color;
        begin
            if(enabled)begin
                number3({xx,1'b1},left,top,color);
                char_at(",",left+48,top,MUTED);
                number3(9'd319-{yy,1'b0},left+64,top,color);
            end else begin
                char_at("-",left,top,MUTED);char_at("-",left+16,top,MUTED);char_at("-",left+32,top,MUTED);
            end
        end
    endtask

    always @* begin
        canvas=BG;text_code=0;font_x=0;font_y=0;text_color=0;
        px=0;py=0;ex=0;ey=0;ddx=0;ddy=0;position=0;
        point_x=0;point_y=0;item_label=0;item_color=GREEN;ascii=0;
        case(motion_status)
            1:runtime_label=46;2:runtime_label=47;3:runtime_label=48;
            4:runtime_label=49;5:runtime_label=50;6:runtime_label=51;
            7:runtime_label=52;8:runtime_label=53;9:runtime_label=54;
            default:runtime_label=63;
        endcase
        if(screen==1 && live_page!=1)begin
            if(live_page==0)begin
                label_at(2,88,24,1,INK);
                for(i=0;i<2;i=i+1)begin
                    py=144+i*120;
                    if(u>=80 && u<720 && v>=py && v<py+80)canvas=WHITE;
                    label_at(94+i,128,py+24,1,INK);
                end
            end else begin
                canvas=camera;
                if(v>=390 && v<394)canvas=GREEN;
                if(u>=48 && u<432 && v>=782 && v<788)canvas=PALE;
                if(u>=48 && u<48+(({1'b0,binary_threshold,1'b0}+binary_threshold)>>1) && v>=782 && v<788)canvas=GREEN;
                if(u+6>=48+(({1'b0,binary_threshold,1'b0}+binary_threshold)>>1) &&
                   u<54+(({1'b0,binary_threshold,1'b0}+binary_threshold)>>1) && v>=770 && v<798)canvas=WHITE;
                bcd3(display_bcd[27][11:0],432,776,WHITE);
            end
        end else if(screen==1 || screen==2 || (screen>=6 && screen<=9))begin
            canvas=camera;
            if(u<72 && v<72)canvas=INK;
            if(screen>=6 && v>=440)begin
                canvas=INK;label_at(runtime_label,96,454,0,WHITE);

            end
            label_at(screen==1?2:screen==2?3:screen==6?6:screen==7?7:screen==8?8:5,88,24,0,WHITE);
            if(screen==2)begin
                if(u>=88 && u<488 && v>=96 && v<200)canvas=BG;
                label_at((control_host===1'b1)?96:control_remote?3:81,104,112,0,INK);
                label_at(speed_edit?82:68,104,156,0,speed_edit?RED:MUTED);
                speed_at(22,304,156,INK);
                if(control_host===1'b1)begin
                    if(u>=88 && u<488 && v>=220 && v<280)canvas=BG;
                    label_at(107,104,240,0,INK);
                    if(u>=520 && u<776 && v>=112 && v<200)canvas=zebra_enable===1'b1 ? GREEN:BG;
                    label_at(108,552,128,0,zebra_enable===1'b1 ? WHITE:INK);
                    label_at(zebra_enable===1'b1 ? 104:103,616,164,0,zebra_can_change===1'b1 ? (zebra_enable===1'b1 ? WHITE:INK):MUTED);
                end
                if(!control_remote && control_host!==1'b1)begin
                    for(i=0;i<6;i=i+1)begin
                        px=i==0 || i==1 || i==4?608:i==2 || i==5?520:696;
                        py=i==0?224:i<5?292:360;
                        if(i==1)py=360;
                        if(u>=px && u<px+80 && v>=py && v<py+56)canvas=i==5?RED:WHITE;
                        label_at(i==0?75:i==1?76:i==2?77:i==3?78:i==4?79:80,
                            px+24,py+20,0,i==5?WHITE:INK);
                    end
                end
            end
            if(screen>=6 && screen<=8)begin
                if(u>=88 && u<488 && v>=100 && v<308)canvas=BG;
                label_at(41,104,120,0,INK);
                range_at(4,328,120,GREEN);
                label_at(42,104,164,0,INK);range_at(1,328,164,INK);
                label_at(43,104,204,0,INK);range_at(2,328,204,INK);
                label_at(44,104,244,0,MUTED);range_at(0,328,244,INK);
                label_at(45,104,280,0,MUTED);range_at(3,328,280,INK);
                if(screen==6)begin
                    if(u>=520 && u<776 && v>=100 && v<308)canvas=BG;
                    for(i=0;i<4;i=i+1)begin
                        char_at("T",536,120+i*44,MUTED);char_at(8'd49+i,552,120+i*44,MUTED);
                        char_at("E",600,120+i*44,MUTED);
                        hex_at(tof_diagnostics[i*8+4+:4],624,120+i*44);
                        hex_at(tof_diagnostics[i*8+:4],640,120+i*44);
                    end
                end
            end
            if(screen==2 || screen==7 || screen==8 || screen==9)begin
                if(u>=88 && u<488 && v>=320 && v<360)canvas=BG;
                label_at(55,104,332,0,INK);signed_at(5,152,332,INK);
                char_at("m",224,332,MUTED);char_at("m",240,332,MUTED);char_at("/",256,332,MUTED);char_at("s",272,332,MUTED);
                label_at(56,312,332,0,INK);signed_at(6,360,332,INK);
            end
            if(screen==9)begin
                if(u>=88 && u<488 && v>=100 && v<300)canvas=BG;
                label_at(57,104,120,0,INK);
                range_at(7,176,120,INK);
                range_at(8,328,120,INK);
                label_at(58,104,164,0,INK);signed_at(9,176,164,INK);
                label_at(59,104,208,0,INK);
                range_at(10,176,208,INK);range_at(11,328,208,INK);
                label_at(58,104,244,0,MUTED);signed_at(12,176,244,INK);
                label_at(60,104,280,0,MUTED);range_at(13,176,280,INK);
                if(u>=528 && u<784 && v>=112 && v<424)canvas=BG;
                char_at("S",544,128,MUTED);char_at("E",560,128,MUTED);char_at("G",576,128,MUTED);
                bcd3(display_bcd[14][11:0],608,128,INK);
                char_at("/",656,128,MUTED);bcd3(display_bcd[15][11:0],672,128,INK);
                range_at(16,560,168,INK);
                char_at("L",544,208,MUTED);bcd3(display_bcd[17][15:4],576,208,INK);
                char_at(".",624,208,INK);char_at(8'd48+display_bcd[17][3:0],640,208,INK);
                char_at("R",544,240,MUTED);bcd3(display_bcd[18][15:4],576,240,INK);
                char_at(".",624,240,INK);char_at(8'd48+display_bcd[18][3:0],640,240,INK);
                char_at("r",672,208,MUTED);char_at("e",688,208,MUTED);char_at("v",704,208,MUTED);
                char_at("r",672,240,MUTED);char_at("e",688,240,MUTED);char_at("v",704,240,MUTED);
                label_at(58,544,280,0,MUTED);signed_at(19,608,280,INK);
                char_at(">",688,280,MUTED);signed_at(20,704,280,INK);
                label_at(56,544,320,0,MUTED);bcd3(display_bcd[21][11:0],608,320,INK);
                if(!profile_ready)label_at(52,544,368,0,RED);
            end
            if((screen==7 || screen==8 || screen==9) && u>=288 && u<512 && v>=368 && v<424)begin
                canvas=RED;label_at(35,336,388,0,WHITE);
            end
            if(screen==7 || screen==8)begin
                if(u>=520 && u<784 && v>=112 && v<224)canvas=BG;
                label_at(screen==8?89:86,536,128,0,INK);bcd3(display_bcd[screen==8?25:24][11:0],536,168,GREEN);
                char_at("c",600,168,MUTED);char_at("m",616,168,MUTED);
                if(motion_phase==5)label_at(88,536,208,0,RED);
            end
        end else begin
            if(v==78 || v==79)canvas=PALE;
            if(screen==0)begin
                label_at(1,32,32,1,INK);
                if(u>=624 && u<776 && v>=16 && v<72)canvas=WHITE;
                label_at(64,664,36,0,INK);
                for(i=0;i<4;i=i+1)begin
                    px=48+(i%2)*368;py=120+(i/2)*160;
                    if(u>=px && u<px+336 && v>=py && v<py+128)begin
                        canvas=WHITE;
                        if(u<px+8)canvas=i==0?GREEN:i==1?BLUE:i==2?RED:16'h5690;
                    end
                    label_at(2+i,px+40,py+48,1,INK);
                end
                if(u>=48 && u<384 && v>=432 && v<476)canvas=WHITE;
                if(u>=416 && u<752 && v>=432 && v<476)canvas=WHITE;
                label_at(105,112,446,0,INK);label_at(106,480,446,0,INK);
            end else if(screen>=10)begin
                label_at(screen==10?64:screen==11?65:screen==12?66:screen==15?85:screen==13 && edit_parameter==1?86:screen==13 && edit_parameter==2?89:screen==13 && edit_parameter==3?90:screen==13?69:70,88,24,1,INK);
                if(screen==10 || screen==11)begin
                    for(i=0;i<2;i=i+1)begin
                        py=144+i*120;
                        if(u>=80 && u<720 && v>=py && v<py+80)canvas=screen==11 && control_host!==1'b1 && control_remote==i?GREEN:WHITE;
                        label_at(screen==10?65+i:i==0?67:3,128,py+24,1,
                            screen==11 && control_host!==1'b1 && control_remote==i?WHITE:INK);
                    end
                    if(screen==10)begin
                        // Static policy, not a switch: map navigation never
                        // acquires the independent obstacle-detour controller.
                        label_at(102,80,388,0,INK);
                        label_at(103,312,384,1,MUTED);
                    end
                    if(screen==11)begin
                        // Third source leaves the two existing button hitboxes intact.
                        if(u>=80 && u<720 && v>=368 && v<432)canvas=control_host?GREEN:WHITE;
                        label_at(96,128,384,1,control_host?WHITE:INK);
                    end
                end else if(screen==12 || screen==15)begin
                    if(screen==15)begin label_at(68,96,128,0,MUTED);speed_at(22,304,128,INK);end
                    for(i=0;i<5;i=i+1)begin
                        py=screen==12?120+i*64:200+i*112;
                        if(screen==12 || i<2)begin
                            if(u>=80 && u<720 && v>=py && v<py+(screen==12?56:80))canvas=WHITE;
                            label_at(screen==12?(i==0?85:i==1?86:i==2?89:i==3?90:109):69+i,128,py+(screen==12?12:20),1,INK);
                            if(screen==12)begin
                                if(i==0 || i==3)speed_at(i==0?22:26,512,py+20,INK);
                                else if(i==4)angle_at(28,544,py+20,INK);
                                else begin bcd3(display_bcd[i==1?24:25][11:0],544,py+20,INK);
                                    char_at("c",608,py+20,MUTED);char_at("m",624,py+20,MUTED);end
                            end
                        end
                    end
                end else if(screen==13)begin
                    label_at(edit_parameter==1?86:edit_parameter==2?89:edit_parameter==3?90:68,64,128,0,MUTED);
                    if(edit_parameter==1 || edit_parameter==2)begin
                        bcd3(display_bcd[edit_parameter==1?24:25][11:0],64,168,INK);label_at(87,64,280,0,MUTED);
                        bcd3(display_bcd[23][11:0],64,216,INK);
                        char_at("c",128,216,MUTED);char_at("m",144,216,MUTED);
                    end else begin speed_at(edit_parameter==3?26:22,64,168,INK);speed_at(23,64,216,INK);end
                    label_at(keypad_error?83:72,64,256,0,keypad_error?RED:GREEN);
                    char_at(edit_parameter==1?"2":edit_parameter==2?"3":edit_parameter==3?"8":"0",64,320,MUTED);
                    char_at(edit_parameter==1 || edit_parameter==2?"5":edit_parameter==3?"0":"-",80,320,MUTED);
                    char_at(edit_parameter==0?"1":"-",96,320,MUTED);
                    char_at(edit_parameter==1?"2":edit_parameter==2?"1":edit_parameter==3?"4":"0",112,320,MUTED);
                    char_at(edit_parameter==2?"5":"0",128,320,MUTED);char_at("0",144,320,MUTED);
                    for(i=0;i<12;i=i+1)begin
                        px=304+(i%3)*144;py=144+(i/3)*72;
                        if(u>=px && u<px+128 && v>=py && v<py+64)canvas=i==11?GREEN:WHITE;
                        if(i==9 || i==11)label_at(i==9?71:72,px+48,py+24,0,i==11?WHITE:INK);
                        else char_at(i==10?"0":8'd49+i,px+56,py+24,INK);
                    end
                end else if(screen==14)begin
                    if(edit_parameter==4)begin
                        label_at(109,96,128,1,INK);angle_at(28,304,144,INK);
                        label_at(111,96,192,0,MUTED);
                        char_at("5",240,192,MUTED);char_at("0",256,192,MUTED);char_at(".",272,192,MUTED);char_at("0",288,192,MUTED);
                        char_at("-",320,192,MUTED);char_at("1",352,192,MUTED);char_at("2",368,192,MUTED);char_at("0",384,192,MUTED);
                        char_at(".",400,192,MUTED);char_at("0",416,192,MUTED);label_at(112,448,192,0,MUTED);
                    end else begin label_at(68,96,144,0,MUTED);speed_at(22,304,144,INK);end
                    for(i=0;i<4;i=i+1)begin
                        px=i<2?80+i*160:416+(i-2)*160;
                        if(u>=px && u<px+144 && v>=240 && v<328)canvas=WHITE;
                        label_at(i==0 || i==3?73:74,px+56,252,0,MUTED);
                        if(edit_parameter==4)begin
                            char_at(i<2?"-":"+",px+32,288,INK);
                            char_at(i==0 || i==3?"1":"0",px+48,288,INK);
                            char_at(".",px+64,288,INK);char_at(i==0 || i==3?"0":"1",px+80,288,INK);
                            label_at(112,px+104,288,0,MUTED);
                        end else begin
                            char_at(i<2?"-":"+",px+40,288,INK);char_at("5",px+64,288,INK);
                            if(i==0 || i==3)char_at("0",px+80,288,INK);
                        end
                    end
                    if(edit_parameter==4)begin
                        label_at(110,96,352,0,MUTED);angle_at(29,272,352,INK);
                        char_at("-",400,352,MUTED);angle_at(30,432,352,INK);
                    end
                    label_at(84,96,384,0,GREEN);
                end
            end else if(screen==3 || screen==4)begin
                label_at(screen==3?4:5,88,24,1,INK);
                for(i=0;i<3;i=i+1)begin
                    if(screen==3 || i<2)begin
                        if(u>=80 && u<720 && v>=128+i*104 && v<200+i*104)canvas=WHITE;
                        label_at(screen==3?6+i:9+i,128,148+i*104,1,INK);
                    end
                end
                if(screen==4)label_at(message,96,368,0,route_ready?GREEN:MUTED);
            end else if(screen==5)begin
                label_at(9,88,24,1,INK);
                label_at(message,400,88,0,message==20 || message==22 || message==23 || message==30 || message>=38?RED:GREEN);
                // Fixed physical dimensions and grid resolution, in centimetres.
                number3(352,24,88,MUTED);char_at("x",72,88,MUTED);
                number3(320,88,88,MUTED);char_at("c",136,88,MUTED);char_at("m",152,88,MUTED);
                char_at("1",280,88,MUTED);char_at("c",296,88,MUTED);char_at("m",312,88,MUTED);
                if(u>=24 && u<376 && v>=112 && v<432)begin
                    canvas=map_blocked?INK:plan_blocked?16'hbdf6:WHITE;
                if(centre_pixel && !map_blocked && !route_ready && !planning)canvas=16'hbdf6;
                    if(barrier_pixel)canvas=RED;
                    if(route_pixel && (route_ready || planning || message==38 || message==39) && !map_blocked)
                        canvas=message==38 || message==39?RED:BLUE;
                    px=u-24;py=v-112;
                    if(barrier_pending && px>=barrier_anchor_x-2 && px<=barrier_anchor_x+2 &&
                        py>=barrier_anchor_y-2 && py<=barrier_anchor_y+2)canvas=RED;
                    for(i=1;i<=9;i=i+1)begin
                        {point_y,point_x}=nav_destination(i);
                        ex=point_x*2+1;ey=point_y*2+1;
                        ddx=px-ex;ddy=py-ey;
                        item_color=selected_point==i?RED:BLUE;
                        if(ddx>=-2 && ddx<=2 && ddy>=-2 && ddy<=2)canvas=item_color;
                        // Offset badges leave the actual centreline point visible.
                        ex=ex<45?ex+9:ex>300?ex-29:ex+10;
                        ey=ey<40?ey+9:ey-29;
                        if(px>=ex && px<ex+20 && py>=ey && py<ey+20)begin
                            canvas=(px==ex || px==ex+19 || py==ey || py==ey+19)?WHITE:item_color;
                            if(px>=ex+2 && px<ex+18 && py>=ey+2 && py<ey+18)begin
                                text_code=nav_ascii(8'd48+i);font_x=px-ex-2;font_y=py-ey-2;text_color=WHITE;
                            end
                        end
                    end
                    for(i=0;i<10;i=i+1)begin
                        point_x=0;point_y=0;item_label=0;item_color=GREEN;
                        if(i==0)begin point_x=sx;point_y=sy;item_label=have_start?31:0;item_color=GREEN;end
                        else if(i==1)begin point_x=gx;point_y=gy;item_label=have_goal?32:0;item_color=RED;end
                        else if(i-2<waypoint_count)begin
                            point_x=waypoint_x[(i-2)*8+:8];point_y=waypoint_y[(i-2)*8+:8];
                            item_label=33;item_color=BLUE;
                        end
                        ex=point_x*2+1;ey=point_y*2+1;
                        ddx=px-ex;ddy=py-ey;
                        if(item_label!=0 && ddx>=-3 && ddx<=3 && ddy>=-3 && ddy<=3)canvas=item_color;
                        // Keep the marker badge inside the map at its edges.
                        ex=ex<10?0:ex>332?336:ex-8;
                        ey=ey<24?ey+7:ey-23;
                        if(item_label!=0 && px>=ex && px<ex+16 && py>=ey && py<ey+16)begin
                            canvas=item_color;
                            text_code=i<2?nav_label(item_label,0):nav_ascii(8'd49+i-2);
                            font_x=px-ex;font_y=py-ey;text_color=WHITE;
                        end
                    end
                end
                for(i=0;i<5;i=i+1)begin
                    if(u>=392 && u<512 && v>=112+i*64 && v<160+i*64)begin
                        canvas=(i<3 && tool==i)?GREEN:i==4?BLUE:WHITE;
                        if(planning && i!=3)canvas=PALE;
                    end
                    label_at(11+i,i==4?404:420,128+i*64,0,
                        (i<3 && tool==i) || i==4?WHITE:INK);
                end
                char_at("X",24,448,MUTED);char_at(":",40,448,MUTED);number3(selected_x_cm,56,448,INK);
                char_at("Y",136,448,MUTED);char_at(":",152,448,MUTED);number3(selected_y_cm,168,448,INK);
                char_at("c",232,448,MUTED);char_at("m",248,448,MUTED);
                label_at(36,536,128,0,MUTED);
                if(u>=600 && u<680 && v>=128 && v<144)begin
                    case((u-600)/16)
                        0:ascii=48+(length_cm/10000)%10;
                        1:ascii=48+(length_cm/1000)%10;
                        2:ascii=48+(length_cm/100)%10;
                        3:ascii=48+(length_cm/10)%10;
                        4:ascii=48+length_cm%10;
                    endcase
                    text_code=nav_ascii(ascii);font_x=(u-600)%16;font_y=v-128;text_color=INK;
                end
                char_at("c",696,128,MUTED);char_at("m",712,128,MUTED);
                label_at(11,536,176,0,GREEN);coordinates(sx,sy,have_start,584,176,INK);
                label_at(12,536,208,0,RED);coordinates(gx,gy,have_goal,584,208,INK);
                if(selected_point!=0)begin
                    char_at("P",536,240,RED);char_at(8'd48+selected_point,552,240,RED);
                end
                for(i=0;i<8;i=i+1)begin
                    if(i<waypoint_count)begin
                        char_at(49+i,536,234+i*18,BLUE);
                        coordinates(waypoint_x[i*8+:8],waypoint_y[i*8+:8],1,584,234+i*18,INK);
                    end
                end
                if(u>=536 && u<784 && v>=384 && v<432)canvas=u<656?(tool==3?RED:WHITE):WHITE;
                label_at(91,552,400,0,tool==3?WHITE:INK);label_at(92,664,400,0,INK);
                if(tool==3)label_at(93,536,440,0,MUTED);
            end
        end
        // Back arrow has a stable 56x56 touch target in every non-home view.
        if(screen!=0 && u>=20 && u<52 && v>=20 && v<52)begin
            ddx=u-20;ddy=v-20;
            if((ddy==16-ddx || ddy==16+ddx) && ddx<12 ||
               (ddx>=2 && ddx<28 && ddy>=15 && ddy<=17))canvas=(screen==1 && live_page!=0) || screen==2 || (screen>=6 && screen<=9)?WHITE:INK;
        end
        if(!valid)canvas=0;
    end
endmodule
