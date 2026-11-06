// One owner for the shared speed and control-source settings. No flash writes.
`include "src/vehicle/vehicle_config.vh"
module vehicle_settings(
    input clk,rst_n,input [3:0] screen,
    input touch_valid,touch_down,input [15:0] touch_x,touch_y,
    input remote_increase,remote_decrease,
    output reg control_remote,control_host,navigation_avoidance,output reg [15:0] configured_speed,
    output reg [15:0] slowdown_cm,bypass_cm,output [15:0] brake_speed,output reg [2:0] edit_parameter,
    output reg [10:0] servo_center_tenths,
    output edit_distance,
    output reg [15:0] keypad_value,output reg [2:0] keypad_digits,
    output reg keypad_error,
    output reg touch_run,output signed [15:0] touch_speed,
    output reg signed [15:0] touch_steer
);
    reg held,reverse_drive;reg [3:0] previous_screen;
    // Optional map-navigation detours, captured by the motion controller at start.
    // Reset stays OFF; changing pages does not discard the operator's selection.
    assign edit_distance=edit_parameter!=0;
    // Keep the legacy port for wiring compatibility, with one speed owner.
    assign brake_speed=configured_speed;
    wire [15:0] minimum_value=edit_parameter==1?25:edit_parameter==2?35:0;
    wire [15:0] maximum_value=edit_parameter==1?200:edit_parameter==2?150:`VEHICLE_MAX_SPEED_MM_S;
    wire tap=touch_valid && touch_down && !held && touch_x<800 && touch_y<480;
    wire [15:0] u=touch_x,v=touch_y;
    reg [3:0] key;reg key_valid;reg [19:0] appended;
    assign touch_speed=touch_run?(reverse_drive?-$signed(configured_speed):$signed(configured_speed)):16'sd0;
    // Keypad: columns at 304/448/592, rows at 144/216/288/360.
    always @*begin
        key=0;key_valid=0;appended=0;
        if(((u>=304 && u<432)||(u>=448 && u<576)||(u>=592 && u<720)) &&
           ((v>=144 && v<208)||(v>=216 && v<280)||(v>=288 && v<352)||(v>=360 && v<424)))begin
            key_valid=1;
            key=(v<208)?1:(v<280)?4:(v<352)?7:10;
            if(v<360)key=key+((u<432)?0:(u<576)?1:2);
            else key=(u<432)?10:(u<576)?0:11;
        end
        appended=keypad_value*10+key;
    end
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin
            control_remote<=0;control_host<=0;navigation_avoidance<=0;configured_speed<=200;held<=0;previous_screen<=0;
            slowdown_cm<=65;bypass_cm<=65;edit_parameter<=0;
            servo_center_tenths<=`VEHICLE_SERVO_CENTER_DEG*10;
            keypad_value<=0;keypad_digits<=0;keypad_error<=0;
            touch_run<=0;reverse_drive<=0;touch_steer<=0;
        end else begin
            previous_screen<=screen;
            if(touch_valid)held<=touch_down;
            if(screen!=2 || control_remote || control_host || screen!=previous_screen)begin
                touch_run<=0;touch_steer<=0;reverse_drive<=0;
            end
            if(screen==13 && previous_screen!=13)begin
                keypad_value<=0;keypad_digits<=0;keypad_error<=0;
            end
            // Only accept remote setting events in remote mode on the driving page.
            if(control_remote && screen==2)begin
                if(remote_increase && !remote_decrease)
                    configured_speed<=configured_speed>`VEHICLE_MAX_SPEED_MM_S-50?`VEHICLE_MAX_SPEED_MM_S:configured_speed+50;
                else if(remote_decrease && !remote_increase)
                    configured_speed<=configured_speed<50?0:configured_speed-50;
            end
            if(tap)begin
                if(screen==0 && v>=432 && v<476 && u>=48 && u<752)begin control_remote<=0;control_host<=1;end
                if(screen==10 && v>=376 && v<432)begin
                    if(u>=240 && u<448)navigation_avoidance<=0;
                    else if(u>=464 && u<720)navigation_avoidance<=1;
                end
                if(screen==12 && u>=80 && u<720)begin
                    if(v>=120 && v<176)edit_parameter<=0;
                    else if(v>=184 && v<240)edit_parameter<=1;
                    else if(v>=248 && v<304)edit_parameter<=2;
                    else if(v>=312 && v<368)edit_parameter<=4;
                end
                if((screen==3 || screen==7 || screen==8) && u>=624 && u<776 && v>=16 && v<72)edit_parameter<=0;
                if(screen==15 && u>=80 && u<720 && ((v>=200 && v<280)||(v>=312 && v<392)))edit_parameter<=0;
                if(screen==11 && u>=80 && u<720)begin
                    if(v>=144 && v<224)begin control_remote<=0;control_host<=0;end
                    else if(v>=264 && v<344)begin control_remote<=1;control_host<=0;end
                    else if(v>=368 && v<432)begin control_remote<=0;control_host<=1;end
                end
                if(screen==13 && key_valid)begin
                    if(key==10)begin
                        keypad_value<=keypad_value/10;
                        if(keypad_digits!=0)keypad_digits<=keypad_digits-1'b1;
                        keypad_error<=0;
                    end else if(key==11)begin
                        if(keypad_digits!=0 && keypad_value>=minimum_value && keypad_value<=maximum_value)begin
                            case(edit_parameter)
                                1:slowdown_cm<=keypad_value;
                                2:bypass_cm<=keypad_value;
                                default:configured_speed<=keypad_value;
                            endcase
                            keypad_error<=0;
                        end else keypad_error<=1;
                    end else if(keypad_digits<4 && appended<=maximum_value)begin
                        keypad_value<=appended[15:0];keypad_digits<=keypad_digits+1'b1;keypad_error<=0;
                    end else keypad_error<=1;
                end
                if(screen==14 && v>=240 && v<328)begin
                    if(edit_parameter==4)begin
                        if(u>=80 && u<224)servo_center_tenths<=servo_center_tenths<`VEHICLE_SERVO_CENTER_MIN_TENTHS+10?
                            `VEHICLE_SERVO_CENTER_MIN_TENTHS:servo_center_tenths-11'd10;
                        else if(u>=240 && u<384)servo_center_tenths<=servo_center_tenths<=`VEHICLE_SERVO_CENTER_MIN_TENTHS?
                            `VEHICLE_SERVO_CENTER_MIN_TENTHS:servo_center_tenths-11'd1;
                        else if(u>=416 && u<560)servo_center_tenths<=servo_center_tenths>=`VEHICLE_SERVO_CENTER_MAX_TENTHS?
                            `VEHICLE_SERVO_CENTER_MAX_TENTHS:servo_center_tenths+11'd1;
                        else if(u>=576 && u<720)servo_center_tenths<=servo_center_tenths>`VEHICLE_SERVO_CENTER_MAX_TENTHS-10?
                            `VEHICLE_SERVO_CENTER_MAX_TENTHS:servo_center_tenths+11'd10;
                    end else begin
                        if(u>=80 && u<224)configured_speed<=configured_speed<50?0:configured_speed-50;
                        else if(u>=240 && u<384)configured_speed<=configured_speed<5?0:configured_speed-5;
                        else if(u>=416 && u<560)configured_speed<=configured_speed>`VEHICLE_MAX_SPEED_MM_S-5?`VEHICLE_MAX_SPEED_MM_S:configured_speed+5;
                        else if(u>=576 && u<720)configured_speed<=configured_speed>`VEHICLE_MAX_SPEED_MM_S-50?`VEHICLE_MAX_SPEED_MM_S:configured_speed+50;
                    end
                end
                if(screen==2 && !control_remote && !control_host)begin
                    if(u>=608 && u<688 && v>=224 && v<280)begin touch_run<=1;reverse_drive<=0;end
                    else if(u>=608 && u<688 && v>=360 && v<416)begin touch_run<=1;reverse_drive<=1;end
                    else if(u>=520 && u<600 && v>=292 && v<348)touch_steer<=`VEHICLE_LEFT_STEER_DEG;
                    else if(u>=696 && u<776 && v>=292 && v<348)touch_steer<=-`VEHICLE_RIGHT_STEER_DEG;
                    else if(u>=608 && u<688 && v>=292 && v<348)touch_steer<=0;
                    else if(u>=520 && u<600 && v>=360 && v<416)begin touch_run<=0;touch_steer<=0;end
                end
            end
        end
    end
endmodule
