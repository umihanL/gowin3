// Landscape upper right, aligned to the 640x480 camera viewport.
// The underlying DDR read proceeds even when a pixel is covered by this OSD.
module attitude_overlay #(parameter AXIS_VECTOR=0)(
    input clk,rst_n,frame_begin,data_req,input portrait_mode,
    input [11:0] x_addr,y_addr,input [15:0] background,
    input online,calibrated,available, input [2:0] sensor_status, input [1:0] cal_state,
    input [7:0] error_reg,error_expected,error_actual,error_mask,error_step,
    input signed [15:0] roll,pitch,yaw,
    output reg [15:0] pixel
);
    reg show_online,show_calibrated,show_available;
    reg [2:0] show_sensor_status; reg [1:0] show_cal_state;
    reg [7:0] show_error_reg,show_error_expected,show_error_actual,show_error_mask,show_error_step;
    reg [2:0] negative;
    reg [11:0] binary[0:2];
    reg [15:0] work_bcd[0:2],bcd[0:2];
    reg [3:0] steps;reg converting;
    integer k,j,i;
    wire [15:0] abs_roll=roll<0 ? -roll : roll;
    wire [15:0] abs_pitch=pitch<0 ? -pitch : pitch;
    wire [15:0] abs_yaw=yaw<0 ? -yaw : yaw;
    reg [15:0] adjusted[0:2];
    always @* begin
        for(k=0;k<3;k=k+1) begin
            adjusted[k]=work_bcd[k];
            for(j=0;j<4;j=j+1)
                if(work_bcd[k][j*4 +:4]>=5) adjusted[k][j*4 +:4]=work_bcd[k][j*4 +:4]+4'd3;
        end
    end
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            show_online<=0;show_calibrated<=0;show_available<=0;show_sensor_status<=0;show_cal_state<=0;negative<=0;steps<=0;converting<=0;
            show_error_reg<=0;show_error_expected<=0;show_error_actual<=0;show_error_mask<=0;show_error_step<=0;
            for(i=0;i<3;i=i+1) begin binary[i]<=0;work_bcd[i]<=0;bcd[i]<=0;end
        end else if(frame_begin) begin
            show_online<=online;show_calibrated<=calibrated;show_available<=available;
            show_sensor_status<=sensor_status;show_cal_state<=cal_state;
            show_error_reg<=error_reg;show_error_expected<=error_expected;show_error_actual<=error_actual;
            show_error_mask<=error_mask;show_error_step<=error_step;
            negative<={yaw<0,pitch<0,roll<0};
            binary[0]<=abs_roll[11:0];
            binary[1]<=abs_pitch[11:0];
            binary[2]<=abs_yaw[11:0];
            work_bcd[0]<=0;work_bcd[1]<=0;work_bcd[2]<=0;steps<=0;converting<=1;
        end else if(converting) begin
            for(i=0;i<3;i=i+1) begin
                work_bcd[i]<={adjusted[i][14:0],binary[i][11]};binary[i]<=binary[i]<<1;
                if(steps==11) bcd[i]<={adjusted[i][14:0],binary[i][11]};
            end
            if(steps==11) converting<=0;else steps<=steps+1'b1;
        end
    end
    function [34:0] glyph;
        input [7:0] c;
        begin case(c)
            "0":glyph=35'b01110_10001_10011_10101_11001_10001_01110;
            "1":glyph=35'b00100_01100_00100_00100_00100_00100_01110;
            "2":glyph=35'b01110_10001_00001_00010_00100_01000_11111;
            "3":glyph=35'b11110_00001_00001_01110_00001_00001_11110;
            "4":glyph=35'b00010_00110_01010_10010_11111_00010_00010;
            "5":glyph=35'b11111_10000_10000_11110_00001_00001_11110;
            "6":glyph=35'b01110_10000_10000_11110_10001_10001_01110;
            "7":glyph=35'b11111_00001_00010_00100_01000_01000_01000;
            "8":glyph=35'b01110_10001_10001_01110_10001_10001_01110;
            "9":glyph=35'b01110_10001_10001_01111_00001_00001_01110;
            "A":glyph=35'b01110_10001_10001_11111_10001_10001_10001;
            "B":glyph=35'b11110_10001_10001_11110_10001_10001_11110;
            "C":glyph=35'b01111_10000_10000_10000_10000_10000_01111;
            "D":glyph=35'b11110_10001_10001_10001_10001_10001_11110;
            "G":glyph=35'b01110_10001_10000_10111_10001_10001_01110;
            "H":glyph=35'b10001_10001_10001_11111_10001_10001_10001;
            "T":glyph=35'b11111_00100_00100_00100_00100_00100_00100;
            "V":glyph=35'b10001_10001_10001_10001_10001_01010_00100;
            "W":glyph=35'b10001_10001_10001_10101_10101_10101_01010;
            "X":glyph=35'b10001_10001_01010_00100_01010_10001_10001;
            "E":glyph=35'b11111_10000_10000_11110_10000_10000_11111;
            "F":glyph=35'b11111_10000_10000_11110_10000_10000_10000;
            "I":glyph=35'b01110_00100_00100_00100_00100_00100_01110;
            "L":glyph=35'b10000_10000_10000_10000_10000_10000_11111;
            "M":glyph=35'b10001_11011_10101_10101_10001_10001_10001;
            "N":glyph=35'b10001_11001_10101_10011_10001_10001_10001;
            "O":glyph=35'b01110_10001_10001_10001_10001_10001_01110;
            "P":glyph=35'b11110_10001_10001_11110_10000_10000_10000;
            "R":glyph=35'b11110_10001_10001_11110_10100_10010_10001;
            "S":glyph=35'b01111_10000_10000_01110_00001_00001_11110;
            "U":glyph=35'b10001_10001_10001_10001_10001_10001_01110;
            "X":glyph=35'b10001_10001_01010_00100_01010_10001_10001;
            "Y":glyph=35'b10001_10001_01010_00100_00100_00100_00100;
            "Z":glyph=35'b11111_00001_00010_00100_01000_10000_11111;
            "a":glyph=35'b00000_00000_01110_00001_01111_10001_01111;
            "c":glyph=35'b00000_00000_01110_10000_10000_10000_01110;
            "h":glyph=35'b10000_10000_11110_10001_10001_10001_10001;
            "i":glyph=35'b00100_00000_01100_00100_00100_00100_01110;
            "l":glyph=35'b01100_00100_00100_00100_00100_00100_01110;
            "o":glyph=35'b00000_00000_01110_10001_10001_10001_01110;
            "t":glyph=35'b00100_00100_11111_00100_00100_00100_00011;
            "w":glyph=35'b00000_00000_10001_10001_10101_10101_01010;
            ":":glyph=35'b00000_00100_00100_00000_00100_00100_00000;
            ".":glyph=35'b00000_00000_00000_00000_00000_00110_00110;
            "+":glyph=35'b00000_00100_00100_11111_00100_00100_00000;
            "-":glyph=35'b00000_00000_00000_11111_00000_00000_00000;
            default:glyph=0;
        endcase end
    endfunction
    function [7:0] hex_digit;
        input [3:0] nibble;
        begin hex_digit=nibble<10 ? (8'd48+{4'd0,nibble}) : (8'd55+{4'd0,nibble});end
    endfunction
    wire portrait=portrait_mode===1'b1;
    wire [11:0] u=portrait?12'd479-y_addr:x_addr,v=portrait?x_addr:y_addr;
    wire [11:0] tx=u-(portrait?12'd144:12'd556),ty=v-12'd10;
    wire [8:0] local_tx=portrait?tx[8:0]:{1'b0,tx[7:0]};
    wire [4:0] column_full=local_tx/12;
    wire [3:0] column=column_full[3:0];
    wire [2:0] gx=(local_tx%12)>>1,gy=ty[3:1];
    wire [2:0] row=ty[6:4];
    reg [7:0] character;
    reg [15:0] digits;
    reg sign_bit;
    reg [34:0] bitmap;
    reg [15:0] color;
    reg [47:0] status_text;
    reg [95:0] hint_text;
    reg [95:0] diagnostic_text;
    wire [3:0] px=local_tx%12;
    wire [3:0] py=ty[3:0];
    reg ink,shadow;
    integer shadow_x,shadow_y;
    // Physical panel uses BGR565 here: low five bits drive red.
    localparam YELLOW=16'h07ff, CYAN=16'hffe0, ORANGE=16'h041f;

    always @* begin
        character=" ";digits=0;sign_bit=0;color=CYAN;
        status_text="INIT  ";hint_text="S0: REZERO  ";diagnostic_text="            ";
        case(show_sensor_status)
            0:status_text="INIT  ";
            1:status_text="WAIT  ";
            3:status_text="ID ERR";
            4:status_text="CFGERR";
            5:status_text="NO DAT";
            6:status_text="FIFO  ";
            2:begin
                if(show_calibrated) status_text="RUN   ";
                else case(show_cal_state)
                    0:status_text="WARM  ";
                    1:status_text="MOVE  ";
                    default:status_text="ZERO  ";
                endcase
            end
        endcase
        if(show_online && !show_calibrated) hint_text="HOLD STILL  ";
        if(row==0) begin
            case(column)
                0:character="I";1:character="M";2:character="U";3:character=":";
            endcase
            if(column>=5 && column<=10) character=status_text[(10-column)*8 +:8];
            color=show_sensor_status>=3 ? ORANGE : (show_calibrated ? 16'h07e0 : YELLOW);
        end else if(row>=1 && row<=3) begin
            case(row)
                1:begin
                    digits=bcd[0];sign_bit=negative[0];color=YELLOW;
                    case(column) 0:character="R";1:character="o";2,3:character="l";endcase
                end
                2:begin
                    digits=bcd[1];sign_bit=negative[1];color=CYAN;
                    case(column) 0:character="P";1:character="i";2:character="t";3:character="c";4:character="h";endcase
                end
                3:begin
                    digits=bcd[2];sign_bit=negative[2];color=ORANGE;
                    case(column) 0:character="Y";1:character="a";2:character="w";endcase
                end
            endcase
            if(AXIS_VECTOR && column<5) begin
                case(column)
                    0:character="R";1:character="o";2:character="t";
                    3:character=row==1 ? "X" : row==2 ? "Y" : "Z";
                    default:character=" ";
                endcase
            end
            case(column)
                5:character=":";
                6:character=sign_bit ? "-" : "+";
                7:character=8'd48+digits[15:12];
                8:character=8'd48+digits[11:8];
                9:character=8'd48+digits[7:4];
                10:character=".";
                11:character=8'd48+digits[3:0];
            endcase
            if((!show_online || !show_available) && column>=6 && column<=11) character="-";
        end else if(row==4) begin
            if(column<12) character=hint_text[(11-column)*8 +:8];
        end
        // On CFGERR, replace unavailable angle rows with actionable telemetry.
        // REG / SET / GOT are hex; M is the comparison mask, I the init step.
        if(show_sensor_status==4 && row>=1 && row<=4) begin
            case(row)
                1:diagnostic_text={"REG: ",hex_digit(show_error_reg[7:4]),hex_digit(show_error_reg[3:0]),"     "};
                2:diagnostic_text={"SET: ",hex_digit(show_error_expected[7:4]),hex_digit(show_error_expected[3:0]),"     "};
                3:diagnostic_text={"GOT: ",hex_digit(show_error_actual[7:4]),hex_digit(show_error_actual[3:0]),"     "};
                4:diagnostic_text={"M:",hex_digit(show_error_mask[7:4]),hex_digit(show_error_mask[3:0]),
                                  " I:",hex_digit(show_error_step[7:4]),hex_digit(show_error_step[3:0]),"   "};
            endcase
            character=column<12 ? diagnostic_text[(11-column)*8 +:8] : " ";
            color=row==3 ? ORANGE : CYAN;
        end
        if(portrait)begin
            character=" ";color=16'h07e0;
            case(column_full/9)
                0:begin digits=bcd[0];sign_bit=negative[0];end
                1:begin digits=bcd[1];sign_bit=negative[1];end
                default:begin digits=bcd[2];sign_bit=negative[2];end
            endcase
            case(column_full%9)
                0:character=column_full<9?"X":column_full<18?"Y":"Z";
                1:character=":";2:character=sign_bit?"-":"+";
                3:character=8'd48+digits[15:12];4:character=8'd48+digits[11:8];
                5:character=8'd48+digits[7:4];6:character=".";7:character=8'd48+digits[3:0];
            endcase
            if((!show_online || !show_available) && column_full%9>=2 && column_full%9<=7)character="-";
        end
        bitmap=glyph(character);pixel=background;
        ink=0;shadow=0;
        shadow_x=(px-1)/2;shadow_y=(py-1)/2;
        if(gx<5 && gy<7) ink=bitmap[34-gy*5-gx];
        // One-pixel black drop shadow only; every other pixel shows the camera.
        if(px>=1 && px<=10 && py>=1 && py<=14)
            shadow=bitmap[34-shadow_y*5-shadow_x];
        if(data_req && (portrait?(u>=144 && u<468 && v>=10 && v<26):(u>=556 && u<712 && v>=10 && v<90))) begin
            if(ink)pixel=color;
            else if(shadow)pixel=16'h0000;
        end
    end
endmodule
