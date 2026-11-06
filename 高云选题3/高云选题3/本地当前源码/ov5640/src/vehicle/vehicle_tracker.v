// Pure pursuit: delta=atan2(2*wheelbase*lateral, squared lookahead).
// Heading/steering are recalculated from measured pose, not open-loop timers.
`include "src/vehicle/vehicle_config.vh"
module vehicle_tracker #(parameter WHEELBASE_MM=`VEHICLE_WHEELBASE_MM,MAX_STEER_DEG=`VEHICLE_MAX_STEER_DEG,
    parameter CONTROL_OFFSET_MM=0)(
    input clk,rst_n,start,
    input precise_bearing,input signed [23:0] bearing_dx_q8,bearing_dy_q8,
    input signed [15:0] target_x,target_y,pose_x,pose_y,
    input signed [16:0] cosine,sine,
    output reg done,output reg [15:0] distance_mm,
    output reg signed [15:0] steering_deg,bearing_tenths,forward_mm,lateral_mm);
    reg [3:0] state;
    reg signed [16:0] dx,dy;
    reg signed [34:0] f0,f1,l0,l1;
    reg [33:0] xx,yy;reg [34:0] square;
    reg atan_start,sqrt_start;reg signed [31:0] ax,ay;
    wire atan_done,sqrt_done;wire signed [31:0] angle;wire [31:0] root;
    reg signed [31:0] angle_scaled;

    // Front-point pursuit: k=2*l/(r^2+2*d*f). For a circular nose path,
    // its short-horizon limit is l/(d*f), the bicycle's front-point kinematics.
    function [31:0] offset_scale;
        input [31:0] value;
        reg [31:0] partial [0:31];
        integer i;
        begin
            for(i=0;i<32;i=i+1)partial[i]=((2*CONTROL_OFFSET_MM)>>i)&1?(value<<i):32'd0;
            for(i=0;i<16;i=i+1)partial[i]=partial[2*i]+partial[2*i+1];
            for(i=0;i<8;i=i+1)partial[i]=partial[2*i]+partial[2*i+1];
            for(i=0;i<4;i=i+1)partial[i]=partial[2*i]+partial[2*i+1];
            for(i=0;i<2;i=i+1)partial[i]=partial[2*i]+partial[2*i+1];
            offset_scale=partial[0]+partial[1];
        end
    endfunction
    wire [31:0] forward_positive=forward_mm>0?{16'd0,forward_mm}:32'd0;
    wire [31:0] offset_term=offset_scale(forward_positive);
    imu_atan2 atan(.clk(clk),.rst_n(rst_n),.start(atan_start),.vx(ax),.vy(ay),.done(atan_done),.angle(angle));
    imu_sqrt sqrt(.clk(clk),.rst_n(rst_n),.start(sqrt_start),.value({29'd0,square}),.done(sqrt_done),.root(root));
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin
            state<=0;done<=0;distance_mm<=0;steering_deg<=0;bearing_tenths<=0;
            forward_mm<=0;lateral_mm<=0;dx<=0;dy<=0;f0<=0;f1<=0;l0<=0;l1<=0;
            xx<=0;yy<=0;square<=0;atan_start<=0;sqrt_start<=0;ax<=0;ay<=0;angle_scaled<=0;
        end else begin
            done<=0;atan_start<=0;sqrt_start<=0;
            case(state)
                0:if(start)begin dx<=$signed(target_x)-$signed(pose_x);dy<=$signed(target_y)-$signed(pose_y);state<=1;end
                1:begin f0<=dx*cosine;f1<=dy*sine;l0<=dy*cosine;l1<=dx*sine;xx<=dx*dx;yy<=dy*dy;state<=2;end
                2:begin forward_mm<=(f0+f1)>>>15;lateral_mm<=(l0-l1)>>>15;square<={1'b0,xx}+yy;state<=3;end
                3:begin
                    sqrt_start<=1;atan_start<=1;
                    // Initial path tangent uses sub-millimetre route samples.
                    // Integer-mm rounding on a short first edge biases yaw.
                    if(precise_bearing)begin ax<=bearing_dx_q8<<<2;ay<=bearing_dy_q8<<<2;end
                    else begin ax<=dx<<<10;ay<=dy<<<10;end
                    state<=4;
                end
                4:if(atan_done)begin angle_scaled<=angle*10;state<=5;end
                5:begin bearing_tenths<=angle_scaled>>>16;state<=6;end
                6:begin
                    // Bound pursuit gain on short samples and mm-level pose noise.
                    ax<=square+offset_term<10000?32'd10000:square[31:0]+offset_term;
                    ay<=lateral_mm<=2 && lateral_mm>=-2?32'sd0:$signed(lateral_mm)*(2*WHEELBASE_MM);
                    atan_start<=1;state<=7;
                end
                7:if(atan_done)begin
                    steering_deg<=angle>MAX_STEER_DEG*65536?MAX_STEER_DEG:
                        angle < -MAX_STEER_DEG*65536?-MAX_STEER_DEG:(angle+32'sd32768)>>>16;
                    done<=1;state<=0;
                end
                default:state<=0;
            endcase
            if(sqrt_done)distance_mm<=root>65535?16'hffff:root[15:0];
        end
    end
endmodule
