// Quaternion -> relative ZYX Euler angles, tenths of a degree.
// SFLP performs gyro-bias compensation. This block establishes an orientation
// reference only after a stationary window, then uses conjugate(q0) * q.
// Roll is about sensor X, Pitch about Y, Yaw about Z (right hand convention).
module imu_attitude #(parameter SETTLE_SAMPLES=120, ZERO_SAMPLES=60)(
    input clk,rst_n,online,recalibrate,sample_valid,fusion_reset,
    input [15:0] qx_half,qy_half,qz_half,
    input signed [15:0] gx,gy,gz,ax,ay,az,
    output reg calibrated,valid,available,
    // 0 fusion warming up, 1 moving, 2 acquiring zero, 3 calibrated
    output reg [1:0] cal_state,
    output reg signed [15:0] roll,pitch,yaw,
    // Complete relative quaternion, Q14. Valid together with the Euler result.
    output reg signed [17:0] relative_w,relative_x,relative_y,relative_z
);
    function signed [17:0] half_q14;
        input [15:0] h; reg [31:0] m;
        begin
            m={21'd0,1'b1,h[9:0]};
            if(h[14:10]==0) m=0;
            else if(h[14:10]>=11) m=m<<(h[14:10]-11);
            else m=m>>(11-h[14:10]);
            half_q14=h[15] ? -$signed(m[17:0]) : $signed(m[17:0]);
        end
    endfunction
    function [18:0] distance;
        input signed [17:0] a,b; reg signed [18:0] d;
        begin d={a[17],a}-{b[17],b};distance=d<0 ? -d : d;end
    endfunction
    function signed [15:0] to_tenths;
        input signed [31:0] a; reg signed [35:0] v;
        begin v=a*36'sd10+36'sd32768;to_tenths=v[31:16];end
    endfunction
    reg [5:0] state;
    reg [15:0] settle_count,zero_count;
    reg recovery_pending;
    reg signed [17:0] qw,qx,qy,qz,rw,rx,ry,rz,w,x,y,z;
    reg signed [17:0] cw,cx,cy,cz;
    // q and -q are the same orientation, including at the +/-180-degree seam.
    wire same_pose=(distance(qw,cw)<=256 && distance(qx,cx)<=256 &&
                    distance(qy,cy)<=256 && distance(qz,cz)<=256) ||
                   (distance(-qw,cw)<=256 && distance(-qx,cx)<=256 &&
                    distance(-qy,cy)<=256 && distance(-qz,cz)<=256);
    reg signed [35:0] p0,p1,p2,p3;
    // Four explicit shared multipliers serve normal quaternion products and
    // recovery. Separate '*' operators in each state would duplicate DSPs.
    reg signed [17:0] ma0,ma1,ma2,ma3,mb0,mb1,mb2,mb3;
    always @*begin
        ma0=qx;mb0=qx;ma1=qy;mb1=qy;ma2=qz;mb2=qz;ma3=0;mb3=0;
        if(state==5)case(component)
            0:begin ma0=rw;mb0=qw;ma1=rx;mb1=qx;ma2=ry;mb2=qy;ma3=rz;mb3=qz;end
            1:begin ma0=rw;mb0=qx;ma1=-rx;mb1=qw;ma2=-ry;mb2=qz;ma3=rz;mb3=qy;end
            2:begin ma0=rw;mb0=qy;ma1=rx;mb1=qz;ma2=-ry;mb2=qw;ma3=-rz;mb3=qx;end
            3:begin ma0=rw;mb0=qz;ma1=-rx;mb1=qy;ma2=ry;mb2=qx;ma3=-rz;mb3=qw;end
        endcase
        else if(state==15)case(component)
            0:begin ma0=qw;mb0=relative_w;ma1=qx;mb1=relative_x;ma2=qy;mb2=relative_y;ma3=qz;mb3=relative_z;end
            1:begin ma0=-qw;mb0=relative_x;ma1=qx;mb1=relative_w;ma2=-qy;mb2=relative_z;ma3=qz;mb3=relative_y;end
            2:begin ma0=-qw;mb0=relative_y;ma1=qx;mb1=relative_z;ma2=qy;mb2=relative_w;ma3=-qz;mb3=relative_x;end
            3:begin ma0=-qw;mb0=relative_z;ma1=-qx;mb1=relative_y;ma2=qy;mb2=relative_x;ma3=qz;mb3=relative_w;end
        endcase
    end
    wire signed [35:0] mp0=ma0*mb0,mp1=ma1*mb1,mp2=ma2*mb2,mp3=ma3*mb3;
    wire signed [37:0] sum4={{2{p0[35]}},p0}+{{2{p1[35]}},p1}+
                            {{2{p2[35]}},p2}+{{2{p3[35]}},p3};
    reg [1:0] component;
    reg signed [35:0] xx,yy,zz,wx,wy,wz,xy,xz,yz;
    wire signed [35:0] roll_denom=p0+zz-xx-yy, roll_numer=(wx+yz)<<<1;
    wire signed [35:0] yaw_denom=p0+xx-yy-zz, yaw_numer=(wz+xy)<<<1;
    wire signed [35:0] pitch_numer=(wy-xz)<<<1;
    reg [31:0] ax2,ay2,az2;
    reg gyro_still,stationary;
    reg sqrt_start,atan_start;
    reg [63:0] sqrt_input;
    wire sqrt_done,atan_done; wire [31:0] sqrt_result;
    reg signed [31:0] atan_x,atan_y;
    wire signed [31:0] angle;
    reg signed [31:0] roll_x,roll_y,yaw_x,yaw_y,sin_pitch;
    // Pitch = atan2(2(wy-xz), hypot(R32,R33)). Unlike asin(2(wy-xz)), this
    // cancels the small norm error left by half/Q14 rounding, even at
    // +/-90 degrees. Reduce the cosine terms from Q28 to Q14, then
    // reuse one 16-bit square multiplier across states 10/11. Restore
    // the Q56 scale before sqrt so its output still matches sin_pitch.
    wire signed [15:0] pitch_cos_component=state==10 ? roll_x[29:14] : roll_y[29:14];
    wire signed [31:0] pitch_cos_square_q28=pitch_cos_component*pitch_cos_component;
    wire [63:0] pitch_cos_square={4'd0,pitch_cos_square_q28,28'd0};
    reg signed [63:0] pitch_square;
    reg signed [15:0] next_roll,next_pitch;
    imu_sqrt u_sqrt(.clk(clk),.rst_n(rst_n),.start(sqrt_start),.value(sqrt_input),.done(sqrt_done),.root(sqrt_result));
    imu_atan2 u_atan(.clk(clk),.rst_n(rst_n),.start(atan_start),.vx(atan_x),.vy(atan_y),.done(atan_done),.angle(angle));
    // Aborting a zero operation does not interrupt the arithmetic units. New
    // work starts only after their bounded latency; normal samples are 16ms apart.
    reg [7:0] restart_guard;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            state<=0;settle_count<=0;zero_count<=0;calibrated<=0;valid<=0;available<=0;cal_state<=0;recovery_pending<=0;
            roll<=0;pitch<=0;yaw<=0;next_roll<=0;next_pitch<=0;
            qw<=0;qx<=0;qy<=0;qz<=0;rw<=16384;rx<=0;ry<=0;rz<=0;
            cw<=16384;cx<=0;cy<=0;cz<=0;
            w<=0;x<=0;y<=0;z<=0;p0<=0;p1<=0;p2<=0;p3<=0;component<=0;
            xx<=0;yy<=0;zz<=0;wx<=0;wy<=0;wz<=0;xy<=0;xz<=0;yz<=0;
            ax2<=0;ay2<=0;az2<=0;gyro_still<=0;stationary<=0;
            sqrt_start<=0;atan_start<=0;sqrt_input<=0;atan_x<=0;atan_y<=0;
            roll_x<=0;roll_y<=0;yaw_x<=0;yaw_y<=0;sin_pitch<=0;pitch_square<=0;
            restart_guard<=0;
            relative_w<=16384;relative_x<=0;relative_y<=0;relative_z<=0;
        end else begin
            sqrt_start<=0;atan_start<=0;valid<=0;
            if(recalibrate) begin
                state<=0;settle_count<=0;zero_count<=0;calibrated<=0;recovery_pending<=0;
                roll<=0;pitch<=0;yaw<=0;restart_guard<=100;available<=0;cal_state<=0;
                rw<=16384;rx<=0;ry<=0;rz<=0;
                relative_w<=16384;relative_x<=0;relative_y<=0;relative_z<=0;
            end else if(fusion_reset)begin
                // Preserve the user's angle through a real sensor restart.
                // On the first fresh quaternion, transport the saved reference
                // into the new sensor coordinates; never re-zero to identity.
                state<=0;available<=0;restart_guard<=100;
                if(calibrated)recovery_pending<=1;
            end else if(!online)begin
                // Preserve the zero reference through a transport interruption.
                // Only S0 or board reset may discard this user reference.
                state<=0;available<=0;restart_guard<=100;
            end else if(restart_guard!=0) restart_guard<=restart_guard-1'b1;
            else case(state)
                0:if(sample_valid) begin
                    qx<=half_q14(qx_half);qy<=half_q14(qy_half);qz<=half_q14(qz_half);
                    ax2<=ax*ax;ay2<=ay*ay;az2<=az*az;
                    // Raw gyro may contain bias before SFLP converges. A 6 dps
                    // gate plus an orientation window rejects rotation without
                    // requiring the raw, uncorrected gyro to already be zero.
                    gyro_still<=gx>-86 && gx<86 && gy>-86 && gy<86 && gz>-86 && gz<86;
                    state<=1;
                end
                1:begin
                    p0<=mp0;p1<=mp1;p2<=mp2;p3<=mp3;
                    // +/-4g: 1g=8197 LSB. Accept 0.85g..1.15g, not free fall.
                    stationary<=gyro_still && ({2'd0,ax2}+{2'd0,ay2}+{2'd0,az2}>34'd48500000)
                                           && ({2'd0,ax2}+{2'd0,ay2}+{2'd0,az2}<34'd88900000);
                    state<=2;
                end
                2:begin
                    // binary16 rounding may make |xyz|^2 slightly larger than 1.
                    if(sum4>38'sd269000000) begin state<=0;zero_count<=0;end
                    else begin
                        sqrt_input<=sum4>=38'sd268435456 ? 0 : 64'd268435456-sum4;
                        sqrt_start<=1;state<=3;
                    end
                end
                3:if(sqrt_done) begin qw<=sqrt_result[17:0];state<=4;end
                4:begin
                    // Live angles are useful BEFORE zero is complete. Until a
                    // reference is committed rw/rx/ry/rz is identity.
                    component<=0;state<=5;
                    if(recovery_pending && calibrated)begin state<=15;end
                    else if(settle_count<SETTLE_SAMPLES) begin settle_count<=settle_count+1'b1;cal_state<=0;end
                    else if(!calibrated) begin
                        if(!stationary) begin zero_count<=0;cal_state<=1;end
                        else if(zero_count==0) begin cw<=qw;cx<=qx;cy<=qy;cz<=qz;zero_count<=1;cal_state<=2;end
                        else if(!same_pose) begin zero_count<=0;cal_state<=1;end
                        else if(zero_count==ZERO_SAMPLES-1) begin
                            rw<=qw;rx<=qx;ry<=qy;rz<=qz;calibrated<=1;
                            roll<=0;pitch<=0;yaw<=0;valid<=1;available<=1;cal_state<=3;state<=0;
                            relative_w<=16384;relative_x<=0;relative_y<=0;relative_z<=0;
                        end else zero_count<=zero_count+1'b1;
                    end
                end
                5:begin // conjugate(reference) multiplied by present orientation
                    p0<=mp0;p1<=mp1;p2<=mp2;p3<=mp3;
                    state<=6;
                end
                6:begin
                    case(component)
                        0:w<=sum4[31:14];1:x<=sum4[31:14];2:y<=sum4[31:14];3:z<=sum4[31:14];
                    endcase
                    if(component==3) state<=7;
                    else begin component<=component+1'b1;state<=5;end
                end
                7:begin
                    xx<=x*x;yy<=y*y;zz<=z*z;wx<=w*x;wy<=w*y;wz<=w*z;
                    xy<=x*y;xz<=x*z;yz<=y*z;state<=8;
                end
                8:begin
                    // Use quaternion norm instead of literal 1 for roll/yaw.
                    // A binary16 packet has a small norm error near w=0.
                    p0<=w*w;state<=9;
                end
                9:begin
                    roll_x<=roll_denom[31:0];roll_y<=roll_numer[31:0];
                    yaw_x<=yaw_denom[31:0];yaw_y<=yaw_numer[31:0];
                    sin_pitch<=pitch_numer[31:0];
                    state<=10;
                end
                10:begin
                    pitch_square<=pitch_cos_square;
                    atan_x<=roll_x;atan_y<=roll_y;atan_start<=1;state<=11;
                end
                11:begin sqrt_input<=pitch_square+pitch_cos_square;sqrt_start<=1;state<=12;end
                12:begin
                    if(atan_done) next_roll<=to_tenths(angle);
                    if(sqrt_done) begin atan_x<=sqrt_result;atan_y<=sin_pitch;atan_start<=1;state<=13;end
                end
                13:if(atan_done) begin
                    next_pitch<=to_tenths(angle);atan_x<=yaw_x;atan_y<=yaw_y;atan_start<=1;state<=14;
                end
                14:if(atan_done) begin
                    roll<=next_roll;pitch<=next_pitch;yaw<=to_tenths(angle);valid<=1;available<=1;state<=0;
                    relative_w<=w;relative_x<=x;relative_y<=y;relative_z<=z;
                end
                15:begin // new_reference = new_sensor * conjugate(last_relative)
                    p0<=mp0;p1<=mp1;p2<=mp2;p3<=mp3;
                    state<=16;
                end
                16:begin
                    case(component)
                        0:rw<=sum4[31:14];1:rx<=sum4[31:14];2:ry<=sum4[31:14];3:rz<=sum4[31:14];
                    endcase
                    if(component==3)begin recovery_pending<=0;component<=0;state<=5;end
                    else begin component<=component+1'b1;state<=15;end
                end
                default:state<=0;
            endcase
        end
    end
endmodule
