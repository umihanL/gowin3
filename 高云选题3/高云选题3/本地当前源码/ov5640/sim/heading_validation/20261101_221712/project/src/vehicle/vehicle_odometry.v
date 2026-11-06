// Rear-axle bicycle pose with optional NOSE output; mm Q8, X right/Y up, CCW+.
// Encoders measure distance; IMU yaw measures direction. Acceleration is not
// double-integrated. Rebase establishes a new count/yaw origin for each run.
`include "src/vehicle/vehicle_config.vh"
module vehicle_odometry #(
    parameter COUNTS_PER_REV=`VEHICLE_COUNTS_PER_REV,
    WHEEL_DIAMETER_MM=`VEHICLE_WHEEL_DIAMETER_MM,IMU_YAW_SIGN=`VEHICLE_IMU_YAW_SIGN,
    FRONT_REFERENCE=0,NOSE_MM=`VEHICLE_NOSE_FROM_REAR_MM,TRACK_MM=`VEHICLE_REAR_TRACK_MM
)(input clk,rst_n,tick,rebase,imu_fresh,imu_restarted,
    input signed [23:0] initial_x_q8,initial_y_q8,
    input signed [15:0] initial_heading,imu_yaw,
    input signed [31:0] encoder1,encoder2,
    output reg signed [23:0] x_q8,y_q8,
    output reg signed [23:0] rear_x_q8,rear_y_q8,
    output reg signed [15:0] heading,
    output reg signed [31:0] travel_q8,
    output signed [31:0] relative1,relative2,
    output reg updated,encoder_fault,
    output reg signed [16:0] cos_heading,sin_heading);
    localparam signed [31:0] MM_PER_COUNT_Q16=
        (64'd65536*WHEEL_DIAMETER_MM*314159+COUNTS_PER_REV*50000)/(COUNTS_PER_REV*100000);
    reg signed [31:0] base1,base2,previous1,previous2;
    reg signed [15:0] yaw_origin,heading_origin;
    localparam signed [31:0] YAW_PER_DIFF_Q16=
        (64'd65536*WHEEL_DIAMETER_MM*1800+COUNTS_PER_REV*TRACK_MM/2)/(COUNTS_PER_REV*TRACK_MM);
    reg signed [31:0] heading_q16,predicted_q16,innovation,correction;
    reg signed [31:0] previous_heading_q16,heading_step_q16;
    reg signed [31:0] yaw_delta_counts,yaw_increment,imu_heading_q16;
    reg sample_imu_fresh;
    reg [3:0] state;
    reg trig_start;reg signed [15:0] trig_angle;
    wire trig_done;wire signed [16:0] cs,sn;
    reg signed [32:0] sum_counts;
    reg signed [63:0] count_product;
    reg signed [17:0] distance_q8;
    reg signed [35:0] dx,dy;
    // Preserve discarded fractions across samples, including reverse motion.
    // These are nonnegative Euclidean remainders for the arithmetic shifts;
    // truncating every 10 ms otherwise accumulates a direction-dependent bias.
    reg [8:0] distance_remainder;
    reg [14:0] x_remainder,y_remainder;
    wire signed [63:0] distance_with_remainder=count_product+$signed({55'd0,distance_remainder});
    wire signed [35:0] dx_with_remainder=dx+$signed({21'd0,x_remainder});
    wire signed [35:0] dy_with_remainder=dy+$signed({21'd0,y_remainder});
    wire signed [31:0] delta1=encoder1-previous1,delta2=encoder2-previous2;
    wire signed [31:0] raw_heading=$signed(heading_origin)+IMU_YAW_SIGN*
        ($signed(imu_yaw)-$signed(yaw_origin));
    wire signed [31:0] imu_heading=raw_heading>1800?raw_heading-3600:raw_heading < -1800?raw_heading+3600:raw_heading;
    // Balanced constant shift/add tree: preserve 32-bit modular multiplication
    // without consuming the DSP columns already needed by the video/navigation.
    function signed [31:0] constant_scale;
        input signed [31:0] value;
        input [31:0] coefficient;
        reg signed [31:0] partial [0:31];
        integer i;
        begin
            for(i=0;i<32;i=i+1)partial[i]=coefficient[i]?(value<<<i):32'sd0;
            for(i=0;i<16;i=i+1)partial[i]=partial[2*i]+partial[2*i+1];
            for(i=0;i<8;i=i+1)partial[i]=partial[2*i]+partial[2*i+1];
            for(i=0;i<4;i=i+1)partial[i]=partial[2*i]+partial[2*i+1];
            for(i=0;i<2;i=i+1)partial[i]=partial[2*i]+partial[2*i+1];
            constant_scale=partial[0]+partial[1];
        end
    endfunction
    wire signed [31:0] offset_x=FRONT_REFERENCE?constant_scale($signed(cs),NOSE_MM)>>>7:32'sd0;
    wire signed [31:0] offset_y=FRONT_REFERENCE?constant_scale($signed(sn),NOSE_MM)>>>7:32'sd0;
    function signed [31:0] wrap_q16;
        input signed [31:0] a;
        begin wrap_q16=a>117964800?a-235929600:a < -117964800?a+235929600:a;end
    endfunction
    function signed [15:0] round_heading;
        input signed [31:0] a;
        begin round_heading=(a+32'sd32768)>>>16;end
    endfunction
    wire signed [31:0] fused_heading_q16=wrap_q16(predicted_q16+correction);
    wire signed [31:0] midpoint_heading_q16=wrap_q16(previous_heading_q16+(heading_step_q16>>>1));
    // Wheel prediction bridges missed frames. Capture one coherent 10 ms
    // sample, then pipeline subtraction/scaling/wrap/innovation/correction;
    // none of these carry chains shares a single 33 MHz timing path.
    wire signed [31:0] desired_correction=sample_imu_fresh &&
        innovation<=13107200 && innovation>=-13107200?innovation>>>2:32'sd0;
    assign relative1=encoder1-base1;
    assign relative2=encoder2-base2;
    vehicle_sincos trig(.clk(clk),.rst_n(rst_n),.start(trig_start),.angle(trig_angle),
        .done(trig_done),.cosine(cs),.sine(sn));
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin
            base1<=0;base2<=0;previous1<=0;previous2<=0;yaw_origin<=0;heading_origin<=0;
            x_q8<=0;y_q8<=0;rear_x_q8<=0;rear_y_q8<=0;heading_q16<=0;heading<=0;travel_q8<=0;state<=0;trig_start<=0;
            predicted_q16<=0;innovation<=0;correction<=0;yaw_delta_counts<=0;
            previous_heading_q16<=0;heading_step_q16<=0;trig_angle<=0;
            yaw_increment<=0;imu_heading_q16<=0;sample_imu_fresh<=0;
            sum_counts<=0;count_product<=0;distance_q8<=0;dx<=0;dy<=0;
            distance_remainder<=0;x_remainder<=0;y_remainder<=0;
            updated<=0;encoder_fault<=0;cos_heading<=32768;sin_heading<=0;
        end else begin
            updated<=0;trig_start<=0;
            if(rebase)begin
                base1<=encoder1;base2<=encoder2;previous1<=encoder1;previous2<=encoder2;
                yaw_origin<=imu_yaw;heading_origin<=initial_heading;heading<=initial_heading;
                x_q8<=initial_x_q8;y_q8<=initial_y_q8;travel_q8<=0;encoder_fault<=0;state<=5;
                heading_q16<=$signed(initial_heading)<<<16;
                trig_angle<=initial_heading;
                distance_remainder<=0;x_remainder<=0;y_remainder<=0;
            end else case(state)
                0:if(tick)begin
                    sum_counts<=$signed({delta1[31],delta1})+$signed({delta2[31],delta2});
                    previous1<=encoder1;previous2<=encoder2;
                    if(delta1>200 || delta1 < -200 || delta2>200 || delta2 < -200)encoder_fault<=1;
                    yaw_delta_counts<=delta2-delta1;
                    imu_heading_q16<=imu_heading<<<16;sample_imu_fresh<=imu_fresh;
                    if(imu_restarted && imu_fresh)begin
                        // Encoder yaw already covers movement during a real
                        // sensor restart. Align the new transport epoch to it;
                        // do not pull the moving car back to a pre-gap angle.
                        yaw_origin<=imu_yaw;heading_origin<=heading;
                        sample_imu_fresh<=0;
                    end
                    state<=7;
                end
                7:begin yaw_increment<=constant_scale(yaw_delta_counts,YAW_PER_DIFF_Q16);state<=8;end
                8:begin predicted_q16<=wrap_q16(heading_q16+yaw_increment);state<=9;end
                9:begin innovation<=wrap_q16(imu_heading_q16-predicted_q16);state<=10;end
                10:begin
                    correction<=desired_correction>327680?32'sd327680:
                        desired_correction < -327680?-32'sd327680:desired_correction;
                    state<=11;
                end
                11:begin
                    previous_heading_q16<=heading_q16;
                    heading_step_q16<=wrap_q16(fused_heading_q16-heading_q16);
                    heading_q16<=fused_heading_q16;
                    heading<=round_heading(fused_heading_q16);state<=12;
                end
                // Integrate this interval at the shortest-arc midpoint, not
                // its final heading. Reuse the same CORDIC a second time for
                // the endpoint direction required by pursuit/nose geometry.
                12:begin trig_angle<=round_heading(midpoint_heading_q16);state<=1;end
                1:begin count_product<=sum_counts*MM_PER_COUNT_Q16;trig_start<=1;state<=2;end
                2:begin
                    distance_q8<=distance_with_remainder>>>9;
                    distance_remainder<=distance_with_remainder[8:0];state<=3;
                end
                3:if(trig_done)begin
                    dx<=distance_q8*cs;dy<=distance_q8*sn;
                    if(trig_angle==heading)begin cos_heading<=cs;sin_heading<=sn;state<=4;end
                    else begin trig_angle<=heading;state<=13;end
                end
                13:begin trig_start<=1;state<=14;end
                14:if(trig_done)begin cos_heading<=cs;sin_heading<=sn;state<=4;end
                4:begin
                    if(!encoder_fault)begin
                        rear_x_q8<=rear_x_q8+(dx_with_remainder>>>15);rear_y_q8<=rear_y_q8+(dy_with_remainder>>>15);
                        x_q8<=rear_x_q8+(dx_with_remainder>>>15)+offset_x;y_q8<=rear_y_q8+(dy_with_remainder>>>15)+offset_y;
                        x_remainder<=dx_with_remainder[14:0];y_remainder<=dy_with_remainder[14:0];
                        travel_q8<=travel_q8+distance_q8;
                    end
                    updated<=1;state<=0;
                end
                5:begin trig_start<=1;state<=6;end
                6:if(trig_done)begin
                    cos_heading<=cs;sin_heading<=sn;
                    rear_x_q8<=x_q8-offset_x;rear_y_q8<=y_q8-offset_y;
                    updated<=1;state<=0;
                end
                default:state<=0;
            endcase
        end
    end
endmodule
