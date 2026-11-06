// Closed-loop Ackermann bypass in a latched local straight-road frame.
// Distances mm, angles 0.1 degree. Serial frame rotation uses no DSP/BSRAM.
// 1 turn out, 7 diagonal clearance, 6 align, 2 pass/tail, 3 merge, 4 done, 5 stop.
`include "src/vehicle/vehicle_config.vh"
module vehicle_bypass #(parameter TAIL_MM=100,MAX_TRAVEL_MM=3000,
    MIN_OFFSET_MM=45,SINGLE_FRONT_SIDE_MM=25,SINGLE_SIDE_OFFSET_MM=25,
    FRONT_INSET_MM=30,EDGE_EXTRA_MM=FRONT_INSET_MM+30,
    FOLLOW_NEAR_MM=35,FOLLOW_FAR_MM=75)(
    input clk,rst_n,tick,start,cancel,choose_left,
    input single_front_entry,
    input signed [15:0] px,py,heading,input signed [16:0] cosine,sine,
    input signed [31:0] travel_q8,
    input [15:0] lf,rf,left_mm,right_mm,
    input [31:0] diagnostics,input [3:0] observations,
    output reg active,done,blocked,output reg [3:0] phase,
    output reg signed [15:0] steering,
    output reg signed [15:0] local_x,local_y,
    output reg tail_confirmed,output reg left_turn);
    reg signed [15:0] origin_x,origin_y,origin_heading;
    reg signed [16:0] base_cos,base_sin;
    reg signed [31:0] start_travel,tail_travel;
    reg edge_confirmed;reg signed [31:0] edge_travel;
    reg [3:0] pending;
    reg single_front_latched;reg [1:0] single_close_samples;
    reg [15:0] entry_inner_front;
    reg [2:0] clear_lf,clear_rf,clear_side;
    reg seen_side;reg [15:0] nearest_side;
    reg [15:0] previous_inner;reg [2:0] falling_samples,rising_samples;
    reg side_edge_rising;
    reg signed [31:0] side_min_travel;
    reg side_previous_valid;
    reg [15:0] first_inner,fall_reference;
    reg [5:0] lf_age,rf_age,side_age;
    reg [3:0] heading_settled;
    reg [15:0] edge_bound,pass_ticks;
    reg signed [15:0] tail_x,clear_x,previous_heading,pass_y;
    reg [15:0] nearest_lf,nearest_rf;
    reg [15:0] elapsed;
    wire [15:0] inner=left_turn?right_mm:left_mm;
    wire [7:0] inner_code=left_turn?diagnostics[31:24]:diagnostics[7:0];
    wire [3:0] event_bits=pending|observations;
    wire inner_event=left_turn?event_bits[3]:event_bits[0];
    wire inner_valid=inner_code==0 && inner>0 && inner<=4000;
    wire inner_noecho=inner_code==8'h11 || inner_code==8'h12;
    wire lf_clear=(diagnostics[15:8]==0 && lf>nearest_lf+250 && lf>500 && lf<=4000) ||
        diagnostics[15:8]==8'h11 || diagnostics[15:8]==8'h12;
    wire rf_clear=(diagnostics[23:16]==0 && rf>nearest_rf+250 && rf>500 && rf<=4000) ||
        diagnostics[23:16]==8'h11 || diagnostics[23:16]==8'h12;
    wire [15:0] outward=left_turn?left_mm:right_mm;
    wire [7:0] outward_code=left_turn?diagnostics[7:0]:diagnostics[31:24];
    wire signed [16:0] raw_heading=$signed(heading)-$signed(origin_heading);
    wire signed [15:0] relative_heading=raw_heading>1800?raw_heading-3600:
        raw_heading < -1800?raw_heading+3600:raw_heading;
    wire signed [15:0] side_offset=left_turn?local_y:-local_y;
    wire signed [16:0] relative_cos,relative_sin;
    wire relative_trig_done;reg edge_math_ready;
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)edge_math_ready<=0;
        else if(start || cancel)edge_math_ready<=0;
        else if(relative_trig_done)edge_math_ready<=1;
    end
    vehicle_sincos edge_trig(.clk(clk),.rst_n(rst_n),.start(tick && active),
        .angle(relative_heading),.cosine(relative_cos),.sine(relative_sin),.done(relative_trig_done));
    wire signed [16:0] outward_sin=left_turn?relative_sin:-relative_sin;
    wire [15:0] inner_front=left_turn?rf:lf;
    wire [7:0] inner_front_code=left_turn?diagnostics[23:16]:diagnostics[15:8];
    wire inner_front_event=left_turn?event_bits[2]:event_bits[1];
    // Entry classification is latched by motion arbitration using the
    // configured trigger distance. A dual-beam obstacle never changes into
    // this shortcut when one beam clears during the manoeuvre.
    wire [15:0] outer_front=left_turn?lf:rf;
    wire [7:0] outer_front_code=left_turn?diagnostics[15:8]:diagnostics[23:16];
    wire outer_front_event=left_turn?event_bits[1]:event_bits[2];
    wire outer_front_clear_now=(outer_front_code==8'h11 || outer_front_code==8'h12) ||
        (outer_front_code==0 && outer_front>500 && outer_front<=4000);
    wire inner_front_fresh=(left_turn?rf_age:lf_age)<30 || inner_front_event;
    wire outer_front_fresh=(left_turn?lf_age:rf_age)<30 || outer_front_event;
    wire single_front_evidence=single_front_latched && inner_front_fresh && outer_front_fresh &&
        inner_front_code==0 && inner_front>0 && inner_front<=entry_inner_front+100 && outer_front_clear_now;
    reg signed [34:0] range_projection,beam_projection,edge_projection;
    always @(posedge clk)begin
        range_projection<=($signed({1'b0,inner_front})+(`VEHICLE_NOSE_FROM_REAR_MM-`VEHICLE_FRONT_LASER_FROM_NOSE_MM))*outward_sin;
        beam_projection<=(`VEHICLE_FRONT_LASER_TRACK_MM/2)*relative_cos;
        edge_projection<=range_projection-beam_projection;
    end
    wire signed [16:0] measured_edge=$signed(side_offset)+(edge_projection>>>15);
    // Side minimum/rebound owns alignment. Do not reintroduce a 175mm
    // body-envelope gate (or an unbounded edge_bound-derived substitute).
    // Clamp even an instance override to the requested 50mm maximum.
    wire [16:0] required_offset=MIN_OFFSET_MM>50?17'd50:MIN_OFFSET_MM;
    wire heading_stable=heading_error<25 && heading_error > -25 &&
        relative_heading-previous_heading<4 && relative_heading-previous_heading > -4;
    wire front_cleared=clear_lf>=2 && clear_rf>=2 && lf_age<30 && rf_age<30 && lf_clear && rf_clear;
    // A no-echo -> finite hit is not a falling trend. Establish the real
    // decreasing sequence only on the settled diagonal, never during yaw.
    wire decline_established=side_previous_valid && falling_samples>=2 && first_inner>=nearest_side+12;
    wire rise_sample=phase==7 && heading_settled>=3 && front_cleared && inner_event && decline_established &&
        ((inner_valid && inner<800 && inner>nearest_side+8 &&
          inner+2>=previous_inner && travel_q8-side_min_travel>=2*256) ||
         (inner_noecho && seen_side));
    wire rise_confirm_now=rise_sample && rising_samples>=1;
    wire single_close_ready=phase==7 && heading_settled>=3 && single_front_evidence &&
        side_offset>=SINGLE_SIDE_OFFSET_MM;
    // Count distinct completed side samples. Front/side I2C channels finish
    // independently; their event pulses must not be required on one clock.
    wire single_front_return=single_close_ready && inner_event && inner_valid &&
        inner<=SINGLE_FRONT_SIDE_MM && single_close_samples>=1;
    wire normal_side_return=phase==7 && front_cleared && side_offset>=0 &&
        ({1'b0,side_offset}>=required_offset) &&
        (side_edge_rising || rise_confirm_now);
    wire merge_done;wire signed [15:0] merge_steer;
    // Rear-axle pursuit anticipates the countersteer while converging to the
    // original line. The former weak y->heading controller spent ~750 mm
    // returning from a 280 mm offset even after the tail had cleared.
    vehicle_tracker #(.CONTROL_OFFSET_MM(0)) merge_tracker(
        .clk(clk),.rst_n(rst_n),.start(tick && active && phase==3),
        .precise_bearing(1'b0),.bearing_dx_q8(24'sd0),.bearing_dy_q8(24'sd0),
        .target_x($signed(local_x)+16'sd140),.target_y(16'sd0),
        .pose_x(local_x),.pose_y(local_y),.cosine(relative_cos),.sine(relative_sin),
        .done(merge_done),.steering_deg(merge_steer),.distance_mm(),
        .bearing_tenths(),.forward_mm(),.lateral_mm());
    // A far trigger must not hold a 35-degree diagonal all the way to the
    // box: that alone creates excessive lateral offset before the side edge.
    // This sets departure angle only. Completed side observations decide
    // alignment, and phase 3 keeps the already accepted merge controller.
    wire [15:0] entry_front=diagnostics[15:8]==0 && diagnostics[23:16]==0?
        (lf<rf?lf:rf):diagnostics[15:8]==0?lf:diagnostics[23:16]==0?rf:16'd650;
    reg signed [15:0] departure_angle;
    reg signed [15:0] desired_heading,heading_error,command;
    reg signed [16:0] merge_angle;
    always @*begin
        desired_heading=0;merge_angle=-$signed(local_y)*3;
        if(phase==1 || phase==7)desired_heading=left_turn?departure_angle:-departure_angle;
        else if(phase==3)desired_heading=merge_angle>350?16'sd350:
            merge_angle < -350?-16'sd350:merge_angle;
        else if(phase==2)begin
            // Hold the parallel line latched after alignment. Side echoes
            // detect the trailing edge only; they must not steer the car.
            desired_heading=($signed(pass_y)-$signed(local_y))*2;
            if(desired_heading>30)desired_heading=30;
            else if(desired_heading < -30)desired_heading=-30;
        end
        heading_error=desired_heading-relative_heading;
        command=(heading_error>>>2)-(($signed(relative_heading)-$signed(previous_heading))>>>1);
        if((phase==2 || phase==6) && heading_error<=8 && heading_error>=-8)command=0;
    end
    // Four signed products for local pose, one serial multiplier.
    reg [2:0] math_state;reg [4:0] bit_index;
    reg [1:0] term;reg negative;reg [16:0] factor;
    reg [33:0] addend,accumulator;reg signed [34:0] sum_x,sum_y;
    reg signed [16:0] frame_dx,frame_dy,multiplicand,coefficient;
    wire [33:0] product=accumulator+(factor[0]?addend:34'd0);
    wire signed [34:0] signed_product=negative?-$signed({1'b0,product}):$signed({1'b0,product});
    always @*begin
        multiplicand=term==0 || term==2?frame_dx:frame_dy;
        coefficient=term==0 || term==3?base_cos:base_sin;
    end
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin
            math_state<=0;bit_index<=0;term<=0;negative<=0;factor<=0;addend<=0;accumulator<=0;
            sum_x<=0;sum_y<=0;frame_dx<=0;frame_dy<=0;local_x<=0;local_y<=0;
        end else if(start || cancel)begin math_state<=0;local_x<=0;local_y<=0;end
        else case(math_state)
            0:if(tick && active)begin
                frame_dx<=$signed(px)-$signed(origin_x);frame_dy<=$signed(py)-$signed(origin_y);
                term<=0;sum_x<=0;sum_y<=0;math_state<=1;
            end
            1:begin
                negative<=multiplicand[16]^coefficient[16];
                factor<=coefficient<0?-coefficient:coefficient;
                addend<=multiplicand<0?-multiplicand:multiplicand;
                accumulator<=0;bit_index<=0;math_state<=2;
            end
            2:begin
                accumulator<=product;factor<=factor>>1;addend<=addend<<1;
                if(bit_index==16)begin
                    case(term)
                        0:sum_x<=signed_product;
                        1:sum_x<=sum_x+signed_product;
                        2:sum_y<=-signed_product;
                        3:sum_y<=sum_y+signed_product;
                    endcase
                    if(term==3)math_state<=3;
                    else begin term<=term+1'b1;math_state<=1;end
                end else bit_index<=bit_index+1'b1;
            end
            3:begin local_x<=sum_x>>>15;local_y<=sum_y>>>15;math_state<=0;end
            default:math_state<=0;
        endcase
    end
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin
            active<=0;done<=0;blocked<=0;phase<=0;steering<=0;left_turn<=0;departure_angle<=350;
            single_front_latched<=0;single_close_samples<=0;entry_inner_front<=0;
            origin_x<=0;origin_y<=0;origin_heading<=0;base_cos<=32768;base_sin<=0;
            start_travel<=0;tail_travel<=0;tail_confirmed<=0;pending<=0;edge_confirmed<=0;edge_travel<=0;
            edge_bound<=0;pass_ticks<=0;tail_x<=0;clear_x<=0;previous_heading<=0;pass_y<=0;
            clear_lf<=0;clear_rf<=0;clear_side<=0;seen_side<=0;nearest_side<=2000;elapsed<=0;nearest_lf<=2000;nearest_rf<=2000;
            previous_inner<=4000;falling_samples<=0;rising_samples<=0;side_edge_rising<=0;side_min_travel<=0;
            side_previous_valid<=0;first_inner<=0;fall_reference<=0;lf_age<=63;rf_age<=63;side_age<=63;heading_settled<=0;
        end else begin
            done<=0;pending<=pending|observations;
            if(merge_done && active && phase==3 && !blocked)steering<=merge_steer;
            if(cancel)begin active<=0;phase<=0;blocked<=0;steering<=0;pending<=0;end
            else if(start && !active)begin
                active<=1;blocked<=0;phase<=1;steering<=0;left_turn<=choose_left;
                single_front_latched<=single_front_entry===1'b1;single_close_samples<=0;
                entry_inner_front<=choose_left?rf:lf;
                departure_angle<=entry_front>900?150:entry_front>500?200:entry_front>350?300:350;
                origin_x<=px;origin_y<=py;origin_heading<=heading;base_cos<=cosine;base_sin<=sine;
                start_travel<=travel_q8;tail_confirmed<=0;clear_lf<=0;clear_rf<=0;clear_side<=0;edge_confirmed<=0;
                seen_side<=0;nearest_side<=2000;elapsed<=0;pending<=0;
                edge_bound<=0;pass_ticks<=0;tail_x<=0;clear_x<=0;previous_heading<=0;pass_y<=0;
                nearest_lf<=diagnostics[15:8]==0?lf:2000;nearest_rf<=diagnostics[23:16]==0?rf:2000;
                previous_inner<=4000;falling_samples<=0;rising_samples<=0;side_edge_rising<=0;side_min_travel<=travel_q8;
                side_previous_valid<=0;first_inner<=0;fall_reference<=0;lf_age<=63;rf_age<=63;side_age<=63;heading_settled<=0;
            end else if(tick && active)begin
                pending<=0;elapsed<=elapsed+1'b1;previous_heading<=relative_heading;
                if(lf_age<63)lf_age<=lf_age+1'b1;
                if(rf_age<63)rf_age<=rf_age+1'b1;
                if(inner_event)side_age<=0;else if(side_age<63)side_age<=side_age+1'b1;
                if((phase==1 || phase==7) && heading_stable)begin
                    if(heading_settled<15)heading_settled<=heading_settled+1'b1;
                end else heading_settled<=0;
                if(blocked || phase==5)steering<=0;
                else if(phase!=3)steering<=command>`VEHICLE_MAX_STEER_DEG?`VEHICLE_MAX_STEER_DEG:
                    command < -`VEHICLE_MAX_STEER_DEG?-`VEHICLE_MAX_STEER_DEG:command;
                if(event_bits[1])begin
                    lf_age<=0;
                    if(phase==1 && diagnostics[15:8]==0 && lf<nearest_lf)nearest_lf<=lf;
                    if(lf_clear)begin if(clear_lf<7)clear_lf<=clear_lf+1'b1;end else clear_lf<=0;
                end
                if(event_bits[2])begin
                    rf_age<=0;
                    if(phase==1 && diagnostics[23:16]==0 && rf<nearest_rf)nearest_rf<=rf;
                    if(rf_clear)begin if(clear_rf<7)clear_rf<=clear_rf+1'b1;end else clear_rf<=0;
                end
                if((phase==7 || phase==6 || phase==2) && inner_event && inner_valid && inner<800)begin
                    seen_side<=1;if(inner<nearest_side)nearest_side<=inner;
                end
                // Track fresh observations after the departure heading has
                // settled. Rotation, stale readings and bus faults break the
                // trend; they cannot manufacture an obstacle trailing edge.
                if(phase==7 && (heading_settled<3 || side_age>=30))begin
                    side_previous_valid<=0;falling_samples<=0;rising_samples<=0;side_edge_rising<=0;nearest_side<=2000;
                end else if(phase==7 && inner_event)begin
                    side_age<=0;
                    if(inner_valid && inner<800)begin
                        previous_inner<=inner;
                        side_previous_valid<=1;
                        if(!side_previous_valid)begin first_inner<=inner;fall_reference<=inner;nearest_side<=inner;side_min_travel<=travel_q8;end
                        // Accumulate millimetre changes across observations;
                        // at high sample rates no individual step need be 4mm.
                        else if(inner+3<fall_reference)begin
                            fall_reference<=inner;
                            if(falling_samples<7)falling_samples<=falling_samples+1'b1;
                            rising_samples<=0;side_edge_rising<=0;
                        end
                        if(inner<nearest_side)begin side_min_travel<=travel_q8;rising_samples<=0;end
                        else if(rise_sample)begin
                            if(rising_samples<7)rising_samples<=rising_samples+1'b1;
                            if(rising_samples>=1)side_edge_rising<=1;
                        end else begin rising_samples<=0;side_edge_rising<=0;end
                    end else if(rise_sample)begin
                        if(rising_samples<7)rising_samples<=rising_samples+1'b1;
                        if(rising_samples>=1)side_edge_rising<=1;
                    end else begin
                        rising_samples<=0;side_edge_rising<=0;
                        if(!inner_noecho)begin side_previous_valid<=0;falling_samples<=0;end
                    end
                end
                if(phase==1 && edge_math_ready && inner_front_event && inner_front_code==0 &&
                   !(left_turn?rf_clear:lf_clear) &&
                   inner_front>0 && inner_front<=4000 && measured_edge>$signed({1'b0,edge_bound}) && measured_edge<17'sd800)
                    edge_bound<=measured_edge;
                if(!single_close_ready || side_age>=30)single_close_samples<=0;
                else if(inner_event)begin
                    if(inner_valid && inner<=SINGLE_FRONT_SIDE_MM)begin
                        if(single_close_samples<2)single_close_samples<=single_close_samples+1'b1;
                    end else if(!inner_valid || inner>SINGLE_FRONT_SIDE_MM+3)single_close_samples<=0;
                end
                if(single_front_return || normal_side_return)begin
                    phase<=6;
                end else if(phase==1)begin
                    if(heading_settled>=3)begin
                        edge_confirmed<=1;edge_travel<=travel_q8;phase<=7;
                        nearest_side<=2000;side_previous_valid<=0;falling_samples<=0;rising_samples<=0;side_edge_rising<=0;side_age<=0;
                    end
                    if(local_x>1200 && !(clear_lf>=2 && clear_rf>=2))begin blocked<=1;phase<=5;end
                end else if(phase==7)begin
                    // A confirmed side-range minimum followed by a rise means
                    // the sensor has reached the obstacle's trailing edge.
                    // Start the countersteer as soon as the measured lateral
                    // body-sweep clearance is met; do not add another encoder
                    // run after the edge event. Missing side evidence is
                    // handled by the manoeuvre watchdog, never by guessing
                    // an obstacle length from encoder travel.
                    if(single_front_return || normal_side_return)begin
                        phase<=6;
                    end
                end else if(phase==6)begin
                    if(relative_heading<15 && relative_heading > -15)begin phase<=2;pass_ticks<=0;pass_y<=local_y;end
                end else if(phase==2)begin
                    if(pass_ticks!=65535)pass_ticks<=pass_ticks+1'b1;
                    if(inner_event && seen_side)begin
                        if((inner_valid && inner>nearest_side+250 && inner>400) || inner_noecho)begin
                            if(clear_side==0)clear_x<=local_x;
                            if(clear_side<7)clear_side<=clear_side+1'b1;
                            if(clear_side>=2 && !tail_confirmed)begin tail_confirmed<=1;tail_travel<=travel_q8;tail_x<=clear_x;end
                        end else if(inner_valid)begin clear_side<=0;tail_confirmed<=0;end
                    end
                    // The side sensor is 210 mm ahead of the tail. Starting
                    // the smooth merge at 100 mm deliberately uses the lateral
                    // sweep clearance while the last 110 mm still passes.
                    if(tail_confirmed && local_x-tail_x>=TAIL_MM)phase<=3;
                    // Lost side evidence cannot cause unbounded straight driving.
                    if(!tail_confirmed && pass_ticks>=2000)begin blocked<=1;phase<=5;end
                end else if(phase==3)begin
                    if(local_y<12 && local_y > -12 && relative_heading<15 && relative_heading > -15)begin
                        active<=0;done<=1;phase<=4;steering<=0;
                    end
                end
                if(travel_q8-start_travel>MAX_TRAVEL_MM*256 || elapsed>=6000 ||
                   relative_heading>650 || relative_heading < -650)begin blocked<=1;phase<=5;end
            end
        end
    end
endmodule
