// Ackermann candidate-path refinement. Coordinates use cm * 256; origin is
// the NOSE midpoint, raster Y down. No motor control is implemented here.
// Locked user points survive smoothing. Curvature and body checks are advisory;
// an explicit forbidden barrier still rejects publication.
`include "src/vehicle/vehicle_config.vh"
module nav_path_refiner #(
    parameter MAX_POINTS=2048, SMOOTH_PASSES=512,
    parameter MIN_RADIUS_MM=`VEHICLE_MIN_RADIUS_MM, HALF_WIDTH_MM=95,
    parameter FRONT_MM=0, REAR_MM=`VEHICLE_BODY_LENGTH_MM, CELL_MARGIN_MM=8
)(
    input clk,rst_n,clear,cancel,
    input load_valid,load_lock, input [7:0] load_x,load_y,
    input start, output reg busy,done, output reg [2:0] error,
    output [14:0] plan_addr, input plan_blocked,
    output [16:0] raw_addr, output raw_request,
    output [15:0] plan_xy,output [17:0] raw_xy,
    input raw_grant,raw_blocked,raw_forbidden,
    output reg route_write,route_bit, output reg [14:0] route_addr,
    output reg sample_valid, output reg [16:0] sample_x_q8,sample_y_q8,
    output reg [3:0] sample_segment,
    output reg [17:0] length_cm,
    // Reuse the validated path RAM for execution. Two clocks read latency.
    input [10:0] execution_index,
    output [11:0] execution_count,
    output [16:0] execution_x_q8,execution_y_q8,
    output execution_lock
);
    localparam IDLE=0,RP=1,WP=2,RC=3,WC=4,RN=5,WN=6,
        CALC=7,PM_WAIT=8,PM_TEST=9,NEXT=10,SQ_AB=11,SQ_BC=12,
        SQ_AC=13,CURVE=14,BODY=15,RAW_ISSUE=16,RAW_WAIT=17,
        RAW_TEST=18,BODY_NEXT=19,ERASE=20,EMIT=21,LINE=22;
    localparam R_Q8=(MIN_RADIUS_MM*256+9)/10;
    localparam F_Q8=((FRONT_MM+CELL_MARGIN_MM)*256+9)/10;
    localparam B_Q8=((REAR_MM+CELL_MARGIN_MM)*256+9)/10;
    localparam W_Q8=((HALF_WIDTH_MM+CELL_MARGIN_MM)*256+9)/10;
    // The entire body extends behind the selected nose point.
    localparam EXTENT=(FRONT_MM>REAR_MM?FRONT_MM:REAR_MM)+HALF_WIDTH_MM/2+CELL_MARGIN_MM;
    localparam BOX=(EXTENT+9)/10;
    reg [4:0] state;
    reg [1:0] phase; // 0 smoothing, 1 validation, 2 ordered output
    reg [35:0] points[0:MAX_POINTS-1];
    reg [35:0] q;
    reg [10:0] index,read_index;
    reg [11:0] count;
    assign execution_count=count;
    assign execution_x_q8=q[16:0];
    assign execution_y_q8=q[33:17];
    assign execution_lock=q[34];
    reg overflow;
    reg [33:0] last_loaded;
    wire [16:0] load_cx={load_x,9'b100000000};
    wire [16:0] load_cy={load_y,9'b100000000};
    wire duplicate=count!=0 && last_loaded=={load_cy,load_cx};
    reg [16:0] ax,ay,bx,by,cx,cy,new_x,new_y;
    reg locked;
    reg [8:0] pass;
    wire [18:0] sum_x={2'b0,ax}+{1'b0,bx,1'b0}+{2'b0,cx};
    wire [18:0] sum_y={2'b0,ay}+{1'b0,by,1'b0}+{2'b0,cy};
    wire candidate_inside=new_x<352*256 && new_y<320*256;
    assign plan_addr=candidate_inside?(new_y[16:9]*15'd176+new_x[16:9]):15'd0;
    assign plan_xy=candidate_inside?{new_y[16:9],new_x[16:9]}:16'd0;
    always @* begin
        read_index=index;
        if(!busy)read_index=execution_index;
        if(state==RP || state==WP)read_index=index==0?12'd0:index-1'b1;
        else if(state==RN || state==WN)read_index=index+1<count?index+1'b1:index;
    end
    always @(posedge clk)begin
        q<=points[read_index];
        if(!busy && load_valid && !clear && !cancel)begin
            if(duplicate && load_lock)points[count-1'b1]<={1'b0,1'b1,last_loaded};
            else if(!duplicate && count<MAX_POINTS)
                points[count[10:0]]<={1'b0,(load_lock || count==0),load_cy,load_cx};
        end else if(state==PM_TEST && !plan_blocked && candidate_inside && !locked && !cancel)
            points[index]<={1'b0,locked,new_y,new_x};
    end
    wire signed [17:0] abx=$signed({1'b0,bx})-$signed({1'b0,ax});
    wire signed [17:0] aby=$signed({1'b0,by})-$signed({1'b0,ay});
    wire signed [17:0] bcx=$signed({1'b0,cx})-$signed({1'b0,bx});
    wire signed [17:0] bcy=$signed({1'b0,cy})-$signed({1'b0,by});
    wire signed [17:0] acx=$signed({1'b0,cx})-$signed({1'b0,ax});
    wire signed [17:0] acy=$signed({1'b0,cy})-$signed({1'b0,ay});
    // Only one square-root request runs at a time. Share its two square
    // multipliers across AB, BC and AC instead of placing six DSP products.
    wire signed [17:0] square_x=state==SQ_AB?bcx:state==SQ_BC?acx:abx;
    wire signed [17:0] square_y=state==SQ_AB?bcy:state==SQ_BC?acy:aby;
    wire [35:0] square_x2=square_x*square_x;
    wire [35:0] square_y2=square_y*square_y;
    wire [63:0] square_sum={28'd0,square_x2}+{28'd0,square_y2};
    // Coordinates remain fixed throughout the three sequential square roots.
    // Pipeline the products and absolute value before the radius comparison.
    reg signed [36:0] cross_product,tangent_dot;
    reg [36:0] cross_abs,cross_xor;
    reg cross_negative;
    always @(posedge clk)begin
        cross_product<=abx*bcy-aby*bcx;
        tangent_dot<=abx*bcx+aby*bcy;
        // Split conditional inversion and carry propagation. Coordinates stay
        // fixed for three square roots, so the extra stage is settled before
        // SQ_AC consumes cross_abs; path geometry and results are unchanged.
        cross_xor<=cross_product^{37{cross_product[36]}};
        cross_negative<=cross_product[36];
        cross_abs<=cross_xor+cross_negative;
    end
    reg sqrt_start;
    reg [63:0] sqrt_value;
    wire sqrt_done;
    wire [31:0] sqrt_result;
    imu_sqrt root(.clk(clk),.rst_n(rst_n),.start(sqrt_start),.value(sqrt_value),
        .done(sqrt_done),.root(sqrt_result));
    reg [17:0] len_ab,len_bc,len_ac;
    reg [47:0] radius_lhs;
    reg [35:0] len_product;
    wire [53:0] radius_rhs=len_product*len_ac;
    reg [31:0] distance_q8;
    reg signed [17:0] heading_x,heading_y;
    reg [17:0] heading_length;
    reg signed [6:0] ox,oy;
    wire signed [10:0] cell_x=$signed({3'b0,bx[16:8]})+ox;
    wire signed [10:0] cell_y=$signed({3'b0,by[16:8]})+oy;
    wire signed [18:0] rel_x=ox*19'sd256+19'sd128-$signed({11'b0,bx[7:0]});
    wire signed [18:0] rel_y=oy*19'sd256+19'sd128-$signed({11'b0,by[7:0]});
    wire signed [37:0] forward=rel_x*heading_x+rel_y*heading_y;
    wire signed [37:0] lateral=rel_y*heading_x-rel_x*heading_y;
    wire signed [37:0] front_limit=heading_length*F_Q8;
    wire signed [37:0] rear_limit=heading_length*B_Q8;
    wire signed [37:0] side_limit=heading_length*W_Q8;
    wire in_body=forward<=front_limit && forward>=-rear_limit &&
                      lateral<=side_limit && lateral>=-side_limit;
    wire in_bounds=cell_x>=0 && cell_x<352 && cell_y>=0 && cell_y<320;
    assign raw_request=state==RAW_ISSUE;
    assign raw_addr=in_bounds?cell_y*17'd352+cell_x:17'd0;
    assign raw_xy=in_bounds?{cell_y[8:0],cell_x[8:0]}:18'd0;
    reg [14:0] erase_index;
    reg signed [9:0] lx,ly,end_x,end_y,ldx,ldy,step_x,step_y;
    reg signed [10:0] line_error;
    wire signed [11:0] double_error=line_error*2;
    reg [7:0] prev_x,prev_y;
    reg raw_hit,forbidden_hit;
    always @* begin
        route_write=0;route_addr=0;route_bit=0;
        if(state==ERASE)begin route_write=1;route_addr=erase_index;end
        else if(state==LINE)begin
            route_write=1;route_bit=1;route_addr=ly*15'd176+lx;
        end
    end
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin
            state<=IDLE;phase<=0;index<=0;count<=0;overflow<=0;last_loaded<=0;
            busy<=0;done<=0;error<=0;sample_valid<=0;sample_x_q8<=0;sample_y_q8<=0;
            sample_segment<=0;length_cm<=0;pass<=0;ax<=0;ay<=0;bx<=0;by<=0;
            cx<=0;cy<=0;new_x<=0;new_y<=0;locked<=0;sqrt_start<=0;sqrt_value<=0;
            len_ab<=0;len_bc<=0;len_ac<=0;radius_lhs<=0;len_product<=0;
            heading_x<=0;heading_y<=0;heading_length<=0;ox<=0;oy<=0;distance_q8<=0;
            erase_index<=0;lx<=0;ly<=0;end_x<=0;end_y<=0;ldx<=0;ldy<=0;
            step_x<=0;step_y<=0;line_error<=0;prev_x<=0;prev_y<=0;raw_hit<=0;forbidden_hit<=0;
        end else begin
            done<=0;sample_valid<=0;sqrt_start<=0;
            if(clear)begin count<=0;overflow<=0;state<=IDLE;busy<=0;error<=0;end
            else if(cancel)begin state<=IDLE;busy<=0;end
            else begin
                if(!busy && load_valid && !duplicate)begin
                    if(count<MAX_POINTS)begin count<=count+1'b1;last_loaded<={load_cy,load_cx};end
                    else overflow<=1;
                end
                case(state)
                    IDLE:if(start)begin
                        error<=0;length_cm<=0;distance_q8<=0;sample_segment<=0;
                        if(overflow || count<2)begin error<=1;done<=1;end
                        else begin busy<=1;index<=1;pass<=0;phase<=0;state<=RP;end
                    end
                    RP:state<=WP;
                    WP:begin ax<=q[16:0];ay<=q[33:17];state<=RC;end
                    RC:state<=WC;
                    WC:begin
                        bx<=q[16:0];by<=q[33:17];locked<=q[34];
                        state<=phase==2?EMIT:RN;
                    end
                    RN:state<=WN;
                    WN:begin cx<=q[16:0];cy<=q[33:17];state<=CALC;end
                    CALC:if(phase==0)begin
                        new_x<=sum_x[18:2];new_y<=sum_y[18:2];state<=PM_WAIT;
                    end else begin
                        sqrt_value<=square_sum;sqrt_start<=1;state<=SQ_AB;
                    end
                    PM_WAIT:state<=PM_TEST;
                    PM_TEST:state<=NEXT;
                    NEXT:if(phase==0)begin
                        if(index+2>=count)begin
                            index<=1;
                            if(pass+1>=SMOOTH_PASSES)begin phase<=1;index<=0;end
                            else pass<=pass+1'b1;
                        end else index<=index+1'b1;
                        state<=RP;
                    end else if(index+1>=count)begin
                        length_cm<=(distance_q8+128)>>8;
                        erase_index<=0;state<=ERASE;
                    end else begin index<=index+1'b1;state<=RP;end
                    SQ_AB:if(sqrt_done)begin
                        len_ab<=sqrt_result[17:0];sqrt_value<=square_sum;
                        sqrt_start<=1;state<=SQ_BC;
                    end
                    SQ_BC:if(sqrt_done)begin
                        len_bc<=sqrt_result[17:0];sqrt_value<=square_sum;
                        sqrt_start<=1;state<=SQ_AC;
                    end
                    SQ_AC:if(sqrt_done)begin
                        len_ac<=sqrt_result[17:0];len_product<=len_ab*len_bc;
                        radius_lhs<=cross_abs*(2*R_Q8);state<=CURVE;
                    end
                    CURVE:begin
                        if((index!=0 && index+1<count &&
                            (len_ab==0 || len_bc==0 || tangent_dot<=0 || radius_lhs>radius_rhs)) ||
                            (len_ab==0 && len_bc==0))begin
                            if(error==0)error<=4;
                        end
                        begin
                            heading_x<=index+1<count?bcx:abx;
                            heading_y<=index+1<count?bcy:aby;
                            heading_length<=index+1<count?len_bc:len_ab;
                            if(index+1<count)distance_q8<=distance_q8+len_bc;
                            ox<=-BOX;oy<=-BOX;state<=BODY;
                        end
                    end
                    BODY:if(!in_body)state<=BODY_NEXT;
                        else if(!in_bounds)state<=BODY_NEXT;
                        else state<=RAW_ISSUE;
                    RAW_ISSUE:if(raw_grant)state<=RAW_WAIT;
                    RAW_WAIT:begin raw_hit<=raw_blocked;forbidden_hit<=raw_forbidden;state<=RAW_TEST;end
                    RAW_TEST:if(forbidden_hit)begin error<=6;done<=1;busy<=0;state<=IDLE;end
                        // Only explicit user barriers are hard forbidden here.
                        else state<=BODY_NEXT;
                    BODY_NEXT:if(ox==BOX && oy==BOX)state<=NEXT;
                        else begin
                            if(ox==BOX)begin ox<=-BOX;oy<=oy+1'b1;end else ox<=ox+1'b1;
                            state<=BODY;
                        end
                    ERASE:if(erase_index==28159)begin phase<=2;index<=0;state<=RC;end
                        else erase_index<=erase_index+1'b1;
                    EMIT:begin
                        sample_valid<=1;sample_x_q8<=bx;sample_y_q8<=by;
                        if(locked && index!=0 && index+1<count)sample_segment<=sample_segment+1'b1;
                        end_x<={2'b0,bx[16:9]};end_y<={2'b0,by[16:9]};
                        lx<=index==0?{2'b0,bx[16:9]}:{2'b0,prev_x};
                        ly<=index==0?{2'b0,by[16:9]}:{2'b0,prev_y};
                        ldx<=index==0?10'd0:(bx[16:9]>prev_x?bx[16:9]-prev_x:prev_x-bx[16:9]);
                        ldy<=index==0?10'd0:-(by[16:9]>prev_y?by[16:9]-prev_y:prev_y-by[16:9]);
                        step_x<=bx[16:9]>prev_x?10'sd1:-10'sd1;
                        step_y<=by[16:9]>prev_y?10'sd1:-10'sd1;
                        line_error<=index==0?11'd0:
                            (bx[16:9]>prev_x?bx[16:9]-prev_x:prev_x-bx[16:9])-
                            (by[16:9]>prev_y?by[16:9]-prev_y:prev_y-by[16:9]);
                        prev_x<=bx[16:9];prev_y<=by[16:9];state<=LINE;
                    end
                    LINE:if(lx==end_x && ly==end_y)begin
                        if(index+1>=count)begin done<=1;busy<=0;state<=IDLE;end
                        else begin index<=index+1'b1;state<=RC;end
                    end else begin
                        if(double_error>=ldy)lx<=lx+step_x;
                        if(double_error<=ldx)ly<=ly+step_y;
                        line_error<=line_error+(double_error>=ldy?ldy:10'd0)+
                            (double_error<=ldx?ldx:10'd0);
                    end
                    default:state<=IDLE;
                endcase
            end
        end
    end
endmodule
