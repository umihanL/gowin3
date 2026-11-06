// Read the already validated path RAM once. Nine ordered user segments keep
// nominal rear-wheel counts, heading endpoints and peak road-wheel steering.
// These are feed-forward estimates; motion still closes the loop on pose.
`include "src/vehicle/vehicle_config.vh"
module vehicle_route_profile(
    input clk,rst_n,route_valid,input [11:0] route_count,
    output reg [10:0] route_index,input [16:0] route_x_q8,route_y_q8,input route_lock,
    input [3:0] selected_segment,
    input [10:0] execution_index,output reg signed [15:0] execution_tangent,
    output reg signed [1:0] execution_curve,
    output reg execution_exit_valid,output reg signed [15:0] execution_exit_heading,
    output reg signed [1:0] execution_upcoming_turn,output reg [7:0] execution_entry_mm,
    output execution_entry_valid,output signed [15:0] execution_entry_x_mm,execution_entry_y_mm,
    output [10:0] execution_entry_index,
    output reg busy,ready,output reg [3:0] segment_count,
    output [15:0] length_mm,left_revs_tenths,right_revs_tenths,
    output signed [15:0] entry_heading,exit_heading,
    output [7:0] peak_steer,
    output signed [31:0] left_counts,right_counts
);
    localparam [31:0] COUNTS_PER_MM_Q16=(64'd65536*`VEHICLE_COUNTS_PER_REV*100000+
        `VEHICLE_WHEEL_DIAMETER_MM*314159/2)/(`VEHICLE_WHEEL_DIAMETER_MM*314159);
    localparam [31:0] HALF_TRACK_RAD_Q16=(64'd65536*`VEHICLE_REAR_TRACK_MM*314159+
        180000000)/360000000;
    reg [4:0] state;
    reg seen,have_bearing,locked_point;
    reg [2:0] wait_count;
    reg [3:0] segment;
    reg [16:0] previous_x,previous_y;
    reg signed [23:0] dx,dy;
    reg signed [17:0] point_dx,point_dy;
    reg [47:0] xx,yy;
    reg [31:0] edge_length,total_length;
    reg signed [15:0] previous_bearing,current_bearing,first_heading,change;
    reg signed [31:0] turn;
    reg [7:0] maximum_steer;
    reg atan_start,sqrt_start;
    reg signed [31:0] ax,ay;
    wire atan_done,sqrt_done;
    wire signed [31:0] angle;
    wire [31:0] root;
    reg signed [47:0] angle_scaled;
    reg signed [63:0] correction,left_product,right_product;
    reg signed [31:0] lc,rc;
    reg [31:0] divide_bits,quotient;
    reg [15:0] remainder;
    reg [5:0] divide_step;
    wire [16:0] trial={remainder,divide_bits[31]};
    wire take_bit=trial>=`VEHICLE_COUNTS_PER_REV;
    wire [31:0] next_quotient={quotient[30:0],take_bit};
    reg [31:0] length_store[0:8];
    reg signed [31:0] left_store[0:8],right_store[0:8];
    reg [15:0] left_turn_store[0:8],right_turn_store[0:8];
    reg signed [15:0] entry_store[0:8],exit_store[0:8];
    reg [7:0] steer_store[0:8];
    reg [25:0] tangent_store[0:2047];
    reg [37:0] corner_store[0:2047];
    reg [10:0] execution_entry_point,scan_entry_point;
    assign execution_entry_valid=ready && execution_upcoming_turn!=0;
    assign execution_entry_x_mm=16'sd0;
    assign execution_entry_y_mm=16'sd0;
    assign execution_entry_index=execution_entry_point;
    reg [25:0] scanned_tangent;
    reg [10:0] scan_index;
    reg [1:0] scan_direction,straight_gap;
    reg signed [15:0] scan_exit,next_heading;
    reg [8:0] scan_remaining;
    reg [1:0] upcoming_direction;
    wire [8:0] remaining_sum=scan_remaining+{1'b0,scanned_tangent[25:18]};
    wire scan_mode=(state>=16 && state<=18) || state==20 || state==21;
    wire [10:0] tangent_read_index=scan_mode?scan_index:execution_index==0?11'd1:execution_index;
    wire [1:0] scanned_curve=scanned_tangent[17:16];
    wire new_curve=scanned_curve!=0 && scanned_curve!=scan_direction;
    wire signed [15:0] scanned_exit=new_curve?next_heading:scan_exit;
    wire [1:0] edge_curve=change>=4?2'b01:change<=-4?2'b11:2'b00;
    // Reuse the existing per-edge atan, not a second pose controller. This
    // synchronous ROM tells motion which way the planned road actually bends.
    always @(posedge clk)begin
        if(state==8)tangent_store[route_index]<={edge_length>=65280?8'd255:edge_length[15:8],edge_curve,current_bearing};
        scanned_tangent<=tangent_store[tangent_read_index];
        {execution_curve,execution_tangent}<=tangent_store[tangent_read_index][17:0];
        {execution_entry_point,execution_upcoming_turn,
         execution_entry_mm,execution_exit_valid,execution_exit_heading}<=corner_store[tangent_read_index];
        if(state==18)corner_store[scan_index]<=
            scanned_curve!=0?{scan_index-1'b1,scanned_curve,8'd0,1'b1,scanned_exit}:
            upcoming_direction!=0 && scan_remaining<240?
                {scan_entry_point,upcoming_direction,scan_remaining[7:0],1'b1,scan_exit}:38'd0;
    end
    wire [3:0] select_index=selected_segment<segment_count?selected_segment:4'd0;
    assign length_mm=ready?length_store[select_index][23:8]:16'd0;
    assign left_counts=ready?left_store[select_index]:32'sd0;
    assign right_counts=ready?right_store[select_index]:32'sd0;
    assign left_revs_tenths=ready?left_turn_store[select_index]:16'd0;
    assign right_revs_tenths=ready?right_turn_store[select_index]:16'd0;
    assign entry_heading=ready?entry_store[select_index]:16'sd0;
    assign exit_heading=ready?exit_store[select_index]:16'sd0;
    assign peak_steer=ready?steer_store[select_index]:8'd0;
    wire signed [16:0] raw_change=$signed(current_bearing)-$signed(previous_bearing);
    imu_sqrt sqrt(.clk(clk),.rst_n(rst_n),.start(sqrt_start),.value({15'd0,1'b0,xx}+{16'd0,yy}),.done(sqrt_done),.root(root));
    imu_atan2 atan(.clk(clk),.rst_n(rst_n),.start(atan_start),.vx(ax),.vy(ay),.done(atan_done),.angle(angle));
    integer i;
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin
            state<=0;seen<=0;have_bearing<=0;locked_point<=0;wait_count<=0;segment<=0;
            route_index<=0;busy<=0;ready<=0;segment_count<=0;previous_x<=0;previous_y<=0;
            dx<=0;dy<=0;point_dx<=0;point_dy<=0;xx<=0;yy<=0;edge_length<=0;total_length<=0;turn<=0;
            previous_bearing<=0;current_bearing<=0;first_heading<=0;change<=0;
            maximum_steer<=0;atan_start<=0;sqrt_start<=0;ax<=0;ay<=0;angle_scaled<=0;
            correction<=0;left_product<=0;right_product<=0;lc<=0;rc<=0;
            divide_bits<=0;quotient<=0;remainder<=0;divide_step<=0;
            scan_index<=0;scan_direction<=0;straight_gap<=0;scan_exit<=0;next_heading<=0;
            scan_remaining<=255;upcoming_direction<=0;
            scan_entry_point<=0;
            for(i=0;i<9;i=i+1)begin
                length_store[i]<=0;left_store[i]<=0;right_store[i]<=0;
                left_turn_store[i]<=0;right_turn_store[i]<=0;
                entry_store[i]<=0;exit_store[i]<=0;steer_store[i]<=0;
            end
        end else begin
            atan_start<=0;sqrt_start<=0;
            if(!route_valid)begin state<=0;seen<=0;ready<=0;busy<=0;end
            else case(state)
                0:if(!seen && route_count>=2)begin
                    seen<=1;busy<=1;ready<=0;segment<=0;segment_count<=0;
                    route_index<=0;wait_count<=0;state<=1;
                    total_length<=0;turn<=0;maximum_steer<=0;have_bearing<=0;
                end
                1:if(wait_count==3)begin
                    previous_x<=route_x_q8;previous_y<=route_y_q8;
                    route_index<=1;wait_count<=0;state<=2;
                end else wait_count<=wait_count+1'b1;
                2:if(wait_count==3)begin
                    point_dx<=$signed({1'b0,route_x_q8})-$signed({1'b0,previous_x});
                    point_dy<=$signed({1'b0,previous_y})-$signed({1'b0,route_y_q8});
                    previous_x<=route_x_q8;previous_y<=route_y_q8;locked_point<=route_lock;
                    state<=3;
                end else wait_count<=wait_count+1'b1;
                3:begin dx<=point_dx*24'sd10;dy<=point_dy*24'sd10;state<=19;end
                19:begin xx<=dx*dx;yy<=dy*dy;ax<=dx<<<2;ay<=dy<<<2;state<=4;end
                4:begin sqrt_start<=1;atan_start<=1;state<=5;end
                5:if(sqrt_done)begin edge_length<=root;angle_scaled<=angle*48'sd10;state<=6;end
                // Graph primitive joins can differ by only one Q8 unit.
                // Such sub-millimetre edges do not define a new direction.
                6:begin current_bearing<=edge_length<256?previous_bearing:angle_scaled>>>16;state<=7;end
                7:begin
                    change<=!have_bearing?16'sd0:raw_change>1800?raw_change-3600:
                        raw_change < -1800?raw_change+3600:raw_change;
                    if(total_length==0)first_heading<=current_bearing;
                    previous_bearing<=current_bearing;have_bearing<=1;state<=8;
                end
                8:begin
                    total_length<=total_length+edge_length;turn<=turn+change;
                    // delta=atan(L*d_heading/distance), with tenths-deg to rad.
                    ax<=edge_length*32'sd573;
                    ay<=change*(`VEHICLE_WHEELBASE_MM*256);
                    atan_start<=1;state<=9;
                end
                9:if(atan_done)begin
                    if((angle<0?-angle:angle)>>>16>maximum_steer)
                        maximum_steer<=(angle<0?-angle:angle)>>>16;
                    state<=10;
                end
                10:if(locked_point || route_index+1>=route_count)begin
                    correction<=turn*$signed({1'b0,HALF_TRACK_RAD_Q16});state<=11;
                end else begin route_index<=route_index+1'b1;wait_count<=0;state<=2;end
                11:begin
                    left_product<=($signed({1'b0,total_length})-(correction>>>8))*$signed({1'b0,COUNTS_PER_MM_Q16});
                    right_product<=($signed({1'b0,total_length})+(correction>>>8))*$signed({1'b0,COUNTS_PER_MM_Q16});
                    state<=12;
                end
                12:begin lc<=left_product>>>24;rc<=right_product>>>24;state<=13;end
                13:begin
                    length_store[segment]<=total_length;
                    left_store[segment]<=lc;right_store[segment]<=rc;
                    entry_store[segment]<=first_heading;exit_store[segment]<=current_bearing;
                    steer_store[segment]<=maximum_steer;
                    divide_bits<=lc>0?lc*32'd10:32'd0;quotient<=0;remainder<=0;divide_step<=0;state<=14;
                end
                // Counts to tenths of a revolution: one shared restoring
                // divider. A 32-bit combinational divide does not meet 33 MHz.
                14,15:begin
                    divide_bits<=divide_bits<<1;quotient<=next_quotient;
                    remainder<=take_bit?trial-`VEHICLE_COUNTS_PER_REV:trial;
                    if(divide_step!=31)divide_step<=divide_step+1'b1;
                    else if(state==14)begin
                        left_turn_store[segment]<=next_quotient>65535?16'hffff:next_quotient[15:0];
                        divide_bits<=rc>0?rc*32'd10:32'd0;quotient<=0;remainder<=0;divide_step<=0;state<=15;
                    end else begin
                    right_turn_store[segment]<=next_quotient>65535?16'hffff:next_quotient[15:0];
                    segment_count<=segment+1'b1;
                    if(route_index+1>=route_count || segment==8)begin
                        // Backward annotate each same-direction curve with
                        // its outgoing tangent. S lobes retain their actual
                        // angle rather than being forced to a quarter-turn.
                        scan_index<=route_index;scan_direction<=0;straight_gap<=0;
                        scan_remaining<=255;upcoming_direction<=0;
                        scan_exit<=current_bearing;next_heading<=current_bearing;state<=16;
                    end
                    else begin
                        segment<=segment+1'b1;total_length<=0;turn<=0;maximum_steer<=0;
                        route_index<=route_index+1'b1;wait_count<=0;state<=2;
                    end
                    end
                end
                // Read the point before this edge through the existing route
                // RAM port. Four clocks cover its registered read latency.
                // This stores a physical entry location, independent of the
                // rear-wheel distance travelled through the previous S-lobe.
                16:begin route_index<=scan_index-1'b1;state<=20;end
                20:state<=21;
                21:state<=17;
                17:state<=18;
                18:begin
                    next_heading<=scanned_tangent[15:0];
                    if(scanned_curve!=0)begin
                        scan_direction<=scanned_curve;straight_gap<=0;scan_exit<=scanned_exit;
                        scan_remaining<=0;upcoming_direction<=scanned_curve;
                        scan_entry_point<=scan_index-1'b1;
                    end else if(straight_gap<3)straight_gap<=straight_gap+1'b1;
                    else scan_direction<=0;
                    if(scanned_curve==0)scan_remaining<=remaining_sum>255?9'd255:remaining_sum;
                    if(scan_index<=1)begin busy<=0;ready<=1;state<=0;end
                    else begin scan_index<=scan_index-1'b1;state<=16;end
                end
                default:state<=0;
            endcase
        end
    end
endmodule
