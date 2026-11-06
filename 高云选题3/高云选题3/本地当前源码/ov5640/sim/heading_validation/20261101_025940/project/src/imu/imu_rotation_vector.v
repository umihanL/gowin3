// Principal quaternion logarithm, expressed as X/Y/Z rotation components.
// Unlike ZYX Euler angles this has no pitch +/-90 degree singularity.
// rv = (2*atan2(length(q.xyz),q.w)) * q.xyz/length(q.xyz), degrees*10.
// The vector magnitude is 0..180 degrees; the branch seam is at 180, not 90.
// No time integration, history-dependent unwrapping, or automatic zeroing.
module imu_rotation_vector(
    input clk, rst_n, online, sample_valid,
    input signed [17:0] q_w, q_x, q_y, q_z,
    output reg valid, available,
    output reg signed [15:0] rot_x, rot_y, rot_z
);
    localparam IDLE=0, SQUARE_X=1, SQUARE_Y=2, SQUARE_Z=3,
               ROOT_START=4, ROOT_WAIT=5, ANGLE_WAIT=6,
               DIV_LOAD=7, DIVIDE=8, STORE=9;
    reg [3:0] state;
    reg signed [17:0] w,x,y,z;
    wire signed [15:0] square_component = state==SQUARE_X ? x[15:0] :
                                         state==SQUARE_Y ? y[15:0] : z[15:0];
    wire signed [31:0] component_square = square_component*square_component;
    reg [33:0] square_sum;
    reg sqrt_start, atan_start;
    wire sqrt_done, atan_done;
    wire [31:0] root;
    wire signed [31:0] angle;
    reg [15:0] vector_length;
    reg [18:0] theta_q8;
    // angle is HALF the rotation in degrees*65536. This is theta*10*256,
    // rounded, using shifts/adds rather than a constant-multiply DSP.
    wire [34:0] theta_scaled = {3'd0,angle}+{1'b0,angle,2'd0}+35'd32;
    reg [1:0] axis;
    wire signed [17:0] axis_value = axis==0 ? x : axis==1 ? y : z;
    wire [15:0] axis_magnitude = axis_value[17] ? -axis_value : axis_value;
    wire [34:0] axis_product = theta_q8*axis_magnitude;
    reg [35:0] division_bits;
    reg [23:0] divisor;
    reg [24:0] remainder;
    wire [24:0] trial_remainder = {remainder[23:0],division_bits[35]};
    reg [15:0] quotient;
    reg [5:0] division_step;
    reg signed [15:0] pending_x,pending_y;
    wire signed [15:0] signed_result = axis_value[17] ? -$signed(quotient) : $signed(quotient);

    wire math_reset_n = rst_n & online;
    imu_sqrt u_sqrt(.clk(clk),.rst_n(math_reset_n),.start(sqrt_start),
        .value({30'd0,square_sum}),.done(sqrt_done),.root(root));
    imu_atan2 u_atan(.clk(clk),.rst_n(math_reset_n),.start(atan_start),
        .vx({w,14'd0}),.vy({2'd0,vector_length,14'd0}),.done(atan_done),.angle(angle));

    // Arithmetic units also reset on an offline/re-zero abort. A new sample
    // cannot consume an unfinished sqrt/atan result from the previous zero.
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            state<=IDLE;valid<=0;available<=0;rot_x<=0;rot_y<=0;rot_z<=0;
            w<=0;x<=0;y<=0;z<=0;square_sum<=0;sqrt_start<=0;atan_start<=0;
            vector_length<=0;theta_q8<=0;axis<=0;division_bits<=0;
            divisor<=0;remainder<=0;quotient<=0;division_step<=0;pending_x<=0;pending_y<=0;
        end else begin
            valid<=0;sqrt_start<=0;atan_start<=0;
            if(!online) begin
                state<=IDLE;available<=0;rot_x<=0;rot_y<=0;rot_z<=0;
            end else case(state)
                IDLE: if(sample_valid) begin
                    // q and -q denote the same orientation. Select w>=0 so
                    // the result is the shortest rotation from the saved zero.
                    w<=q_w[17] ? -q_w : q_w;
                    x<=q_w[17] ? -q_x : q_x;
                    y<=q_w[17] ? -q_y : q_y;
                    z<=q_w[17] ? -q_z : q_z;
                    state<=SQUARE_X;
                end
                SQUARE_X: begin square_sum<={2'd0,component_square};state<=SQUARE_Y;end
                SQUARE_Y: begin square_sum<=square_sum+{2'd0,component_square};state<=SQUARE_Z;end
                SQUARE_Z: begin square_sum<=square_sum+{2'd0,component_square};state<=ROOT_START;end
                ROOT_START: begin sqrt_start<=1;state<=ROOT_WAIT;end
                ROOT_WAIT: if(sqrt_done) begin
                    if(root==0) begin
                        rot_x<=0;rot_y<=0;rot_z<=0;valid<=1;available<=1;state<=IDLE;
                    end else begin vector_length<=root[15:0];atan_start<=1;state<=ANGLE_WAIT;end
                end
                ANGLE_WAIT: if(atan_done) begin theta_q8<=theta_scaled[24:6];axis<=0;state<=DIV_LOAD;end
                DIV_LOAD: begin
                    // Round the final signed tenth of a degree. Share one
                    // product and one restoring divider across all three axes.
                    division_bits<={1'b0,axis_product}+{13'd0,vector_length,7'd0};
                    divisor<={vector_length,8'd0};remainder<=0;quotient<=0;
                    division_step<=0;state<=DIVIDE;
                end
                DIVIDE: begin
                    division_bits<=division_bits<<1;
                    if(trial_remainder>={1'b0,divisor}) begin
                        remainder<=trial_remainder-{1'b0,divisor};quotient<={quotient[14:0],1'b1};
                    end else begin remainder<=trial_remainder;quotient<={quotient[14:0],1'b0};end
                    if(division_step==35) state<=STORE;
                    else division_step<=division_step+1'b1;
                end
                STORE: begin
                    if(axis==0) pending_x<=signed_result;
                    else if(axis==1) pending_y<=signed_result;
                    else begin
                        rot_x<=pending_x;rot_y<=pending_y;rot_z<=signed_result;
                        valid<=1;available<=1;
                    end
                    if(axis==2) state<=IDLE;
                    else begin axis<=axis+1'b1;state<=DIV_LOAD;end
                end
                default:state<=IDLE;
            endcase
        end
    end
endmodule
