// Eight-neighbour A*: octile heuristic, axial cost 5, diagonal cost 7.
// Diagonals require BOTH adjacent axial cells free (no obstacle corner cutting).
// One 18-bit record per cell: {state[1:0], parent_direction[2:0], g[12:0]}.
// A streaming minimum scan avoids a large heap and has a bounded RAM footprint.
module astar_grid #(
    parameter WIDTH=176, HEIGHT=160, NODES=WIDTH*HEIGHT, REVERSE_DRIVE=0
)(
    input clk,rst_n,start,cancel,
    input [7:0] start_x,start_y,goal_x,goal_y,
    output reg busy,done, output reg [2:0] error,
    output [14:0] map_addr, input map_blocked,
    output [15:0] map_xy,
    input [7:0] map_directions,
    input map_central,
    output reg path_valid, output reg [14:0] path_addr,
    output reg [7:0] path_x,path_y,
    output reg [13:0] path_cost,
    output reg [15:0] expanded
);
    localparam IDLE=0, CLEAR=1, CHECK_S=2, WAIT_S=3, TEST_S=4,
        WAIT_G=5, TEST_G=6, SEED=7, SCAN=8, DRAIN=9, PICK=10,
        CLOSE=11, NEIGHBOUR=12, WAIT_N=13, TEST_N=14,
        TRACE=15, WAIT_T=16, STEP_T=17, WAIT_C1=18, TEST_C1=19,
        WAIT_C2=20, TEST_C2=21;
    reg [4:0] state;
    reg [17:0] records[0:NODES-1];
    reg [17:0] q;
    reg [14:0] read_addr,write_addr;
    reg [17:0] write_data;
    reg write_en;
    always @(posedge clk) begin
        q<=records[read_addr];
        if(write_en) records[write_addr]<=write_data;
    end
    reg [14:0] index,sid,gid,current,best,neighbour,trace_id;
    reg [7:0] sx,sy,gx,gy,cx,cy,bx,by,nx,ny,scan_x,scan_y,dx,dy,tx,ty;
    reg [14:0] delayed_index;
    reg scan_valid,found,neighbour_valid;
    reg [14:0] best_f;
    reg [12:0] best_g,current_g;
    reg [14:0] trace_steps;
    reg [2:0] direction;
    // Free-grid distance only: no precomputed body-heading or clearance cost.
    wire [12:0] edge_cost=direction<4?13'd5:13'd7;
    wire [13:0] candidate_g={1'b0,current_g}+{1'b0,edge_cost};
    reg cost_limited;
    wire [8:0] hx=dx>gx ? dx-gx : gx-dx;
    wire [8:0] hy=dy>gy ? dy-gy : gy-dy;
    wire [14:0] heuristic=hx>hy?hx*15'd5+hy*15'd2:hy*15'd5+hx*15'd2;
    wire [14:0] score={2'b0,q[12:0]}+heuristic;
    assign map_addr=(state==WAIT_C1 || state==TEST_C1)?cy*WIDTH+nx:
                    (state==WAIT_C2 || state==TEST_C2)?ny*WIDTH+cx:read_addr;
    assign map_xy=(state==WAIT_C1 || state==TEST_C1)?{cy,nx}:
                  (state==WAIT_C2 || state==TEST_C2)?{ny,cx}:
                  (state==CHECK_S || state==WAIT_S || state==TEST_S)?{sy,sx}:
                  (state==WAIT_G || state==TEST_G)?{gy,gx}:
                  (state==WAIT_N || state==TEST_N)?{ny,nx}:16'd0;
    always @* begin
        read_addr=0;write_addr=0;write_data=0;write_en=0;
        case(state)
            CLEAR:begin write_en=1;write_addr=index;end
            CHECK_S,WAIT_S,TEST_S:read_addr=sid;
            WAIT_G,TEST_G:read_addr=gid;
            SEED:begin write_en=1;write_addr=sid;write_data={2'b01,3'b000,13'd0};end
            SCAN,DRAIN:read_addr=index;
            CLOSE:begin
                write_en=1;write_addr=current;
                write_data={2'b10,3'b000,current_g};
                // The parent is needed for traceback, including closed nodes.
                write_data[15:13]=best_parent;
            end
            WAIT_N,TEST_N:begin
                read_addr=neighbour;
                if(state==TEST_N && neighbour_valid && !map_blocked &&
                   candidate_g<14'd8180 && (q[17:16]==0 ||
                   (q[17:16]==1 && candidate_g<{1'b0,q[12:0]}))) begin
                    write_en=1;write_addr=neighbour;
                    write_data={2'b01,(direction^3'b001),candidate_g[12:0]};
                end
            end
            TRACE,WAIT_T,STEP_T:read_addr=trace_id;
            default:;
        endcase
        if(cancel) write_en=0;
    end
    reg [2:0] best_parent;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            state<=IDLE;busy<=0;done<=0;error<=0;path_valid<=0;path_addr<=0;
            path_x<=0;path_y<=0;path_cost<=0;expanded<=0;
            index<=0;sid<=0;gid<=0;current<=0;best<=0;neighbour<=0;trace_id<=0;
            sx<=0;sy<=0;gx<=0;gy<=0;cx<=0;cy<=0;bx<=0;by<=0;nx<=0;ny<=0;
            scan_x<=0;scan_y<=0;dx<=0;dy<=0;delayed_index<=0;
            scan_valid<=0;found<=0;neighbour_valid<=0;best_f<=0;best_g<=0;
            current_g<=0;direction<=0;best_parent<=0;tx<=0;ty<=0;trace_steps<=0;cost_limited<=0;
        end else begin
            done<=0;path_valid<=0;
            if(cancel) begin state<=IDLE;busy<=0;error<=0;scan_valid<=0;end
            else begin
                if((state==SCAN || state==DRAIN) && scan_valid && q[17:16]==1 &&
                   (!found || score<best_f || (score==best_f && q[12:0]>best_g))) begin
                    found<=1;best_f<=score;best_g<=q[12:0];best<=delayed_index;
                    bx<=dx;by<=dy;best_parent<=q[15:13];
                end
                case(state)
                    IDLE:if(start) begin
                        error<=0;expanded<=0;path_cost<=0;cost_limited<=0;
                        if(start_x>=WIDTH || goal_x>=WIDTH || start_y>=HEIGHT || goal_y>=HEIGHT) begin
                            error<=1;done<=1;
                        end else begin
                            sx<=start_x;sy<=start_y;gx<=goal_x;gy<=goal_y;
                            sid<=start_y*WIDTH+start_x;gid<=goal_y*WIDTH+goal_x;
                            index<=0;busy<=1;state<=CLEAR;
                        end
                    end
                    CLEAR:if(index==NODES-1) state<=CHECK_S;else index<=index+1'b1;
                    CHECK_S:state<=WAIT_S;
                    WAIT_S:state<=TEST_S;
                    TEST_S:if(map_blocked) begin error<=2;done<=1;busy<=0;state<=IDLE;end
                           else state<=WAIT_G;
                    WAIT_G:state<=TEST_G;
                    TEST_G:if(map_blocked) begin error<=2;done<=1;busy<=0;state<=IDLE;end
                           else state<=SEED;
                    SEED:begin
                        index<=0;scan_x<=0;scan_y<=0;scan_valid<=0;found<=0;state<=SCAN;
                    end
                    SCAN:begin
                        scan_valid<=1;delayed_index<=index;dx<=scan_x;dy<=scan_y;
                        if(index==NODES-1) state<=DRAIN;
                        else begin
                            index<=index+1'b1;
                            if(scan_x==WIDTH-1) begin scan_x<=0;scan_y<=scan_y+1'b1;end
                            else scan_x<=scan_x+1'b1;
                        end
                    end
                    DRAIN:begin scan_valid<=0;state<=PICK;end
                    PICK:if(!found) begin error<=cost_limited?4:3;done<=1;busy<=0;state<=IDLE;end
                        else begin
                            current<=best;current_g<=best_g;cx<=bx;cy<=by;
                            expanded<=expanded+1'b1;
                            if(best==gid) begin
                                trace_id<=gid;tx<=gx;ty<=gy;trace_steps<=0;
                                path_cost<=best_g;state<=TRACE;
                            end else if(best_g>=13'd8180) begin
                                error<=4;busy<=0;done<=1;state<=IDLE;
                            end else state<=CLOSE;
                        end
                    CLOSE:begin direction<=0;state<=NEIGHBOUR;end
                    NEIGHBOUR:begin
                        neighbour_valid<=1;nx<=cx;ny<=cy;
                        case(direction)
                            0:begin neighbour<=current+1'b1;nx<=cx+1'b1;if(cx==WIDTH-1)neighbour_valid<=0;end
                            1:begin neighbour<=current-1'b1;nx<=cx-1'b1;if(cx==0)neighbour_valid<=0;end
                            2:begin neighbour<=current+WIDTH;ny<=cy+1'b1;if(cy==HEIGHT-1)neighbour_valid<=0;end
                            3:begin neighbour<=current-WIDTH;ny<=cy-1'b1;if(cy==0)neighbour_valid<=0;end
                            4:begin neighbour<=current+WIDTH+1;nx<=cx+1'b1;ny<=cy+1'b1;if(cx==WIDTH-1 || cy==HEIGHT-1)neighbour_valid<=0;end
                            5:begin neighbour<=current-WIDTH-1;nx<=cx-1'b1;ny<=cy-1'b1;if(cx==0 || cy==0)neighbour_valid<=0;end
                            6:begin neighbour<=current-WIDTH+1;nx<=cx+1'b1;ny<=cy-1'b1;if(cx==WIDTH-1 || cy==0)neighbour_valid<=0;end
                            7:begin neighbour<=current+WIDTH-1;nx<=cx-1'b1;ny<=cy+1'b1;if(cx==0 || cy==HEIGHT-1)neighbour_valid<=0;end
                        endcase
                        state<=direction<4?WAIT_N:WAIT_C1;
                    end
                    WAIT_C1:state<=TEST_C1;
                    TEST_C1:if(!neighbour_valid || map_blocked)begin
                        neighbour_valid<=0;state<=WAIT_N;
                    end else state<=WAIT_C2;
                    WAIT_C2:state<=TEST_C2;
                    TEST_C2:begin if(map_blocked)neighbour_valid<=0;state<=WAIT_N;end
                    WAIT_N:state<=TEST_N;
                    TEST_N:begin
                        if(neighbour_valid && !map_blocked && candidate_g>=14'd8180)cost_limited<=1;
                        if(direction==7)begin
                            index<=0;scan_x<=0;scan_y<=0;scan_valid<=0;found<=0;state<=SCAN;
                        end else begin direction<=direction+1'b1;state<=NEIGHBOUR;end
                    end
                    TRACE:state<=WAIT_T;
                    WAIT_T:state<=STEP_T;
                    STEP_T:begin
                        path_valid<=1;path_addr<=trace_id;path_x<=tx;path_y<=ty;
                        trace_steps<=trace_steps+1'b1;
                        if(trace_id==sid) begin done<=1;busy<=0;state<=IDLE;end
                        else if(trace_steps>=NODES) begin error<=5;done<=1;busy<=0;state<=IDLE;end
                        else begin
                            case(q[15:13])
                                0:begin trace_id<=trace_id+1'b1;tx<=tx+1'b1;end
                                1:begin trace_id<=trace_id-1'b1;tx<=tx-1'b1;end
                                2:begin trace_id<=trace_id+WIDTH;ty<=ty+1'b1;end
                                3:begin trace_id<=trace_id-WIDTH;ty<=ty-1'b1;end
                                4:begin trace_id<=trace_id+WIDTH+1;tx<=tx+1'b1;ty<=ty+1'b1;end
                                5:begin trace_id<=trace_id-WIDTH-1;tx<=tx-1'b1;ty<=ty-1'b1;end
                                6:begin trace_id<=trace_id-WIDTH+1;tx<=tx+1'b1;ty<=ty-1'b1;end
                                7:begin trace_id<=trace_id+WIDTH-1;tx<=tx-1'b1;ty<=ty+1'b1;end
                            endcase
                            state<=TRACE;
                        end
                    end
                    default:state<=IDLE;
                endcase
            end
        end
    end
endmodule
