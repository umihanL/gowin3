// Directed tangent road graph. Backward dynamic programming computes the
// cost of ALL remaining ordered waypoints before a forward route is emitted.
// Forward legs use A*: f=g+h with a proven conservative map-distance heuristic.
// Backward waypoint suffix DP uses Dijkstra (h=0) in the same work RAM.
`include "nav_centerline_config.vh"
module nav_road_planner(
 input clk,rst_n,clear,cancel,start,
 input start_heading_valid,input signed [15:0] start_heading, // Map degrees * 10, X right/Y up.
 input region_preference_enabled,input [7:0] region_x0,region_y0,region_x1,region_y1,
 input snap_start,input [7:0] snap_x,snap_y,
 output reg snap_done,snap_accept,output reg [7:0] snapped_x,snapped_y,
 output reg snapped_heading_valid,output reg signed [15:0] snapped_heading,
 input [7:0] sx,sy,gx,gy,input [3:0] waypoint_count,input [63:0] waypoint_x,waypoint_y,
 input [3:0] barrier_valid,input [35:0] barrier_x0,barrier_y0,barrier_x1,barrier_y1,
 output reg busy,done,output reg [2:0] error,output reg [17:0] length_cm,
 output reg route_write,output route_bit,output reg [14:0] route_addr,
 output reg sample_valid,output reg [16:0] sample_x_q8,sample_y_q8,
 output reg [3:0] sample_segment,
 output reg anchor_valid,output reg [2:0] anchor_index,output reg [7:0] anchor_x,anchor_y,
 input [10:0] execution_index,output reg [11:0] execution_count,
 output [16:0] execution_x_q8,execution_y_q8,output execution_lock,
 // Independent synchronous read port; never steals the execution index.
 input [10:0] telemetry_index,output [16:0] telemetry_x_q8,telemetry_y_q8
);
 localparam B=`ROAD_BITS,N=`ROAD_NODES,W=`ROAD_WORD_BITS,INVALID={B{1'b1}};
 localparam IDLE=0,NEAR_ISSUE=1,NEAR_WAIT=2,NEAR_READ=3,INIT_ISSUE=4,INIT_WAIT=5,INIT_READ=6,
 SCAN_ISSUE=7,SCAN_WAIT=8,SCAN_READ=9,POP_ISSUE=10,POP_WAIT=11,POP_READ=12,
 EDGE=13,RELAX_WAIT=14,RELAX_READ=15,TRACE_ISSUE=16,TRACE_WAIT=17,TRACE_READ=18,
 OUTPUT_ISSUE=19,OUTPUT_WAIT=20,OUTPUT_GRAPH=21,OUTPUT_GWAIT=22,EMIT=23,LINE=24,NEXT_SEG=25,
 POINT_FINISH=26,CONFIGURE=27,DP_FINISH=28,REGION_EDGE_WAIT=29,REGION_EDGE_READ=30;
 reg [4:0] state;
 reg [W-1:0] graph[0:(1<<B)-1];reg [W-1:0] gq;
 reg signed [8:0] start_tangent[0:(1<<B)-1],tangent_q;
 reg heading_filter;reg signed [15:0] requested_heading;
 reg start_fallback,region_active;reg [7:0] region_left,region_top,region_right,region_bottom;
 reg [16:0] physical_start_x,physical_start_y;
 reg [3*(B+4)-1:0] reverse_graph[0:(1<<B)-1],reverse_q;
 initial begin
  `include "nav_centerline_init.vh"
  `include "nav_centerline_reverse.vh"
  `include "nav_centerline_heading.vh"
 end
 reg [B-1:0] gaddr,waddr,scan,min_node,current,arrival,best_goal_node;
 // Preserve the unweighted predecessor-edge length while searching weighted
 // regional costs. 32 bits fit the existing 36-bit work-memory organization.
 reg [B+20:0] work[0:(1<<B)-1];reg [B+20:0] wq;
 reg [B-1:0] stack[0:2047];reg [B-1:0] stack_q;
 reg [10:0] stack_addr,stack_count;
 reg [34:0] route[0:2047];reg [34:0] rq,telemetry_q;
 always @(posedge clk)begin
  gq<=graph[gaddr];reverse_q<=reverse_graph[gaddr];tangent_q<=start_tangent[gaddr];
  wq<=work[waddr];stack_q<=stack[stack_addr];rq<=route[execution_index];
  telemetry_q<=route[telemetry_index];
 end
 assign telemetry_x_q8=telemetry_q[16:0];assign telemetry_y_q8=telemetry_q[33:17];
 assign execution_x_q8=rq[16:0];assign execution_y_q8=rq[33:17];assign execution_lock=rq[34];
 assign route_bit=1'b1;
 wire [16:0] nx=gq[16:0],ny=gq[33:17];
 wire signed [15:0] tangent_tenths=$signed(tangent_q)*16'sd10;
 wire signed [16:0] heading_delta=$signed(tangent_tenths)-$signed(requested_heading);
 wire signed [16:0] heading_error=heading_delta>1800?heading_delta-3600:heading_delta < -1800?heading_delta+3600:heading_delta;
 wire start_direction_allowed=!heading_filter || (tangent_q!=9'sd255 && heading_error>=-350 && heading_error<=350);
 wire [17:0] start_dx=nx>physical_start_x?nx-physical_start_x:physical_start_x-nx;
 wire [17:0] start_dy=ny>physical_start_y?ny-physical_start_y:physical_start_y-ny;
 wire [18:0] start_separation={1'b0,start_dx}+start_dy;
 // A 2-cm backward allowance only covers input-cell quantization. A nearby
 // opposite-direction road alias never becomes a valid start candidate.
 wire start_forward=requested_heading>=-450 && requested_heading<=450 ? {1'b0,nx}+18'd512>=physical_start_x:
    requested_heading>450 && requested_heading<1350 ? ny<={1'b0,physical_start_y}+18'd512:
    requested_heading>=1350 || requested_heading<=-1350 ? nx<={1'b0,physical_start_x}+18'd512:
    {1'b0,ny}+18'd512>=physical_start_y;
 wire strict_start=nx==source_x && ny==source_y && start_direction_allowed;
 wire nearby_start=start_separation<=19'd1536 && start_forward && start_direction_allowed;
 wire initial_seed=segment==0 && (start_fallback?nearby_start:strict_start);
 wire [15:0] approach_cost=start_fallback?(start_separation+19'd255)>>8:16'd0;
 function [15:0] preference_cost;
  input [3:0] base_cost;input [16:0] x,y;
  begin
   if(region_active && (x[16:9]<region_left || x[16:9]>region_right ||
       y[16:9]<region_top || y[16:9]>region_bottom))
    preference_cost={12'd0,base_cost}+({12'd0,base_cost}<<5);
   else preference_cost={12'd0,base_cost};
  end
 endfunction
 function forbidden;
  input [16:0] x,y;integer j;reg [9:0] xx,yy;
  begin
   xx={1'b0,x[16:8]};yy={1'b0,y[16:8]};forbidden=0;
   for(j=0;j<4;j=j+1)
    if(barrier_valid[j] && xx+11>=barrier_x0[j*9+:9] && xx<=barrier_x1[j*9+:9]+10'd11 &&
       yy+11>=barrier_y0[j*9+:9] && yy<=barrier_y1[j*9+:9]+10'd11)forbidden=1;
  end
 endfunction
 reg snapping,backward;reg [3:0] segment,point_index,dp_stage,waypoints;
 reg [16:0] point_x[0:9],point_y[0:9];
 // One synchronous suffix-distance RAM, reused by all waypoint layers.
 // Storing every node allows a touch region to select the correct fillet,
 // rather than forcing an exact straight-line sample and a needless loop.
 reg [15:0] suffix_cost[0:8*(1<<B)-1],suffix_q;
 reg stage_reachable;
 function junction_near;
  input [16:0] x,y;
  begin
   junction_near=0;
   `include "nav_centerline_junctions.vh"
  end
 endfunction
 function [16:0] touch_radius;
  input [16:0] x,y;
  // A quarter-circle fillet's closest Manhattan distance to its junction
  // is R*(2-sqrt(2)): about 18.4cm for our 31.4cm nose radius.
  begin touch_radius=junction_near(x,y)?17'd5120:17'd1536;end
 endfunction
 reg [16:0] query_x,query_y,nearest_x,nearest_y,source_x,source_y,goal_x,goal_y;
 reg [18:0] nearest;reg nearest_heading_valid;reg signed [15:0] nearest_heading;
 wire [17:0] dx=nx>query_x?nx-query_x:query_x-nx,dy=ny>query_y?ny-query_y:query_y-ny;
 wire [18:0] separation={1'b0,dx}+dy;
 wire terminal=backward?dp_stage==waypoints:segment==waypoints;
 wire [2:0] suffix_stage=backward?dp_stage[2:0]:segment[2:0];
 wire [2:0] write_stage=dp_stage[2:0]-3'd1;
 always @(posedge clk)suffix_q<=suffix_cost[{suffix_stage,gaddr}];
 wire [15:0] remaining_cost=terminal?16'd0:suffix_q;
 reg [16:0] goal_radius,collect_radius,collect_x,collect_y;
 wire [17:0] goal_dx=nx>goal_x?nx-goal_x:goal_x-nx,goal_dy=ny>goal_y?ny-goal_y:goal_y-ny;
 wire goal_match=({1'b0,goal_dx}+goal_dy)<=goal_radius;
 wire [17:0] collect_dx=nx>collect_x?nx-collect_x:collect_x-nx,collect_dy=ny>collect_y?ny-collect_y:collect_y-ny;
 wire collect_match=({1'b0,collect_dx}+collect_dy)<=collect_radius;
 reg [15:0] minimum,current_distance,best_goal_score,best_goal_distance;
 // ROM audit: every directed edge cost >= ceil(L1 coordinate delta / 512).
 // Coordinates are cm Q8; subtracting the full goal touch radius makes
 // this a lower bound to *any* accepted goal node, including fillets.
 // Backward suffix propagation needs all costs, so its heuristic is zero.
 wire [18:0] goal_l1={1'b0,goal_dx}+{1'b0,goal_dy};
 wire [16:0] heuristic=backward || goal_l1<=goal_radius ? 17'd0:(goal_l1-goal_radius)>>9;
 wire [16:0] scan_priority={1'b0,wq[B+:16]}+heuristic;
 reg [16:0] min_priority;
 reg [11:0] expansions;reg best_goal_valid;
 wire [16:0] goal_score={1'b0,minimum}+{1'b0,remaining_cost};
 reg [1:0] slot;
 wire [B+3:0] edge_word=backward?(reverse_q>>(slot*(B+4))):(gq>>(34+slot*(B+4)));
 wire [B-1:0] neighbor=edge_word[B-1:0];wire [3:0] cost=edge_word[B+3:B];
 reg [B-1:0] edge_node;reg [15:0] proposed;reg [3:0] edge_cost;
 wire [16:0] edge_sum={1'b0,current_distance}+preference_cost(cost,nx,ny);
 wire [16:0] reverse_edge_sum={1'b0,current_distance}+preference_cost(edge_cost,nx,ny);
 reg [7:0] line_x,line_y,dest_x,dest_y;
 reg [16:0] last_x,last_y;
 wire duplicate=execution_count!=0 && last_x==nx && last_y==ny;
 wire end_of_segment=stack_count==0;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin
   state<=IDLE;busy<=0;done<=0;error<=0;length_cm<=0;route_write<=0;route_addr<=0;
   heading_filter<=0;requested_heading<=0;
   start_fallback<=0;physical_start_x<=0;physical_start_y<=0;
   region_active<=0;region_left<=0;region_top<=0;region_right<=0;region_bottom<=0;edge_cost<=0;
   sample_valid<=0;sample_x_q8<=0;sample_y_q8<=0;sample_segment<=0;execution_count<=0;
   snapped_heading_valid<=0;snapped_heading<=0;nearest_heading_valid<=0;nearest_heading<=0;
   snap_done<=0;snap_accept<=0;snapped_x<=0;snapped_y<=0;snapping<=0;backward<=0;
   segment<=0;point_index<=0;dp_stage<=0;waypoints<=0;stage_reachable<=0;
   anchor_valid<=0;anchor_index<=0;anchor_x<=0;anchor_y<=0;
   goal_radius<=0;collect_radius<=0;collect_x<=0;collect_y<=0;
   gaddr<=0;waddr<=0;scan<=0;min_node<=0;current<=0;arrival<=0;stack_addr<=0;stack_count<=0;
   query_x<=0;query_y<=0;nearest_x<=0;nearest_y<=0;source_x<=0;source_y<=0;goal_x<=0;goal_y<=0;
   nearest<=0;minimum<=65535;min_priority<=17'h1ffff;current_distance<=0;expansions<=0;best_goal_node<=0;
   best_goal_score<=65535;best_goal_distance<=0;best_goal_valid<=0;
   slot<=0;edge_node<=0;proposed<=0;line_x<=0;line_y<=0;dest_x<=0;dest_y<=0;last_x<=0;last_y<=0;
  end else begin
   done<=0;snap_done<=0;sample_valid<=0;route_write<=0;anchor_valid<=0;
   if(cancel || clear)begin state<=IDLE;busy<=0;error<=0;if(clear)execution_count<=0;end
   else case(state)
    IDLE:begin
     if(snap_start)begin
      snapping<=1;scan<=0;nearest<=19'h7ffff;nearest_heading_valid<=0;
      query_x<={snap_x,9'b100000000};query_y<={snap_y,9'b100000000};state<=NEAR_ISSUE;
     end else if(start)begin
      busy<=1;error<=0;length_cm<=0;execution_count<=0;segment<=0;snapping<=0;backward<=0;
      heading_filter<=start_heading_valid===1'b1;requested_heading<=start_heading;
      start_fallback<=0;physical_start_x<={sx,9'b100000000};physical_start_y<={sy,9'b100000000};
      region_active<=region_preference_enabled===1'b1;
      region_left<=region_x0;region_top<=region_y0;region_right<=region_x1;region_bottom<=region_y1;
      point_index<=0;waypoints<=waypoint_count>8?4'd8:waypoint_count;scan<=0;nearest<=19'h7ffff;
      query_x<={sx,9'b100000000};query_y<={sy,9'b100000000};
      state<=NEAR_ISSUE;
     end
    end
    NEAR_ISSUE:begin gaddr<=scan;state<=NEAR_WAIT;end
    NEAR_WAIT:state<=NEAR_READ;
    NEAR_READ:begin
     // The selected coordinate identifies the physical road point. Search
     // initialization below accepts only aliases facing the declared heading.
     if(!forbidden(nx,ny) && (separation<nearest ||
         (snapping && separation==nearest && !nearest_heading_valid && tangent_q!=9'sd255)))begin
      nearest<=separation;nearest_x<=nx;nearest_y<=ny;
      nearest_heading<=tangent_tenths;nearest_heading_valid<=tangent_q!=9'sd255;
     end
     if(scan==N-1)state<=POINT_FINISH;
     else begin scan<=scan+1'b1;state<=NEAR_ISSUE;end
    end
    POINT_FINISH:begin
     if(snapping)begin
      snapped_heading<=nearest_heading;snapped_heading_valid<=nearest_heading_valid;
      snapped_x<=nearest_x[16:9];snapped_y<=nearest_y[16:9];snap_accept<=nearest<=18*256;
      snap_done<=1;state<=IDLE;
     end else if(nearest==19'h7ffff)begin error<=2;busy<=0;done<=1;state<=IDLE;end
     else begin
      point_x[point_index]<=nearest_x;point_y[point_index]<=nearest_y;
      if(point_index==waypoints+1)begin
       backward<=waypoints!=0;dp_stage<=waypoints;state<=CONFIGURE;
      end else begin
       point_index<=point_index+1'b1;scan<=0;nearest<=19'h7ffff;
       query_x<=point_index<waypoints?{waypoint_x[point_index*8+:8],9'b100000000}:{gx,9'b100000000};
       query_y<=point_index<waypoints?{waypoint_y[point_index*8+:8],9'b100000000}:{gy,9'b100000000};
       state<=NEAR_ISSUE;
      end
     end
    end
    CONFIGURE:begin
     scan<=0;stage_reachable<=0;
     source_x<=point_x[0];source_y<=point_y[0];
     goal_x<=backward?point_x[dp_stage+1]:point_x[segment+1];
     goal_y<=backward?point_y[dp_stage+1]:point_y[segment+1];
     // A terminal has no required departure tangent. Requiring the single
     // straight-road alias at a near-corner goal can force a full block loop
     // although a legal fillet passes within the normal 6 cm arrival window.
     goal_radius<=terminal?17'd1536:backward?touch_radius(point_x[dp_stage+1],point_y[dp_stage+1]):
         touch_radius(point_x[segment+1],point_y[segment+1]);
     collect_x<=point_x[dp_stage];collect_y<=point_y[dp_stage];
     collect_radius<=touch_radius(point_x[dp_stage],point_y[dp_stage]);state<=INIT_ISSUE;
    end
    INIT_ISSUE:begin gaddr<=scan;state<=INIT_WAIT;end
    INIT_WAIT:state<=INIT_READ;
    INIT_READ:begin
     work[scan]<={4'd0,forbidden(nx,ny),
       backward?(goal_match?remaining_cost:16'hffff):
        (initial_seed?approach_cost:(segment!=0 && scan==arrival)?16'd0:16'hffff),INVALID};
     if(backward)suffix_cost[{write_stage,scan}]<=65535;
     if(scan==N-1)begin
      scan<=0;minimum<=65535;min_priority<=17'h1ffff;expansions<=0;best_goal_score<=65535;best_goal_distance<=0;best_goal_valid<=0;state<=SCAN_ISSUE;
     end else begin scan<=scan+1'b1;state<=INIT_ISSUE;end
    end
    SCAN_ISSUE:begin waddr<=scan;gaddr<=scan;state<=SCAN_WAIT;end
    SCAN_WAIT:state<=SCAN_READ;
    SCAN_READ:begin
     if(!wq[B+16] && wq[B+:16]!=65535 && scan_priority<min_priority)begin
      min_priority<=scan_priority;minimum<=wq[B+:16];min_node<=scan;
     end
     if(scan==N-1)state<=POP_ISSUE;
     else begin scan<=scan+1'b1;state<=SCAN_ISSUE;end
    end
    POP_ISSUE:begin gaddr<=min_node;waddr<=min_node;state<=POP_WAIT;end
    POP_WAIT:state<=POP_READ;
    POP_READ:begin
     if(backward && (minimum==65535 || expansions>=N))state<=DP_FINISH;
     else if(!backward && best_goal_valid && (minimum==65535 || min_priority>={1'b0,best_goal_score}))begin
      arrival<=best_goal_node;current<=best_goal_node;stack_count<=0;state<=TRACE_ISSUE;
     end else if(minimum==65535 || expansions>=N)begin
      // Quantized stop poses at a fillet can be closest to an unrelated
      // straight alias. Retry bounded forward/tangent-compatible neighbors;
      // never substitute a made-up heading or seed the opposite lane.
      if(!backward && segment==0 && heading_filter && !start_fallback)begin start_fallback<=1;state<=CONFIGURE;end
      else begin error<=2;busy<=0;done<=1;execution_count<=0;state<=IDLE;end
     end
     else begin
      if(backward)begin
       suffix_cost[{write_stage,min_node}]<=minimum;
       if(collect_match)stage_reachable<=1;
      end
      if(!backward && goal_match && remaining_cost!=65535 && goal_score<best_goal_score)begin
       best_goal_score<=goal_score[15:0];best_goal_distance<=minimum;best_goal_node<=min_node;best_goal_valid<=1;
      end
      current<=min_node;current_distance<=minimum;work[min_node]<={wq[B+20:B+17],1'b1,wq[B+15:0]};
      slot<=0;expansions<=expansions+1'b1;state<=EDGE;
     end
    end
    DP_FINISH:begin
     if(!stage_reachable)begin error<=2;done<=1;busy<=0;state<=IDLE;end
     else begin
      if(dp_stage==1)begin backward<=0;segment<=0;end else dp_stage<=dp_stage-1'b1;
      state<=CONFIGURE;
     end
    end
    EDGE:begin
     if(neighbor==INVALID)begin
      if(slot==2)begin scan<=0;minimum<=65535;min_priority<=17'h1ffff;state<=SCAN_ISSUE;end else slot<=slot+1'b1;
     end else begin
      edge_node<=neighbor;edge_cost<=cost;waddr<=neighbor;
      // Reverse DP must price the ORIGINAL directed edge's source. Read its
      // coordinate before relaxation, then restore the current graph word.
      if(backward && region_active)begin gaddr<=neighbor;state<=REGION_EDGE_WAIT;end
      else begin proposed<=edge_sum>=65535?16'hffff:edge_sum[15:0];state<=RELAX_WAIT;end
     end
    end
    REGION_EDGE_WAIT:state<=REGION_EDGE_READ;
    REGION_EDGE_READ:begin
     proposed<=reverse_edge_sum>=65535?16'hffff:reverse_edge_sum[15:0];gaddr<=current;state<=RELAX_WAIT;
    end
    RELAX_WAIT:state<=RELAX_READ;
    RELAX_READ:begin
     if(!wq[B+16] && proposed<wq[B+:16])work[edge_node]<={edge_cost,1'b0,proposed,current};
     if(slot==2)begin scan<=0;minimum<=65535;min_priority<=17'h1ffff;state<=SCAN_ISSUE;end
     else begin slot<=slot+1'b1;state<=EDGE;end
    end
    TRACE_ISSUE:begin waddr<=current;state<=TRACE_WAIT;end
    TRACE_WAIT:state<=TRACE_READ;
    TRACE_READ:begin
     stack[stack_count]<=current;
     length_cm<=length_cm+wq[B+20:B+17];
     if(wq[B-1:0]==INVALID)state<=OUTPUT_ISSUE;
     else if(stack_count>=2046)begin error<=1;done<=1;busy<=0;execution_count<=0;state<=IDLE;end
     else begin stack_count<=stack_count+1'b1;current<=wq[B-1:0];state<=TRACE_ISSUE;end
    end
    OUTPUT_ISSUE:begin stack_addr<=stack_count;state<=OUTPUT_WAIT;end
    OUTPUT_WAIT:state<=OUTPUT_GRAPH;
    OUTPUT_GRAPH:begin gaddr<=stack_q;state<=OUTPUT_GWAIT;end
    OUTPUT_GWAIT:state<=EMIT;
    EMIT:begin
     if(execution_count>=2047)begin error<=1;done<=1;busy<=0;execution_count<=0;state<=IDLE;end
     else begin
      if(end_of_segment && segment<waypoints)begin
       anchor_valid<=1;anchor_index<=segment[2:0];anchor_x<=nx[16:9];anchor_y<=ny[16:9];
      end
      if(!duplicate)begin
       route[execution_count[10:0]]<={end_of_segment || execution_count==0,ny,nx};
       execution_count<=execution_count+1'b1;sample_valid<=1;sample_x_q8<=nx;sample_y_q8<=ny;sample_segment<=segment;
       last_x<=nx;last_y<=ny;
      end else if(end_of_segment)route[execution_count-1'b1]<={1'b1,ny,nx};
      dest_x<=nx[16:9];dest_y<=ny[16:9];
      if(execution_count==0)begin line_x<=nx[16:9];line_y<=ny[16:9];end
      state<=LINE;
     end
    end
    LINE:begin
     route_write<=1;route_addr<=line_y*15'd176+line_x;
     if(line_x==dest_x && line_y==dest_y)begin
      if(stack_count==0)state<=NEXT_SEG;
      else begin stack_count<=stack_count-1'b1;state<=OUTPUT_ISSUE;end
     end else begin
      if(line_x!=dest_x)line_x<=line_x<dest_x?line_x+1'b1:line_x-1'b1;
      if(line_y!=dest_y)line_y<=line_y<dest_y?line_y+1'b1:line_y-1'b1;
     end
    end
    NEXT_SEG:begin
     if(segment==waypoints)begin
      busy<=0;done<=1;state<=IDLE;if(execution_count<2)error<=1;
     end else begin segment<=segment+1'b1;state<=CONFIGURE;end
    end
    default:state<=IDLE;
   endcase
  end
 end
endmodule
