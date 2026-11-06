// Host extension controller. Distances use encoder feedback, not elapsed time.
// Software ACK, commanded stop and measured wheel stillness remain distinct.
`include "src/vehicle/vehicle_config.vh"
module vehicle_extension #(
 parameter LEASE_TICKS=40, STALL_TICKS=150, MAX_TICKS=60000,
 parameter MM_PER_COUNT_Q16=(`VEHICLE_WHEEL_DIAMETER_MM*314159*64'd65536/100000/`VEHICLE_COUNTS_PER_REV)
)(
 input clk,rst_n,tick,scope,drive_page,link_ok,
 input command_valid,input [7:0] opcode,sequence_id,
 input [31:0] arguments,output reg [7:0] result,
 input signed [31:0] encoder1,encoder2,
 input signed [15:0] actual1,actual2,
 input front_valid,input [15:0] front_mm,
 input zebra_update,zebra_detected,
 input zebra_toggle,output reg zebra_enable,output zebra_can_change,
 input legacy_run,input [15:0] legacy_speed,
 output owned,run,brake,output signed [15:0] speed,steer,
 output reg [7:0] task_mode,phase,task_id,
 output [7:0] flags,output [15:0] remaining_mm,
 output reg event_valid,output reg [7:0] event_code,event_task,event_status
);
 localparam IDLE=0,MOVE=1,VELOCITY=2,STOPPING=3,DONE=4,FAULT=5,WAIT_LIGHT=6;
 wire signed [15:0] arg_distance=$signed(arguments[15:0]);
 wire [15:0] arg_speed=arguments[31:16];
 wire signed [7:0] arg_steer=$signed(arguments[23:16]);
 wire [15:0] abs_distance=arg_distance<0?-arg_distance:arg_distance;
 wire moving=phase==MOVE || phase==VELOCITY;
 reg signed [15:0] requested_speed,requested_steer,applied_speed;
 reg signed [31:0] previous1,previous2;
 wire signed [31:0] delta1=encoder1-previous1,delta2=encoder2-previous2;
 wire signed [32:0] delta_sum=$signed(delta1)+$signed(delta2);
 wire signed [16:0] bounded_delta_sum=delta_sum[16:0];
 localparam signed [15:0] MM_SCALE=MM_PER_COUNT_Q16;
 wire signed [32:0] sum_q16=bounded_delta_sum*MM_SCALE;
 wire signed [31:0] delta_q16=sum_q16>>>1;
 wire signed [31:0] directed_delta=requested_speed<0?-delta_q16:delta_q16;
 reg signed [31:0] progress_q16;
 reg [31:0] target_q16;
 wire [31:0] distance_left=progress_q16<0?target_q16:
     $unsigned(progress_q16)>=target_q16?32'd0:target_q16-progress_q16;
 assign remaining_mm=phase==MOVE?distance_left[31:16]:16'd0;
 reg [15:0] elapsed_ticks,lease_ticks,stall_ticks;
 reg [7:0] still_ticks;
 reg [7:0] stop_reason;
 reg traffic_session,previous_legacy,wait_event_sent;
 reg zebra_armed;reg [3:0] zebra_clear_count;
 reg duplicate_valid;reg [7:0] previous_move_seq;reg [31:0] previous_move_args;
 reg velocity_session_seen;reg [7:0] velocity_sequence;
 wire wheels_still=still_ticks>=10;
 assign owned=scope && drive_page && (task_mode>=2 || phase==STOPPING);
 assign run=owned && link_ok && moving;
 assign brake=owned && (!moving || !link_ok);
 assign speed=run?applied_speed:16'sd0;
 assign steer=run?requested_steer:16'sd0;
 assign flags={3'd0,zebra_enable,scope,phase==WAIT_LIGHT,wheels_still,run};
 assign zebra_can_change=scope && wheels_still && !moving && phase!=STOPPING && (!legacy_run || phase==WAIT_LIGHT);
 wire duplicate_move=duplicate_valid && previous_move_seq==sequence_id && previous_move_args==arguments;
 always @*begin
   result=0;
   case(opcode)
    8'h05:result=0;
    8'h10:if(!scope || !drive_page || task_mode!=2)result=1;
      else if(arg_distance==0 || abs_distance>10000 || arg_speed<20 || arg_speed>200)result=2;
      else if(!duplicate_move && (moving || phase==STOPPING || !wheels_still))result=4;
    8'h11:if(!scope || !drive_page || task_mode!=2)result=1;
      else if(arg_distance==0 || arg_distance>200 || arg_distance< -200 || arg_steer>28 || arg_steer< -28 || arguments[31:24]!=0)result=2;
      else if((moving && (phase!=VELOCITY || task_id!=sequence_id || arg_distance[15]!=requested_speed[15])) || phase==STOPPING)result=4;
      else if(!moving && velocity_session_seen && velocity_sequence==sequence_id)result=3;
    8'h12:if(!scope)result=1;else if(arguments[7:0]>3 || arguments[31:8]!=0)result=2;
      else if((moving || phase==STOPPING || legacy_run) && arguments[7:0]!=0)result=4;
    8'h13:if(!scope || task_mode!=3)result=1;else if(arguments>2)result=2;
      else if(arguments==2 && (phase!=WAIT_LIGHT || !zebra_enable))result=3;
    8'h17:if(!scope)result=1;else if(arguments>1)result=2;else if(!zebra_can_change)result=4;
    8'h14,8'h15:result=0;
    8'h16:result=5;
    default:result=5;
   endcase
 end
 task stop_motion(input [7:0] reason);begin
   phase<=STOPPING;applied_speed<=0;requested_speed<=0;requested_steer<=0;
   stop_reason<=reason;still_ticks<=0;
 end endtask
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin
   task_mode<=0;phase<=IDLE;task_id<=0;requested_speed<=0;requested_steer<=0;applied_speed<=0;
   previous1<=0;previous2<=0;progress_q16<=0;target_q16<=0;elapsed_ticks<=0;lease_ticks<=0;stall_ticks<=0;still_ticks<=0;
   stop_reason<=0;traffic_session<=0;previous_legacy<=0;wait_event_sent<=0;zebra_armed<=1;zebra_clear_count<=0;
   event_valid<=0;event_code<=0;event_task<=0;event_status<=0;duplicate_valid<=0;previous_move_seq<=0;previous_move_args<=0;
   velocity_session_seen<=0;velocity_sequence<=0;
   zebra_enable<=0;
  end else begin
   event_valid<=0;previous_legacy<=legacy_run;
   if(tick)begin
    previous1<=encoder1;previous2<=encoder2;
    if(delta1==0 && delta2==0 && actual1<5 && actual1> -5 && actual2<5 && actual2> -5)begin
      if(still_ticks<255)still_ticks<=still_ticks+1'b1;
    end else still_ticks<=0;
    if(moving)begin
     elapsed_ticks<=elapsed_ticks+1'b1;
     if(!traffic_session && phase==VELOCITY && lease_ticks<LEASE_TICKS)lease_ticks<=lease_ticks+1'b1;
     if((delta1==0 || delta2==0) && (applied_speed>=40 || applied_speed<= -40))stall_ticks<=stall_ticks+1'b1;
     else stall_ticks<=0;
     if(delta1<=100 && delta1>= -100 && delta2<=100 && delta2>= -100)progress_q16<=progress_q16+directed_delta;
     if(applied_speed<requested_speed-2)applied_speed<=applied_speed+2;
     else if(applied_speed>requested_speed+2)applied_speed<=applied_speed-2;
     else applied_speed<=requested_speed;
     // Slow the last 5 cm, then brake at the measured endpoint.
     if(phase==MOVE && remaining_mm<=50 && (applied_speed>40 || applied_speed< -40))
       applied_speed<=requested_speed<0?-16'sd40:16'sd40;
     if(phase==MOVE && distance_left<=32'd65536)stop_motion(3);
     else if(delta1>100 || delta1< -100 || delta2>100 || delta2< -100 || directed_delta< -262144)stop_motion(8);
     else if(stall_ticks>=STALL_TICKS)stop_motion(9);
     else if(elapsed_ticks>=MAX_TICKS || (!traffic_session && phase==VELOCITY && lease_ticks>=LEASE_TICKS))stop_motion(4);
     else if(requested_speed>0 && front_valid && front_mm<100)stop_motion(5);
    end
    if(phase==STOPPING && wheels_still)begin
      if(stop_reason==6)begin phase<=WAIT_LIGHT;wait_event_sent<=0;end
      else begin
       phase<=stop_reason==0?IDLE:stop_reason==3?DONE:FAULT;
       if(stop_reason!=0)begin event_valid<=1;event_code<=stop_reason==3?8'hf0:8'hf1;event_task<=task_id;event_status<=stop_reason;end
      end
    end
   end
   // Report crossing only after mechanical stillness, once per crossing.
   if(phase==WAIT_LIGHT && !wait_event_sent)begin event_valid<=1;event_code<=8'hf3;event_task<=task_id;event_status<=1;wait_event_sent<=1;end
   if(zebra_update && zebra_enable)begin
    if(zebra_detected)zebra_clear_count<=0;
    else if(zebra_clear_count<10)zebra_clear_count<=zebra_clear_count+1'b1;
    if(!zebra_detected && zebra_clear_count>=9)zebra_armed<=1;
    if(task_mode==3 && moving && zebra_detected && zebra_armed)begin zebra_armed<=0;stop_motion(6);end
   end
   if(scope && drive_page && task_mode==3 && zebra_enable && legacy_run && !previous_legacy && link_ok)begin
     phase<=VELOCITY;traffic_session<=1;requested_speed<=legacy_speed>200?16'sd200:$signed(legacy_speed);
     requested_steer<=0;applied_speed<=0;elapsed_ticks<=0;stall_ticks<=0;progress_q16<=0;task_id<=sequence_id;
   end
   if(command_valid)begin
    if(opcode==8'h05)begin stop_motion(0);traffic_session<=0;end
    else if(result==0)case(opcode)
     8'h10:if(!duplicate_move)begin
       phase<=MOVE;requested_speed<=arg_distance<0?-$signed(arg_speed):$signed(arg_speed);requested_steer<=0;applied_speed<=0;
       task_id<=sequence_id;target_q16<={abs_distance,16'd0};progress_q16<=0;elapsed_ticks<=0;stall_ticks<=0;traffic_session<=0;
       previous1<=encoder1;previous2<=encoder2;duplicate_valid<=1;previous_move_seq<=sequence_id;previous_move_args<=arguments;
     end
     8'h11:begin
       requested_speed<=arg_distance;requested_steer<=arg_steer;
       if(phase!=VELOCITY)begin phase<=VELOCITY;requested_speed<=arg_distance;requested_steer<=arg_steer;
        velocity_session_seen<=1;velocity_sequence<=sequence_id;
        applied_speed<=0;task_id<=sequence_id;elapsed_ticks<=0;stall_ticks<=0;traffic_session<=0;
        previous1<=encoder1;previous2<=encoder2;progress_q16<=0;end
       lease_ticks<=0;
     end
     8'h12:begin task_mode<=arguments[7:0];stop_motion(0);traffic_session<=0;duplicate_valid<=0;end
     8'h13:if(arguments==2)begin
       phase<=VELOCITY;requested_speed<=legacy_speed>200?16'sd200:$signed(legacy_speed);requested_steer<=0;
       applied_speed<=0;elapsed_ticks<=0;stall_ticks<=0;traffic_session<=1;
      end else if(moving)stop_motion(6);
     8'h17:begin zebra_enable<=arguments[0];if(phase!=WAIT_LIGHT)zebra_armed<=1;zebra_clear_count<=0;end
     default:;
    endcase
   end
   // UART wins a simultaneous local tap. A consumed crossing remains consumed
   // while waiting, even if its detector is turned off and back on.
   if(zebra_toggle===1'b1 && zebra_can_change && !(command_valid && opcode==8'h17 && result==0))begin
     zebra_enable<=!zebra_enable;if(phase!=WAIT_LIGHT)zebra_armed<=1;zebra_clear_count<=0;
   end
   if(scope && !link_ok && (moving || phase==WAIT_LIGHT))begin stop_motion(4);traffic_session<=0;end
   if(!scope)begin task_mode<=0;phase<=IDLE;requested_speed<=0;requested_steer<=0;applied_speed<=0;traffic_session<=0;duplicate_valid<=0;end
  end
 end
endmodule
