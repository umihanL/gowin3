// Short encoder-bounded motion inside a paused navigation session. This
// controller never writes route RAM, its cursor, map pose or IMU zero.
`include "src/vehicle/vehicle_config.vh"
module vehicle_retained_jog #(
 parameter STALL_TICKS=150,MAX_TICKS=1000,
 parameter MM_PER_COUNT_Q16=(`VEHICLE_WHEEL_DIAMETER_MM*314159*64'd65536/100000/`VEHICLE_COUNTS_PER_REV)
)(
 input clk,rst_n,tick,scope,link_ok,pose_valid,inertial_healthy,
 input pump_low,input [15:0] generation,input [10:0] route_cursor,
 input command_valid,stop,input [7:0] sequence_id,input [31:0] arguments,
 input signed [31:0] encoder1,encoder2,
 input signed [31:0] travel_q8,
 input signed [15:0] actual1,actual2,heading,applied_steer,
 output reg [7:0] result,output [31:0] status_word,
 output active,run,resume_blocked,
 output signed [15:0] speed,steer
);
 localparam IDLE=0,MOVING=1,STOPPING=2,DONE=3,ABORTED=4;
 reg [7:0] phase,reason,accepted_sequence;
 reg [15:0] accepted_generation,elapsed_ticks,stall_ticks;
 reg [10:0] accepted_cursor;
 reg signed [15:0] requested_speed,applied_speed,hold_heading;
 reg signed [31:0] previous1,previous2,start_travel_q8;
 reg [31:0] target_q16,previous_arguments;
 reg [7:0] still_ticks;
 reg duplicate_valid,steering_ready;
 wire signed [15:0] arg_distance=$signed(arguments[15:0]);
 wire [15:0] arg_speed=arguments[31:16];
 wire [15:0] absolute_distance=arg_distance<0?-arg_distance:arg_distance;
 wire signed [31:0] delta1=encoder1-previous1,delta2=encoder2-previous2;
 wire signed [32:0] delta_sum=$signed({delta1[31],delta1})+$signed({delta2[31],delta2});
 wire signed [16:0] bounded_delta=delta_sum[16:0];
 localparam signed [15:0] MM_SCALE=MM_PER_COUNT_Q16;
 wire signed [32:0] distance_product=bounded_delta*MM_SCALE;
 wire signed [31:0] distance_step=distance_product>>>1;
 wire signed [31:0] directed_step=requested_speed<0?-distance_step:distance_step;
 // Both map pose and this displacement now consume the SAME committed
 // encoder/IMU odometry. Raw encoder deltas below remain independent fault
 // and stall evidence, but no longer maintain a second distance integrator.
 wire signed [31:0] travelled_q8=travel_q8-start_travel_q8;
 wire signed [31:0] directed_travel_q8=requested_speed<0?-travelled_q8:travelled_q8;
 wire signed [31:0] progress_q16=directed_travel_q8<<<8;
 wire [31:0] remaining=progress_q16<0?target_q16:
      $unsigned(progress_q16)>=target_q16?32'd0:target_q16-progress_q16;
 wire wheels_still=still_ticks>=10;
 wire sensor_ready=pose_valid && inertial_healthy;
 wire navigation_ready=scope && sensor_ready && generation!=0 && generation!=16'hffff;
 wire duplicate=duplicate_valid && sequence_id==accepted_sequence && arguments==previous_arguments;
 wire same_sequence=duplicate_valid && sequence_id==accepted_sequence;
 wire signed [16:0] raw_error=$signed(hold_heading)-$signed(heading);
 wire signed [16:0] heading_error=raw_error>1800?raw_error-3600:raw_error < -1800?raw_error+3600:raw_error;
 // Positive steering gives positive yaw when moving forward. Reverse travel
 // needs the opposite steering sign to correct the same world-heading error.
 wire signed [16:0] direction_error=requested_speed<0?-heading_error:heading_error;
 wire signed [15:0] hold_steer=direction_error>50?16'sd5:direction_error < -50?-16'sd5:direction_error/10;
 assign active=phase==MOVING || phase==STOPPING;
 assign run=phase==MOVING && steering_ready && navigation_ready && link_ok && pump_low && !stop;
 assign speed=run?applied_speed:16'sd0;
 assign steer=phase==MOVING && steering_ready?hold_steer:16'sd0;
 assign resume_blocked=active || (duplicate_valid && !wheels_still);
 wire [7:0] flags={5'd0,run,navigation_ready,wheels_still};
 assign status_word={flags,accepted_sequence,reason,phase};
 always @*begin
  result=0;
  if(!scope)result=1;
  else if(arg_distance==0 || absolute_distance>100 || arg_speed<20 || arg_speed>60)result=2;
  else if(same_sequence && !duplicate)result=2;
  else if(!navigation_ready || !link_ok)result=3;
  else if(!pump_low || (!duplicate && (active || !wheels_still)))result=4;
 end
 task stop_motion(input [7:0] why);begin
  phase<=STOPPING;reason<=why;applied_speed<=0;steering_ready<=0;still_ticks<=0;
 end endtask
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin
   phase<=IDLE;reason<=0;accepted_sequence<=0;accepted_generation<=0;accepted_cursor<=0;
   elapsed_ticks<=0;stall_ticks<=0;requested_speed<=0;applied_speed<=0;hold_heading<=0;
   previous1<=0;previous2<=0;start_travel_q8<=0;target_q16<=0;previous_arguments<=0;
   still_ticks<=0;duplicate_valid<=0;steering_ready<=0;
  end else begin
   if(tick)begin
    previous1<=encoder1;previous2<=encoder2;
    if(delta1==0 && delta2==0 && actual1<5 && actual1> -5 && actual2<5 && actual2> -5)begin
     if(still_ticks<255)still_ticks<=still_ticks+1'b1;
    end else still_ticks<=0;
    if(phase==MOVING)begin
     elapsed_ticks<=elapsed_ticks+1'b1;
     if(!steering_ready)begin
      // The cruise steering servo slews at 1 degree/10 ms; keep braking until
      // its command has returned to centre before the signed straight jog.
      if(applied_steer<=1 && applied_steer>=-1)steering_ready<=1;
     end else begin
      if((delta1==0 || delta2==0) && (applied_speed>=20 || applied_speed<=-20))stall_ticks<=stall_ticks+1'b1;
      else stall_ticks<=0;
      if(applied_speed<requested_speed-2)applied_speed<=applied_speed+2;
      else if(applied_speed>requested_speed+2)applied_speed<=applied_speed-2;
      else applied_speed<=requested_speed;
      if(remaining<=20*65536 && (applied_speed>20 || applied_speed< -20))
       applied_speed<=requested_speed<0?-16'sd20:16'sd20;
     end
     if(remaining<=65536)stop_motion(0);
     else if(delta1>100 || delta1< -100 || delta2>100 || delta2< -100 || directed_step< -262144)stop_motion(7);
     else if(stall_ticks>=STALL_TICKS)stop_motion(5);
     else if(elapsed_ticks>=MAX_TICKS)stop_motion(6);
     else if(heading_error>150 || heading_error< -150)stop_motion(8);
    end
    if(phase==STOPPING && wheels_still)phase<=reason==0?DONE:ABORTED;
   end
   if(command_valid && result==0 && !duplicate)begin
    phase<=MOVING;reason<=0;accepted_sequence<=sequence_id;previous_arguments<=arguments;duplicate_valid<=1;
    accepted_generation<=generation;accepted_cursor<=route_cursor;hold_heading<=heading;
    requested_speed<=arg_distance<0?-$signed(arg_speed):$signed(arg_speed);applied_speed<=0;
    start_travel_q8<=travel_q8;target_q16<={absolute_distance,16'd0};elapsed_ticks<=0;stall_ticks<=0;
    previous1<=encoder1;previous2<=encoder2;steering_ready<=0;
   end
   // STOP wins over a command in the same cycle. Replayed packets cannot
   // restart a cancelled/completed displacement with the same sequence ID.
   if(phase==MOVING || (command_valid && result==0 && !duplicate))begin
    if(stop)stop_motion(1);
    else if(!scope || !link_ok)stop_motion(2);
    // 3 remains an old-firmware reason. New terminals expose the failed
    // source so the host can distinguish pose/IMU/route identity failures.
    else if(phase==MOVING)begin
     if(!inertial_healthy)stop_motion(10);
     else if(generation!=accepted_generation)stop_motion(11);
     else if(route_cursor!=accepted_cursor)stop_motion(12);
     else if(!pose_valid)stop_motion(9);
     else if(!pump_low)stop_motion(4);
    end else if(!pump_low)stop_motion(4);
   end
   if(!scope && !active)duplicate_valid<=0;
  end
 end
endmodule
