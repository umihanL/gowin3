`timescale 1ns/1ps
module retained_jog_tb;
 reg clk=0;always #5 clk=~clk;
 reg rst=0,tick=0,scope=1,link_ok=1,pose_valid=1,inertial_healthy=1,pump_low=1;
 reg command=0,stop=0;reg [7:0] seq=0;reg [31:0] args=0;
 reg [15:0] generation=9;reg [10:0] cursor=12;
 reg signed [31:0] enc=0;reg signed [15:0] actual=0,heading=0,servo=0;
 // Unit fixture supplies the motion owner's cumulative distance. The UART
 // integration fixture instantiates the actual vehicle_odometry pipeline.
 wire signed [63:0] model_distance_q16=$signed(enc)*64'sd8711;
 wire signed [31:0] model_travel_q8=model_distance_q16>>>8;
 wire [7:0] result;wire [31:0] status;wire active,run,blocked;wire signed [15:0] speed,steer;
 vehicle_retained_jog dut(.clk(clk),.rst_n(rst),.tick(tick),.scope(scope),.link_ok(link_ok),
 .pose_valid(pose_valid),.inertial_healthy(inertial_healthy),.travel_q8(model_travel_q8),.pump_low(pump_low),
 .generation(generation),.route_cursor(cursor),.command_valid(command),.stop(stop),
 .sequence_id(seq),.arguments(args),.encoder1(enc),.encoder2(enc),.actual1(actual),.actual2(actual),
 .heading(heading),.applied_steer(servo),.result(result),.status_word(status),
 .active(active),.run(run),.resume_blocked(blocked),.speed(speed),.steer(steer));
 integer fraction=0,delta=0,steps=0;reg encoder_model=1;
 task step;begin
  repeat(20)@(negedge clk);
  if(encoder_model)begin fraction=fraction+speed*1560;delta=fraction/20735;fraction=fraction-delta*20735;enc=enc+delta;end
  actual=speed;
  if(servo<steer)servo=servo+1;else if(servo>steer)servo=servo-1;
  tick=1;@(negedge clk);tick=0;repeat(3)@(negedge clk);steps=steps+1;
 end endtask
 task settle;integer n;begin for(n=0;n<15;n=n+1)step;end endtask
 task start_move(input [7:0] id,input integer distance,speed_value,input [7:0] wanted);begin
  seq=id;args={16'(speed_value),16'(distance)};repeat(3)@(negedge clk);
  if(result!==wanted)$fatal(1,"bad result id=%0d got=%0d wanted=%0d",id,result,wanted);
  command=1;@(negedge clk);command=0;repeat(3)@(negedge clk);
 end endtask
 task finish_move;integer n;begin
  n=0;while(active && n<1200)begin step;n=n+1;end
  if(n>=1200)$fatal(1,"move timeout");
 end endtask
 integer begin_enc,end_enc;
 initial begin
  repeat(5)@(negedge clk);rst=1;settle;
  start_move(1,0,40,2);start_move(1,101,40,2);start_move(1,-101,40,2);
  start_move(1,20,19,2);start_move(1,20,61,2);
  begin_enc=enc;servo=15;start_move(1,50,40,0);
  repeat(8)step;if(run || enc!=begin_enc)$fatal(1,"moved before steering centred");
  start_move(2,30,40,4);start_move(1,51,40,2);
  finish_move;end_enc=enc;
  if(status[7:0]!=3 || status[23:16]!=1 || !status[24] || blocked || end_enc-begin_enc<360 || end_enc-begin_enc>390)
   $fatal(1,"forward endpoint/state incorrect counts=%0d status=%h",end_enc-begin_enc,status);
  start_move(1,50,40,0);settle;if(enc!=end_enc || active)$fatal(1,"duplicate re-executed completed jog");
  heading=0;start_move(2,-50,40,0);repeat(5)step;heading=50;repeat(3)step;
  if(steer<=0)$fatal(1,"reverse heading correction must reverse steering sign, got %0d",steer);
  heading=0;finish_move;
  if(status[7:0]!=3 || enc-begin_enc>8 || enc-begin_enc< -8)$fatal(1,"signed reverse failed return %0d",enc-begin_enc);
  start_move(3,100,40,0);repeat(8)step;heading=50;repeat(2)step;
  if(steer>=0)$fatal(1,"forward heading correction wrong %0d",steer);
  heading=0;stop=1;@(negedge clk);stop=0;finish_move;
  if(status[7:0]!=4 || status[15:8]!=1 || status[23:16]!=3)$fatal(1,"STOP not cancelled");
  end_enc=enc;start_move(3,100,40,0);settle;if(enc!=end_enc)$fatal(1,"cancelled jog replayed");
  start_move(4,-100,40,0);repeat(4)step;link_ok=0;finish_move;
  if(status[7:0]!=4 || status[15:8]!=2)$fatal(1,"link loss did not abort");
  link_ok=1;settle;start_move(5,100,40,0);pump_low=0;finish_move;
  if(status[7:0]!=4 || status[15:8]!=4)$fatal(1,"pump high did not abort");
  start_move(6,50,40,4);pump_low=1;settle;start_move(6,100,40,0);inertial_healthy=0;finish_move;
  if(status[7:0]!=4 || status[15:8]!=10)$fatal(1,"shared inertial health loss did not abort");
  inertial_healthy=1;settle;start_move(7,100,40,0);cursor=13;finish_move;
  if(status[7:0]!=4 || status[15:8]!=12)$fatal(1,"cursor change did not abort");
  settle;start_move(8,100,40,0);generation=10;finish_move;
  if(status[7:0]!=4 || status[15:8]!=11)$fatal(1,"generation change did not abort");
  settle;start_move(9,100,40,0);scope=0;finish_move;
  if(status[7:0]!=4 || status[15:8]!=2)$fatal(1,"scope exit did not abort");
  scope=1;settle;start_move(10,-100,40,0);encoder_model=0;finish_move;
  if(status[7:0]!=4 || status[15:8]!=5)$fatal(1,"reverse stalled wheel did not abort %h",status);
  encoder_model=1;settle;start_move(11,100,40,0);pose_valid=0;finish_move;
  if(status[7:0]!=4 || status[15:8]!=9)$fatal(1,"invalid committed pose did not abort");
  $display("PASS retained jog: signed shared-odometry bounds, servo-centre wait, reverse heading sign, exact sequence/idempotency, STOP/link/pump/inertial/pose/cursor/generation/scope/stall aborts");$finish;
 end
 initial begin #100000000;$fatal(1,"timeout");end
endmodule
