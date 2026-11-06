`timescale 1ns/1ps
module motor_brake_plant_tb;
 reg clk=0;always #5 clk=~clk;
 reg rst=0,enabled=0,brake=0;reg signed [15:0] target=0;
 reg [1:0] ab=0;wire [1:0] dir;wire pwm;
 wire signed [15:0] actual,converted;wire signed [19:0] counts;
 motor_target_counts convert(.clk(clk),.speed_mm_s(target),.converted_speed(converted),.counts_q4(counts));
 motor_closed_loop #(.CLK_HZ(100000),.ENCODER_FILTER_CYCLES(1),.REVERSE_BLANK_CYCLES(2)) dut(
 .clk(clk),.rst_n(rst),.drive_enabled(enabled),.brake_request(brake),.speed_command(target),
 .converted_speed(converted),.target_counts_q4(counts),.encoder_A(ab[1]),.encoder_B(ab[0]),
 .dir(dir),.pwm_out(pwm),.actual_speed_mm_s(actual),.encoder_position());
 real velocity=0,position=0,fraction=0,acceleration=0,origin=0;
 integer edges=0,ticks=0,step=0;
 always @(negedge clk)if(rst)begin
   if(dir==3 && pwm)acceleration=-1500;
   else if(dir==1 && pwm)acceleration=(900-velocity)/0.15;
   else acceleration=-velocity/0.8;
   velocity=velocity+acceleration*0.00001;if(velocity<0)velocity=0;
   position=position+velocity*0.00001;
   fraction=fraction+velocity*0.00001*1560.0/(66*3.14159265);
   if(fraction>=1)begin
     fraction=fraction-1;
     case(ab)0:ab=1;1:ab=3;3:ab=2;2:ab=0;endcase
   end
 end
 initial begin
   repeat(10)@(negedge clk);rst=1;enabled=1;target=400;
   repeat(500000)@(negedge clk);
   if(velocity<350 || velocity>450)$fatal(1,"did not reach cruise %f",velocity);
   origin=position;brake=1;
   for(step=0;step<32;step=step+1)begin
     target=step>=30?0:400-step*13;
     repeat(1000)@(negedge clk);
   end
   repeat(60000)@(negedge clk);
   if(velocity>5 || position-origin>165)$fatal(1,"PWM brake stopping distance %f speed %f",position-origin,velocity);
   $display("PASS motor brake plant: actual 400mm/s, real quadrature/PI/PWM, 300ms deceleration, stopping travel=%fmm final=%fmm/s",position-origin,velocity);
   $finish;
 end
endmodule
