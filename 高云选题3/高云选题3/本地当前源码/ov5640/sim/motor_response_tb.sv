`timescale 1ns/1ps
module motor_response_tb;
 reg clk=0;always #5 clk=~clk;
 reg rst_n=0,sv=0;reg signed [15:0] speed=0;
 wire signed [19:0] target;wire signed [15:0] converted;
 motor_target_counts converter(.clk(clk),.speed_mm_s(speed),.counts_q4(target),.converted_speed(converted));
 reg signed [15:0] measured_new=0,measured_old=0;
 wire signed [11:0] pwm_new,pwm_old;
 motor_pi #(.KP_Q8(6144),.KI_Q8(128),.STARTUP_KI_Q8(512),.UNWIND_KI_Q8(512),.PROPORTIONAL_AVERAGE_LOG2(1)) improved(
  .clk(clk),.rst_n(rst_n),.sample_valid(sv),.target_counts_q4(target),.measured_counts(measured_new),.pwm_command(pwm_new));
 motor_pi_before_settings #(.KP_Q8(6144),.KI_Q8(32),.STARTUP_KI_Q8(128),.UNWIND_KI_Q8(128),.PROPORTIONAL_AVERAGE_LOG2(2)) previous(
  .clk(clk),.rst_n(rst_n),.sample_valid(sv),.target_counts_q4(target),.measured_counts(measured_old),.pwm_command(pwm_old));
 integer i,j,expected,n_new,n_old,cp_new,cp_old,last_new,last_old;
 real vnew,vold,posnew,posold,drive_new,drive_old,peak_new,err_new;
 real friction=300.0,gain=0.6,tau=0.12;
 task tick;
 begin
  @(negedge clk);sv=1;@(negedge clk);sv=0;repeat(80)@(negedge clk);
  drive_new=pwm_new>friction?(pwm_new-friction)*gain:pwm_new < -friction?(pwm_new+friction)*gain:0;
  drive_old=pwm_old>friction?(pwm_old-friction)*gain:pwm_old < -friction?(pwm_old+friction)*gain:0;
  vnew=vnew+(drive_new-vnew)*(0.01/tau);
  vold=vold+(drive_old-vold)*(0.01/tau);
  posnew=posnew+vnew*0.01*1560/(66*3.141592653589793);
  posold=posold+vold*0.01*1560/(66*3.141592653589793);
  cp_new=$rtoi(posnew);cp_old=$rtoi(posold);
  measured_new=cp_new-last_new;measured_old=cp_old-last_old;
  last_new=cp_new;last_old=cp_old;
 end endtask
 task variation(input real f,g,t,input integer command);
 begin
  rst_n=0;speed=command;friction=f;gain=g;tau=t;
  vnew=0;vold=0;posnew=0;posold=0;last_new=0;last_old=0;measured_new=0;measured_old=0;peak_new=0;
  repeat(40)@(negedge clk);rst_n=1;
  for(j=0;j<350;j=j+1)begin tick;if($abs(vnew)>peak_new)peak_new=$abs(vnew);end
  if($abs(vnew-command)>12 || peak_new>$abs(command)*1.25)
   $fatal(1,"plant variation f=%f g=%f tau=%f command=%d final=%f peak=%f",f,g,t,command,vnew,peak_new);
 end endtask
 initial begin
  repeat(40)@(negedge clk);
  for(i=-1000;i<=1000;i=i+1)begin
   speed=i;repeat(40)@(negedge clk);
   expected=$rtoi(i*1560*0.01*16/(66*3.141592653589793)+(i<0?-0.5:0.5));
   if(converted!=i || $abs(target-expected)>1)$fatal(1,"converter %d -> %d count %d expected %d",i,converted,target,expected);
  end
  speed=200;repeat(40)@(negedge clk);rst_n=1;
  vnew=0;vold=0;posnew=0;posold=0;last_new=0;last_old=0;n_new=0;n_old=0;peak_new=0;
  for(i=1;i<=500;i=i+1)begin
   tick;
   if(vnew>=190 && n_new==0)n_new=i;
   if(vold>=190 && n_old==0)n_old=i;
   if(vnew>peak_new)peak_new=vnew;
  end
  $display("INFO response 0->200: new=%d ms old=%d ms peak=%f final=%f",n_new*10,n_old*10,peak_new,vnew);
  if(n_new==0 || n_new>=n_old || n_new>800 || peak_new>240 || $abs(vnew-200)>8)$fatal(1,"startup response");
  speed=600;repeat(40)@(negedge clk);n_new=0;n_old=0;peak_new=0;
  for(i=1;i<=600;i=i+1)begin
   tick;
   if(vnew>=570 && n_new==0)n_new=i;
   if(vold>=570 && n_old==0)n_old=i;
   if(vnew>peak_new)peak_new=vnew;
  end
  $display("INFO response 200->600: new=%d ms old=%d ms (0 means >6000 ms) peak=%f final=%f",n_new*10,n_old*10,peak_new,vnew);
  if(n_new==0 || (n_old!=0 && n_new>=n_old) || n_new>1000 || peak_new>690 || $abs(vnew-600)>12)$fatal(1,"step response");
  speed=150;repeat(40)@(negedge clk);
  for(i=0;i<400;i=i+1)tick;
  if($abs(vnew-150)>8)$fatal(1,"down-step settling");
  variation(200.0,0.9,0.08,200);
  variation(450.0,0.4,0.20,400);
  variation(600.0,0.7,0.25,600);
  variation(450.0,0.5,0.18,-200);
  rst_n=0;repeat(5)@(negedge clk);if(pwm_new!=0)$fatal(1,"reset PWM");
  $display("PASS motor response: all 2001 signed speed conversions, 0->200/200->600 faster than prior PI, bounded overshoot, downstep settling, four load/friction/inertia variants including reverse, reset; independent friction/inertia plant, not physical measurements");$finish;
 end
endmodule
