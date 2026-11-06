`timescale 1ns/1ps
module servo_pwm_tenths_tb;
 reg clk=0;always #5 clk=~clk;
 reg rst=0;reg [10:0] angle=900;wire pulse;
 servo_pwm #(.CLK_HZ(33000000)) dut(.clk(clk),.rst_n(rst),.angle_tenths(angle),.pwm_out(pulse));
 integer n,start,cycle=0,i,expected;
 always @(posedge clk)cycle=cycle+1;
 task measure(input integer value);integer width;begin
  angle=value;repeat(8)@(negedge clk);
  // Fast-forward idle time only. A real rollover must load the requested
  // width; measure every output pulse, and one full frame without a jump.
  @(negedge clk);dut.frame_count=659999;
  @(posedge pulse);start=cycle;@(negedge pulse);width=cycle-start;
  expected=33*(500+(2000*(value>1800?1800:value)+900)/1800);
  if(width!=expected)$fatal(1,"33MHz pulse angle=%0d width=%0d expected=%0d",value,width,expected);
  if(value==900)begin @(posedge pulse);if(cycle-start!=660000)$fatal(1,"20ms frame changed");end
 end endtask
 initial begin
  repeat(5)@(negedge clk);rst=1;repeat(5)@(negedge clk);
  for(i=0;i<2048;i=i+1)if(dut.pulse_rom[i]!=(500+(2000*(i>1800?1800:i)+900)/1800))
   $fatal(1,"PWM ROM %0d",i);
  measure(900);measure(901);measure(923);measure(423);measure(1523);measure(0);measure(1800);measure(2047);
  // Mid-pulse changes must not truncate or stretch the pulse already active.
  angle=900;repeat(8)@(negedge clk);dut.frame_count=659999;@(posedge pulse);start=cycle;
  repeat(33000)@(negedge clk);angle=0;
  @(negedge pulse);if(cycle-start!=49500)$fatal(1,"mid-frame command glitched active pulse");
  @(posedge pulse);start=cycle;@(negedge pulse);
  if(cycle-start!=16500)$fatal(1,"next frame did not apply request");
  rst=0;#1;if(pulse)$fatal(1,"reset PWM must be low");
  $display("PASS PWM at production 33MHz: all 2048 ROM entries, 20ms frames, 0.1-degree distinct pulses, full 0..180 limits, frame-boundary updates, reset");$finish;
 end
 initial begin #400000000;$fatal(1,"PWM timeout");end
endmodule
