`timescale 1ns/1ps
// Compare identical encoder/IMU samples against analytic wheel-odometry paths.
// This validates numerical integration, not tyre slip or physical calibration.
module inertial_accuracy_tb;
 reg clk=0;always #5 clk=~clk;
 reg rst=0,tick=0,rebase=0,fresh=0,restarted=0;
 reg signed [23:0] ix=0,iy=0;
 reg signed [15:0] ih=0,yaw=0;
 reg signed [31:0] e1=0,e2=0;
 wire signed [23:0] x,y,rx,ry,bx,by,brx,bry;
 wire signed [15:0] heading,bheading;
 wire signed [31:0] travel,btravel;
 wire signed [16:0] cs,sn,bcs,bsn;
 wire updated,bupdated,fault,bfault;
 vehicle_odometry #(.FRONT_REFERENCE(1)) dut(.clk(clk),.rst_n(rst),.tick(tick),.rebase(rebase),
   .imu_fresh(fresh),.imu_restarted(restarted),.initial_x_q8(ix),.initial_y_q8(iy),
   .initial_heading(ih),.imu_yaw(yaw),.encoder1(e1),.encoder2(e2),
   .x_q8(x),.y_q8(y),.rear_x_q8(rx),.rear_y_q8(ry),.heading(heading),.travel_q8(travel),
   .updated(updated),.encoder_fault(fault),.cos_heading(cs),.sin_heading(sn));
 vehicle_odometry_before #(.FRONT_REFERENCE(1)) baseline(.clk(clk),.rst_n(rst),.tick(tick),.rebase(rebase),
   .imu_fresh(fresh),.imu_restarted(restarted),.initial_x_q8(ix),.initial_y_q8(iy),
   .initial_heading(ih),.imu_yaw(yaw),.encoder1(e1),.encoder2(e2),
   .x_q8(bx),.y_q8(by),.rear_x_q8(brx),.rear_y_q8(bry),.heading(bheading),.travel_q8(btravel),
   .updated(bupdated),.encoder_fault(bfault),.cos_heading(bcs),.sin_heading(bsn));
 localparam real PI=3.141592653589793;
 localparam real MM_COUNT=66.0*PI/1560.0;
 real wx,wy,wh,distance_ref,turn,previous_h,expected_travel;
 real er,ber,tx,ty;
 integer latency,max_latency=0,checks=0,i,j,s,n;
 reg seen,base_seen;
 function real rabs(input real value);begin rabs=value<0?-value:value;end endfunction
 task wait_both;begin
   latency=0;seen=0;base_seen=0;
   while(!(seen&&base_seen) && latency<120)begin
     @(negedge clk);latency=latency+1;
     if(updated)seen=1;
     if(bupdated)base_seen=1;
   end
   if(!(seen&&base_seen))$fatal(1,"odometry sample not complete in120clocks");
   if(latency>max_latency)max_latency=latency;
 end endtask
 task origin(input integer direction);begin
   tick=0;rebase=0;fresh=0;restarted=0;ih=direction;yaw=0;
   @(negedge clk);rebase=1;@(negedge clk);rebase=0;wait_both();
   wh=direction*PI/1800.0;
   // Each implementation's initial nose offset is checked separately below.
   wx=$itor($signed(rx))/256.0;wy=$itor($signed(ry))/256.0;
   expected_travel=0;
   if(dut.distance_remainder || dut.x_remainder || dut.y_remainder)
     $fatal(1,"rebase retained a fractional residual");
 end endtask
 task sample(input integer c1,c2);begin
   previous_h=wh;distance_ref=(c1+c2)*0.5*MM_COUNT;turn=(c2-c1)*MM_COUNT/168.0;
   wh=wh+turn;
   if(turn==0)begin wx=wx+distance_ref*$cos(wh);wy=wy+distance_ref*$sin(wh);end
   else begin
     wx=wx+distance_ref/turn*($sin(wh)-$sin(previous_h));
     wy=wy-distance_ref/turn*($cos(wh)-$cos(previous_h));
   end
   expected_travel=expected_travel+distance_ref;
   @(negedge clk);e1=e1+c1;e2=e2+c2;tick=1;
   @(negedge clk);tick=0;wait_both();
   if(fault||bfault)$fatal(1,"ordinary encoder stimulus faulted");
   // Endpoint direction must describe current heading, never midpoint heading.
   if(rabs($itor($signed(cs))-32768.0*$cos($itor($signed(heading))*PI/1800.0))>8 ||
      rabs($itor($signed(sn))-32768.0*$sin($itor($signed(heading))*PI/1800.0))>8)
     $fatal(1,"endpoint CORDIC direction mismatch h=%0d cs=%0d sn=%0d expected=%0d,%0d",heading,cs,sn,$rtoi(32768.0*$cos($signed(heading)*PI/1800.0)),$rtoi(32768.0*$sin($signed(heading)*PI/1800.0)));
   if(rabs($itor($signed(x))-$itor($signed(rx))-219.0*256.0*$cos($itor($signed(heading))*PI/1800.0))>16 ||
      rabs($itor($signed(y))-$itor($signed(ry))-219.0*256.0*$sin($itor($signed(heading))*PI/1800.0))>16)
     $fatal(1,"nose output used midpoint instead of endpoint heading");
 end endtask
 task metric(input string name,input real max_error,input integer require_better);begin
   tx=$itor($signed(rx))/256.0-wx;ty=$itor($signed(ry))/256.0-wy;er=$sqrt(tx*tx+ty*ty);
   tx=$itor($signed(brx))/256.0-wx;ty=$itor($signed(bry))/256.0-wy;ber=$sqrt(tx*tx+ty*ty);
   $display("METRIC %s old_mm=%0.6f new_mm=%0.6f travel_old_error_mm=%0.6f travel_new_error_mm=%0.6f",
      name,ber,er,$itor($signed(btravel))/256.0-expected_travel,$itor($signed(travel))/256.0-expected_travel);
   if(er>max_error)$fatal(1,"new integration exceeds independent path tolerance %s",name);
   if(require_better && er>=ber*0.35)$fatal(1,"integration did not improve materially %s",name);
   checks=checks+1;
 end endtask
 initial begin
   repeat(5)@(negedge clk);rst=1;
   // Slow travel creates maximum opportunities for repeated fractional loss.
   origin(0);for(i=0;i<10000;i=i+1)sample(1,1);metric("slow_forward_1.329m",0.5,1);
   for(i=0;i<10000;i=i+1)sample(-1,-1);metric("slow_forward_reverse_closure",0.02,1);
   origin(370);for(i=0;i<5000;i=i+1)sample(7,7);metric("diagonal_forward_4.652m",1.0,1);
   origin(-370);for(i=0;i<5000;i=i+1)sample(-7,-7);metric("diagonal_reverse_4.652m",1.0,1);
   // Analytic constant-radius arcs with physically consistent wheel yaw.
   origin(0);for(i=0;i<400;i=i+1)sample(10,16);metric("left_arc_108deg",0.4,1);
   origin(0);for(i=0;i<400;i=i+1)sample(16,10);metric("right_arc_108deg",0.4,1);
   origin(1790);for(i=0;i<200;i=i+1)sample(10,16);metric("positive_180_wrap",0.4,1);
   origin(-1790);for(i=0;i<200;i=i+1)sample(16,10);metric("negative_180_wrap",0.4,1);
   origin(0);for(i=0;i<400;i=i+1)sample(-10,-16);metric("reverse_arc",0.4,1);
   // Reset/rebase must not inherit the previous run's fractional history.
   origin(0);sample(1,1);@(negedge clk);rst=0;repeat(3)@(negedge clk);rst=1;
   if(dut.distance_remainder || dut.x_remainder || dut.y_remainder)$fatal(1,"reset residual");
   origin(0);sample(0,0);metric("reset_stationary",0.02,0);
   // IMU correction path, zero movement and restart: retain valid endpoint and no false translation.
   fresh=1;yaw=10;for(i=0;i<20;i=i+1)sample(0,0);
   if($abs($signed(heading)-10)>1)$fatal(1,"IMU correction stopped working");
   restarted=1;yaw=700;sample(0,0);restarted=0;
   if($abs($signed(heading)-10)>1)$fatal(1,"IMU restart reset physical heading");
   if(max_latency>=100)$fatal(1,"sample latency exceeded100clocks");
   $display("PASS numerical inertial accuracy cases=%0d max_latency_clocks=%0d; no hardware",checks,max_latency);
   $finish;
 end
 initial begin #100000000;$fatal(1,"timeout");end
endmodule
