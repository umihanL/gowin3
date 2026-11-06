`timescale 1ns/1ps
module navigation_modes_filter_tb;
 reg clk=0;always #5 clk=~clk;
 reg rst=0,request=0,update=0,frame_valid=0,zebra=0,pause=0,imu=1;
 reg [3:0] mode=0;
 reg [15:0] front=1500;
 reg signed [31:0] e1=0,e2=0;real travel=0;
 wire tick,drive;wire [10:0] index;wire [3:0] status;
 wire signed [15:0] speed,steer;wire signed [23:0] px,py;
 reg [16:0] rx=0,ry=0;
 always @(posedge clk)begin rx<=(index==0?500:2500)*256/10;ry<=2700*256/10;end
 vehicle_motion #(.CLK_HZ(20000),.IMU_TIMEOUT_MS(2000),.START_DELAY_MS(0)) dut(
   .clk(clk),.rst_n(rst),.mode(mode),.navigation_request(request),
   .route_valid(1'b1),.profile_ready(1'b1),.route_count(12'd2),.route_index(index),
   .route_x_q8(rx),.route_y_q8(ry),.route_lock(1'b1),.route_curve(2'sd0),
   
   
   .configured_speed(16'd200),.slowdown_cm(16'd65),.bypass_cm(16'd65),.brake_speed(16'd200),
   .encoder1(e1),.encoder2(e2),.imu_available(imu),.imu_update(imu),.imu_yaw(16'sd0),.imu_restart(1'b0),
   .range_frame(1'b1),.range_samples(4'hf),.range_observations(4'hf),.range_diagnostics(32'd0),
   .left_mm(16'd1200),.right_mm(16'd1200),.left_front_mm(front),.right_front_mm(front),
   .remote_link(1'b0),.remote_run(1'b0),.remote_speed(16'sd0),.remote_steer(16'sd0),.host_pause(pause),
   
   .zebra_update(update),.zebra_detected(zebra),
   .drive_enabled(drive),.speed_mm_s(speed),.steer_deg(steer),.status(status),.tick_10ms(tick),
   .pose_x_q8(px),.pose_y_q8(py));
 integer ticks=0,started,held_x,attempt;
 always @(negedge clk)if(rst && tick)begin
   ticks=ticks+1;
   if(drive)travel=travel+speed*0.01;
   e1=$rtoi(travel*1560.0/(66.0*3.141592654));e2=e1;
   update=frame_valid && ticks%3==0;
 end else update=0;
 task wait_ticks(input integer n);integer last;begin last=ticks+n;wait(ticks>=last);@(negedge clk);end endtask
 task reset_run;
   begin rst=0;mode=0;request=0;frame_valid=0;zebra=0;pause=0;imu=1;front=1500;travel=0;e1=0;e2=0;
     repeat(8)@(negedge clk);rst=1;wait_ticks(3);
     started=ticks;mode=9;request=1;wait_ticks(8);
     if(!drive || status!=2)$fatal(1,"camera-free start failed status=%0d state=%0d",status,dut.state);
     if(ticks-started>=200)$fatal(1,"navigation retained two-second startup delay");
   end
 endtask
 initial begin
   reset_run;wait_ticks(140);
   if(speed!=200 || steer!=0)$fatal(1,"camera-free inertial cruise failed speed=%0d steer=%0d",speed,steer);
   frame_valid=1;zebra=1;wait_ticks(45);
   if(speed!=60 || steer!=0)$fatal(1,"zebra did not slow pure navigation to 60 speed=%0d steer=%0d",speed,steer);
   frame_valid=0;wait_ticks(12);
   if(speed!=60)$fatal(1,"missing frames released zebra cap");
   frame_valid=1;zebra=0;wait_ticks(80);
   if(speed!=200 || steer!=0)$fatal(1,"clear frame did not restore original speed");
   pause=1;wait_ticks(3);held_x=px;wait_ticks(20);
   if(drive || speed || px!=held_x)$fatal(1,"host pause moved/rebased pose");
   pause=0;wait_ticks(10);if(!drive || px<held_x)$fatal(1,"resume lost route pose");
   $display("PASS pure navigation: camera-free startup, zebra 200->60->200, invalid-frame hold, steering independent, pause/resume");
   // Enter navigation while the previous automatic function is computing
   // a different target. No reset or two-second countdown may hide it.
   frame_valid=0;
   for(attempt=0;attempt<8;attempt=attempt+1)begin
     mode=7;request=0;wait_ticks(8);
     wait(dut.tracker_inflight && dut.calculation_target_x!=2500);
     repeat(attempt*2+1)@(negedge clk);
     mode=9;request=1;wait_ticks(12);
     if(!drive || status!=2 || !dut.heading_aligned)
       $fatal(1,"navigation trapped behind prior target attempt=%0d state=%0d",attempt,dut.state);
   end
   $display("PASS repeated mode handoff: eight in-flight prior targets cannot trap navigation startup");
   front=40;wait_ticks(4);if(drive || speed || status!=5)$fatal(1,"emergency obstacle check removed");
   reset_run;imu=0;wait_ticks(220);
   if(drive || speed || status!=4)$fatal(1,"IMU watchdog removed");
   $display("PASS pure navigation emergency obstacle/IMU stops");
   $finish;
 end
 initial begin #100000000;$fatal(1,"navigation modes timeout");end
endmodule
