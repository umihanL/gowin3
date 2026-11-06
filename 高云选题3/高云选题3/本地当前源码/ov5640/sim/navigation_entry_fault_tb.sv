`timescale 1ns/1ps
// Public-port reproductions: future visual bend, reversed pursuit, exit bias.
module navigation_entry_fault_tb;
 parameter CASE=0;
 reg clk=0;always #5 clk=~clk;
 reg rst=0,tick=0,update=0,valid=1,moving=0,entry=0,due=0;
 reg [2:0] visibility=7;
 reg signed [15:0] route=0,yaw=0,exit_heading=900;
 reg signed [8:0] near_error=0,heading_error=0;
 reg signed [1:0] planned=0;
 reg signed [31:0] travel=0;
 wire blocked,active,recovery;wire signed [15:0] steer;
 vehicle_vision_fusion dut(.clk(clk),.rst_n(rst),.tick(tick),.enabled(1'b1),
 .road_update(update),.road_valid(valid),.road_junction(1'b0),.road_visibility(visibility),
 .terminal_approach(1'b0),.road_near(near_error),.road_heading(heading_error),
 .route_steer(route),.pose_heading(yaw),.travel_q8(travel),.moving(moving),
 .planned_turn(planned),.planned_exit_valid(1'b1),.planned_exit_heading(exit_heading),
`ifndef BASELINE
 .turn_entry_ready(entry),.mapped_entry_due(due),.incoming_heading_error(16'sd0),
`endif
 .route_guidance_valid(1'b1),.blocked(blocked),.active(active),.recovery_active(recovery),.steering(steer));
 integer n=0,k,side,applied;real lateral,angle,ds,kappa,oldangle;
 task step;begin
  @(negedge clk);tick=1;update=n%3==0;n=n+1;
  @(negedge clk);tick=0;update=0;
 end endtask
 task steps(input integer count);integer j;begin for(j=0;j<count;j=j+1)step;end endtask
 initial begin
  for(side=-1;side<=1;side=side+2)begin
   rst=0;n=0;entry=0;due=0;moving=0;valid=1;visibility=7;route=0;yaw=0;
   exit_heading=side*900;planned=0;near_error=0;heading_error=0;travel=0;
   repeat(5)@(negedge clk);rst=1;steps(30);moving=1;
   if(CASE==0)begin
    planned=side;route=side*28;near_error=-side*40;heading_error=-side*20;
    steps(90);
    if(blocked || recovery || $abs(steer)>1)$fatal(1,"FUTURE_BEND_EARLY_LOCK side=%0d steer=%0d",side,steer);
    // Actual entry is due even if the visible junction still looks straight
    // or shows the wrong branch. The route must choose the signed exit.
    entry=1;due=1;near_error=side*40;heading_error=side*20;steps(55);
    if(blocked || !recovery || steer*side<15)$fatal(1,"JUNCTION_DID_NOT_ENTER_PLANNED_TURN side=%0d steer=%0d",side,steer);
    $display("PASS fixed entry side=%0d: future curved image holds incoming heading; measured entry selects planned branch",side);
   end else begin
    planned=side;route=side*28;valid=0;visibility=6;entry=1;due=1;steps(60);
    if(blocked || !recovery)$fatal(1,"corner setup failed");
    if(CASE==1)begin
     yaw=side*450;route=-side*28;steps(60);
     if(blocked || !recovery || steer*side<15)$fatal(1,"PLANNED_TURN_REVERSED side=%0d steer=%0d",side,steer);
     $display("PASS signed corner side=%0d: behind-target pursuit cannot reverse the committed turn",side);
    end else begin
     yaw=side*900;valid=1;visibility=side==1?5:6;near_error=side*80;heading_error=0;
     planned=0;route=0;due=0;steps(35);
     if(blocked || recovery || !active || steer*side>=0)$fatal(1,"EXIT_CENTRE_NOT_ACQUIRED");
     // Independent rear-axle bicycle; the controlled point is 219mm ahead.
     // Keep observations in physical mm rather than feeding DUT internals.
     lateral=side*80;angle=0;applied=steer;
     for(k=0;k<1200;k=k+1)begin
      near_error=$rtoi(lateral+210*angle);heading_error=$rtoi(110*angle);
      yaw=side*900+$rtoi(angle*1800/3.141592654);step;
      if(applied<steer)applied=applied+1;else if(applied>steer)applied=applied-1;
      kappa=$tan(applied*0.01745329252)/142;ds=1;oldangle=angle;angle=angle+ds*kappa;
      lateral=lateral+ds*$sin((oldangle+angle)/2)+219*($sin(angle)-$sin(oldangle));
      travel=travel+256;
      if(blocked)$fatal(1,"EXIT_CENTRING_ABORTED");
     end
     if($abs(lateral)>5 || $abs(angle)>0.04)$fatal(1,"EXIT_RESIDUAL side=%0d mm=%f yaw=%f",side,lateral,angle);
     $display("PASS physical exit centring side=%0d: 80mm initial offset converges to %f mm",side,lateral);
    end
   end
  end
  $finish;
 end
 initial begin #10000000;$fatal(1,"entry fault timeout");end
endmodule
