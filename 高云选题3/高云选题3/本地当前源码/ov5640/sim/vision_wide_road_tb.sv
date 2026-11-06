`timescale 1ns/1ps
module vision_wide_road_tb;
 parameter EXTENDED=0;
 parameter NETLIST_SMOKE=0;
 parameter NOISE_PERCENT=0;
 parameter NOISE_SMOKE=0;
`ifdef GOWIN_NETLIST
 GSR GSR(.GSRI(1'b1));
`endif
 reg clk=0; always #5 clk=~clk;
 reg rst=0,fb=0,req=0,pv=0,tick=0;
 reg [11:0] x=0,y=0; reg [15:0] rgb=0;
 wire update,valid,junction; wire [2:0] visibility;
 wire signed [8:0] near_error,heading; wire signed [15:0] steer;
 wire start_ready,blocked; wire [15:0] preview;
 reg [31:0] noise_rng=32'h13579bdf;
 vision_video dut(.clk(clk),.rst_n(rst),.frame_begin(fb),.data_req(req),.pixel_valid(pv),
   .x(x),.y(y),.rgb(rgb),.threshold(8'd96),.turn_hint(2'sd0),.preview(preview),
   .road_update(update),.road_valid(valid),.road_junction(junction),
   .road_visibility(visibility),.road_near(near_error),.road_heading(heading));
 vehicle_vision_fusion fusion(.clk(clk),.rst_n(rst),.tick(tick),.enabled(1'b1),
   .road_update(update),.road_valid(valid),.road_junction(junction),.road_visibility(visibility),
   .terminal_approach(1'b0),.road_near(near_error),.road_heading(heading),.route_steer(16'sd0),
   .pose_heading(16'sd0),.travel_q8(32'sd0),.moving(1'b0),.planned_turn(2'sd0),
   .planned_exit_valid(1'b0),.planned_exit_heading(16'sd0),.route_guidance_valid(1'b1),
   .route_target_behind(1'b0),.turn_entry_ready(1'b0),.incoming_heading_error(16'sd0),
   .mapped_entry_due(1'b0),.blocked(blocked),.steering(steer),.start_ready(start_ready));
 task reset;
 begin @(negedge clk);rst=0;fb=0;req=0;pv=0;repeat(5)@(negedge clk);rst=1;end
 endtask
 // Independent pinhole geometry feeds the native 640x480 pixel interface.
 // width is a physical measurement, not the implementation's lookup table.
 task frame(input integer width_mm,offset,yaw_tenths,kind);
 integer xx,yy,n; real pitch,q,den,depth,forward,centre,half_width,theta;begin
   pitch=$atan2(185.0,395.0);theta=yaw_tenths*3.141592653589793/1800.0;
   for(yy=0;yy<480;yy=yy+1)begin
     q=(yy-239.5)/554.2562584;den=$sin(pitch)+q*$cos(pitch);
     depth=185.0/den;forward=185.0*($cos(pitch)-q*$sin(pitch))/den-15.0;
     centre=319.5+554.2562584*(2.0+offset+forward*$tan(theta))/depth;
     if(kind==4 || kind==5)centre=319.5+554.2562584*(2.0+offset+
       (kind==4?1.0:-1.0)*0.0008*(forward-100.0)*(forward-100.0))/depth;
     if(kind==6 || kind==7)centre=319.5+554.2562584*(2.0+offset+
       (forward>350.0?(kind==6?1.0:-1.0)*0.002*(forward-350.0)*(forward-350.0):0.0))/depth;
     half_width=554.2562584*width_mm/(2.0*depth);
     for(xx=0;xx<640;xx=xx+1)begin
       @(negedge clk);req=1;pv=1;x=80+xx;y=yy;
       rgb=den>0 && xx>centre-half_width && xx<centre+half_width?16'h0000:16'hffff;
       if(kind==1)rgb=16'h0000;
       if(kind==2)rgb=16'hffff;
       if(kind==3 && xx>=318 && xx<=326)rgb=16'hffff;
       noise_rng={noise_rng[30:0],noise_rng[31]^noise_rng[21]^noise_rng[1]^noise_rng[0]};
       if(NOISE_PERCENT!=0 && noise_rng%100<NOISE_PERCENT)rgb=~rgb;
     end
   end
   @(negedge clk);req=0;pv=0;fb=1;@(negedge clk);fb=0;
   wait(update);@(negedge clk);
   for(n=0;n<16;n=n+1)begin tick=1;@(negedge clk);tick=0;@(negedge clk);end
   $display("frame width=%0d offset=%0d yaw=%0d kind=%0d valid=%b edges=%b near=%0d heading=%0d steer=%0d ready=%b",width_mm,offset,yaw_tenths,kind,valid,visibility,near_error,heading,steer,start_ready);
 end endtask
 task check(input integer width_mm,offset,yaw_tenths);
 integer target,heading_target,near_delta,heading_delta;begin
   reset;repeat(4)frame(width_mm,offset,yaw_tenths,0);
   target=offset+$rtoi(210.0*$tan(yaw_tenths*3.141592653589793/1800.0));
   heading_target=$rtoi(110.0*$tan(yaw_tenths*3.141592653589793/1800.0));
   near_delta=$signed(near_error)-target;heading_delta=$signed(heading)-heading_target;
   if(!valid || blocked || !start_ready || visibility[1:0]==0)
     $fatal(1,"widened lane cannot qualify navigation start");
   if(near_delta>12 || near_delta < -12 || heading_delta>8 || heading_delta < -8)
     $fatal(1,"widened lane metric error target=%0d near=%0d heading=%0d",target,near_error,heading);
   if(offset==0 && yaw_tenths==0 && (visibility!=7 || junction || steer>1 || steer < -1))
     $fatal(1,"centred wide straight was mistaken for bend/junction or preloaded steering");
   if(offset>=30 && steer>=0 || offset<=-30 && steer<=0)
     $fatal(1,"lane correction sign incorrect");
   $display("PASS width=%0d offset=%0d yaw=%0d",width_mm,offset,yaw_tenths);
 end endtask
 initial begin
   if(NOISE_SMOKE)begin
     check(340,30,0);check(340,-30,0);check(373,30,0);check(373,-30,0);
     $display("PASS native wide-road video with %0d percent salt/pepper pixels",NOISE_PERCENT);$finish;
   end
   if(NETLIST_SMOKE)begin
     check(373,30,0);
     reset;frame(340,0,0,0);
     if(!valid || near_error>8 || near_error < -8 || visibility!=7)
       $fatal(1,"synthesized inner road centre or visibility mismatch");
     $display("PASS synthesized measured wide-road centre and correction");$finish;
   end
   if(EXTENDED)begin
     // Width changes must not bias the centre or invent a junction.
     reset;repeat(4)frame(340,0,0,0);repeat(3)frame(373,0,0,0);
     if(!valid || visibility!=7 || junction || near_error>6 || near_error < -6)
       $fatal(1,"inner-to-outer width change invented centring error");
     repeat(3)frame(340,0,0,0);
     if(!valid || visibility!=7 || junction || near_error>6 || near_error < -6)
       $fatal(1,"outer-to-inner width change invented centring error");
     reset;frame(340,0,0,4);
     if(!valid || heading<=0)$fatal(1,"right-curved inner road lost its direction");
     reset;frame(340,0,0,5);
     if(!valid || heading>=0)$fatal(1,"left-curved inner road lost its direction");
     reset;frame(373,0,0,4);
     if(!valid || heading<=0)$fatal(1,"right-curved outer road lost its direction");
     reset;frame(373,0,0,5);
     if(!valid || heading>=0)$fatal(1,"left-curved outer road lost its direction");
     // A future bend may be visible without authorising a turn now.
     reset;frame(340,0,0,6);
     if(!valid || near_error>8 || near_error < -8 || heading>8 || heading < -8)
       $fatal(1,"future right bend pulled visible straight lane");
     reset;frame(340,0,0,7);
     if(!valid || near_error>8 || near_error < -8 || heading>8 || heading < -8)
       $fatal(1,"future left bend pulled visible straight lane");
     $display("PASS widened lane transitions and both curved contours");$finish;
   end
   check(340,0,0);check(373,0,0);
   check(340,30,0);check(340,-30,0);check(373,30,0);check(373,-30,0);
   check(340,60,0);check(340,-60,0);check(373,60,0);check(373,-60,0);
   check(340,0,50);check(340,0,-50);check(373,0,50);check(373,0,-50);
   reset;frame(340,0,0,3);
   if(!valid || near_error>8 || near_error < -8)$fatal(1,"painted stripe split widened road");
   frame(373,0,0,1);if(valid || visibility!=4)$fatal(1,"all-dark image invented boundaries");
   frame(373,0,0,2);if(valid || visibility!=4)$fatal(1,"all-light image accepted");
   $display("PASS widened road native video and startup/correction integration");$finish;
 end
 initial begin #400000000;$fatal(1,"wide road test timeout");end
endmodule
