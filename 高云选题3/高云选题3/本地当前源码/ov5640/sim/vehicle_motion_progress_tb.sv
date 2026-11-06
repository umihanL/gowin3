`timescale 1ns/1ps
module vehicle_motion_progress_tb;
initial begin repeat(10) begin #1000000; $display("PROGRESS t=%0t ticks=%0d mode=%0d state=%0d status=%0d x=%f",$time,ticks,mode,dut.state,status,world_x); end $finish; end
    reg clk=0;always #5 clk=~clk;
    reg rst_n=0;reg [3:0] mode=0;reg request=0,route_valid=1;
    wire [10:0] index;reg [16:0] rx,ry;reg route_lock;
    reg [16:0] path_x[0:255],path_y[0:255];reg locks[0:255];reg [11:0] count=0;
    always @(posedge clk)begin rx<=path_x[index];ry<=path_y[index];route_lock<=locks[index];end
    reg signed [31:0] encoder1=0,encoder2=0;
    reg imu_available=1,imu_update=1;reg signed [15:0] yaw=0;
    reg [15:0] slowdown=65,cruise=200,bypass_setting=65,brake_setting=200;reg profile_ready=1;
    reg range_frame=1;reg [31:0] diagnostics=0;
    reg [3:0] sample_suppress=0;
    reg [15:0] left=1200,lf=1500,rf=1500,right=1200;
    reg [3:0] drop_mask=0;reg [7:0] drop_code=8'h11;
    reg pulsed_frames=0;integer frame_clock=0;
    // Real scan completion is a one-clock pulse asynchronous to the 10 ms
    // controller tick. It must not be lost by the tail-confirmation logic.
    always @(negedge clk)if(pulsed_frames)begin
        range_frame=frame_clock==17;
        frame_clock=(frame_clock+1)%1200;
    end
    reg remote_link=0,remote_run=0;reg signed [15:0] remote_speed=0,remote_steer=0;
    wire drive,tick,fvalid;wire signed [15:0] speed,steer,heading,target_x,target_y,bearing;
    wire signed [23:0] px,py;wire signed [31:0] relative1,relative2;
    wire [3:0] status,phase;wire [15:0] avg,target_distance;
    // Clock-scaled simulation; preserve 10 ms plant steps and allow >100
    // clocks per update for the actual sequential CORDIC/RAM pipelines.
    vehicle_motion #(.REQUIRE_VISION(0),.CLK_HZ(20000),.STALL_MS(3000)) dut(.route_exit_valid(1'b0),.route_exit_heading(16'sd0),.route_curve(2'sd0),.route_upcoming_turn(2'sd0),.route_entry_mm(8'd0),.route_tangent(16'sd0),.imu_restart(1'b0),.clk(clk),.rst_n(rst_n),.mode(mode),
        .configured_speed(cruise),.bypass_cm(bypass_setting),.brake_speed(brake_setting),.slowdown_cm(slowdown),.profile_ready(profile_ready),
        .navigation_request(request),.route_valid(route_valid),.route_count(count),
        .route_index(index),.route_x_q8(rx),.route_y_q8(ry),.route_lock(route_lock),
        .encoder1(encoder1),.encoder2(encoder2),.imu_available(imu_available),.imu_update(imu_update),.imu_yaw(yaw),
        .road_visibility(3'b000),.road_update(1'b0),.road_valid(1'b0),.road_junction(1'b0),.road_near(9'sd0),.road_heading(9'sd0),
        .zebra_update(1'b0),.zebra_detected(1'b0),
        .range_observations(range_frame?4'hf:4'h0),.range_frame(range_frame),.range_samples(range_frame?(4'hf & ~sample_suppress):4'h0),.range_diagnostics(diagnostics),
        .left_mm(left),.left_front_mm(lf),.right_front_mm(rf),.right_mm(right),
        .remote_link(remote_link),.remote_run(remote_run),.remote_speed(remote_speed),.remote_steer(remote_steer),
        .drive_enabled(drive),.speed_mm_s(speed),.steer_deg(steer),.status(status),.phase(phase),
        .front_average_mm(avg),.front_valid(fvalid),.pose_x_q8(px),.pose_y_q8(py),.pose_heading(heading),
        .encoder_relative1(relative1),.encoder_relative2(relative2),.target_x_mm(target_x),.target_y_mm(target_y),
        .target_heading(bearing),.target_distance_mm(target_distance),.tick_10ms(tick));
    real world_x=0,world_y=0,world_h=0,traveled=0,wheel1=0,wheel2=0;
    real ds,curvature,ratio,old_h;
    real bx,by,cx,cy,ac,as_,sep_x,sep_y,sep_l,sep_w;
    real box_x0=1100,box_x1=1400,box_y0=-70,box_y1=70;
    real box_cx,box_cy,box_l,box_w;
    reg overlap;
    task check_body;
      begin
        // Independent measured SAT body: 320x200, rear 101, front 219 mm.
        // Rear axle is the pose reference; obstacle is x=1100..1400,y=-70..70.
        ac=$cos(world_h);as_=$sin(world_h);
        bx=world_x+59*ac;by=world_y+59*as_;
        box_cx=(box_x0+box_x1)/2;box_cy=(box_y0+box_y1)/2;
        box_l=(box_x1-box_x0)/2;box_w=(box_y1-box_y0)/2;
        cx=bx-box_cx;cy=by-box_cy;
        sep_x=$abs(cx)-(box_l+160*$abs(ac)+100*$abs(as_));
        sep_y=$abs(cy)-(box_w+160*$abs(as_)+100*$abs(ac));
        sep_l=$abs(cx*ac+cy*as_)-(160+box_l*$abs(ac)+box_w*$abs(as_));
        sep_w=$abs(-cx*as_+cy*ac)-(100+box_l*$abs(as_)+box_w*$abs(ac));
        overlap=sep_x<0 && sep_y<0 && sep_l<0 && sep_w<0;
        if(overlap)$fatal(1,"body collision x=%f y=%f h=%f phase=%d",world_x,world_y,world_h,phase);
      end
    endtask

    function real beam;
        input real ox,oy,dx,dy;
        real near_t,far_t,a,b,tmp;
        begin
            near_t=0;far_t=2000;
            if($abs(dx)<0.000001)begin if(ox<box_x0 || ox>box_x1)far_t=-1;end
            else begin
                a=(box_x0-ox)/dx;b=(box_x1-ox)/dx;
                if(a>b)begin tmp=a;a=b;b=tmp;end
                if(a>near_t)near_t=a;if(b<far_t)far_t=b;
            end
            if($abs(dy)<0.000001)begin if(oy<box_y0 || oy>box_y1)far_t=-1;end
            else begin
                a=(box_y0-oy)/dy;b=(box_y1-oy)/dy;
                if(a>b)begin tmp=a;a=b;b=tmp;end
                if(a>near_t)near_t=a;if(b<far_t)far_t=b;
            end
            beam=far_t>=near_t && far_t>=0?near_t:65535;
        end
    endfunction
    real ch,sh;
    integer actual_steer=0;
    integer model=0,ticks=0,last_speed=0,speed_step=0,first_motion_tick=0,arm_tick=0;
    reg simulate_wheels=1,freeze_encoder1=0,reverse_feedback=0;
    integer yaw_bias=0;
    integer base_count1=0,base_count2=0;
    // Independent bicycle plant. Counts are quantized from travelled wheel
    // distance and IMU reports physical orientation; no DUT pose is fed back.
    always @(negedge clk)if(rst_n && tick)begin
        ticks=ticks+1;
        if(actual_steer<steer)actual_steer=actual_steer+1;
        else if(actual_steer>steer)actual_steer=actual_steer-1;
        if(drive && speed!=0 && simulate_wheels)begin
            curvature=$sin(actual_steer*3.141592653589793/180)/
                $cos(actual_steer*3.141592653589793/180)/142;
            // The subsystem requests outer-wheel speed. Rear-axle centre
            // speed is lower on a turn, using the physical 168 mm track.
            ds=speed*0.01/(1+84*$abs(curvature));
            old_h=world_h;world_h=world_h+ds*curvature;
            world_x=world_x+ds*$cos((old_h+world_h)/2);
            world_y=world_y+ds*$sin((old_h+world_h)/2);
            wheel1=wheel1+ds*(1-84*curvature);wheel2=wheel2+ds*(1+84*curvature);
            traveled=traveled+ds;
        end
        yaw=$rtoi(world_h*1800/3.141592653589793)+yaw_bias;
        while(yaw>1800)yaw=yaw-3600;
        while(yaw < -1800)yaw=yaw+3600;
        if(!freeze_encoder1)encoder1=base_count1+(reverse_feedback?-1:1)*$rtoi(wheel1*1560/(66*3.141592653589793));
        encoder2=base_count2+$rtoi(wheel2*1560/(66*3.141592653589793));
        if(drive && speed>0 && first_motion_tick==0)first_motion_tick=ticks;
        speed_step=$signed(speed);speed_step=speed_step-last_speed;
        if((mode==7 || mode==8 || mode==9) && status==2 && (speed_step>2 || speed_step < (mode==7?(brake_setting<=120?-2:-40):-2)))
            $fatal(1,"speed step %d -> %d",last_speed,speed);
        last_speed=speed;
        if(model==1)begin
            lf=900-world_x;rf=lf;
        end
        if(model==2 || model==3)begin
            ch=$cos(world_h);sh=$sin(world_h);
            lf=$rtoi(beam(world_x+189*ch-56*sh,world_y+189*sh+56*ch,ch,sh));
            rf=$rtoi(beam(world_x+189*ch+56*sh,world_y+189*sh-56*ch,ch,sh));
            left=$rtoi(beam(world_x+109*ch-72*sh,world_y+109*sh+72*ch,-sh,ch));
            right=$rtoi(beam(world_x+109*ch+72*sh,world_y+109*sh-72*ch,sh,-ch));
            if(model==3 && !dut.bypass_active)right=200;
            diagnostics={(right==65535?8'h11:8'h00),(rf==65535?8'h11:8'h00),
                (lf==65535?8'h11:8'h00),(left==65535?8'h11:8'h00)};
            check_body;
        end
        if(drop_mask[0])begin left=65535;diagnostics[7:0]=drop_code;end
        if(drop_mask[1])begin lf=65535;diagnostics[15:8]=drop_code;end
        if(drop_mask[2])begin rf=65535;diagnostics[23:16]=drop_code;end
        if(drop_mask[3])begin right=65535;diagnostics[31:24]=drop_code;end
    end
    task wait_ticks;
        input integer n;integer stop_tick;
        begin stop_tick=ticks+n;wait(ticks>=stop_tick);@(negedge clk);end
    endtask
    task leave;
        begin
            @(negedge clk);mode=0;request=0;repeat(5)@(negedge clk);
            if(drive || speed!=0)$fatal(1,"menu exit must stop");
        end
    endtask
    task reset_plant;
        begin
            leave;world_x=0;world_y=0;world_h=0;traveled=0;wheel1=0;wheel2=0;
            actual_steer=0;yaw=0;encoder1=0;encoder2=0;lf=1500;rf=1500;left=1200;right=1200;
            last_speed=0;first_motion_tick=0;model=0;simulate_wheels=1;
            freeze_encoder1=0;reverse_feedback=0;yaw_bias=0;base_count1=0;base_count2=0;
            pulsed_frames=0;frame_clock=0;range_frame=1;imu_update=1;imu_available=1;diagnostics=0;drop_mask=0;drop_code=8'h11;sample_suppress=0;
        end
    endtask
    task launch;
        input integer next_mode;
        begin @(negedge clk);mode=next_mode;request=next_mode==9;arm_tick=ticks;end
    endtask
    task await_finish;
        input integer limit;integer timeout_tick;
        begin
            timeout_tick=ticks+limit;
            while(status<3 && ticks<timeout_tick)@(negedge clk);
            if(status!=3)$fatal(1,"run failed status=%d mode=%d phase=%d pos=%f,%f head=%f target=%d,%d index=%d state=%d speed=%d dist=%d",status,mode,phase,world_x,world_y,world_h*180/3.14159265,target_x,target_y,index,dut.state,speed,target_distance);
            wait_ticks(4);
            if(speed || drive)$fatal(1,"finished run moving");
        end
    endtask
    task await_status;
        input integer expected,limit;integer timeout_tick;
        begin
            timeout_tick=ticks+limit;
            while(status<3 && ticks<timeout_tick)@(negedge clk);
            if(status!=expected)$fatal(1,"expected status %d got %d, phase %d x %f",expected,status,phase,world_x);
            wait_ticks(4);
            if(drive || speed)$fatal(1,"fault/blocked run still moving");
        end
    endtask
    integer i,count_error;real theta;
    initial begin
        for(i=0;i<256;i=i+1)begin path_x[i]=0;path_y[i]=0;locks[i]=0;end
        repeat(4)@(negedge clk);rst_n=1;wait_ticks(2);
        mode=6;wait_ticks(10);if(drive || speed)$fatal(1,"range-only page drives");
        lf=123;rf=456;repeat(4)@(negedge clk);
        if(avg!=289 || !fvalid)$fatal(1,"front arithmetic mean");
        rf=65535;repeat(4)@(negedge clk);if(fvalid)$fatal(1,"invalid front accepted");
        reset_plant;mode=2;remote_link=1;remote_run=1;remote_speed=180;remote_steer=-10;
        repeat(5)@(negedge clk);if(!drive || speed!=180 || steer!=-10)$fatal(1,"remote arbitration");
        remote_link=0;repeat(5)@(negedge clk);if(drive || speed)$fatal(1,"remote lost");
        reset_plant;model=1;launch(7);wait_ticks(190);if(drive || speed)$fatal(1,"two second delay");
        await_finish(1500);
        if(first_motion_tick-arm_tick<200)$fatal(1,"started before two seconds");
        if(world_x<650 || world_x>850)$fatal(1,"stop location %f",world_x);
        wait_ticks(100);if(drive)$fatal(1,"automatic restart");
        $display("PASS gradual stop: real two-second delay, slew limit, 200 mm/s cruise, range slowdown and no restart");
        reset_plant;launch(7);wait_ticks(230);imu_available=0;imu_update=0;wait_ticks(10);
        if(status!=2 || !drive)$fatal(1,"brief IMU transport loss stopped motion");
        wait_ticks(890);
        if(status!=2 || !drive)$fatal(1,"nine-second IMU loss stopped motion");
        wait_ticks(110);
        if(status!=4 || drive || speed)$fatal(1,"sustained 10-second IMU loss does not stop");imu_available=1;imu_update=1;
        reset_plant;launch(7);wait_ticks(230);lf=20;rf=1000;wait_ticks(2);
        if(status!=5 || drive)$fatal(1,"nearest sensor emergency hidden by mean");
        reset_plant;launch(7);wait_ticks(230);range_frame=0;imu_update=0;wait_ticks(1010);
        if(status!=4 || drive)$fatal(1,"stale data does not stop");range_frame=1;imu_update=1;
        reset_plant;launch(7);simulate_wheels=0;wait_ticks(650);
        if(status!=9 || drive)$fatal(1,"stall detection %d",status);
        reset_plant;launch(7);freeze_encoder1=1;wait_ticks(650);
        if(status!=9 || drive)$fatal(1,"single encoder loss %d",status);
        $display("PASS mode exits, IMU fault, stale range/yaw, single-beam emergency, encoder stall");
        reset_plant;
        // Straight planned path, world origin (500,500), nose-reference points.
        count=51;
        for(i=0;i<count;i=i+1)begin path_x[i]=(500+i*20)*256/10;path_y[i]=(3200-500)*256/10;locks[i]=(i==0 || i==25 || i==50);end
        route_valid=0;launch(9);wait_ticks(5);
        if(status!=7 || drive)$fatal(1,"unplanned navigation started");
        leave;route_valid=1;
        base_count1=123456;base_count2=-4567;wait_ticks(5);
        launch(9);await_finish(1800);
        if(world_x<970 || world_x>1030 || $abs(world_y)>25)$fatal(1,"straight destination");
        count_error=$signed(relative1);count_error=count_error-$rtoi(traveled*1560/(66*3.141592653589793));
        if(count_error>5 || count_error < -5)$fatal(1,"relative count origin count=%d traveled=%f error=%d",relative1,traveled,count_error);
        $display("PASS route execution: ordered path RAM, start coordinates, locked waypoint, encoder origin and endpoint");
        // A vertical straight has a 90-degree map tangent. Align odometry to
        // that tangent before motion and ensure the initial command is straight.
        reset_plant;count=51;
        for(i=0;i<count;i=i+1)begin path_x[i]=500*256/10;path_y[i]=(3200-500-i*20)*256/10;locks[i]=(i==0 || i==25 || i==50);end
        world_h=1.5707963267948966;yaw=900;launch(9);wait(drive);wait_ticks(20);
        if($abs(steer)>3 || $abs(actual_steer)>3)$fatal(1,"vertical straight launch steered %0d/%0d initial=%0d",steer,actual_steer,dut.initial_heading);
        await_finish(1800);
        if($abs(world_x)>25 || world_y<970 || world_y>1030)$fatal(1,"vertical straight drift %f,%f",world_x,world_y);
        $display("PASS vertical straight startup: route tangent rebased before drive, no launch turn and bounded lateral drift");
        reset_plant;launch(9);wait_ticks(230);route_valid=0;wait_ticks(1);
        if(status!=7 || drive)$fatal(1,"invalidated moving route did not stop");route_valid=1;
        reset_plant;
        // Smooth 90 degree left arc radius 300 mm then straight north.
        count=65;
        for(i=0;i<40;i=i+1)begin
            theta=i*3.141592653589793/78;
            path_x[i]=$rtoi((500+300*$sin(theta))*256/10);
            path_y[i]=$rtoi((3200-500-300*(1-$cos(theta)))*256/10);locks[i]=(i==0);
        end
        for(i=40;i<count;i=i+1)begin path_x[i]=800*256/10;path_y[i]=(3200-800-(i-39)*20)*256/10;locks[i]=i==count-1;end
        yaw_bias=1790;yaw=1790;launch(9);await_finish(2500);
        if($abs(world_x+228*$cos(world_h)-528)>40 || $abs(world_y+228*$sin(world_h)-800)>40 || $abs(world_h*180/3.141592653589793-90)>15)
            $fatal(1,"arc destination %f,%f h=%f pose=%f,%f heading=%d initial=%d origin=%d",world_x,world_y,world_h*180/3.141592653589793,$signed(px)/256.0,$signed(py)/256.0,heading,dut.initial_heading,dut.odometry.yaw_origin);
        $display("PASS Ackermann curve tracking: independent bicycle model, wheel counts and IMU, 90-degree corner");
        reset_plant;model=2;launch(8);wait(phase==1);wait(dut.bypass_active);
        if(dut.left_detour)$fatal(1,"right priority ignored");await_finish(6000);
        if(world_x<2000 || $abs(world_y)>35 || $abs(world_h*180/3.141592653589793)>10)
            $fatal(1,"detour did not return to line %f,%f",world_x,world_y);
        $display("PASS obstacle bypass: depart line, observe inner-side obstacle, pass, merge back, continue and stop");
        reset_plant;model=3;right=200;launch(8);wait(phase==1);wait(dut.bypass_active);
        if(!dut.left_detour)$fatal(1,"left fallback ignored");await_finish(6000);
        if(world_x<2000 || $abs(world_y)>35 || $abs(world_h*180/3.141592653589793)>10)
            $fatal(1,"right detour did not return to line %f,%f",world_x,world_y);
        $display("PASS right bypass, nonzero encoder baseline, single-wheel feedback loss, missing/cancelled route");
        // Mode 8 locked its detour direction at launch; the front sensors see
        // the original obstacle throughout the turn. That repeat echo must not
        // re-run the side decision or trip the approach emergency stop.
        reset_plant;model=3;right=200;launch(8);wait(dut.bypass_active);
        begin : lock_direction
            integer locked_side;
            locked_side=dut.left_detour;
            wait(world_x>300);model=0;lf=30;rf=30;range_frame=1;diagnostics[15:8]=0;diagnostics[23:16]=0;
            wait_ticks(5);
            if(status==5 || !drive || !dut.bypass_active || dut.left_detour!=locked_side)
                $fatal(1,"committed bypass was interrupted/reselected by the same front obstacle");
            model=3;
        end
        leave;
        $display("PASS committed bypass: startup direction stays latched; same-obstacle emergency echo is ignored while detouring");
        reset_plant;slowdown=200;model=1;launch(7);await_finish(2000);
        if(world_x>820)$fatal(1,"2m gradual stop");
        reset_plant;slowdown=50;model=1;launch(7);await_finish(2000);
        if(world_x<650 || world_x>850)$fatal(1,"50cm stop %f",world_x);
        reset_plant;slowdown=25;model=1;launch(7);await_finish(2500);
        if(world_x>850)$fatal(1,"25cm stop %f",world_x);
        reset_plant;slowdown=65;model=1;left=200;right=200;launch(8);
        wait(phase==5);wait_ticks(2);
        if(status==5 || !drive || speed<=0)$fatal(1,"blocked detour cut drive abruptly");
        wait(status==5);wait_ticks(2);if(drive || speed)$fatal(1,"blocked did not stop");
        reset_plant;profile_ready=0;launch(9);wait_ticks(20);
        if(status!=1 || drive)$fatal(1,"profile wait");profile_ready=1;await_finish(2500);
        reset_plant;reverse_feedback=1;launch(7);wait_ticks(500);
        if(status!=8 || drive || speed)$fatal(1,"reversed feedback must inhibit automatic motion");
        $display("PASS configurable braking: 25/50/200cm, right priority, left fallback, blocked gradual stop, profile-ready wait, yaw wrap, reverse-feedback stop, swept body collision check");
        reset_plant;lf=3500;rf=3500;left=3000;right=3000;launch(7);wait_ticks(450);
        if(status!=2 || !drive || speed!=brake_setting || !dut.known_lf || !dut.known_rf)
            $fatal(1,"VL53L1X valid range beyond 2m rejected by motion controller");
        leave;
        $display("PASS L1X motion range: valid 3.5m front samples authorize cruise and retain distance memory");
        reset_plant;model=1;drop_mask=4'b1101;wait_ticks(2);launch(7);await_finish(2000);
        if(world_x<650 || world_x>850)$fatal(1,"single front stop %f",world_x);
        $display("PASS range dropout: only one front sensor required; both side sensors may be absent");
        reset_plant;model=1;launch(7);wait(world_x>180);drop_mask=15;
        await_finish(2000);
        if(world_x<650 || world_x>850)$fatal(1,"cached stop before slowdown %f",world_x);
        $display("PASS range dropout: all echoes lost before braking, odometry completes stop at remembered obstacle");
        reset_plant;model=1;launch(7);wait(world_x>400);drop_code=8'h09;drop_mask=15;
        await_finish(2000);
        if(world_x<650 || world_x>850)$fatal(1,"cached stop during braking %f",world_x);
        $display("PASS range dropout: communication loss during braking preserves the distance plan");
        reset_plant;model=1;launch(7);wait(world_x>250);range_frame=0;
        await_finish(2000);
        if(world_x<650 || world_x>850)$fatal(1,"frozen scan reused as new distance %f",world_x);
        $display("PASS range dropout: frozen scan is spent using measured travel, not refreshed stale data");
        reset_plant;model=1;launch(7);wait(world_x>250);model=0;sample_suppress=6;
        await_finish(2000);
        if(world_x<650 || world_x>850)$fatal(1,"not-ready channels refreshed by healthy side scans %f",world_x);
        $display("PASS range dropout: healthy side scans cannot refresh not-ready front channels holding old E00 data");
        reset_plant;drop_mask=15;wait_ticks(2);launch(7);wait_ticks(450);
        if(status!=2 || speed!=brake_setting || !drive)$fatal(1,"completed noecho must permit configured brake cruise");leave;
        $display("PASS range dropout: completed noecho cruises at configured initial speed");
        reset_plant;drop_mask=15;drop_code=8'h09;wait_ticks(2);launch(7);await_status(4,300);
        if(world_x!=0 || first_motion_tick!=0)$fatal(1,"initial bus fault drove blindly");
        reset_plant;model=1;launch(7);wait(world_x>300);drop_mask=2;wait_ticks(2);
        model=0;rf=20;diagnostics[23:16]=0;wait_ticks(2);
        if(status!=5 || drive)$fatal(1,"reacquired near obstacle ignored");
        $display("PASS range dropout: startup bus failure does not authorize motion; surviving/recovered near beam still stops");
        reset_plant;model=2;launch(8);wait(world_x>80);drop_mask=8;
        wait(phase==1);wait(dut.bypass_active);if(dut.left_detour)$fatal(1,"cached right-side space lost");
        drop_mask=14;await_finish(6500);
        if($abs(world_y)>35)$fatal(1,"front dropout bypass return");
        $display("PASS range dropout: cached right clearance selects right bypass, front beams may disappear throughout maneuver");
        reset_plant;model=3;right=200;pulsed_frames=1;launch(8);wait(phase==1);wait(dut.bypass_active);
        if(!dut.left_detour)$fatal(1,"left choice");
        wait(dut.passing_clear);drop_mask=15;await_finish(6500);
        if($abs(world_y)>35)$fatal(1,"all dropout after tail return");
        $display("PASS range dropout: asynchronous scan pulses confirm tail; all four beams may vanish before merge and final stop");
        reset_plant;model=2;launch(8);wait(phase==1);wait(dut.bypass_active);drop_mask=15;
        await_status(5,6500);
        if(phase!=5 || $abs(world_y)<180 || traveled>3400)$fatal(1,"unknown tail guessed clear phase=%d y=%f s=%f",phase,world_y,traveled);
        $display("PASS range dropout: missing unobserved obstacle tail limits parallel travel and brakes without merging blindly");
        reset_plant;model=1;launch(7);wait(world_x>250);drop_mask=15;wait_ticks(2);
        leave;
        if(dut.known_lf || dut.known_rf)$fatal(1,"range memory not cleared on exit");
        launch(7);wait_ticks(450);
        if(dut.known_lf || dut.known_rf)$fatal(1,"previous run range memory leaked");
        leave;
        $display("PASS range dropout: exit/re-entry clears prior-run obstacle and side certificates");
        reset_plant;drop_mask=15;wait_ticks(2);launch(7);wait(world_x>100);
        drop_mask=0;diagnostics=0;model=1;await_finish(2400);
        if(world_x<650 || world_x>850)$fatal(1,"search reacquisition did not complete distance stop %f",world_x);
        $display("PASS range recovery: a real front echo replaces bounded search and completes odometry stop");
        // A newer near reading before the trigger revokes old side clearance.
        reset_plant;model=2;launch(8);wait(world_x>80);
        right=100;model=0;wait_ticks(5);drop_mask=8;drop_code=8'h09;
        model=2;wait(phase==1);wait(dut.bypass_active);
        if(!dut.left_detour)$fatal(1,"new near right echo failed to revoke old clearance");
        leave;
        $display("PASS side clearance: a newer near reading revokes remembered open right side and selects left");
        reset_plant;lf=600;rf=600;left=1800;right=700;launch(8);wait(dut.bypass_active);
        if(!dut.left_detour)$fatal(1,"larger left space not preferred");leave;
        reset_plant;lf=1500;rf=600;left=800;right=1600;launch(8);wait(dut.bypass_active);
        if(!dut.left_detour)$fatal(1,"right-front-only obstacle must prefer clear left");leave;
        reset_plant;lf=600;rf=1500;left=1600;right=800;launch(8);wait(dut.bypass_active);
        if(dut.left_detour)$fatal(1,"left-front-only obstacle must prefer clear right");leave;
        reset_plant;lf=600;rf=600;left=65535;right=500;diagnostics=32'h00000011;
        launch(8);wait(dut.bypass_active);if(!dut.left_detour)$fatal(1,"completed no-echo side not usable");leave;
        reset_plant;lf=600;rf=600;left=65535;right=500;diagnostics=32'h00000009;
        launch(8);wait(dut.bypass_active);if(dut.left_detour)$fatal(1,"bus fault misclassified free");leave;
        reset_plant;bypass_setting=35;model=2;launch(8);wait_ticks(230);
        if(phase!=0)$fatal(1,"bypass started before configured threshold");
        wait(dut.bypass_active);if(world_x<450)$fatal(1,"35cm trigger too early x=%f",world_x);leave;bypass_setting=65;
        $display("PASS bypass policy: larger side, each single-front obstacle, noecho versus E09, adjustable delayed trigger");
        reset_plant;slowdown=25;brake_setting=400;model=1;launch(7);await_finish(2200);
        if(world_x>850 || world_x<650)$fatal(1,"400mm/s short-zone braking x=%f",world_x);
        reset_plant;slowdown=200;brake_setting=80;model=1;launch(7);await_finish(3500);
        if(world_x>850 || world_x<650)$fatal(1,"80mm/s long-zone braking x=%f",world_x);
        brake_setting=200;slowdown=65;
        $display("PASS adaptive braking: independent 80/400mm/s initial speeds, 25/200cm zones, odometry stop");
        reset_plant;slowdown=100;brake_setting=80;model=1;launch(7);
        begin : low_speed_slew
            integer previous_command,large_steps;
            previous_command=0;large_steps=0;
            while(status<3)begin
                @(negedge clk);
                if(speed<previous_command && previous_command-speed>2)large_steps=large_steps+1;
                previous_command=speed;
            end
            if(large_steps!=0)$fatal(1,"80mm/s / 100cm stop had coarse speed steps: %0d",large_steps);
        end
        if(world_x>1050)$fatal(1,"80mm/s / 100cm smooth stop exceeded zone: %f",world_x);
        $display("PASS smooth low-speed stop: 80mm/s over 100cm uses no >2mm/s command step");
        // Leave forward route after the 320mm-body rejoin. A goal passed
        // during the manoeuvre is covered separately by the physical-beam TB.
        reset_plant;model=2;count=176;
        for(i=0;i<count;i=i+1)begin path_x[i]=(500+i*20)*256/10;path_y[i]=2700*256/10;locks[i]=i==0 || i==175;end
        launch(9);wait(dut.bypass_active);await_finish(7000);
        if($abs(world_x-3500)>40 || $abs(world_y)>50)$fatal(1,"navigation bypass endpoint %f,%f",world_x,world_y);
        $display("PASS navigation dynamic obstacle: bypass, same-line rejoin, skip passed samples, resume and stop at route goal");
        $finish;
    end
    initial begin #500000000;$fatal(1,"simulation timeout");end
endmodule
