`timescale 1ns/1ps
module vehicle_close_bypass_tb;
    parameter ENABLE_VISION=0;
    reg vision_update=0;reg signed [8:0] vision_near=0,vision_heading=0;
    integer vision_rejoin_ticks=0;reg completed_bypass=0;
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
    vehicle_motion #(.REQUIRE_VISION(ENABLE_VISION),.CLK_HZ(20000),.STALL_MS(3000)) dut(.route_exit_valid(1'b0),.route_exit_heading(16'sd0),.route_curve(2'sd0),.route_upcoming_turn(2'sd0),.route_entry_mm(8'd0),.route_tangent(16'sd0),.imu_restart(1'b0),.clk(clk),.rst_n(rst_n),.mode(mode),
        .configured_speed(cruise),.bypass_cm(bypass_setting),.brake_speed(brake_setting),.slowdown_cm(slowdown),.profile_ready(profile_ready),
        .navigation_request(request),.route_valid(route_valid),.route_count(count),
        .route_index(index),.route_x_q8(rx),.route_y_q8(ry),.route_lock(route_lock),
        .encoder1(encoder1),.encoder2(encoder2),.imu_available(imu_available),.imu_update(imu_update),.imu_yaw(yaw),
        .road_visibility(ENABLE_VISION?3'b111:3'b000),.road_update(vision_update),.road_valid(ENABLE_VISION!=0),.road_junction(1'b0),.road_near(vision_near),.road_heading(vision_heading),
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
    real sh,ch;integer side_choice=0;real max_heading=0,max_lateral=0,tail_at=0;
    reg merge_checked=0;
    real pass_line_y=0,max_pass_drift=0;reg pass_tracking=0;
    integer actual_steer=0;
    integer model=0,ticks=0,last_speed=0,speed_step=0,first_motion_tick=0,arm_tick=0;
    reg simulate_wheels=1,freeze_encoder1=0,reverse_feedback=0;
    integer yaw_bias=0;
    integer base_count1=0,base_count2=0;
    // Independent bicycle plant. Counts are quantized from travelled wheel
    // distance and IMU reports physical orientation; no DUT pose is fed back.
    always @(negedge clk)if(rst_n && tick)begin
        ticks=ticks+1;
        if(ENABLE_VISION)begin
            vision_near=$rtoi(world_y+210*$tan(world_h));
            vision_heading=$rtoi(110*$tan(world_h));
            vision_update=ticks%3==0;
            if(dut.bypass_done)completed_bypass=1;
            if(completed_bypass && dut.vision_active && !dut.nav_resume)vision_rejoin_ticks=vision_rejoin_ticks+1;
        end
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
        if((mode==7 || mode==8 || mode==9) && status==2 && (speed_step>2 || speed_step < (mode==7?-40:-2)))
            $fatal(1,"speed step %d -> %d",last_speed,speed);
        last_speed=speed;
        if(model==1)begin
            lf=900-world_x;rf=lf;
        end
        if(model==2)begin
            // Obstacle front at x=1100; side beam sees it only alongside.
            lf=(world_y<150 && world_y > -150 && world_x<1050)?1100-world_x-190:1500;
            rf=lf;right=1200;
            left=(world_x>=1100 && world_x<=1400 && world_y < -200)?-world_y-70:1200;
        end
        if(model==2 || model==3)check_body;
        if(model==3)begin
            lf=(world_y<150 && world_y > -150 && world_x<1050)?1100-world_x-190:1500;
            rf=lf;left=1200;
            right=world_x<300?200:(world_x>=1100 && world_x<=1400 && world_y>200)?world_y-70:1200;
        end

        if(model==4)begin
            if(phase==2)begin
                if(!pass_tracking)begin pass_tracking=1;pass_line_y=world_y;end
                if($abs(world_y-pass_line_y)>max_pass_drift)max_pass_drift=$abs(world_y-pass_line_y);
                if($abs(world_y-pass_line_y)>15)$fatal(1,"parallel bypass line wanders: %f mm",world_y-pass_line_y);
            end else pass_tracking=0;
            if(phase==4 && ($abs(world_y)>25 || $abs(world_h)*180/3.14159265359>2.5))
                $fatal(1,"post-merge straight is not aligned y=%f heading=%f",world_y,world_h);
            if(phase==4 && tail_at>0 && !merge_checked)begin
                merge_checked=1;
                if(world_x-tail_at>650)
                    $fatal(1,"merge still takes excessive forward travel: %f mm",world_x-tail_at);
                $display("PASS merge travel bound: %f mm",world_x-tail_at);
            end
            ch=$cos(world_h);sh=$sin(world_h);
            lf=$rtoi(beam(world_x+189*ch-56*sh,world_y+189*sh+56*ch,ch,sh));
            rf=$rtoi(beam(world_x+189*ch+56*sh,world_y+189*sh-56*ch,ch,sh));
            left=$rtoi(beam(world_x+109*ch-72*sh,world_y+109*sh+72*ch,-sh,ch));
            right=$rtoi(beam(world_x+109*ch+72*sh,world_y+109*sh-72*ch,sh,-ch));
            if(side_choice==1 && !dut.bypass_active)right=240; // narrow wall on right: use left
            if(side_choice==2 && !dut.bypass_active)left=240;
            diagnostics={(right==65535?8'h11:8'h00),(rf==65535?8'h11:8'h00),
                (lf==65535?8'h11:8'h00),(left==65535?8'h11:8'h00)};
            if(dut.bypass_active)begin
                if($abs(world_h)*180/3.14159265359>max_heading)max_heading=$abs(world_h)*180/3.14159265359;
                if($abs(world_y)>max_lateral)max_lateral=$abs(world_y);
                if(phase==3 && tail_at==0)begin
                    tail_at=world_x;
                    // Earlier convergence is allowed once the side scan has
                    // confirmed the trailing edge. The swept-body SAT check
                    // below remains active on every plant step.
                    if(world_x<box_x1-30 || world_x>box_x1+150)$fatal(1,"tail merge timing outside safe corridor, x=%f box_end=%f",world_x,box_x1);
                end
            end
            check_body;
        end
        if(drop_mask[0])begin left=65535;diagnostics[7:0]=drop_code;end
        if(drop_mask[1])begin lf=65535;diagnostics[15:8]=drop_code;end
        if(drop_mask[2])begin rf=65535;diagnostics[23:16]=drop_code;end
        if(drop_mask[3])begin right=65535;diagnostics[31:24]=drop_code;end
    end else vision_update=0;
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
            leave;world_x=0;world_y=0;world_h=0;traveled=0;wheel1=0;wheel2=0;merge_checked=0;
            actual_steer=0;yaw=0;encoder1=0;encoder2=0;lf=1500;rf=1500;left=1200;right=1200;
            last_speed=0;first_motion_tick=0;model=0;simulate_wheels=1;
            freeze_encoder1=0;reverse_feedback=0;yaw_bias=0;base_count1=0;base_count2=0;
            pulsed_frames=0;frame_clock=0;range_frame=1;imu_update=1;imu_available=1;diagnostics=0;drop_mask=0;drop_code=8'h11;sample_suppress=0;
            pass_tracking=0;max_pass_drift=0;vision_rejoin_ticks=0;completed_bypass=0;
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

    reg [3:0] prior_phase=0;
    always @(posedge clk)if(phase!=prior_phase)begin
        prior_phase=phase;
        $display("TRACE phase %d x=%f y=%f heading=%f local=%d,%d side=%d diag=%h seen=%b tail=%b",phase,world_x,world_y,world_h*180/3.14159265,dut.bypass.local_x,dut.bypass.local_y,dut.bypass.inner,diagnostics,dut.bypass.seen_side,dut.bypass.tail_confirmed);
    end
    integer i,k;
    initial begin
        for(i=0;i<256;i=i+1)begin path_x[i]=0;path_y[i]=0;locks[i]=0;end
        repeat(4)@(negedge clk);rst_n=1;wait_ticks(2);
        if(ENABLE_VISION)begin
            for(k=0;k<2;k=k+1)begin
                reset_plant;model=4;side_choice=k+1;count=126;max_heading=0;max_lateral=0;tail_at=0;
                for(i=0;i<count;i=i+1)begin path_x[i]=(500+i*20)*256/10;path_y[i]=2700*256/10;locks[i]=i==0 || i==125;end
                launch(9);wait(dut.bypass_active);wait(!dut.bypass_active);
                completed_bypass=1;await_finish(7500);
                if($abs(world_x-2500)>40 || $abs(world_y)>15 || vision_rejoin_ticks<50)
                    $fatal(1,"visual bypass rejoin/goal failed x=%f y=%f visualTicks=%0d",world_x,world_y,vision_rejoin_ticks);
                $display("PASS vision navigation bypass side=%0d: measured-body collision check, route resume, visual road-centre handback (%0d ticks), original goal",k,vision_rejoin_ticks);
            end
            $finish;
        end
        // Check physical clearance at both configurable trigger endpoints,
        // as well as the default, for both bypass directions.
        for(k=0;k<6;k=k+1)begin
            reset_plant;model=4;side_choice=k%2+1;max_heading=0;max_lateral=0;tail_at=0;
            bypass_setting=k<2?35:k<4?65:150;
            launch(8);wait(dut.bypass_active);
            if(dut.left_detour!=(k%2==0))$fatal(1,"geometric side selection");
            await_finish(7500);
            if($abs(world_y)>35 || max_heading>=65 || max_lateral>410 || tail_at==0)
                $fatal(1,"close bypass y=%f maxhead=%f maxoffset=%f tail=%f",world_y,max_heading,max_lateral,tail_at);
            $display("PASS physical beams %d trigger=%dcm: 320x200 body no overlap, heading=%f deg offset=%f mm tail merge=%f mm",
                k,bypass_setting,max_heading,max_lateral,tail_at);
        end
        bypass_setting=65;
        reset_plant;model=4;side_choice=2;count=126;max_heading=0;max_lateral=0;tail_at=0;
        for(i=0;i<count;i=i+1)begin path_x[i]=(500+i*20)*256/10;path_y[i]=2700*256/10;locks[i]=i==0 || i==125;end
        launch(9);wait(dut.bypass_active);await_finish(7500);
        if($abs(world_x-2500)>40 || $abs(world_y)>35)$fatal(1,"dynamic navigation goal %f,%f",world_x,world_y);
        $display("PASS navigation with physical beams: close bypass, route rejoin and original goal");
        reset_plant;model=4;side_choice=2;count=86;max_heading=0;max_lateral=0;tail_at=0;
        for(i=0;i<count;i=i+1)begin path_x[i]=(500+i*20)*256/10;path_y[i]=2700*256/10;locks[i]=i==0 || i==85;end
        launch(9);wait(dut.bypass_active);await_status(6,7500);
        $display("PASS passed navigation goal: bounded tracking stop after bypass, no indefinite forward resume");
        for(k=0;k<4;k=k+1)begin
            reset_plant;model=4;side_choice=k%2+1;max_heading=0;max_lateral=0;tail_at=0;
            box_x1=k<2?1160:1400;
            box_y0=k<2?-70:k==2?-140:20;
            box_y1=k<2?70:k==2?-20:140;
            pulsed_frames=1;
            launch(8);wait(dut.bypass_active);await_finish(7500);
            // A stronger return reaches ~47deg instead of the old 35deg.
            // The complete 320x200 swept-body collision test remains active.
            if($abs(world_y)>35 || max_heading>50 || tail_at==0)$fatal(1,"short/one-beam box bypass failed");
            $display("PASS bypass edge case %d: short or single-front-beam box, pulsed ToF, body/tail clearance and rejoin",k);
        end
        reset_plant;model=4;box_x1=1400;box_y0=-70;box_y1=70;side_choice=2;
        launch(8);wait(dut.bypass_active);wait(phase==2);
        // Communication loss on the box-facing side does not mean the box
        // ended. It must result in a bounded stop, not straight driving forever.
        drop_mask=1;drop_code=8'h09;await_status(5,5000);
        if(steer!=0)$fatal(1,"blocked bypass keeps countersteering into box");
        $display("PASS bypass missing side edge: bounded stop, no endless straight run, zero blocked steering");
        $finish;
    end
    initial begin #400000000;$fatal(1,"close bypass timeout");end
endmodule
