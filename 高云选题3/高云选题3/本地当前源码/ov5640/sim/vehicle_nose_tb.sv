`timescale 1ns/1ps
module vehicle_nose_tb;
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
    wire drive,tick,fvalid,brake;wire signed [15:0] speed,steer,heading,target_x,target_y,bearing;
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
        .drive_enabled(drive),.brake_enable(brake),.speed_mm_s(speed),.steer_deg(steer),.status(status),.phase(phase),
        .front_average_mm(avg),.front_valid(fvalid),.pose_x_q8(px),.pose_y_q8(py),.pose_heading(heading),
        .encoder_relative1(relative1),.encoder_relative2(relative2),.target_x_mm(target_x),.target_y_mm(target_y),
        .target_heading(bearing),.target_distance_mm(target_distance),.tick_10ms(tick));
    real world_x=0,world_y=0,world_h=0,traveled=0,wheel1=0,wheel2=0;
    real ds,curvature,ratio,old_h;
    real bx,by,cx,cy,ac,as_,sep_x,sep_y,sep_l,sep_w;
    reg overlap;
    task check_body;
      begin
        // SAT: 260x190mm car, axle L=138, centred overhang 61mm.
        // Rear axle is the pose reference; obstacle is x=1100..1400,y=-70..70.
        ac=$cos(world_h);as_=$sin(world_h);
        bx=world_x+80*ac;by=world_y+80*as_;
        cx=bx-1250;cy=by;
        sep_x=$abs(cx)-(150+150*$abs(ac)+95*$abs(as_));
        sep_y=$abs(cy)-(70+150*$abs(as_)+95*$abs(ac));
        sep_l=$abs(cx*ac+cy*as_)-(150+150*$abs(ac)+70*$abs(as_));
        sep_w=$abs(-cx*as_+cy*ac)-(95+150*$abs(as_)+70*$abs(ac));
        overlap=sep_x<0 && sep_y<0 && sep_l<0 && sep_w<0;
        if(overlap)$fatal(1,"body collision x=%f y=%f h=%f phase=%d",world_x,world_y,world_h,phase);
      end
    endtask
    real plant_speed=0,plant_tau=0.25;reg lag_enabled=0;
    real obstacle_x=100000,clearance,actual_nose_x,actual_nose_y;
    reg dynamic_obstacle=0,use_short_brake=0;
    real minimum_clearance=100000;
    reg braking_latched=0;integer stop_after_brake=0;
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
        if(lag_enabled)begin
            // Independently model a slow power response, much slower coast,
            // and bounded electrical braking. Acceleration is a plant constant,
            // not calculated from the DUT's envelope or measured range.
            if(use_short_brake && brake && (speed==0 || plant_speed>speed+12))begin
                plant_speed=plant_speed-15;
                if(plant_speed<0)plant_speed=0;
                braking_latched=1;
            end else plant_speed=plant_speed+((drive?speed:0)-plant_speed)*
                (0.01/((!drive || speed==0)?0.8:plant_tau));
        end
        else plant_speed=drive?speed:0;
        if(plant_speed>0.05 && simulate_wheels)begin
            curvature=$sin(actual_steer*3.141592653589793/180)/
                $cos(actual_steer*3.141592653589793/180)/142;
            // The subsystem requests outer-wheel speed. Rear-axle centre
            // speed is lower on a turn, using the physical 168 mm track.
            ds=plant_speed*0.01/(1+84*$abs(curvature));
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
            left=(world_x>=1100 && world_x<=1400 && world_y < -180)?-world_y-70:1200;
        end
        if(model==2 || model==3)check_body;
        if(model==3)begin
            lf=(world_y<150 && world_y > -150 && world_x<1050)?1100-world_x-190:1500;
            rf=lf;left=1200;
            right=world_x<300?200:(world_x>=1100 && world_x<=1400 && world_y>180)?world_y-70:1200;
        end
        actual_nose_x=world_x+219*$cos(world_h);actual_nose_y=world_y+219*$sin(world_h);
        if(dynamic_obstacle)begin
            clearance=obstacle_x-actual_nose_x;
            if(clearance<minimum_clearance)minimum_clearance=clearance;
            // Five control ticks = 50 ms completed front range observations.
            if(ticks%5==0)begin lf=$rtoi(clearance);rf=lf;end
            if(clearance<=0)$fatal(1,"physical nose hit obstacle at speed=%f",plant_speed);
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


    integer i,k,limit,first_dropout;real theta,err_x,err_y,max_error,max_recovery_step,previous_heading;
    initial begin
        for(i=0;i<256;i=i+1)begin path_x[i]=0;path_y[i]=0;locks[i]=0;end
        repeat(4)@(negedge clk);rst_n=1;wait_ticks(2);
        for(k=0;k<2;k=k+1)begin
            reset_plant;plant_speed=0;lag_enabled=1;plant_tau=k==0?0.25:0.40;
            use_short_brake=1;braking_latched=0;slowdown=25;brake_setting=400;
            launch(7);wait_ticks(650);
            if(plant_speed<395 || speed!=400)$fatal(1,"must test an already moving 400 mm/s car");
            obstacle_x=world_x+219+250;dynamic_obstacle=1;minimum_clearance=250;
            lf=250;rf=250;
            await_finish(800);wait_ticks(100);
            if(minimum_clearance<50 || plant_speed>1 || !braking_latched)
                $fatal(1,"fast braking clearance=%f actual=%f",minimum_clearance,plant_speed);
            $display("PASS nose fast brake: actual 400mm/s before 25cm trigger, 50ms range, coast tau=0.8s, drive tau=%f, clearance=%fmm",plant_tau,minimum_clearance);
            dynamic_obstacle=0;lag_enabled=0;
        end
        reset_plant;count=111;
        // Absolute physical rear pose is initial selected NOSE minus 219mm.
        world_x=281;world_y=500;drop_mask=6;left=150;right=150;
        for(i=0;i<26;i=i+1)begin path_x[i]=(500+i*20)*256/10;path_y[i]=2700*256/10;locks[i]=i==0;end
        for(i=26;i<66;i=i+1)begin
            theta=(i-25)*3.141592653589793/80;
            path_x[i]=$rtoi((1000+500*$sin(theta))*256/10);
            path_y[i]=$rtoi((2700-500*(1-$cos(theta)))*256/10);locks[i]=0;
        end
        for(i=66;i<count;i=i+1)begin path_x[i]=1500*256/10;path_y[i]=(2200-(i-65)*20)*256/10;locks[i]=i==count-1;end
        launch(9);wait(status==2);max_error=0;max_recovery_step=0;first_dropout=0;
        for(k=0;k<4000 && status==2;k=k+1)begin
            if(first_dropout==0 && world_h>0.25)begin first_dropout=k;imu_available=0;imu_update=0;end
            if(first_dropout!=0 && k==first_dropout+70)begin imu_available=1;imu_update=1;end
            previous_heading=heading;wait_ticks(1);
            err_x=$signed(px)/256.0-(world_x+219*$cos(world_h));
            err_y=$signed(py)/256.0-(world_y+219*$sin(world_h));
            if($sqrt(err_x*err_x+err_y*err_y)>max_error)max_error=$sqrt(err_x*err_x+err_y*err_y);
            if(first_dropout!=0 && k>first_dropout+70 && k<first_dropout+90 && $abs(heading-previous_heading)>max_recovery_step)
                max_recovery_step=$abs(heading-previous_heading);
            if(k%200==0)$display("NOSE trace k=%d rear=%f,%f yaw=%f nose=%f,%f index=%d status=%d steer=%d",k,world_x,world_y,world_h*180/3.14159265,actual_nose_x,actual_nose_y,index,status,steer);
        end
        if(status!=3 || max_error>20 || max_recovery_step>15 || first_dropout==0 ||
           $abs(actual_nose_x-1500)>35 || $abs(actual_nose_y-1900)>35)
            $fatal(1,"nose goal/dropout status=%d nose=%f,%f err=%f step=%f",status,actual_nose_x,actual_nose_y,max_error,max_recovery_step);
        $display("PASS nose navigation: absolute nose start/goal, straight/90deg curve, 700ms IMU outage while turning, max pose error=%fmm recovery step=%f tenths",max_error,max_recovery_step);
        // A sustained outage still stops instead of using wheel yaw indefinitely.
        reset_plant;launch(7);wait(status==2);wait_ticks(150);imu_update=0;imu_available=0;
        wait_ticks(900);if(status!=2)$fatal(1,"nine-second dropout stopped bounded odometry");
        wait_ticks(110);if(status!=4 || drive)$fatal(1,"sustained IMU outage did not stop");
        $display("PASS nose IMU watchdog: 9s gap tolerated; sustained 10s loss disarms");
        leave;$finish;
    end
    initial begin #400000000;$fatal(1,"nose regression timeout");end
endmodule
