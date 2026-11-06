`timescale 1ns/1ps
module vehicle_io_tb;
    localparam HZ=1000000;
    reg clk=0;always #500 clk=~clk;
    reg control_remote=1,control_host=0,touch_run=0;reg [15:0] speed_limit=200;
    reg signed [15:0] touch_speed=0,touch_steer=0;
    reg rst_n=0,connected=1,imu_update=1;reg [3:0] mode=0;
    reg [7:0] lx=128,ry=128;reg [15:0] buttons=16'hffff;
    reg [1:0] encoder1=0,encoder2=0;
    wire signed [15:0] target1,target2,actual1,actual2;
    wire cs,sclk,mosi,miso,servo,pwm1,pwm2,tx;
    wire [1:0] dir1,dir2;wire [3:0] status,phase;
    wire [15:0] l,lf,rf,r,average;wire valid;
    wire signed [15:0] speed,steer;
    tri1 sda,scl;wire [3:0] xs;
    vehicle_subsystem #(.CLK_HZ(HZ)) dut(.zebra_toggle(1'b0),.host_route_page(1'b0),.host_route_result(8'd5),.host_route_status(8'd0),.imu_restart(1'b0),.host_point_select_ready(1'b0),.host_point_route_ready(1'b0),.host_point_planning(1'b0),.selected_point(4'd0),.host_point_error(3'd0),.clk(clk),.rst_n(rst_n),.mode(mode),
        .servo_center_tenths(11'd900),.navigation_avoidance(1'b0),.control_host(control_host),.host_rx(1'b1),
        .bypass_cm(16'd65),.brake_speed(speed_limit),.slowdown_cm(16'd65),.control_remote(control_remote),.configured_speed(speed_limit),.touch_run(touch_run),.touch_speed(touch_speed),.touch_steer(touch_steer),
        .navigation_request(1'b0),.route_valid(1'b0),.route_count(12'd0),.route_index(),
        .route_x_q8(17'd0),.route_y_q8(17'd0),.route_lock(1'b0),
        .imu_available(1'b1),.imu_update(imu_update),.imu_yaw(16'sd0),
        .zebra_update(1'b0),.zebra_detected(1'b0),
        .encoder_A(encoder1[1]),.encoder_B(encoder1[0]),.encoder2_A(encoder2[1]),.encoder2_B(encoder2[0]),
        .dir(dir1),.dir2(dir2),.pwm_out(pwm1),.pwm_out2(pwm2),.servo_pwm_out(servo),
        .ps2_cs_n(cs),.ps2_clk(sclk),.ps2_mosi(mosi),.ps2_miso(miso),
        .tof_sda(sda),.tof_scl(scl),.tof_xs1(xs[0]),.tof_xs2(xs[1]),.tof_xs3(xs[2]),.tof_xs4(xs[3]),.tx(tx),
        .left_mm(l),.left_front_mm(lf),.right_front_mm(rf),.right_mm(r),
        .front_average_mm(average),.front_valid(valid),.status(status),.phase(phase),
        .commanded_speed(speed),.commanded_steer(steer),
        .target_speed1(target1),.target_speed2(target2),.actual_speed1(actual1),.actual_speed2(actual2));
    ps2_receiver_model receiver(.cs_n(cs),.sclk(sclk),.mosi(mosi),.miso(miso),
        .connected(connected),.bad_header(1'b0),.bad_id(1'b0),.left_x(lx),.right_y(ry),
        .left_y(8'd128),.right_x(8'd128),.buttons(buttons),.forced_mode(8'd0),
        .active_mode(),.transactions(),.polls());
    vl53l1x_sensor_model #(.SENSOR(0),.DISTANCE(123)) s0(rst_n,xs[0],1'b1,1'b0,1'b0,1'b0,sda,scl);
    vl53l1x_sensor_model #(.SENSOR(1),.DISTANCE(456)) s1(rst_n,xs[1],1'b1,1'b0,1'b0,1'b0,sda,scl);
    vl53l1x_sensor_model #(.SENSOR(2),.DISTANCE(1000)) s2(rst_n,xs[2],1'b1,1'b0,1'b0,1'b0,sda,scl);
    vl53l1x_sensor_model #(.SENSOR(3),.DISTANCE(1800)) s3(rst_n,xs[3],1'b1,1'b0,1'b0,1'b0,sda,scl);
    task milliseconds;
        input integer n;begin repeat(n*(HZ/1000))@(negedge clk);end
    endtask
    task press_start;
        begin buttons[3]=0;milliseconds(160);buttons[3]=1;milliseconds(160);end
    endtask
    task encoder_steps;
        input integer reverse_direction;
        integer n;
        begin
            // Physical polarity model: M1=01 and the corrected M2=10 both
            // produce the same forward raw AB sequence on this installation.
            for(n=0;n<160;n=n+1)begin
                case(encoder1)
                    0:encoder1=reverse_direction?2:1;
                    1:encoder1=reverse_direction?0:3;
                    3:encoder1=reverse_direction?1:2;
                    2:encoder1=reverse_direction?3:0;
                endcase
                encoder2=encoder1;
                repeat(700)@(negedge clk);
            end
        end
    endtask
    integer clocks=0,start_clock,pulse_clocks,period_clocks;
    integer source_case,speed_case,mode_case;
    always @(posedge clk)begin
      clocks=clocks+1;
      if(rst_n && (dut.servo_angle_tenths<400 || dut.servo_angle_tenths>1500))$fatal(1,"servo command exceeded 40..150");
    end
    initial begin
        repeat(8)@(negedge clk);rst_n=1;milliseconds(10);
        if($test$plusargs("shared_speed_wiring_only"))begin
            for(source_case=0;source_case<2;source_case=source_case+1)begin
                control_host=source_case;
                for(speed_case=0;speed_case<4;speed_case=speed_case+1)begin
                    case(speed_case)0:speed_limit=0;1:speed_limit=50;2:speed_limit=350;3:speed_limit=1000;endcase
                    for(mode_case=0;mode_case<4;mode_case=mode_case+1)begin
                        case(mode_case)0:mode=2;1:mode=7;2:mode=8;3:mode=9;endcase
                        repeat(4)@(negedge clk);
                        if(mode==9)begin
                            if(dut.motion.configured_speed!=(control_host?dut.host_limit:200))
                                $fatal(1,"navigation speed contaminated by shared setting");
                        end else if(dut.motion.configured_speed!=speed_limit)
                            $fatal(1,"non-navigation setting overwritten by host source");
                        if(dut.motion.brake_speed!=speed_limit || dut.remote.speed_limit!=speed_limit)
                            $fatal(1,"remote/obstacle consumers differ from shared setting");
                    end
                end
            end
            $display("PASS speed wiring: 32 mode/source/value combinations, navigation isolated, host selection cannot override standalone avoidance");
            $finish;
        end
        if(dut.servo_offset_rom[8]!=10 || dut.servo_offset_rom[11]!=20 ||
           dut.servo_offset_rom[15]!=30 || dut.servo_offset_rom[20]!=40 ||
           dut.servo_offset_rom[24]!=50 || dut.servo_offset_rom[28]!=60 ||
           dut.servo_offset_rom[64+4]!=10 || dut.servo_offset_rom[64+11]!=20 ||
           dut.servo_offset_rom[64+20]!=30 || dut.servo_offset_rom[64+26]!=40 ||
           dut.servo_offset_rom[64+28]!=50)
          $fatal(1,"measured nonlinear steering knots lost");
        for(start_clock=29;start_clock<64;start_clock=start_clock+1)
          if(dut.servo_offset_rom[start_clock]!=60 || dut.servo_offset_rom[64+start_clock]!=50)
            $fatal(1,"steering ROM extrapolated beyond measured hard stops");
        if(pwm1 || pwm2 || dir1 || dir2)$fatal(1,"reset/home motor output");
        mode=6;wait(valid);milliseconds(80);
        if(l!=123 || lf!=456 || rf!=1000 || r!=1800 || average!=728 || pwm1 || pwm2)
            $fatal(1,"four range channel assignment %d %d %d %d average=%d",l,lf,rf,r,average);
        $display("INFO vehicle I/O: four range sensors ready");
        mode=2;wait(dut.remote_link);milliseconds(80);
        if(dut.remote_run || speed)$fatal(1,"remote armed on entry");
        press_start;
        if(!dut.remote_run || speed!=200)$fatal(1,"START did not engage remote");
        if(dir1!=2'b01 || dir2!=2'b10)$fatal(1,"forward M1/M2 polarity %b %b",dir1,dir2);
        encoder_steps(0);
        if(dut.encoder_position1!=160 || dut.encoder_position2!=160 || actual1<=0 || actual2<=0)
            $fatal(1,"forward encoder feedback %d %d speeds %d %d",dut.encoder_position1,dut.encoder_position2,actual1,actual2);
        if(target1!=dut.speed1 || target2!=dut.speed2 || target1!=200 || target2!=200)
            $fatal(1,"exported target speeds do not match PI commands");
        $display("INFO vehicle I/O: forward M2 polarity and encoder feedback verified");
        speed_limit=600;milliseconds(100);
        if(speed!=600 || target1!=600 || target2!=600 || dut.converted1!=600 || dut.counts1<720 || dut.counts1>725 || dir1!=1 || dir2!=2)
            $fatal(1,"speed above old 200 cap: command=%d target=%d converted=%d counts=%d",speed,target1,dut.converted1,dut.counts1);
        speed_limit=200;milliseconds(80);
        lx=255;milliseconds(450);
        if(steer!=-28 || dut.servo_angle_tenths!=400 || dut.speed1!=200 || dut.speed2<103 || dut.speed2>106)
            $fatal(1,"linkage/differential s=%d servo=%d wheels=%d,%d",steer,dut.servo_angle_tenths,dut.speed1,dut.speed2);
        @(posedge servo);start_clock=clocks;@(negedge servo);pulse_clocks=clocks-start_clock;
        @(posedge servo);period_clocks=clocks-start_clock;
        if(pulse_clocks<942 || pulse_clocks>946 || period_clocks!=20000)
            $fatal(1,"servo frame/pulse %d / %d",period_clocks,pulse_clocks);
        lx=0;milliseconds(800);
        // R=142/tan(28deg), rear track 168mm: inner/outer ~=0.521.
        if(steer!=28 || dut.servo_angle_tenths!=1500 || dut.speed2!=200 || dut.speed1<103 || dut.speed1>106)
            $fatal(1,"opposite linkage/differential s=%d servo=%d wheels=%d,%d",steer,dut.servo_angle_tenths,dut.speed1,dut.speed2);
        @(posedge servo);start_clock=clocks;@(negedge servo);pulse_clocks=clocks-start_clock;
        if(pulse_clocks<2165 || pulse_clocks>2169)$fatal(1,"150-degree servo pulse %d",pulse_clocks);
        lx=128;ry=255;milliseconds(450);
        if(speed!=-200 || dut.servo_angle_tenths!=900 || dir1!=2'b10 || dir2!=2'b01)
            $fatal(1,"reverse M1/M2 polarity speed=%d servo=%d dir=%b,%b",speed,dut.servo_angle_tenths,dir1,dir2);
        encoder_steps(1);
        if(dut.encoder_position1!=0 || dut.encoder_position2!=0 || actual1>=0 || actual2>=0)
            $fatal(1,"reverse encoder feedback %d %d speeds %d %d",dut.encoder_position1,dut.encoder_position2,actual1,actual2);
        $display("INFO vehicle I/O: 40/150 servo pulses, reverse M2 polarity and encoder feedback verified");
        ry=128;milliseconds(120);
        mode=0;repeat(8)@(negedge clk);
        if(pwm1 || pwm2 || dir1 || dir2 || speed)$fatal(1,"remote exit did not stop pins");
        buttons[3]=0;mode=2;milliseconds(300);
        if(dut.remote_run || speed)$fatal(1,"held START rearmed on mode entry");
        buttons[3]=1;milliseconds(160);press_start;
        if(!dut.remote_run)$fatal(1,"fresh START rejected");
        connected=0;milliseconds(300);
        if(dut.remote_link || dut.remote_run || pwm1 || pwm2 || speed)$fatal(1,"remote link loss did not stop pins");
        mode=0;milliseconds(20);control_remote=0;mode=2;touch_speed=350;touch_steer=28;touch_run=1;milliseconds(500);
        if(speed!=350 || steer!=28 || target2!=350 || target1<181 || target1>184 || dut.remote_run)
            $fatal(1,"manual/remote source isolation speed=%d steer=%d targets=%d,%d",speed,steer,target1,target2);
        touch_run=0;milliseconds(10);if(pwm1 || pwm2 || dir1 || dir2)$fatal(1,"touch stop outputs");
        $display("PASS integrated I/O: four real VL53L1X I2C models/order/mean, PS2 handshaking, menu rearm, link loss, 40/90/150 servo pulses, 142/168mm differential with steering slew, M2 bridge swap and forward/reverse encoder feedback, per-wheel UART target exports, 600mm/s beyond old cap, manual/remote hardware arbitration");$finish;
    end
    initial begin #8000000000;$fatal(1,"I/O timeout state=%d link=%d ranges=%d,%d,%d,%d",dut.receiver.state,dut.remote_link,l,lf,rf,r);end
endmodule
