`timescale 1ns/1ps
module motor_brake_tb;
    reg clk=0;always #5 clk=~clk;
    reg rst_n=0,enabled=0,brake=0;
    reg signed [15:0] target=0;
    reg [1:0] ab=0;
    wire [1:0] dir;wire pwm;
    wire signed [15:0] actual;
    wire signed [31:0] position;
    motor_closed_loop #(.CLK_HZ(100000),.SAMPLE_MS(10),
        .ENCODER_FILTER_CYCLES(1),.REVERSE_BLANK_CYCLES(2),.SPEED_AVERAGE_LOG2(0)) dut(
        .clk(clk),.rst_n(rst_n),.drive_enabled(enabled),.brake_request(brake),
        .speed_command(target),.converted_speed(target),.target_counts_q4(20'sd480),
        .encoder_A(ab[1]),.encoder_B(ab[0]),.dir(dir),.pwm_out(pwm),
        .actual_speed_mm_s(actual),.encoder_position(position));
    task cycles(input integer n);repeat(n)@(negedge clk);endtask
    task edges(input integer n);integer i;begin
        for(i=0;i<n;i=i+1)begin
            case(ab)0:ab=1;1:ab=3;3:ab=2;2:ab=0;endcase
            cycles(10);
        end
    end endtask
    task coast;begin cycles(2);if(dir!==0 || pwm!==0)$fatal(1,"disabled/reset drove motor");end endtask
    integer i,brake_cycles,coast_cycles;reg signed [23:0] integral_before;
    initial begin
        cycles(5);coast;rst_n=1;cycles(1100);coast;
        enabled=1;target=400;cycles(2100);
        if(dut.braking)$fatal(1,"ordinary acceleration short-braked");
        brake=1;cycles(2100);
        if(dut.braking)$fatal(1,"underspeed acceleration short-braked");
        integral_before=dut.u_pi.integral_q12;
        edges(40);wait(dut.sample_valid);cycles(2);
        if(actual<450)$fatal(1,"no overspeed feedback: %d",actual);
        if(dut.brake_duty!=128)$fatal(1,"brake duty must ramp, got %d",dut.brake_duty);
        if(dut.u_pi.integral_q12==0 || !dut.motor_rst_n)$fatal(1,"braking reset PI");
        brake_cycles=0;coast_cycles=0;
        for(i=0;i<800;i=i+1)begin
            cycles(1);
            if(dir==3 && pwm)brake_cycles=brake_cycles+1;
            else if(dir==0 && pwm)coast_cycles=coast_cycles+1;
            else $fatal(1,"brake PWM truth table");
        end
        if(brake_cycles==0 || coast_cycles==0)$fatal(1,"no modulated brake/coast phases");
        cycles(2200);if(dut.braking)$fatal(1,"brake did not release below target");
        target=0;cycles(1100);if(dut.brake_duty!=128)$fatal(1,"zero speed ramp");
        enabled=0;cycles(2);if(dir!=3 || pwm!=1)$fatal(1,"final stopped hold");
        brake=0;coast;
        brake=1;rst_n=0;coast;
        $display("PASS motor modulated brake: real encoder overspeed, 128/2000 duty slew, PI preserved, PWM brake/coast truth table, release, zero hold, reset/menu coast");
        $finish;
    end
    initial begin #10000000;$fatal(1,"motor brake timeout");end
endmodule
