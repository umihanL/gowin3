`timescale 1ns/1ps
module vehicle_math_tb;
    reg clk=0;always #5 clk=~clk;
    reg rst_n=0,start=0;reg signed [15:0] angle=0;
    wire done;wire signed [16:0] cs,sn;
    vehicle_sincos trig(.clk(clk),.rst_n(rst_n),.start(start),.angle(angle),.done(done),.cosine(cs),.sine(sn));
    integer a;real expected;
    reg track_start=0;reg signed [15:0] tx=0,ty=0,px=0,py=0;
    wire track_done;wire [15:0] distance;wire signed [15:0] steer,bearing,forward,lateral;
    vehicle_tracker tracker(.clk(clk),.rst_n(rst_n),.start(track_start),.target_x(tx),.target_y(ty),
        .precise_bearing(1'b0),.bearing_dx_q8(24'sd0),.bearing_dy_q8(24'sd0),
        .pose_x(px),.pose_y(py),.cosine(17'sd32768),.sine(17'sd0),.done(track_done),
        .distance_mm(distance),.steering_deg(steer),.bearing_tenths(bearing),.forward_mm(forward),.lateral_mm(lateral));
    task track;
        input integer x,y,expected_distance,expected_steer,expected_bearing;
        begin
            @(negedge clk);tx=x;ty=y;track_start=1;@(negedge clk);track_start=0;
            wait(track_done);@(negedge clk);
            if(distance<expected_distance-1 || distance>expected_distance+1 ||
                steer<expected_steer-1 || steer>expected_steer+1 ||
                bearing<expected_bearing-1 || bearing>expected_bearing+1)
                $fatal(1,"tracker %d,%d d=%d s=%d b=%d",x,y,distance,steer,bearing);
        end
    endtask
    initial begin
        repeat(3)@(negedge clk);rst_n=1;
        for(a=-1800;a<=1800;a=a+25)begin
            @(negedge clk);angle=a;start=1;@(negedge clk);start=0;
            wait(done);@(negedge clk);
            expected=$cos(a*3.141592653589793/1800)*32768;
            if($abs(cs-expected)>30)$fatal(1,"cos %d got %d expected %f",a,cs,expected);
            expected=$sin(a*3.141592653589793/1800)*32768;
            if($abs(sn-expected)>30)$fatal(1,"sin %d got %d expected %f",a,sn,expected);
        end
        track(100,0,100,0,0);track(100,100,141,28,450);
        track(100,-100,141,-28,-450);track(0,100,100,28,900);
        track(-100,0,100,0,1800);
        $display("PASS vehicle math: full-circle sin/cos and signed pure pursuit distance/bearing/steering");$finish;
    end
    initial begin #1000000;$fatal(1,"math timeout");end
endmodule
