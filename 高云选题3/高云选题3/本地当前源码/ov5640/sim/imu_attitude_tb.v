`timescale 1ns/1ps
module imu_attitude_tb;
    reg clk=0;always #5 clk=~clk;
    reg rst_n=0,online=0,recalibrate=0,sample_valid=0,fusion_reset=0;
    reg [15:0] qx=0,qy=0,qz=0;
    reg signed [15:0] gx=0,gy=0,gz=0,ax=0,ay=0,az=8197;
    wire calibrated,valid,available;wire [1:0] cal_state;wire signed [15:0] roll,pitch,yaw;
    imu_attitude #(.SETTLE_SAMPLES(4),.ZERO_SAMPLES(8)) dut(.fusion_reset(fusion_reset),
        .clk(clk),.rst_n(rst_n),.online(online),.recalibrate(recalibrate),.sample_valid(sample_valid),
        .qx_half(qx),.qy_half(qy),.qz_half(qz),.gx(gx),.gy(gy),.gz(gz),.ax(ax),.ay(ay),.az(az),
        .calibrated(calibrated),.valid(valid),.available(available),.cal_state(cal_state),.roll(roll),.pitch(pitch),.yaw(yaw));
    reg [95:0] vectors[0:255];integer i,errors=0;
    integer er,ep,ey,dr,dp,dy;
    task sample;
        input [47:0] halfs;
        begin
            @(negedge clk);{qz,qy,qx}=halfs;sample_valid=1;
            @(negedge clk);sample_valid=0;
            repeat(250) @(negedge clk);
        end
    endtask
    task check;
        input [95:0] v;
        begin
            sample(v[47:0]);
            er=$signed(v[63:48]);ep=$signed(v[79:64]);ey=$signed(v[95:80]);
            dr=roll-er;dp=pitch-ep;dy=yaw-ey;
            if(dr>1800)dr=dr-3600;if(dr< -1800)dr=dr+3600;
            if(dy>1800)dy=dy-3600;if(dy< -1800)dy=dy+3600;
            if(dr>12 || dr< -12 || dp>12 || dp< -12 || dy>12 || dy< -12) begin
                $display("vector %0d got %0d,%0d,%0d expected %0d,%0d,%0d",i,roll,pitch,yaw,er,ep,ey);errors=errors+1;
            end
        end
    endtask
    initial begin
        $readmemh("sim/imu_vectors.mem",vectors);
        repeat(4) @(negedge clk);rst_n=1;online=1;repeat(120) @(negedge clk);
        gx=300;
        repeat(20) sample(0);
        if(calibrated)$fatal(1,"must not zero while rotating");
        if(!available || cal_state!=1)$fatal(1,"live pre-zero angles or movement status missing");
        gx=0;az=0;
        repeat(12)sample(0);
        if(calibrated)$fatal(1,"must not zero during free fall");
        az=8197;
        // Sensor can be stationary with raw bias > old 1.96dps gate.
        gx=40;gy=-35;gz=31;
        repeat(8)sample(0);
        gx=0;gy=0;gz=0;
        if(!calibrated)$fatal(1,"stationary zero failed");
        for(i=0;i<200;i=i+1)check(vectors[i]);
        // Set a nontrivial reference: quaternion subtraction must not be used.
        @(negedge clk);recalibrate=1;@(negedge clk);recalibrate=0;
        repeat(120)@(negedge clk);
        repeat(12)sample(vectors[200][47:0]);
        if(!calibrated)$fatal(1,"re-zero failed");
        for(i=200;i<240;i=i+1)check(vectors[i]);
        // Quaternion sign can flip around 180 degrees without any motion.
        @(negedge clk);recalibrate=1;@(negedge clk);recalibrate=0;
        repeat(120)@(negedge clk);az=-8197;
        for(i=0;i<12;i=i+1)sample(i[0] ? 48'h0000_0000_bc00 : 48'h0000_0000_3c00);
        if(!calibrated)$fatal(1,"equivalent q/-q at 180 degrees rejected during zero");
        sample(48'h0000_0000_3c00);
        if(roll>1 || roll< -1 || pitch>1 || pitch< -1 || yaw>1 || yaw< -1)
            $fatal(1,"180-degree relative zero is not identity");
        online=0;repeat(4)@(negedge clk);
        if(!calibrated || available)$fatal(1,"transport disconnect must preserve zero but mark unavailable");
        online=1;repeat(120)@(negedge clk);sample(48'h0000_0000_3c00);
        if(!calibrated || !available || roll>1 || roll< -1)$fatal(1,"reconnect lost saved reference");
        // A complete sensor restart must retain a nonzero user-relative
        // orientation; only the sensor frame changes, never the user zero.
        az=8197;recalibrate=1;@(negedge clk);recalibrate=0;
        repeat(120)@(negedge clk);repeat(12)sample(0);
        sample(48'h3000_0000_0000); // about 14.36 deg yaw
        ey=yaw;
        if(ey<100 || ey>180)$fatal(1,"nonzero reference stimulus missing");
        online=0;fusion_reset=1;@(negedge clk);fusion_reset=0;
        repeat(120)@(negedge clk);online=1;repeat(120)@(negedge clk);
        sample(0); // new sensor coordinate starts at identity
        if(!calibrated || !available || yaw<ey-3 || yaw>ey+3)
            $fatal(1,"full sensor restart silently re-zeroed %0d -> %0d",ey,yaw);
        sample(48'h3000_0000_0000);
        if(yaw<2*ey-5 || yaw>2*ey+5)$fatal(1,"post-restart incremental yaw lost %0d",yaw);
        $display("PASS IMU full restart reference bridge: nonzero angle retained, fresh rotation still measured, no user re-zero");
        recalibrate=1;@(negedge clk);recalibrate=0;
        if(calibrated)$fatal(1,"explicit fusion reset must invalidate reference");
        if(errors)$fatal(1,"%0d quaternion cases failed",errors);
        $display("PASS imu_attitude: 240 quaternion cases, arbitrary zero reference, raw bias tolerance, q/-q zero, pre-zero live angles, motion/freefall rejection, disconnect");
        $finish;
    end
    initial begin #20000000;$fatal(1,"timeout");end
endmodule
