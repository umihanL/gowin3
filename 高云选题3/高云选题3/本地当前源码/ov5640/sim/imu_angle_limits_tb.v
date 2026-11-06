`timescale 1ns/1ps
module imu_angle_limits_tb #(
    parameter REFERENCE_COUNT=8, ROLL_CASES=20, PITCH_CASES=27
);
    reg clk=0;always #5 clk=~clk;
    reg rst_n=0,online=0,recalibrate=0,sample_valid=0;
    reg [15:0] qx_half=0,qy_half=0,qz_half=0;
    reg signed [15:0] gx=0,gy=0,gz=0,ax=0,ay=0,az=8197;
    wire calibrated,valid,available;
    wire [1:0] cal_state;
    wire signed [15:0] roll,pitch,yaw;
    wire signed [17:0] relative_w,relative_x,relative_y,relative_z;
    imu_attitude #(.SETTLE_SAMPLES(4),.ZERO_SAMPLES(8)) dut(.fusion_reset(1'b0),.*);
    localparam RECORDS=REFERENCE_COUNT*(1+ROLL_CASES+PITCH_CASES);
    reg [95:0] vectors[0:RECORDS-1];
    integer reference_index,case_index,index=0,errors=0,checks=0;
    integer dr,dp,dy,er,ep,ey,pitch_tolerance;
    task sample;
        input [47:0] halfs;
        begin
            @(negedge clk);{qz_half,qy_half,qx_half}=halfs;sample_valid=1;
            @(negedge clk);sample_valid=0;
            repeat(250)@(negedge clk);
        end
    endtask
    task check_angles;
        input [95:0] vector;
        input check_outer_angles;
        begin
            sample(vector[47:0]);
            if(!calibrated || !available || cal_state!=3)
                $fatal(1,"normal rotation discarded calibration");
            er=$signed(vector[63:48]);ep=$signed(vector[79:64]);ey=$signed(vector[95:80]);
            if(!check_outer_angles && reference_index<2 && (ep==900 || ep==-900))
                $display("INFO imu pitch ref=%0d expected=%0d measured=%0d tenths of a degree",
                    reference_index,ep,pitch);
            dr=roll-er;dp=pitch-ep;dy=yaw-ey;
            if(dr>1800)dr=dr-3600;if(dr< -1800)dr=dr+3600;
            if(dy>1800)dy=dy-3600;if(dy< -1800)dy=dy+3600;
            // Near absolute 180 degrees, binary16 XYZ leaves little
            // information for reconstructing W. Retain the existing 1.2
            // degree bound for full roll sweeps; pitch sweeps use 0.3.
            pitch_tolerance=check_outer_angles ? 12 : 3;
            if(dp>pitch_tolerance || dp< -pitch_tolerance ||
               (check_outer_angles && (dr>12 || dr< -12 || dy>12 || dy< -12))) begin
                if(errors<12)
                    $display("angle boundary ref=%0d case=%0d outer=%0d got=%0d,%0d,%0d expected=%0d,%0d,%0d",
                        reference_index,case_index,check_outer_angles,roll,pitch,yaw,er,ep,ey);
                errors=errors+1;
            end
            checks=checks+1;
        end
    endtask
    initial begin
        $readmemh("sim/imu_angle_limits_vectors.mem",vectors);
        repeat(4)@(negedge clk);rst_n=1;online=1;
        for(reference_index=0;reference_index<REFERENCE_COUNT;reference_index=reference_index+1) begin
            @(negedge clk);recalibrate=1;@(negedge clk);recalibrate=0;
            repeat(120)@(negedge clk);
            repeat(12)sample(vectors[index][47:0]);index=index+1;
            if(!calibrated)$fatal(1,"boundary reference did not calibrate");
            for(case_index=0;case_index<ROLL_CASES;case_index=case_index+1) begin
                check_angles(vectors[index],1'b1);index=index+1;
            end
            for(case_index=0;case_index<PITCH_CASES;case_index=case_index+1) begin
                check_angles(vectors[index],1'b0);index=index+1;
            end
        end
        if(errors)$fatal(1,"%0d/%0d angle boundary cases failed",errors,checks);
        $display("PASS imu_angle_limits: %0d rotations, %0d zero references, pitch +/-90 within 0.3 degrees, roll through +/-180, calibration retained",
            checks,REFERENCE_COUNT);
        $finish;
    end
    initial begin #20000000;$fatal(1,"angle boundary timeout");end
endmodule
