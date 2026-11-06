`timescale 1ns/1ps
module imu_rotation_tb #(parameter REFERENCES=4, CASES=816);
    reg clk=0;always #5 clk=~clk;
    reg rst_n=0,online=0,recalibrate=0,sample_valid=0;
    reg [15:0] qx_half=0,qy_half=0,qz_half=0;
    reg signed [15:0] gx=0,gy=0,gz=0,ax=0,ay=0,az=8197;
    wire calibrated,valid,available;wire [1:0] cal_state;
    wire signed [15:0] roll,pitch,yaw;
    wire signed [17:0] relative_w,relative_x,relative_y,relative_z;
    wire vector_valid,vector_available;
    wire signed [15:0] rot_x,rot_y,rot_z;
    reg direct_mode=0,direct_valid=0;
    reg signed [17:0] direct_w=16384,direct_x=0,direct_y=0,direct_z=0;
    imu_attitude #(.SETTLE_SAMPLES(4),.ZERO_SAMPLES(8)) attitude(.fusion_reset(1'b0),.*);
    imu_rotation_vector vector(.clk(clk),.rst_n(rst_n),.online(online & available & !recalibrate),
        .sample_valid(direct_mode ? direct_valid : valid),
        .q_w(direct_mode ? direct_w : relative_w),.q_x(direct_mode ? direct_x : relative_x),
        .q_y(direct_mode ? direct_y : relative_y),.q_z(direct_mode ? direct_z : relative_z),
        .valid(vector_valid),.available(vector_available),.rot_x(rot_x),.rot_y(rot_y),.rot_z(rot_z));
    reg [95:0] vectors [0:REFERENCES*(CASES+1)-1];
    integer index=0,reference_index,case_index,checks=0,errors=0,pulses=0;
    integer ex,ey,ez,dx,dy,dz,previous_pulses;
    integer trace;
    reg [47:0] reference_sample;
    always @(negedge clk) begin
        if(vector_valid) pulses=pulses+1;
    end
    task sample;
        input [47:0] halfs;
        begin
            @(negedge clk);{qz_half,qy_half,qx_half}=halfs;sample_valid=1;
            @(negedge clk);sample_valid=0;
            repeat(480)@(negedge clk);
        end
    endtask
    task direct_check;
        input signed [17:0] qw,qx,qy,qz;
        input integer expected_x,expected_y,expected_z;
        begin
            @(negedge clk);direct_mode=1;direct_w=qw;direct_x=qx;direct_y=qy;direct_z=qz;direct_valid=1;
            @(negedge clk);direct_valid=0;
            repeat(240)@(negedge clk);
            dx=rot_x-expected_x;dy=rot_y-expected_y;dz=rot_z-expected_z;
            if(dx>1 || dx< -1 || dy>1 || dy< -1 || dz>1 || dz< -1)
                $fatal(1,"Direct quaternion got %0d,%0d,%0d expected %0d,%0d,%0d",
                    rot_x,rot_y,rot_z,expected_x,expected_y,expected_z);
        end
    endtask
    task check;
        input [95:0] v;
        begin
            previous_pulses=pulses;
            sample(v[47:0]);
            if(pulses!=previous_pulses+1 || !vector_available || !calibrated || cal_state!=3)
                $fatal(1,"Lost sample or calibration ref=%0d case=%0d",reference_index,case_index);
            ex=$signed(v[63:48]);ey=$signed(v[79:64]);ez=$signed(v[95:80]);
            dx=rot_x-ex;dy=rot_y-ey;dz=rot_z-ez;
            if(dx>3 || dx< -3 || dy>3 || dy< -3 || dz>3 || dz< -3) begin
                if(errors<12)$display("rotation ref=%0d case=%0d actual=%0d,%0d,%0d expected=%0d,%0d,%0d",
                    reference_index,case_index,rot_x,rot_y,rot_z,ex,ey,ez);
                errors=errors+1;
            end
            if(case_index>=CASES-12 && case_index<CASES-6 &&
               (rot_x>1 || rot_x< -1 || rot_y>1 || rot_y< -1 || rot_z>1 || rot_z< -1))
                $fatal(1,"Round trip accumulated a false zero offset");
            if(case_index>=CASES-6 && (rot_y<95 || rot_y>105))
                $fatal(1,"Real stationary 10 degree offset was concealed");
            checks=checks+1;
            if(reference_index==0 && case_index>=603 && case_index<804)
                $fwrite(trace,"%0d,%0d,%0d,%0d,%0d,%0d,%0d\n",case_index-603,
                    roll,pitch,yaw,rot_x,rot_y,rot_z);
        end
    endtask
    initial begin
        trace=$fopen("sim/imu_rotation_trace.csv","w");
        $fwrite(trace,"sample,euler_roll,euler_pitch,euler_yaw,rot_x,rot_y,rot_z\n");
        $readmemh("sim/imu_rotation_vectors.mem",vectors);
        repeat(4)@(negedge clk);rst_n=1;online=1;
        for(reference_index=0;reference_index<REFERENCES;reference_index=reference_index+1) begin
            @(negedge clk);recalibrate=1;@(negedge clk);recalibrate=0;
            repeat(120)@(negedge clk);
            reference_sample=vectors[index][47:0];index=index+1;
            repeat(12)sample(reference_sample);
            if(!calibrated)$fatal(1,"Reference calibration failed");
            for(case_index=0;case_index<CASES;case_index=case_index+1) begin
                check(vectors[index]);index=index+1;
            end
        end
        direct_check(0,0,16384,0,0,1800,0);
        direct_check(0,0,-16384,0,0,-1800,0);
        direct_check(0,16384,0,0,1800,0,0);
        direct_check(0,0,0,16384,0,0,1800);
        direct_check(11585,0,11585,0,0,900,0);
        direct_check(-11585,0,-11585,0,0,900,0);
        direct_check(16384,0,1,0,0,0,0);
        direct_check(16384,0,0,0,0,0,0);
        @(negedge clk);direct_mode=0;
        // Abort in the middle of sqrt/atan/divide, then recover at a new zero.
        @(negedge clk);sample_valid=1;
        @(negedge clk);sample_valid=0;
        repeat(190)@(negedge clk);online=0;
        repeat(3)@(negedge clk);
        if(vector_available || vector_valid)$fatal(1,"Offline left a valid vector");
        online=1;repeat(120)@(negedge clk);
        repeat(12)sample(reference_sample);
        if(!calibrated || !vector_available || rot_x!=0 || rot_y!=0 || rot_z!=0)
            $fatal(1,"Offline/recalibration recovery consumed stale math");
        if(errors)$fatal(1,"%0d/%0d rotation-vector cases failed",errors,checks);
        $fclose(trace);
        $display("PASS imu_rotation: %0d compressed-sensor cases, X/Y/Z +/-150 round trips, +/-90 crossings, 9-degree axis misalignment, arbitrary zero, <=0.3-degree math error, no accumulated return bias, genuine 10-degree offset retained, +/-180 endpoints, q/-q equivalence, tiny angles, disconnect recovery",checks);
        $finish;
    end
    initial begin #100000000;$fatal(1,"rotation timeout");end
endmodule
