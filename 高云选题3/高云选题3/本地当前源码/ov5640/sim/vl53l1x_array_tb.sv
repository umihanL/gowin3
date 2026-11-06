`timescale 1ns/1ps
module vl53l1x_array_tb;
    localparam integer CLK_HZ=2000000;
    reg clk=0; always #250 clk=~clk;
    reg rst_n=0;
    reg [3:0] present=4'b1111,invalid_range=0,never_ready=0,nack_config=0;
    tri1 sda,scl;
    wire [3:0] xs;
    wire [15:0] d0,d1,d2,d3;
    wire frame_valid;
    wire [3:0] samples,observations;
    wire [47:0] debug0,debug1,debug2,debug3;
    vl53l1x_array_controller #(.CLK_HZ(CLK_HZ),.I2C_HZ(100000),
        .RESET_WAIT_MS(1),.ENABLE_WAIT_MS(1),.SAMPLE_MS(60),.STALE_MS(180),.RETRY_MS(180)) dut(
        .clk(clk),.rst_n(rst_n),.xs1(xs[0]),.xs2(xs[1]),.xs3(xs[2]),.xs4(xs[3]),
        .i2c_sda(sda),.i2c_scl(scl),.distance0(d0),.distance1(d1),
        .distance2(d2),.distance3(d3),.frame_valid(frame_valid),
        .sample_valid(samples),.observation_valid(observations),
        .debug0(debug0),.debug1(debug1),.debug2(debug2),.debug3(debug3));
    vl53l1x_sensor_model #(.SENSOR(0),.DISTANCE(123)) s0(rst_n,xs[0],present[0],invalid_range[0],never_ready[0],nack_config[0],sda,scl);
    vl53l1x_sensor_model #(.SENSOR(1),.DISTANCE(456)) s1(rst_n,xs[1],present[1],invalid_range[1],never_ready[1],nack_config[1],sda,scl);
    vl53l1x_sensor_model #(.SENSOR(2),.DISTANCE(1000)) s2(rst_n,xs[2],present[2],invalid_range[2],never_ready[2],nack_config[2],sda,scl);
    vl53l1x_sensor_model #(.SENSOR(3),.DISTANCE(2345)) s3(rst_n,xs[3],present[3],invalid_range[3],never_ready[3],nack_config[3],sda,scl);

    integer pulse_count=0;
    always @(posedge clk) if(samples!=0) pulse_count=pulse_count+1;
    initial begin
        #5000; rst_n=1;
        wait(frame_valid);
        if(xs!==4'b1111 || s0.address!=7'h2a || s1.address!=7'h2b ||
           s2.address!=7'h2c || s3.address!=7'h2d)
            $fatal(1,"VL53L1X address assignment failed: %b",xs);
        if({d3,d2,d1,d0}!=={16'd2345,16'd1000,16'd456,16'd123})
            $fatal(1,"VL53L1X distance readback failed: %0d %0d %0d %0d",d0,d1,d2,d3);
        if(s0.config_count<91 || s1.config_count<91 || s2.config_count<91 || s3.config_count<91)
            $fatal(1,"VL53L1X config incomplete: %0d %0d %0d %0d",s0.config_count,s1.config_count,s2.config_count,s3.config_count);
        if(s0.start_count<2 || s1.start_count<2 || s2.start_count<2 || s3.start_count<2 ||
           s0.stop_count!=1 || s1.stop_count!=1 || s2.stop_count!=1 || s3.stop_count!=1)
            $fatal(1,"VL53L1X first-range calibration sequence incomplete");
        if((debug0[47:40]|debug1[47:40]|debug2[47:40]|debug3[47:40])!=0)
            $fatal(1,"VL53L1X unexpected channel diagnostic");
        if(pulse_count!=4) $fatal(1,"sample-valid pulses missing: %0d",pulse_count);
        @(negedge clk);invalid_range[2]=1;
        wait(d2==16'hffff);
        if(debug2[47:40]!=8'h11 || observations[2]!==1'b1)
            $fatal(1,"invalid status was not reported");
        $display("PASS VL53L1X array: four XSHUT addresses, ST configuration, calibration, 16-bit register I2C, range/status readback and sample pulses");
        @(negedge clk);invalid_range[2]=0;
        wait(d2==1000);
        // Remove one physical slave, then force its configuration to NACK.
        // The three other assigned addresses must remain usable throughout.
        @(negedge clk);present[1]=0;
        wait(d1==16'hffff);wait(!xs[1]);
        if(d0!=123 || d2!=1000 || d3!=2345 || debug1[47:40]==0)
            $fatal(1,"one missing L1X corrupted healthy channels");
        nack_config[1]=1;present[1]=1;
        wait(s1.config_count>20);wait(!xs[1]);
        if(d1!=16'hffff || d0!=123 || d2!=1000 || d3!=2345)
            $fatal(1,"configuration NACK was ignored or stalled array");
        nack_config[1]=0;wait(d1==456);
        if(s0.address!=7'h2a || s1.address!=7'h2b || s2.address!=7'h2c || s3.address!=7'h2d)
            $fatal(1,"channel retry changed another L1X address");
        @(negedge clk);never_ready[3]=1;
        wait(d3==16'hffff);
        if(debug3[47:40]!=8'h10 || d0!=123 || d1!=456 || d2!=1000)
            $fatal(1,"stale L1X did not isolate its own channel");
        never_ready[3]=0;wait(d3==2345);
        $display("PASS VL53L1X recovery: invalid range, missing slave, configuration NACK and data-ready timeout isolate one channel; retry restores its address and samples");
        $finish;
    end
    initial begin #3000000000;$fatal(1,"VL53L1X array timeout xs=%b state=%0d op=%0d",xs,dut.state,dut.operation);end
endmodule
