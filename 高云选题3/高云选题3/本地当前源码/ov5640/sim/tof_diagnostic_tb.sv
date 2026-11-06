`timescale 1ns/1ps
module tof_diagnostic_tb;
    reg clk=0;always #5 clk=~clk;
    reg rst_n=0;
    reg [3:0] present=15,invalid=0,not_ready=0;
    tri1 sda,scl;wire [3:0] xs;
    wire [15:0] d0,d1,d2,d3;wire frame;
    wire [47:0] e0,e1,e2,e3;
    wire [3:0] samples;integer sample_count[0:3];integer j,before_count;
    always @(posedge clk)if(rst_n)for(j=0;j<4;j=j+1)if(samples[j])sample_count[j]=sample_count[j]+1;
    vl53l0x_array_controller #(.CLK_HZ(200000),.I2C_HZ(100000),
        .ENABLE_WAIT_MS(1),.RESET_WAIT_MS(1),.POLL_WAIT_MS(1),
        .READY_TIMEOUT_MS(30),.SAMPLE_MS(10),.STALE_MS(40),.RETRY_MS(50)) dut(
        .clk(clk),.rst_n(rst_n),.xs1(xs[0]),.xs2(xs[1]),.xs3(xs[2]),.xs4(xs[3]),
        .i2c_sda(sda),.i2c_scl(scl),.distance0(d0),.distance1(d1),.distance2(d2),.distance3(d3),
        .frame_valid(frame),.sample_valid(samples),.debug0(e0),.debug1(e1),.debug2(e2),.debug3(e3));
    vl53l0x_sensor_model #(.SENSOR(0),.DISTANCE(123)) s0(rst_n,xs[0],present[0],invalid[0],not_ready[0],1'b0,1'b0,1'b0,sda,scl);
    vl53l0x_sensor_model #(.SENSOR(1),.DISTANCE(456)) s1(rst_n,xs[1],present[1],invalid[1],not_ready[1],1'b0,1'b0,1'b0,sda,scl);
    vl53l0x_sensor_model #(.SENSOR(2),.DISTANCE(1000)) s2(rst_n,xs[2],present[2],invalid[2],not_ready[2],1'b0,1'b0,1'b0,sda,scl);
    vl53l0x_sensor_model #(.SENSOR(3),.DISTANCE(1800)) s3(rst_n,xs[3],present[3],invalid[3],not_ready[3],1'b0,1'b0,1'b0,sda,scl);
    initial begin
        for(j=0;j<4;j=j+1)sample_count[j]=0;
        repeat(4)@(negedge clk);rst_n=1;
        wait(d0==123 && d1==456 && d2==1000 && d3==1800);
        if(e0[47:40] || e1[47:40] || e2[47:40] || e3[47:40])$fatal(1,"valid ToF error code");
        invalid[1]=1;wait(e1[47:40]==8'h11);@(negedge clk);
        if(d1!=65535 || d0!=123 || d2!=1000 || d3!=1800)$fatal(1,"invalid range isolation");
        invalid[1]=0;wait(d1==456);
        present[3]=0;wait(e3[47:40]==1);@(negedge clk);
        if(d3!=65535)$fatal(1,"NACK was displayed as a distance");
        present[3]=1;wait(d3==1800);
        not_ready[2]=1;repeat(20)@(negedge clk);before_count=sample_count[2];
        wait(e2[47:40]==8'h10);@(negedge clk);
        if(d2!=65535)$fatal(1,"stale range remains visible");
        if(sample_count[2]!=before_count)$fatal(1,"not-ready channel emitted a new range pulse");
        not_ready[2]=0;wait(d2==1000);
        repeat(3)@(negedge clk);
        for(j=0;j<4;j=j+1)if(sample_count[j]==0)$fatal(1,"missing per-channel measurement pulses");
        $display("PASS ToF diagnostics: four independent sensors, E00 valid, E01 NACK, E11 invalid, E10 stale, isolation and recovery");
        $display("PASS ToF sample pulses: only new valid per-channel measurements refresh distance memory; not-ready scans do not");
        $finish;
    end
    initial begin #100000000;$fatal(1,"ToF recovery timeout");end
endmodule
