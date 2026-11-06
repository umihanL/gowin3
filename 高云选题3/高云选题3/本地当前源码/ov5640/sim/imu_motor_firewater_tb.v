`timescale 1ns/1ps
module imu_motor_firewater_tb #(
    parameter integer CLK_HZ=33000000, BAUD=115200
);
    localparam integer BIT_CYCLES=(CLK_HZ+BAUD/2)/BAUD;
    localparam integer BIT_NS=BIT_CYCLES*10, PERIOD=CLK_HZ/20;
    reg clk=0;always #5 clk=~clk;
    reg rst_n=0,available=0;
    reg signed [15:0] roll=123,pitch=-456,yaw=789;
    reg signed [15:0] er=-111,ep=222,ey=-333;
    reg signed [17:0] qw=8192,qx=0,qy=-8192,qz=11585;
    reg signed [15:0] target1=200,actual1=187,target2=75,actual2=69;
    wire tx,busy;
    // sample_valid deliberately never arrives. Motor telemetry is periodic.
    imu_firewater_tx #(.CLK_HZ(CLK_HZ),.BAUD(BAUD),.SEND_HZ(20),
        .EXTENDED_POSE(0),.MOTOR_TELEMETRY(1)) dut(
        .clk(clk),.rst_n(rst_n),.sample_valid(1'b0),.available(available),
        .roll(roll),.pitch(pitch),.yaw(yaw),.euler_roll(er),.euler_pitch(ep),.euler_yaw(ey),
        .q_w(qw),.q_x(qx),.q_y(qy),.q_z(qz),
        .motor_target1(target1),.motor_actual1(actual1),.motor_target2(target2),.motor_actual2(actual2),
        .tx(tx),.busy(busy));
    integer cycle=0,frames=0,bit_number,line_start;
    integer starts[0:15];reg [7:0] rx_byte;string line="";string received[0:15];
    always @(posedge clk)cycle=cycle+1;
    always @(negedge rst_n)line="";
    initial forever begin
        @(negedge tx);
        if(rst_n)begin
            fork:receive_byte
                begin
                    if(line.len()==0)line_start=cycle;
                    #(BIT_NS/2);if(tx!==0)$fatal(1,"UART start bit");
                    for(bit_number=0;bit_number<8;bit_number=bit_number+1)begin
                        #BIT_NS;rx_byte[bit_number]=tx;
                    end
                    #BIT_NS;if(tx!==1)$fatal(1,"UART stop bit or baud");
                    line=$sformatf("%s%c",line,rx_byte);
                    if(rx_byte==10)begin
                        if(frames>=16)$fatal(1,"too many frames");
                        received[frames]=line;starts[frames]=line_start;frames=frames+1;line="";
                    end
                end
                begin @(negedge rst_n);end
            join_any
            disable receive_byte;
        end
    end
    task expect_frame;
        input integer n;input string expected;
        begin
            wait(frames>=n);
            if(frames!=n || received[n-1]!=expected)$fatal(1,"frame %0d expected [%s] got [%s]",n,expected,received[n-1]);
            wait(!busy);@(negedge clk);
        end
    endtask
    integer n;
    initial begin
        repeat(4)@(negedge clk);rst_n=1;
        // Startup with absent IMU still sends motor data at 20 Hz.
        expect_frame(1,"samples:0.0,0.0,0.0,200,187,75,69\015\n");
        available=1;
        wait(busy);repeat(50)@(negedge clk);
        // Data/availability changes during the frame must not mix its fields.
        available=0;target1=-200;actual1=-182;target2=-100;actual2=-93;
        roll=0;pitch=0;yaw=0;
        expect_frame(2,"samples:12.3,-45.6,78.9,200,187,75,69\015\n");
        expect_frame(3,"samples:0.0,0.0,0.0,-200,-182,-100,-93\015\n");
        // Integer speed extrema must not overflow the angle tenths format.
        target1=-32768;actual1=32767;target2=0;actual2=-1;
        expect_frame(4,"samples:0.0,0.0,0.0,-32768,32767,0,-1\015\n");
        available=1;roll=1;pitch=-1;yaw=1800;
        er=-900;ep=899;ey=1799;qw=16384;qx=0;qy=0;qz=0;
        target1=200;actual1=199;target2=200;actual2=201;
        expect_frame(5,"samples:0.1,-0.1,180.0,200,199,200,201\015\n");
        for(n=1;n<5;n=n+1)if(starts[n]-starts[n-1]!=PERIOD)
            $fatal(1,"20 Hz period %0d",starts[n]-starts[n-1]);
        wait(busy);repeat(2*BIT_CYCLES)@(negedge clk);rst_n=0;
        repeat(40)@(negedge clk);if(tx!==1 || busy)$fatal(1,"reset failed");
        rst_n=1;available=0;target1=0;actual1=0;target2=0;actual2=0;
        expect_frame(6,"samples:0.0,0.0,0.0,0,0,0,0\015\n");
        $display("PASS imu_motor_firewater: CLK=%0d BAUD=%0d; exactly seven TX-wire channels, absent/offline IMU zeros with live motor targets/feedback, integer mm/s signs/extrema, atomic snapshots, recovery, exact 20 Hz, reset",CLK_HZ,BAUD);
        $finish;
    end
    initial begin #(PERIOD*10*10);$fatal(1,"motor telemetry timeout");end
endmodule
