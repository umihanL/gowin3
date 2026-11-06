`timescale 1ns/1ps
module imu_firewater_tb #(
    parameter integer CLK_HZ = 33000000,
    parameter integer BAUD = 115200,
    parameter integer SEND_HZ = 20,
    parameter integer EXTENDED_POSE = 0
);
    localparam integer BIT_CYCLES = (CLK_HZ + BAUD/2) / BAUD;
    localparam integer BIT_NS = BIT_CYCLES * 10;
    localparam integer PERIOD = CLK_HZ / SEND_HZ;
    reg clk = 0;
    always #5 clk = ~clk;
    reg rst_n = 0, sample_valid = 0, available = 0;
    reg signed [15:0] roll = 0, pitch = 0, yaw = 0;
    reg signed [15:0] euler_roll=0,euler_pitch=0,euler_yaw=0;
    reg signed [17:0] q_w=16384,q_x=0,q_y=0,q_z=0;
    wire tx, busy;
    imu_firewater_tx #(.CLK_HZ(CLK_HZ),.BAUD(BAUD),.SEND_HZ(SEND_HZ),.EXTENDED_POSE(EXTENDED_POSE)) dut(
        .clk(clk),.rst_n(rst_n),.sample_valid(sample_valid),.available(available),
        .roll(roll),.pitch(pitch),.yaw(yaw),
        .euler_roll(euler_roll),.euler_pitch(euler_pitch),.euler_yaw(euler_yaw),
        .q_w(q_w),.q_x(q_x),.q_y(q_y),.q_z(q_z),.tx(tx),.busy(busy));

    // Independent 8N1 receiver: observe only the actual UART wire, at bit centers.
    integer cycle = 0, frames = 0, bit_number, line_start;
    integer starts [0:15];
    reg [7:0] rx_byte;
    string line = "";
    string received [0:15];
    always @(posedge clk) cycle = cycle + 1;
    always @(negedge rst_n) line = "";
    initial forever begin
        @(negedge tx);
        if (rst_n) begin
            fork : receive_byte
                begin
                    if (line.len() == 0) line_start = cycle;
                    #(BIT_NS/2);
                    if (tx !== 0) $fatal(1,"UART start bit");
                    for (bit_number = 0; bit_number < 8; bit_number = bit_number + 1) begin
                        #BIT_NS;
                        if (tx !== 0 && tx !== 1) $fatal(1,"UART unknown data bit");
                        rx_byte[bit_number] = tx;
                    end
                    #BIT_NS;
                    if (tx !== 1) $fatal(1,"UART stop bit / baud mismatch");
                    line = $sformatf("%s%c",line,rx_byte);
                    if (rx_byte == 8'h0a) begin
                        if (frames >= 16) $fatal(1,"Unexpected repeated frames");
                        received[frames] = line;
                        starts[frames] = line_start;
                        frames = frames + 1;
                        line = "";
                    end
                end
                begin @(negedge rst_n); end
            join_any
            disable receive_byte;
        end
    end

    task sample;
        input signed [15:0] r,p,y;
        begin
            @(negedge clk);roll=r;pitch=p;yaw=y;sample_valid=1;
            @(negedge clk);sample_valid=0;
        end
    endtask
    task expect_frame;
        input integer count;
        input string expected;
        begin
            wait(frames >= count);
            if (frames != count || received[count-1] != expected)
                $fatal(1,"Frame %0d expected [%s] got [%s]",count,expected,received[count-1]);
            wait(!busy);
            @(negedge clk);
        end
    endtask
    task expect_quiet;
        input integer clocks,count;
        begin
            repeat(clocks) @(negedge clk);
            if (frames != count || tx !== 1 || busy)
                $fatal(1,"Stale/offline sample transmitted: expected %0d frames got %0d",count,frames);
        end
    endtask

    initial begin
        repeat(4) @(negedge clk);rst_n=1;
        sample(1,2,3);
        expect_quiet(100,0);
        available=1;
        expect_quiet(100,0);

        euler_roll=-1799;euler_pitch=899;euler_yaw=-1234;
        q_w=-8192;q_x=1;q_y=-11585;q_z=16384;
        sample(123,-456,1799);
        if (!busy) $fatal(1,"First valid sample not accepted");
        // Neither a newer sample nor communication loss may tear this frame.
        euler_roll=0;euler_pitch=0;euler_yaw=0;
        q_w=16384;q_x=0;q_y=0;q_z=0;
        sample(-2,3,-4);
        repeat(BIT_CYCLES*12) @(negedge clk);
        available=0;
        if(EXTENDED_POSE) expect_frame(1,"samples:12.3,-45.6,179.9,-179.9,89.9,-123.4,-0.5000,0.0001,-0.7071,1.0000\015\n");
        else expect_frame(1,"samples:12.3,-45.6,179.9\015\n");
        sample(50,60,70);
        expect_quiet(PERIOD+BIT_CYCLES*12,1);
        available=1;
        expect_quiet(BIT_CYCLES*12,1);

        sample(0,-1,1);
        if(EXTENDED_POSE) expect_frame(2,"samples:0.0,-0.1,0.1,0.0,0.0,0.0,1.0000,0.0000,0.0000,0.0000\015\n");
        else expect_frame(2,"samples:0.0,-0.1,0.1\015\n");
        // A pulse during the rate limit must not be replayed later.
        sample(900,-900,1800);
        expect_quiet(PERIOD+BIT_CYCLES*12,2);

        euler_roll=-32768;euler_pitch=32767;euler_yaw=0;
        q_w=0;q_x=16384;q_y=-16384;q_z=8192;
        sample(1800,-1800,16'sh8000);
        if(EXTENDED_POSE) expect_frame(3,"samples:180.0,-180.0,-3276.8,-3276.8,3276.7,0.0,0.0000,1.0000,-1.0000,0.5000\015\n");
        else expect_frame(3,"samples:180.0,-180.0,-3276.8\015\n");
        euler_roll=0;euler_pitch=0;euler_yaw=0;
        q_w=8191;q_x=-8191;q_y=0;q_z=-1;
        @(negedge clk);roll=32767;pitch=100;yaw=-100;sample_valid=1;
        if(EXTENDED_POSE) expect_frame(4,"samples:3276.7,10.0,-10.0,0.0,0.0,0.0,0.4999,-0.4999,0.0000,-0.0001\015\n");
        else expect_frame(4,"samples:3276.7,10.0,-10.0\015\n");
        sample_valid=0;
        if (starts[3]-starts[2] < PERIOD || starts[3]-starts[2] > PERIOD+4)
            $fatal(1,"Rate limit mismatch: %0d cycles",starts[3]-starts[2]);
        expect_quiet(2*PERIOD,4);

        // Reset in the middle of a UART byte must return TX to idle, then
        // start the next fresh sample with a new complete FireWater prefix.
        sample(-999,999,0);
        repeat(2*BIT_CYCLES) @(negedge clk);
        rst_n=0;
        expect_quiet(50,4);
        rst_n=1;
        q_w=16384;q_x=0;q_y=0;q_z=0;
        sample(-900,900,0);
        if(EXTENDED_POSE) expect_frame(5,"samples:-90.0,90.0,0.0,0.0,0.0,0.0,1.0000,0.0000,0.0000,0.0000\015\n");
        else expect_frame(5,"samples:-90.0,90.0,0.0\015\n");
        if(EXTENDED_POSE)
            $display("PASS imu_pose_firewater: CLK=%0d BAUD=%0d; TX-wire 8N1, ten channels RotXYZ/Euler/WXYZ, signed tenths/four decimals, quaternion rounding, atomic frame, 20 Hz, offline, reset",CLK_HZ,BAUD);
        else
            $display("PASS imu_firewater: CLK=%0d BAUD=%0d; TX-wire 8N1, Roll/Pitch/Yaw, signed tenths, zero, extrema, atomic frame, 20 Hz limit, no stale/offline replay, reset",CLK_HZ,BAUD);
        $finish;
    end
    initial begin
        #(PERIOD*10*20);
        $fatal(1,"FireWater simulation timeout");
    end
endmodule
