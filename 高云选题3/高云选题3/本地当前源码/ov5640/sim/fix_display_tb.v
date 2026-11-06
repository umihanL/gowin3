`timescale 1ns/1ps
module fix_display_tb;
    reg clk = 0;
    reg clock_enable = 1;
    always #5 if (clock_enable) clk = ~clk;
    reg locked = 0;
    wire rst_n;
    reset_sync reset_dut (.clk(clk), .arst_n(locked), .rst_n(rst_n));
    reg [15:0] fifo_q = 16'hb96a;
    wire req, hs, vs, de, pclk, frame_begin;
    wire [11:0] x, y;
    wire [15:0] rgb;
    disp_driver dut (
        .ClkDisp(clk), .Rst_n(rst_n), .Data(fifo_q), .DataReq(req),
        .H_Addr(x), .V_Addr(y), .Disp_HS(hs), .Disp_VS(vs),
        .Disp_Red(rgb[15:11]), .Disp_Green(rgb[10:5]),
        .Disp_Blue(rgb[4:0]), .Frame_Begin(frame_begin),
        .Disp_DE(de), .Disp_PCLK(pclk)
    );
    // Model the existing first-word-fall-through FIFO contract.
    always @(posedge clk)
        if (req) fifo_q <= fifo_q + 1'b1;

    integer f, h, v, pixels, starts;
    reg expected_active;
    reg [15:0] expected_rgb;
    initial begin
        #2;
        if (rst_n !== 0) $fatal(1, "reset must start asserted");
        repeat (4) @(negedge clk);
        locked = 1;
        repeat (2) begin
            @(posedge clk); #1;
            if (rst_n !== 0) $fatal(1, "reset released before three clocks");
        end
        @(posedge clk); #1;
        if (rst_n !== 1) $fatal(1, "reset did not release");
        for (f = 0; f < 2; f = f + 1) begin
            pixels = 0; starts = 0;
            for (v = 0; v < 525; v = v + 1)
                for (h = 0; h < 1056; h = h + 1) begin
                    @(negedge clk);
                    expected_active = h >= 215 && h < 1015 && v >= 34 && v < 514;
                    expected_rgb = expected_active ? fifo_q : 16'h0000;
                    if (req !== expected_active) $fatal(1, "request timing mismatch");
                    if (expected_active && (x != h-215 || y != v-34))
                        $fatal(1, "pixel address mismatch");
                    @(posedge clk); #1;
                    if (de !== expected_active || rgb !== expected_rgb)
                        $fatal(1, "RGB and DE not aligned at %0d,%0d", h, v);
                    if (hs !== (h >= 128) || vs !== (v >= 2))
                        $fatal(1, "sync timing mismatch");
                    if (de) pixels = pixels + 1;
                    if (frame_begin) starts = starts + 1;
                end
            if (pixels != 384000 || starts != 1)
                $fatal(1, "bad frame size or frame pulse count");
        end
        @(negedge clk); clock_enable = 0;
        #2; locked = 0; #1;
        if (rst_n !== 0 || de !== 0 || rgb !== 0 || frame_begin !== 0)
            $fatal(1, "PLL loss must reset display without a clock");
        $display("PASS: two 800x480 frames, 1056x525 raster, FWFT pixel alignment, PLL reset");
        $finish;
    end
endmodule
