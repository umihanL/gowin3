`timescale 1ns/1ps
module fix_fifo_tb;
    reg wclk=0, uclk=0, rclk=0;
    always #7 wclk=~wclk;
    always #5 uclk=~uclk;
    always #11 rclk=~rclk;
    reg reset=1, wen=0, ren=0;
    reg [15:0] data=0;
    wire [127:0] packed_data;
    wire [15:0] q;
    wire wempty, wfull, rempty, rfull;
    wire [9:0] groups;
    wire transfer = !reset && !wempty && !rfull;
    wr_data_fifo wr (.Data(data), .Reset(reset), .WrClk(wclk), .RdClk(uclk),
        .WrEn(wen), .RdEn(transfer), .Wnum(), .Rnum(groups), .Almost_Empty(),
        .Almost_Full(), .Q(packed_data), .Empty(wempty), .Full(wfull));
    rd_data_fifo rd (.Data(packed_data), .Reset(reset), .WrClk(uclk), .RdClk(rclk),
        .WrEn(transfer), .RdEn(ren && !rempty), .Wnum(), .Rnum(), .Almost_Empty(),
        .Almost_Full(), .Q(q), .Empty(rempty), .Full(rfull));
    integer i, received=0;
    function [15:0] pattern;
        input integer n;
        pattern = (n * 97) ^ 16'ha65c;
    endfunction
    always @(posedge rclk) begin
        if (reset) received=0;
        else if (ren && !rempty) begin
            if (q !== pattern(received))
                $fatal(1,"FIFO pixel %0d: got %h, expected %h",received,q,pattern(received));
            received=received+1;
        end
    end
    task send_pixels;
        begin
            for(i=0;i<1024;i=i+1) begin
                @(negedge wclk);
                while(wfull) @(negedge wclk);
                wen=1; data=pattern(i);
            end
            @(negedge wclk); wen=0;
            wait(received==1024);
            repeat(20) @(negedge rclk);
            if (!rempty || received!=1024) $fatal(1,"FIFO did not drain exactly");
        end
    endtask
    initial begin
        #1000; reset=0; #500; ren=1;
        send_pixels;
        @(negedge rclk); reset=1;
        #1000; reset=0; #500;
        send_pixels;
        $display("PASS: regenerated FIFOs, 16/128/16-bit order, FWFT, async clocks and reset");
        $finish;
    end
    initial begin #200000; $fatal(1,"FIFO timeout"); end
endmodule
