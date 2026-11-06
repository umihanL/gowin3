`timescale 1ns/1ps
module range_limit_tb;
    reg clk=0;always #5 clk=~clk;
    reg rst_n=0;reg [15:0] distance_mm=0,slow_mm=650,cruise=200;
    wire [15:0] limit;
    vehicle_range_limit dut(.clk(clk),.rst_n(rst_n),.distance_mm(distance_mm),
        .slow_mm(slow_mm),.stop_mm(16'd160),.cruise(cruise),.speed_limit(limit));
    integer zone,d,v,expected,cases=0;
    initial begin
        repeat(4)@(negedge clk);rst_n=1;
        for(zone=250;zone<=2000;zone=zone+10)begin
            slow_mm=zone;
            for(v=0;v<=200;v=v+20)begin
                cruise=v;
                for(d=0;d<=2100;d=d+37)begin
                    distance_mm=d;repeat(100)@(negedge clk);
                    expected=d<=160?0:d>=zone?v:((d-160)*v)/(zone-160);
                    if(limit!==expected)$fatal(1,"range limiter d=%d zone=%d v=%d expected=%d actual=%d",d,zone,v,expected,limit);
                    cases=cases+1;
                end
            end
        end
        $display("PASS distance limiter: %d independent integer-reference cases, all 1cm zones 25..200cm",cases);$finish;
    end
endmodule
