`timescale 1ns/1ps
module vehicle_profile_tb;
    reg clk=0;always #5 clk=~clk;
    reg rst_n=0,valid=0;reg [11:0] count=0;reg [3:0] selected=0;
    wire [10:0] index;reg [16:0] x,y;reg lock;
    reg [16:0] xs[0:255],ys[0:255];reg locks[0:255];
    always @(posedge clk)begin x<=xs[index];y<=ys[index];lock<=locks[index];end
    wire busy,ready;wire [3:0] segments;wire [15:0] length,lr,rr;
    wire signed [15:0] entry,exit_heading;wire [7:0] peak;
    wire signed [31:0] lc,rc;
    vehicle_route_profile dut(.execution_curve(),.execution_index(11'd1),.execution_tangent(),.clk(clk),.rst_n(rst_n),.route_valid(valid),.route_count(count),
        .route_index(index),.route_x_q8(x),.route_y_q8(y),.route_lock(lock),.selected_segment(selected),
        .busy(busy),.ready(ready),.segment_count(segments),.length_mm(length),
        .left_revs_tenths(lr),.right_revs_tenths(rr),.entry_heading(entry),.exit_heading(exit_heading),
        .peak_steer(peak),.left_counts(lc),.right_counts(rc));
    integer i;real theta;
    initial begin
        for(i=0;i<256;i=i+1)begin xs[i]=0;ys[i]=0;locks[i]=0;end
        repeat(3)@(negedge clk);rst_n=1;
        count=51;
        for(i=0;i<count;i=i+1)begin xs[i]=(500+i*20)*256/10;ys[i]=2700*256/10;locks[i]=i==25 || i==50;end
        valid=1;wait(ready);@(negedge clk);
        if(segments!=2 || length<499 || length>501 || lc<3759 || lc>3765 || rc!=lc || lr!=24 || rr!=24 || peak!=0)
            $fatal(1,"straight profile seg=%d len=%d l=%d r=%d rev=%d peak=%d",segments,length,lc,rc,lr,peak);
        selected=1;#1;if(length<499 || length>501)$fatal(1,"ordered segment selection");
        @(negedge clk);valid=0;repeat(3)@(negedge clk);if(ready || busy)$fatal(1,"profile invalidation");
        selected=0;count=80;
        for(i=0;i<count;i=i+1)begin
            theta=i*3.141592653589793/158;
            xs[i]=$rtoi((500+300*$sin(theta))*256/10);
            ys[i]=$rtoi((2700-300*(1-$cos(theta)))*256/10);locks[i]=i==count-1;
        end
        valid=1;wait(ready);@(negedge clk);
        if(segments!=1 || length<469 || length>473 || lc<2500 || lc>2600 || rc<4490 || rc>4590 || peak<23 || peak>34 || exit_heading<885 || exit_heading>900)
            $fatal(1,"curve profile len=%d l=%d r=%d peak=%d entry=%d exit=%d",length,lc,rc,peak,entry,exit_heading);
        $display("PASS route profile: ordered locked segments, millimetres, per-wheel counts/revolutions, yaw endpoints, curvature and invalidation");$finish;
    end
    initial begin #10000000;$fatal(1,"profile timeout");end
endmodule
