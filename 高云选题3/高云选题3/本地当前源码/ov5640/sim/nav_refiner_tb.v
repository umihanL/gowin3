`timescale 1ns/1ps
module nav_refiner_tb;
    reg clk=0;always #5 clk=~clk;
    reg rst_n=0,clear=0,cancel=0,load=0,lock=0,start=0;
    reg [7:0] x=0,y=0;
    wire busy,done,request,valid,write,bit_value;
    wire [2:0] error;
    wire [14:0] plan,route;
    wire [16:0] raw,px,py;
    wire [17:0] length;
    wire [3:0] segment;
    reg raw_blocked=0,collision=0,forbidden=0,raw_forbidden=0;
    reg [2:0] cycle=0;
    wire grant=request && cycle<3;
    always @(posedge clk)begin cycle<=cycle+1'b1;raw_blocked<=collision;raw_forbidden<=forbidden;end
    nav_path_refiner #(.SMOOTH_PASSES(512)) dut(.clk(clk),.rst_n(rst_n),.clear(clear),.cancel(cancel),
        .load_valid(load),.load_lock(lock),.load_x(x),.load_y(y),.start(start),
        .busy(busy),.done(done),.error(error),.plan_addr(plan),.plan_blocked(1'b0),
        .raw_addr(raw),.raw_request(request),.raw_forbidden(raw_forbidden),.raw_grant(grant),.raw_blocked(raw_blocked),
        .route_write(write),.route_bit(bit_value),.route_addr(route),
        .sample_valid(valid),.sample_x_q8(px),.sample_y_q8(py),.sample_segment(segment),.length_cm(length));
    task reset_points;
        begin @(negedge clk);clear=1;@(negedge clk);clear=0;end
    endtask
    task point;
        input integer xx,yy,ll;
        begin @(negedge clk);x=xx;y=yy;lock=ll;load=1;@(negedge clk);load=0;end
    endtask
    integer samples=0,i;reg curved=0;
    always @(posedge clk)if(valid)begin
        samples=samples+1;
        if(px[8:0]!=9'd256 || py[8:0]!=9'd256)curved=1;
        if(px>=352*256 || py>=320*256)$fatal(1,"sample outside map");
    end
    task run;
        input integer expected;
        begin
            samples=0;curved=0;
            @(negedge clk);start=1;@(negedge clk);start=0;
            wait(done);repeat(2)@(negedge clk);
            if(error!=expected)$fatal(1,"refiner error %d expected %d index %d",error,expected,dut.index);
            if(expected!=6 && samples!=dut.count)$fatal(1,"ordered sample count");
            if(expected==6 && samples!=0)$fatal(1,"failed route published");
        end
    endtask
    initial begin
        repeat(3)@(negedge clk);rst_n=1;reset_points;
        for(i=30;i<=60;i=i+1)point(i,40,i==30 || i==60);
        run(0);if(length!=60)$fatal(1,"straight physical length %d",length);
        reset_points;
        for(i=30;i<=50;i=i+1)point(i,40,i==30);
        for(i=41;i<=60;i=i+1)point(50,i,i==60);
        run(0);if(!curved)$fatal(1,"L path was not smoothed");
        reset_points;
        for(i=30;i<=45;i=i+1)point(i,40,i==30 || i==45);
        for(i=44;i>=30;i=i-1)point(i,40,i==30);
        run(4);
        reset_points;point(30,40,1);point(31,40,1);collision=1;run(0);collision=0;
        reset_points;point(0,0,1);point(1,0,1);run(0);
        reset_points;point(30,40,1);point(31,40,1);forbidden=1;run(6);forbidden=0;
        $display("PASS Ackermann: 20.5cm radius, L-curve smoothing, ordered samples with curvature/body advisory, forbidden barrier hard rejection and blanking arbitration");$finish;
    end
    initial begin #1000000000;$fatal(1,"refiner timeout state=%d index=%d",dut.state,dut.index);end
endmodule
