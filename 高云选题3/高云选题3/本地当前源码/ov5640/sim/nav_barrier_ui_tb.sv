`timescale 1ns/1ps
module nav_barrier_ui_tb;
    reg clk=0;always #5 clk=~clk;
    reg rst_n=0,tv=0,td=0,req=0;reg [15:0] tx=0,ty=0;reg [11:0] x=0,y=0;
    wire [3:0] screen;wire [15:0] pixel;wire ready;
    navigation_ui ui(.clk(clk),.rst_n(rst_n),.data_req(req),.x_addr(x),.y_addr(y),.video(16'd0),
        .touch_valid(tv),.touch_down(td),.touch_x(tx),.touch_y(ty),.operation_mode(screen),.pixel(pixel),.route_valid(ready));
    task tap(input integer a,b);
        begin
            @(negedge clk);tx=a;ty=b;tv=1;td=1;@(negedge clk);tv=0;
            repeat(8)@(negedge clk);tv=1;td=0;@(negedge clk);tv=0;repeat(8)@(negedge clk);
        end
    endtask
    task sample(input integer a,b);
        begin @(negedge clk);x=a;y=b;req=1;repeat(2)@(negedge clk);#1;end
    endtask
    initial begin
        repeat(4)@(negedge clk);rst_n=1;wait(ui.control.state==1);
        tap(580,340);tap(180,164);if(screen!=5)$fatal(1,"map menu");
        tap(592,400);wait(ui.control.state==1);if(ui.tool!=3)$fatal(1,"barrier tool");
        tap(40,232);if(!ui.barrier_pending)$fatal(1,"first endpoint");
        tap(84,232);wait(ui.control.state==1);
        if(ui.barrier_valid!=1 || ui.barrier_pending || ui.barrier_x0[8:0]!=16 || ui.barrier_x1[8:0]!=60 ||
            ui.barrier_y0[8:0]!=120 || ui.barrier_y1[8:0]!=120 || ready)$fatal(1,"barrier endpoints/invalidation");
        sample(51,232);if(pixel!=16'h295b)$fatal(1,"barrier not visible red: %h",pixel);
        req=0;tap(712,400);wait(ui.control.state==1);
        if(ui.barrier_valid || ready)$fatal(1,"clear barriers");
        tap(592,400);wait(ui.control.state==1);tap(80,180);tap(80,240);wait(ui.control.state==1);
        if(ui.barrier_x0[8:0]!=56 || ui.barrier_x1[8:0]!=56 || ui.barrier_y0[8:0]!=68 || ui.barrier_y1[8:0]!=128)
            $fatal(1,"vertical barrier");
        $display("PASS barrier touch UI: horizontal/vertical endpoints, red raster, tool selection, route invalidation and clear");$finish;
    end
    initial begin #10000000;$fatal(1,"barrier UI timeout");end
endmodule
