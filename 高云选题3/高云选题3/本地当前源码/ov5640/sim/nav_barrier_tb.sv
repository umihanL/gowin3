`timescale 1ns/1ps
module nav_barrier_tb;
    reg clk=0;always #5 clk=~clk;
    reg [3:0] valid=0;reg [35:0] x0=0,y0=0,x1=0,y1=0;
    reg [16:0] raw_addr=0;reg [14:0] probe=0;
    wire raw_hit,plan_hit,display_hit;
    wire direct_raw,direct_plan,direct_display;
    reg rst_n=0,start=0,use_planner=0;
    wire [14:0] ma,pa;
    wire [14:0] mapped=(50+(ma>>4))*176+30+ma[3:0];
    wire [7:0] px,py;wire busy,done,pv;wire [2:0] error;
    nav_barrier_overlay overlay(.clk(clk),.valid(valid),.x0(x0),.y0(y0),.x1(x1),.y1(y1),
        .plan_addr(use_planner?mapped:probe),.display_plan_addr(probe),.raw_addr(raw_addr),
        .plan_hit(plan_hit),.display_plan_hit(display_hit),.raw_hit(raw_hit));
    wire [14:0] active_addr=use_planner?mapped:probe;
    wire [8:0] plan_x=(active_addr%176)*2+1,plan_y=(active_addr/176)*2+1;
    wire [8:0] display_x=(probe%176)*2+1,display_y=(probe/176)*2+1;
    wire [8:0] raw_x=raw_addr%352,raw_y=raw_addr/352;
    nav_barrier_overlay #(.DIRECT_COORDINATES(1)) direct(.clk(clk),.valid(valid),
        .x0(x0),.y0(y0),.x1(x1),.y1(y1),.plan_addr(active_addr),.display_plan_addr(probe),.raw_addr(raw_addr),
        .plan_coordinates({plan_y,plan_x}),.display_coordinates({display_y,display_x}),.raw_coordinates({raw_y,raw_x}),
        .plan_hit(direct_plan),.display_plan_hit(direct_display),.raw_hit(direct_raw));
    always @(negedge clk)begin
        #1;
        if(direct_raw!==raw_hit || direct_plan!==plan_hit || direct_display!==display_hit)
            $fatal(1,"direct barrier coordinates changed occupancy");
    end
    astar_grid #(.WIDTH(16),.HEIGHT(16)) astar(.clk(clk),.rst_n(rst_n),.start(start),.cancel(1'b0),
        .start_x(8'd2),.start_y(8'd1),.goal_x(8'd2),.goal_y(8'd14),
        .busy(busy),.done(done),.error(error),.map_addr(ma),.map_blocked(plan_hit),
        .map_central(1'b1),.map_directions(8'hff),.path_valid(pv),.path_addr(pa),.path_x(px),.path_y(py),.path_cost(),.expanded());
    function expected;
        input integer xx,yy,margin;integer j;
        begin
            expected=0;
            for(j=0;j<4;j=j+1)
                if(valid[j] && xx+margin>=x0[j*9+:9] && xx<=x1[j*9+:9]+margin &&
                   yy+margin>=y0[j*9+:9] && yy<=y1[j*9+:9]+margin)expected=1;
        end
    endfunction
    integer i,steps=0;
    always @(posedge clk)if(pv)begin
        steps=steps+1;
        if(expected(2*(30+px)+1,2*(50+py)+1,11))$fatal(1,"A* traversed forbidden barrier");
    end
    task plan(input integer wanted_error);
        begin
            @(negedge clk);steps=0;start=1;@(negedge clk);start=0;wait(done);@(negedge clk);
            if(error!=wanted_error || (wanted_error==0 && steps<13))$fatal(1,"barrier plan error=%d steps=%d",error,steps);
        end
    endtask
    initial begin
        x0={9'd330,9'd100,9'd0,9'd60};x1={9'd351,9'd180,9'd0,9'd62};
        y0={9'd319,9'd200,9'd0,9'd116};y1={9'd319,9'd200,9'd80,9'd116};valid=15;
        repeat(4)@(negedge clk);rst_n=1;
        for(i=0;i<112640;i=i+1)begin
            raw_addr=i;probe=i%28160;@(negedge clk);
            if(raw_hit!==expected(i%352,i/352,1))$fatal(1,"raw barrier raster %d",i);
            if(plan_hit!==expected(2*((i%28160)%176)+1,2*((i%28160)/176)+1,11) || plan_hit!==display_hit)
                $fatal(1,"inflated barrier raster %d",i);
        end
        valid=1;use_planner=1;plan(0);
        x0[8:0]=0;x1[8:0]=351;plan(3);
        valid=0;plan(0);
        $display("PASS forbidden barriers: all raw/coarse addresses, four segments and edge clipping, A* detour, full-road rejection, clear/replan");$finish;
    end
    initial begin #100000000;$fatal(1,"barrier timeout");end
endmodule
