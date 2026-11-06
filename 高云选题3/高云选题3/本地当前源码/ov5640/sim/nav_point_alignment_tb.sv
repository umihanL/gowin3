`timescale 1ns/1ps
module nav_point_alignment_tb;
    reg clk=0;always #5 clk=~clk;
    reg rst_n=0,tv=0,td=0,blocked=0;
    reg [15:0] tx=0,ty=0;
    wire [14:0] addr;
    wire [7:0] sx,sy,gx,gy;
    wire [63:0] wx,wy;
    wire [8:0] selected_x,selected_y;
    nav_control dut(.clk(clk),.rst_n(rst_n),.edit_distance(1'b0),
        .touch_valid(tv),.touch_down(td),.touch_x(tx),.touch_y(ty),
        .sx(sx),.sy(sy),.gx(gx),.gy(gy),.waypoint_x(wx),.waypoint_y(wy),
        .selected_x_cm(selected_x),.selected_y_cm(selected_y),
        .map_addr(addr),.map_blocked(blocked),.map_directions(8'hff),.map_central(1'b1),
        .refine_done(1'b0),.refine_error(3'd0),.refined_length_cm(18'd0));
    // Same one-cycle map latency as the real ROM; one wall at projected cell.
    always @(posedge clk)blocked<=addr==20*176+30;
    task tap(input integer x,y);begin
        @(negedge clk);tx=x;ty=y;tv=1;td=1;
        @(negedge clk);tv=0;repeat(6)@(negedge clk);
        td=0;tv=1;@(negedge clk);tv=0;repeat(6)@(negedge clk);
    end endtask
    task point(input integer x,y);begin
        tap(25+2*x,113+2*y);wait(dut.state==1);repeat(2)@(negedge clk);
        if(selected_x!=2*dut.probe_x+1 || selected_y!=319-2*dut.probe_y)
            $fatal(1,"visible coordinate does not match committed point");
    end endtask
    task fresh;begin
        @(negedge clk);rst_n=0;repeat(3)@(negedge clk);rst_n=1;
        wait(dut.state==1);tap(580,340);tap(180,164);
    end endtask
    initial begin
        fresh;point(20,20);tap(452,200);point(50,22);
        if(gx!=50 || gy!=20)$fatal(1,"horizontal finger error not aligned");
        tap(452,264);point(35,23);
        if(wx[7:0]!=35 || wy[7:0]!=20)$fatal(1,"first waypoint not aligned");
        point(37,50);
        if(wx[15:8]!=35 || wy[15:8]!=50)$fatal(1,"vertical finger error not aligned");
        tap(452,200);point(65,52);
        if(gx!=65 || gy!=50)$fatal(1,"goal not aligned to last waypoint");
        fresh;point(20,20);tap(452,200);point(30,22);
        if(gx!=30 || gy!=22)$fatal(1,"blocked projection did not preserve original free point");
        point(50,50);
        if(gx!=50 || gy!=50)$fatal(1,"real corner was flattened");
        point(25,23);
        if(gx!=25 || gy!=23)$fatal(1,"short local segment incorrectly snapped");
        fresh;tap(452,200);point(50,50);tap(452,136);point(52,20);
        if(sx!=50 || sy!=20)$fatal(1,"start replacement not aligned to existing goal");
        $display("PASS point alignment: horizontal/vertical starts, goals and ordered waypoints; visible coordinates; wall fallback; real corners and short segments preserved");
        $finish;
    end
    initial begin #10000000;$fatal(1,"point alignment timeout");end
endmodule
