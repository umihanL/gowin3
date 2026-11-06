`timescale 1ns/1ps
module nav_ui_tb;
    reg clk=0;always #5 clk=~clk;
    reg rst_n=0,req=0,tv=0,td=0;
    reg [11:0] x=0,y=0;
    reg [15:0] tx=0,ty=0;
    wire [15:0] pixel;
    wire [3:0] mode,segment;wire active,ready,pv;wire [7:0] px,py;
    navigation_ui dut(.clk(clk),.rst_n(rst_n),.data_req(req),.x_addr(x),.y_addr(y),.video(16'h1234),
        .touch_valid(tv),.touch_down(td),.touch_x(tx),.touch_y(ty),.pixel(pixel),
        .operation_mode(mode),.navigation_request(active),.route_valid(ready),
        .path_valid(pv),.path_x(px),.path_y(py),.path_segment(segment),
        .left_mm(16'd123),.left_front_mm(16'd456),.right_front_mm(16'd1000),.right_mm(16'd1800),
        .tof_diagnostics(32'd0),.front_average_mm(16'd728),.front_valid(1'b1),.motion_status(4'd2),.motion_phase(4'd1),
        .commanded_speed(16'sd120),.commanded_steer(-16'sd18),.pose_heading(16'sd450),.target_heading(16'sd520),
        .pose_x_q8(24'sd230400),.pose_y_q8(24'sd409600),.target_x_mm(16'sd930),.target_y_mm(16'sd1640),
        .target_distance_mm(16'd50),.profile_ready(1'b1),.profile_segments(4'd3),.profile_length_mm(16'd800),
        .profile_left_revs(16'd31),.profile_right_revs(16'd43),
        .profile_entry_heading(16'sd250),.profile_exit_heading(16'sd850),.profile_peak_steer(8'd24));
    task tap;
        input integer u,v;
        begin
            @(negedge clk);tx=u;ty=v;tv=1;td=1;
            @(negedge clk);tv=0;
            repeat(6)@(negedge clk);
            tv=1;td=0;@(negedge clk);tv=0;
            repeat(6)@(negedge clk);
        end
    endtask
    task map_tap;
        input integer xx,yy;
        begin tap(24+2*xx+1,112+2*yy+1);wait(dut.control.state==1);repeat(2)@(negedge clk);end
    endtask
    task sample;
        input integer uu,vv;
        begin
            @(negedge clk);x=uu;y=vv;req=1;
            repeat(2)@(negedge clk);#1;
            req=0;
        end
    endtask
    integer out,index,uu,vv,j;
    task snapshot;
        input [255:0] filename;
        begin
            out=$fopen(filename,"wb");
            $fwrite(out,"P6\n800 480\n255\n");
            for(vv=0;vv<480;vv=vv+1)for(uu=0;uu<800;uu=uu+1)begin
                sample(uu,vv);
                $fwrite(out,"%c%c%c",{pixel[4:0],pixel[4:2]},
                    {pixel[10:5],pixel[10:9]},{pixel[15:11],pixel[15:13]});
            end
            $fclose(out);req=0;
        end
    endtask
    integer vertices=0;reg [9:0] segments_seen=0;
    always @(posedge clk)if(pv)begin
        vertices=vertices+1;segments_seen[segment]=1;
        if(dut.maps.plan_map[(py*176+px)/16][(py*176+px)%16])$fatal(1,"path on blocked map cell");
    end
    initial begin
        repeat(3)@(negedge clk);rst_n=1;
        if(dut.renderer.nav_ascii("S")==0 || dut.renderer.nav_ascii("L")==0 ||
           dut.renderer.nav_ascii("R")==0 || dut.renderer.nav_ascii("r")==0 ||
           dut.renderer.nav_ascii("e")==0 || dut.renderer.nav_ascii("v")==0 || dut.renderer.nav_ascii("s")==0)
            $fatal(1,"missing navigation/units font glyphs");
        wait(dut.control.state==1);repeat(2)@(negedge clk);
        if(mode!=0 || !dut.metadata_valid)$fatal(1,"startup/menu/metadata ROM");
        if($test$plusargs("snapshot")) snapshot("sim/menu_home.ppm");
        tap(180,200);if(mode!=1)$fatal(1,"live menu");
        tap(180,180);if(dut.control.live_page!=1)$fatal(1,"normal image submenu");
        sample(200,430);if(pixel!==16'h1234)$fatal(1,"full-screen live video");
        tap(32,32);tap(180,300);if(dut.control.live_page!=2)$fatal(1,"binary image submenu");
        tap(654,456);if(dut.binary_threshold!=255)$fatal(1,"threshold max");
        @(negedge clk);tv=1;td=1;tx=144;ty=456;
        @(negedge clk);tx=336;
        @(negedge clk);tx=654;
        @(negedge clk);td=0;
        @(negedge clk);tv=0;
        if(dut.binary_threshold!=255)$fatal(1,"continuous slider drag");
        tap(32,32);tap(32,32);tap(580,180);if(mode!=2)$fatal(1,"remote menu");
        tap(32,32);tap(180,340);if(mode!=3)$fatal(1,"avoid menu");
        tap(180,164);if(mode!=6)$fatal(1,"detect submenu");
        if($test$plusargs("snapshot"))snapshot("sim/menu_detect.ppm");
        tap(32,32);tap(180,268);if(mode!=7)$fatal(1,"stop submenu");
        tap(32,32);tap(180,372);if(mode!=8)$fatal(1,"dodge submenu");
        tap(32,32);tap(32,32);tap(580,340);if(mode!=4)$fatal(1,"navigation menu");
        tap(180,164);if(mode!=5)$fatal(1,"path setup");
        sample(24,112);if(pixel!==16'h3986)$fatal(1,"obstacle ROM missing");
        sample(24+27,112+105);if(pixel!==16'hffff)$fatal(1,"road ROM missing");
        tap(452,392);if(dut.control.message!=21)$fatal(1,"missing endpoint message");
        map_tap(0,0);if(dut.control.have_start || dut.control.message!=22)$fatal(1,"obstacle selection accepted");
        map_tap(13,50);if(!dut.control.have_start)$fatal(1,"start not selected");
        map_tap(7,52);if(!dut.control.have_start || dut.control.sx<8 || dut.control.sx>13)$fatal(1,"nearby touch did not snap to safe road");
        map_tap(13,52);if(dut.control.sx!=13 || dut.control.sy!=52)$fatal(1,"start replacement");
        sample(24+27,112+105);if(pixel!==16'h5403)$fatal(1,"start marker not visible");
        tap(452,200);map_tap(13,68);if(!dut.control.have_goal)$fatal(1,"goal not selected");
        sample(24+27,112+137);if(pixel!==16'h295b)$fatal(1,"goal marker not visible");
        tap(452,264);map_tap(13,60);map_tap(13,64);
        if(dut.control.waypoint_count!=2)$fatal(1,"waypoint count");
        sample(24+27,112+121);if(pixel!==16'hdc61)$fatal(1,"waypoint marker not visible");
        if(dut.control.selected_x_cm!=27 || dut.control.selected_y_cm!=191)$fatal(1,"physical coordinate conversion");
        tap(452,392);wait(!dut.control.planning);
        repeat(3)@(negedge clk);
        if(!ready || dut.control.length_cm!=32)$fatal(1,"multi-waypoint A* route/order/length: ready=%b seen=%b len=%d",ready,segments_seen,dut.control.length_cm);
        for(j=52;j<=68;j=j+1)if(dut.route[j*176+13]!==1'b1)$fatal(1,"route bitmap missing node");
        if($test$plusargs("snapshot"))snapshot("sim/menu_map.ppm");
        tap(32,32);tap(180,268);if(mode!=9 || !active)$fatal(1,"start navigation request");
        if($test$plusargs("snapshot"))snapshot("sim/menu_navigation.ppm");
        tap(560,396);if(active || mode!=4)$fatal(1,"stop navigation request");
        tap(180,164);tap(452,136);map_tap(13,54);
        if(ready)$fatal(1,"route not invalidated by edited point");
        tap(452,328);wait(dut.control.state==1);
        if(dut.control.have_start || dut.control.have_goal || dut.control.waypoint_count!=0)$fatal(1,"clear points");
        for(j=0;j<28160;j=j+1)if(dut.route[j]!==1'b0)$fatal(1,"clear bitmap");
        tap(452,136);map_tap(13,52);tap(452,200);map_tap(13,68);
        tap(452,264);map_tap(13,64);map_tap(13,60);
        tap(452,392);wait(!dut.control.planning);repeat(3)@(negedge clk);
        if(!ready || dut.control.message!=38)$fatal(1,"Ackermann reversal accepted message=%d",dut.control.message);
        for(j=0;j<6;j=j+1)map_tap(13,60);
        if(dut.control.waypoint_count!=8)$fatal(1,"eight waypoint capacity");
        map_tap(13,60);if(dut.control.waypoint_count!=8 || dut.control.message!=23)$fatal(1,"waypoint overflow");
        tap(452,392);tap(452,328);wait(dut.control.state==1);
        if(dut.control.planning || ready || dut.control.waypoint_count!=0)$fatal(1,"clear during planning");
        tap(452,136);map_tap(13,125);tap(452,200);map_tap(45,146);
        tap(452,392);wait(!dut.control.planning);repeat(3)@(negedge clk);
        if(!ready || dut.control.message==39)$fatal(1,"body footprint must no longer create a collision warning, message=%d",dut.control.message);
        if($test$plusargs("snapshot"))snapshot("sim/menu_map_rejected.ppm");
        tap(32,32);tap(180,268);
        if(mode!=9 || !active || !ready || dut.control.message==39)
            $fatal(1,"valid route did not start");
        tap(560,396);if(active || mode!=4)$fatal(1,"advisory run stop");
        $display("PASS UI: all menus, safe-cell snapping, point replacement/rejection, colored markers, waypoint insertion order/backtracking, eight-point limit, ROM metadata, route bitmap/length, navigation start/stop, invalidation and clear during planning");$finish;
    end
    initial begin #1000000000;$fatal(1,"UI timeout state=%d planner=%d message=%d",dut.control.state,dut.control.planner.state,dut.control.message);end
endmodule
