`timescale 1ns/1ps
// Real DVP byte framing, vendor FIFO netlists and landscape scan cadence.
module video_capture_tb;
`ifdef FULL_SIZE
    localparam W=640,H=480,N=W*H,SLOT=524288;
`else
    localparam W=640,H=24,N=W*H,SLOT=16384;
`endif
    reg ui=0,cam=0,disp=0,rst_n=0;
    always #5 ui=~ui;
    always #8.5 cam=~cam;
    always #15.1515 disp=~disp;
    reg vsync=0,href=0;
    reg [7:0] data=0;
    wire fs,pv,line_active;
    wire [15:0] rgb;
    DVP_Capture capture(.Rst_n(rst_n),.PCLK(cam),.Vsync(vsync),.Href(href),.Data(data),
        .ImageState(fs),.DataValid(pv),.DataPixel(rgb),.DataHs(line_active),
        .DataVs(),.Xaddr(),.Yaddr());
    reg rf=0,req=0;
    reg [11:0] x=0,y=0;
    reg rdy=0,wdy=0,rv=0,stall=0;
    reg [127:0] rd=0;
    wire [27:0] addr;
    wire en,we,wend,empty,available,overflow,ren,underflow;
    wire [2:0] cmd;
    wire [127:0] wd;
    wire [15:0] q,pixel;
    portrait_ddr_adapter #(.WIDTH(W),.HEIGHT(H),.FRAME_WORDS(N),.SLOT_WORDS(SLOT),.CHECK_LINES(1)) dut(
        .ui_clk(ui),.rst_n(rst_n),.init_calib_complete(rst_n),
        .app_rdy(rdy),.app_wdf_rdy(wdy),.app_rd_data_valid(rv),.app_rd_data(rd),
        .app_addr(addr),.app_en(en),.app_wdf_wren(we),.app_wdf_end(wend),.app_cmd(cmd),.app_wdf_data(wd),
        .cam_clk(cam),.cam_rst_n(rst_n),.frame_start(fs),.pixel_valid(pv),.rgb(rgb),.line_active(line_active),
        .rd_clk(disp),.rd_frame(rf),.rfifo_rden(ren),.rdfifo_empty(empty),.rfifo_dout(q),
        .frame_available(available),.overflow(overflow));
    portrait_display #(.VIDEO_HEIGHT(H)) display_source(.clk(disp),.rst_n(rst_n),.frame_begin(rf),
        .data_req(req),.x_addr(x),.y_addr(y),.frame_available(available),.fifo_empty(empty),.fifo_data(q),
        .fifo_read(ren),.pixel_data(pixel),.underflow(underflow));
    reg [15:0] memory[0:2*SLOT-1];
    reg [127:0] queue[0:255];
    integer due[0:255];
    integer head=0,tail=0,cycle=0,j;
    integer source_id=0,scan_id=0,write_words=0,scan_words=0;
    reg check_writes=0,check_scan=0;
    function [15:0] pattern;
        input integer id,px,py;
        pattern=((py*W+px)*97)^16'ha65c^(id*16'h1357);
    endfunction
    always @(posedge ui) if(rst_n && en) begin
        if(!rdy || addr%8!=0 || addr>=2*SLOT) $fatal(1,"DDR address/command contract");
        if(cmd==0)begin
            if(!we || !wdy || !wend) $fatal(1,"DDR write contract");
            if(check_writes && addr%SLOT!=write_words) $fatal(1,"DDR row address drift");
            for(j=0;j<8;j=j+1)begin
                if(check_writes && wd[j*16+:16]!==pattern(source_id,(write_words+j)%W,(write_words+j)/W))
                    $fatal(1,"DVP/DDR pack pixel=%d got=%h",write_words+j,wd[j*16+:16]);
                memory[addr+j]=wd[j*16+:16];
            end
            if(check_writes) write_words=write_words+8;
        end else begin
            for(j=0;j<8;j=j+1) queue[tail%256][j*16+:16]=memory[addr+j];
            due[tail%256]=cycle+29+cycle%17; tail=tail+1;
            if(tail-head>32) $fatal(1,"unbounded DDR reads");
        end
    end
    always @(posedge ui) if(rst_n && dut.fifo_count>200)
        $fatal(1,"display FIFO prefetch exceeded capacity watermark");
    always @(posedge ui) if(rst_n && dut.scan_fifo.Full)
        $fatal(1,"display FIFO overflow");
    always @(negedge ui)begin
        cycle=cycle+1; rdy=!stall && cycle%13!=0; wdy=!stall && cycle%19!=0; rv=0;
        if(tail>head && cycle>=due[head%256] && cycle%5!=0)begin
            rd=queue[head%256];head=head+1;rv=1;
        end
    end
    always @(posedge disp) if(check_scan && req) begin
        if(x>=80 && x<720)begin
            if(!ren || pixel!==pattern(scan_id,x-80,y))
                $fatal(1,"landscape scan id=%d x=%d y=%d got=%h expected=%h empty=%b",scan_id,x,y,pixel,pattern(scan_id,x-80,y),empty);
            scan_words=scan_words+1;
        end else if(ren || pixel!==0) $fatal(1,"side bar consumed image");
    end
    task start_camera;
        begin
            @(negedge cam);vsync=1;href=0;
            repeat(8)@(negedge cam);vsync=0;
            repeat(16)@(negedge cam);
        end
    endtask
    task camera_rows;
        input integer id,rows,pixels_per_line;
        integer cx,cy;reg [15:0] p;
        begin
            for(cy=0;cy<rows;cy=cy+1)begin
                for(cx=0;cx<pixels_per_line;cx=cx+1)begin
                    p=pattern(id,cx,cy);
                    @(negedge cam);href=1;data=p[15:8];
                    @(negedge cam);data=p[7:0];
                end
                @(negedge cam);href=0;data=0;
                repeat(64)@(negedge cam);
            end
            repeat(100)@(negedge cam);
        end
    endtask
    task good_camera;
        input integer id;
        begin
            source_id=id;write_words=0;check_writes=1;
            $display("INFO video %0dx%0d: capture %0d at %0t",W,H,id,$time);
            start_camera;camera_rows(id,H,W);wait(dut.pending_frame);
            if(write_words!=N) $fatal(1,"incomplete good frame");
            $display("INFO video: DDR frame %0d complete at %0t",id,$time);
            check_writes=0;
        end
    endtask
    task start_scan;
        begin
            @(negedge disp);rf=1;
            @(negedge disp);rf=0;
            repeat(600)@(negedge disp);
        end
    endtask
    task scan;
        input integer id;
        integer sx,sy;
        begin
            scan_id=id;scan_words=0;check_scan=1;
            $display("INFO video: scan %0d at %0t",id,$time);
            for(sy=0;sy<H;sy=sy+1)begin
                for(sx=0;sx<800;sx=sx+1)begin
                    @(negedge disp);req=1;x=sx;y=sy;
                end
                @(negedge disp);req=0;
                repeat(256)@(negedge disp);
            end
            check_scan=0;
            if(scan_words!=N || underflow) $fatal(1,"scan size/underflow");
            $display("INFO video: scan %0d matched %0d pixels at %0t",id,scan_words,$time);
        end
    endtask
    initial begin
        #1000;rst_n=1;repeat(100)@(negedge cam);
        // Seven residual pixels in a partial 128-bit group must be discarded.
        start_camera;camera_rows(9,1,W-1);
        wait(overflow);repeat(150)@(negedge cam);
        if(dut.pending_frame || available) $fatal(1,"short row published");
        // A frame boundary before the final row also invalidates the frame.
        start_camera;camera_rows(8,2,W);start_camera;
        repeat(150)@(negedge cam);
        if(dut.pending_frame || available) $fatal(1,"short frame published");
        good_camera(1);start_scan;
        fork
            scan(1);
            good_camera(2);
        join
        start_scan;scan(2);
        // Overflow recovery must preserve the last complete display slot.
        stall=1;start_camera;camera_rows(7,20,W);
        repeat(150)@(negedge cam);stall=0;
        repeat(150)@(negedge cam);
        if(dut.pending_frame || dut.capture_active) $fatal(1,"overflow was not discarded");
        good_camera(3);start_scan;scan(3);
        $display("PASS video pipeline: DVP bytes, HREF rows, RGB565, DDR/FWFT order, 640-wide scan, side bars, partial-row/frame rejection, overflow reset, concurrent capture, three frame swaps");
        $finish;
    end
    initial begin #200000000;$fatal(1,"video pipeline timeout offset=%d pixels=%d active=%b pending=%b",dut.write_offset,dut.pixel_count,dut.capture_active,dut.pending_frame);end
endmodule
