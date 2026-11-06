`timescale 1ns/1ps
module portrait_ddr_tb;
`ifdef FULL_SIZE
    localparam W=640,H=480,SW=640,N=307200,SLOT=524288,FRAMES=2;
`else
    localparam W=16,H=24,SW=16,N=384,SLOT=512,FRAMES=3;
`endif
    reg ui=0,cam=0,disp=0,rst_n=0;
    always #5 ui=~ui;
    always #14 cam=~cam;
    always #17 disp=~disp;
    reg fs=0,pv=0,rf=0,ren=0;
    reg [15:0] rgb=0;
    reg rdy=0,wdy=0,rv=0;reg [127:0] rd=0;
    wire [27:0] addr;wire en,we,wend;wire [2:0] cmd;wire [127:0] wd;
    wire empty,available,overflow;wire [15:0] q;
    portrait_ddr_adapter #(.WIDTH(W),.HEIGHT(H),.SCREEN_WIDTH(SW),.FRAME_WORDS(N),.SLOT_WORDS(SLOT)) dut(
        .ui_clk(ui),.rst_n(rst_n),.init_calib_complete(rst_n),
        .app_rdy(rdy),.app_wdf_rdy(wdy),.app_rd_data_valid(rv),.app_rd_data(rd),
        .app_addr(addr),.app_en(en),.app_wdf_wren(we),.app_wdf_end(wend),.app_cmd(cmd),.app_wdf_data(wd),
        .cam_clk(cam),.cam_rst_n(rst_n),.frame_start(fs),.pixel_valid(pv),.rgb(rgb),.line_active(1'b1),
        .rd_clk(disp),.rd_frame(rf),.rfifo_rden(ren&&!empty),
        .rdfifo_empty(empty),.rfifo_dout(q),.frame_available(available),.overflow(overflow));
    reg [15:0] memory[0:2*SLOT-1];
    reg [127:0] queue[0:255];integer head=0,tail=0,cycle=0;
    integer j,i,xx,yy,f=0,received=0,expect_word,slot;
    reg checking=0;
    always @(posedge ui)begin
        if(rst_n)begin
            if(en)begin
                if(!rdy)$fatal(1,"command without ready");
                if(cmd==0)begin
                    if(!we || !wdy || !wend)$fatal(1,"write handshake");
                    if(addr%8!=0)$fatal(1,"unaligned burst");
                    for(j=0;j<8;j=j+1)memory[addr+j]=wd[j*16+:16];
                end else begin
                    for(j=0;j<8;j=j+1)queue[tail%256][j*16+:16]=memory[addr+j];
                    tail=tail+1;
                end
            end
        end
    end
    always @(negedge ui)begin
        cycle=cycle+1;rdy=(cycle%7!=0);wdy=(cycle%11!=0);rv=0;
        if(rst_n && tail>head && cycle%3==0)begin rd=queue[head%256];head=head+1;rv=1;end
    end
    task framepulse;
        begin @(negedge disp);rf=1;@(negedge disp);rf=0;end
    endtask
    task camera_frame;
        input integer n;
        integer cx,cy;
        begin
            @(negedge cam);fs=1;@(negedge cam);fs=0;
            for(cy=0;cy<H;cy=cy+1)for(cx=0;cx<W;cx=cx+1)begin
                @(negedge cam);pv=1;rgb=n*1000+cy*W+cx;
                @(negedge cam);pv=0;
            end
            repeat(10)@(negedge cam);
        end
    endtask
    always @(posedge disp)if(checking && ren && !empty)begin
        xx=received%SW;yy=received/SW;
        if(xx<W)expect_word=f*1000+yy*W+xx;
        else expect_word=16'h5555;
        if(q!==expect_word[15:0])$fatal(1,"DDR landscape/pack f=%d pixel=%d got=%h expected=%h",f,received,q,expect_word[15:0]);
        received=received+1;
    end
    initial begin
        for(i=0;i<2*SLOT;i=i+1)memory[i]=16'h5555;
        #500;rst_n=1;repeat(50)@(negedge ui);
        camera_frame(0);
        for(f=0;f<FRAMES;f=f+1)begin
            wait(dut.pending_frame);
            if(f==0 && available)$fatal(1,"frame published before vblank");
            framepulse;wait(available && !dut.pending_frame);
            repeat(200)@(negedge disp);
            received=0;checking=1;ren=1;
            fork
                begin if(f<FRAMES-1)camera_frame(f+1);end
                begin wait(received==N);@(negedge disp);ren=0;checking=0;end
            join
            repeat(30)@(negedge disp);
            if(overflow)$fatal(1,"unexpected tile overrun");
        end
        $display("PASS: full-screen RGB DDR capture/scanout, ready stalls, aspect-preserving row order, vendor FIFO order, frame swaps");$finish;
    end
    initial begin #200000000;$fatal(1,"DDR test timeout write=%d pending=%b available=%b",dut.write_offset,dut.pending_frame,available);end
endmodule
