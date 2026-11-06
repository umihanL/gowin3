`timescale 1ns/1ps
module imu_spi_tb;
    // Fault injection, not claims about undocumented silicon behaviour:
    // 0 normal, 1 bad selector echo, 2 lost first switch after every POR,
    // 3 all entry writes lost, 4 false-success echo, 5 all exit writes lost.
    parameter BANK_FAULT=0;
    reg clk=0;always #5 clk=~clk;
    reg rst_n=0,miso=0,connected=0,make_fifo=1;
    wire [2:0] status;
    wire [7:0] error_reg,error_expected,error_actual,error_mask,error_step;
    integer stale_reads=0,saved_por=0;
    reg drop_setting=1; reg pending=0; reg inject_overrun=0,inject_bad_status=0;
    integer produce_count=0,por_count=0,bad_status_reads=0;
    integer bank_attempts=0,exit_attempts=0,embedded_writes=0;
    reg fusion_ready=0;
    wire sclk,mosi,cs_n,online,sample_valid;
    wire [15:0] qx,qy,qz;
    wire signed [15:0] gx,gy,gz,ax,ay,az;
    lsm6dsv16x_spi #(.CLK_HZ(100000),.SPI_HALF(5),.BOOT_CYCLES(40),.POLL_CYCLES(200),.TIMEOUT_CYCLES(30000),.STARTUP_CYCLES(50000)) dut(
        .clk(clk),.rst_n(rst_n),.miso(miso),.sclk(sclk),.mosi(mosi),.cs_n(cs_n),
        .online(online),.sample_valid(sample_valid),.status(status),
        .error_reg(error_reg),.error_expected(error_expected),.error_actual(error_actual),
        .error_mask(error_mask),.error_step(error_step),.qx_half(qx),.qy_half(qy),.qz_half(qz),
        .gx(gx),.gy(gy),.gz(gz),.ax(ax),.ay(ay),.az(az));
    reg [7:0] regs[0:127],emb[0:127];
    reg bank=0,rw;reg [6:0] addr;reg [7:0] cmd,byte_in,byte_out;
    integer bits=0,transactions=0,samples=0,k;
    function [7:0] read_byte;
        input [6:0] a;
        begin
            if(!connected) read_byte=0;
            else if(a==1)begin
                if(BANK_FAULT==1)read_byte=0;
                else if(BANK_FAULT==4 && bank_attempts!=0)read_byte=8'h80;
                else read_byte=bank ? 8'h80:0;
            end
            else if(bank)read_byte=(a==7'h5e) ? (emb[a] ^ 8'h01) : emb[a]; // vary a reserved readback bit
            else case(a)
                'h0f:read_byte='h70;
                'h15:read_byte=stale_reads!=0 ? 0 : (regs[a]|8'h20); // unused LPF field differs
                'h1b:read_byte=inject_bad_status?8'hff:(pending ? 1 : 0);
                'h1c:read_byte=inject_bad_status?8'hff:(inject_overrun ? 8'h40 : 0);
                'h78:read_byte='h98; // quaternion tag=0x13
                'h79:read_byte='h00;'h7a:read_byte='h34; // 0.25
                'h7b:read_byte='h00;'h7c:read_byte='hb8; // -0.5
                'h7d:read_byte='h00;'h7e:read_byte='h38; // +0.5
                'h22:read_byte='hfe;'h23:read_byte='hff;
                'h24:read_byte='h34;'h25:read_byte='h12;
                'h26:read_byte='h00;'h27:read_byte='h80;
                'h28:read_byte='h01;'h29:read_byte='h00;
                'h2a:read_byte='h02;'h2b:read_byte='h00;
                'h2c:read_byte='h05;'h2d:read_byte='h20;
                default:read_byte=regs[a];
            endcase
        end
    endfunction
    always @(negedge cs_n) begin
        if(sclk!==1)$fatal(1,"SPI must idle high");
        bits=0;cmd=0;byte_in=0;transactions=transactions+1;
    end
    always @(negedge sclk) if(!cs_n) begin
        if(bits>=8 && rw) begin
            byte_out=read_byte(addr);miso=byte_out[7-(bits%8)];
        end else miso=0;
    end
    always @(posedge sclk) if(!cs_n) begin
        if(bits<8) begin
            cmd={cmd[6:0],mosi};
            if(bits==7)begin rw=cmd[7];addr=cmd[6:0];end
        end else begin
            byte_in={byte_in[6:0],mosi};
            if(bits%8==7) begin
                if(!rw && connected) begin
                    if(addr=='h5e || addr=='h44 || addr=='h04)embedded_writes=embedded_writes+1;
                    if(addr==1 && byte_in==4) begin
                        por_count=por_count+1;bank=0;fusion_ready=0;pending=0;
                        bank_attempts=0;exit_attempts=0;
                        for(k=0;k<128;k=k+1)begin regs[k]=0;emb[k]=0;end
                        regs['h12]='h44;
                        regs['h02]='h23;emb['h02]='h01;emb['h5e]='h5b;
                    end else if(addr==1) begin
                        if(byte_in[7])begin
                            bank_attempts=bank_attempts+1;
                            if(BANK_FAULT!=3 && BANK_FAULT!=4 && !(BANK_FAULT==2 && bank_attempts==1))bank=1;
                        end else begin
                            if(bank)exit_attempts=exit_attempts+1;
                            if(BANK_FAULT!=5)bank=0;
                        end
                    end
                    else if(bank)begin
                        if(addr=='h44 && drop_setting) drop_setting=0;
                        else emb[addr]=byte_in;
                        // Embedded algorithms cannot run with sensors in power down.
                        if(addr==4 && byte_in==2) begin
                            if(por_count==0 || regs['h10]!=5 || regs['h11]!=5)
                                $fatal(1,"SFLP enabled before sensors/global reset");
                            fusion_ready=1;
                        end
                    end
                    else if(addr=='h12 && byte_in[0])begin
                        for(k=0;k<128;k=k+1)begin regs[k]=0;emb[k]=0;end
                        regs['h12]='h44;
                    end else regs[addr]=byte_in;
                end
                if(!rw && addr==7'h15 && !bank) stale_reads=1;
                if(rw && cmd==8'hf8 && addr==7'h7e)pending=0;
                addr=addr+1'b1;
            end
        end
        bits=bits+1;
    end
    always @(posedge cs_n) if(rst_n && bits!=0) begin
        if(cmd==8'h9b && inject_bad_status)begin
            if(bad_status_reads>1)bad_status_reads=bad_status_reads-1;
            else inject_bad_status=0;
        end
        if(cmd==8'h95 && stale_reads!=0) stale_reads=stale_reads-1;
        if(bits%8!=0 || bits<16)$fatal(1,"CS truncated transaction");
        if(sclk!==1)$fatal(1,"CS released at wrong clock phase");
        if(cmd=='hf8 && bits!=64)$fatal(1,"quaternion burst must be 8 bytes with command");
        if(cmd=='ha2 && bits!=104)$fatal(1,"raw burst must be 13 bytes with command");
    end
    always @(posedge clk) if(sample_valid) begin
        samples=samples+1;
        if({qz,qy,qx}!==48'h3800_b800_3400)$fatal(1,"quaternion byte order");
        if(gx!=-2 || gy!=4660 || gz!=-32768 || ax!=1 || ay!=2 || az!=8197)$fatal(1,"raw byte order");
    end
    always @(posedge clk) begin
        if(fusion_ready && make_fifo && connected) begin
            if(produce_count==7999)begin pending=1;produce_count=0;end
            else produce_count=produce_count+1;
        end else produce_count=0;
    end
    initial begin
        for(k=0;k<128;k=k+1)begin regs[k]=0;emb[k]=0;end
        repeat(5)@(negedge clk);rst_n=1;
        repeat(2000)@(negedge clk);
        if(online || samples)$fatal(1,"absent device accepted");
        if(BANK_FAULT>=3)drop_setting=0;
        connected=1;
        if(BANK_FAULT>=3)begin
            wait(status==4);
            if(online || samples)$fatal(1,"unverified bank accepted");
            if(BANK_FAULT==5)begin
                if(error_reg!=1 || error_expected!=0 || error_actual!=8'h80 || error_mask!=8'hc0 || error_step!=10)
                    $fatal(1,"failed return to main bank not identified");
                if(exit_attempts!=3 || embedded_writes!=2)$fatal(1,"incorrect return retries or writes after failed return");
            end else begin
                if(bank_attempts!=3 || por_count!=1 || embedded_writes!=0)
                    $fatal(1,"must retry bank write locally; must not write unverified embedded addresses");
                if(BANK_FAULT==3 && (error_reg!=1 || error_expected!=8'h80 || error_actual!=0 || error_mask!=8'hc0 || error_step!=7))
                    $fatal(1,"hardware REG01 SET80 GOT00 M C0 I07 not reproduced");
                if(BANK_FAULT==4 && (error_reg!=8'h5e || error_expected!=8'h18 || error_actual!=0 || error_mask!=8'h38 || error_step!=7))
                    $fatal(1,"selector echo alone incorrectly accepted a wrong bank");
            end
            $display("PASS imu_spi bank fault %0d: failed entry/exit stays offline; local retries and actual-page checks",BANK_FAULT);
            $finish;
        end
        wait(status==4); // a lost embedded write MUST be caught by readback
        if(online)$fatal(1,"bad config reported online");
        if(error_reg!=8'h44 || error_expected!=2 || error_actual!=0 || error_mask!=8'hf2 || error_step!=9)
            $fatal(1,"failure telemetry does not identify the lost embedded write");
        wait(samples==3);
        if(regs['h03]!=1 || regs['h12]!='h44 || regs['h15]!=4 || regs['h17]!=1 ||
           regs['h10]!=5 || regs['h11]!=5 || regs['h0a]!=6 || emb['h5e]!='h53 || emb['h44]!=2 || emb['h04]!=2)
            $fatal(1,"sensor configuration mismatch");
        if(por_count!=2)$fatal(1,"reserved/unused bit difference or delayed read triggered false CFGERR");
        if(BANK_FAULT==2 && bank_attempts!=3)$fatal(1,"lost first switch must recover without POR");
        // SPI identity remains valid, but absence of FIFO data must stay visible.
        make_fifo=0;pending=0;wait(!online);
        if(status!=5)$fatal(1,"missing stream not reported as NO DAT");
        make_fifo=1;wait(samples>=5);
        k=por_count;inject_bad_status=1;
        wait(!inject_bad_status);repeat(1000)@(negedge clk);
        if(!online || por_count!=k)$fatal(1,"one corrupt SPI status forced fusion reset");
        // More than the old three-read threshold must recover locally too.
        bad_status_reads=8;inject_bad_status=1;
        wait(!inject_bad_status);wait(sample_valid);repeat(20)@(negedge clk);
        if(!online || por_count!=k)$fatal(1,"brief repeated corrupt SPI status forced fusion reset");
        $display("PASS IMU burst recovery: eight consecutive FFFF FIFO status reads preserve fusion reference and resume valid samples");
        inject_overrun=1;wait(status==6);inject_overrun=0;
        wait(samples>=7);
        if(!online || por_count!=k)$fatal(1,"FIFO flush reset fusion engine");
        $display("PASS IMU local recovery: isolated SPI error and FIFO overflow preserve fusion reference without SW_POR");
        connected=0;pending=0;wait(!online);
        wait(status==3);connected=1;wait(samples>=9);
        // An actual dead embedded engine/configuration does NOT resume merely
        // because the host asks for FIFO again. Only full init repairs it.
        saved_por=por_count;fusion_ready=0;pending=0;regs['h10]=0;emb['h04]=0;
        wait(por_count>saved_por);wait(sample_valid);
        if(!online || regs['h10]!=5 || emb['h04]!=2)
            $fatal(1,"dead SFLP/config not recovered without board reset");
        $display("PASS IMU dead engine: FIFO retries escalate to verified full initialization and streaming resumes without board reset");
        // Freeze the SPI clock divider before the next transaction sends its
        // first bit. The host must abort this transfer, release CS, perform
        // verified initialization and resume without a board reset.
        saved_por=por_count;
        @(negedge cs_n);force dut.u_spi.divider=0;
        wait(dut.spi_abort);
        @(negedge clk);release dut.u_spi.divider;
        wait(por_count>saved_por);wait(sample_valid);
        if(!online || !cs_n)$fatal(1,"stalled SPI transaction did not recover");
        $display("PASS IMU SPI watchdog: stuck transfer aborted, CS released, verified initialization and samples resume without board reset");
        $display("PASS imu_spi bank fault %0d: page verification/retry, masked readback, retained diagnostics, ID/data/overflow recovery, mode3/endian",BANK_FAULT);
        $finish;
    end
    initial begin #10000000;$fatal(1,"SPI test timeout state=%0d status=%0d",dut.state,status);end
endmodule
