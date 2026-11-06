`timescale 1ns/1ps
module portrait_touch_tb;
    reg clk=0,rst_n=0,slave_low=0;
    always #5 clk=~clk;
    tri1 scl,sda;tri intr;wire reset_out;
    assign sda=slave_low?1'b0:1'bz;
    wire valid,down,online;wire [15:0] x,y;
    gt911_touch #(.CLOCK_HZ(400000),.RESET_CYCLES(20),.INT_HOLD_CYCLES(20),
        .STARTUP_CYCLES(30),.POLL_CYCLES(100)) dut(
        .clk(clk),.rst_n(rst_n),.tp_scl(scl),.tp_sda(sda),.tp_int(intr),.tp_rst_n(reset_out),
        .sample_valid(valid),.pressed(down),.x(x),.y(y),.online(online));
    reg [7:0] address,hi,lo,value,rx;
    reg [15:0] pointer;
    integer i,clears=0,samples=0;
    reg no_touch=0;
    task receive;
        output [7:0] b;
        integer k;
        begin for(k=7;k>=0;k=k-1)begin @(posedge scl);b[k]=sda;end end
    endtask
    task ack;
        begin @(negedge scl);slave_low=1;@(posedge scl);@(negedge scl);slave_low=0;end
    endtask
    task transmit;
        input [7:0] b;
        integer k;
        begin
            for(k=7;k>=0;k=k-1)begin
                slave_low=!b[k];@(posedge scl);@(negedge scl);
            end
            slave_low=0;@(posedge scl);
            if(sda!==1)$fatal(1,"master must NACK single-byte read");
            @(negedge scl);
        end
    endtask
    initial forever begin
        @(negedge sda);
        if(scl===1 && rst_n && reset_out)begin
            receive(address);
            if(address[7:1]!=7'h5d)$fatal(1,"unexpected I2C address %h",address);
            ack;
            if(!address[0])begin
                receive(hi);ack;receive(lo);ack;pointer={hi,lo};
                if(dut.operation==5)begin
                    receive(rx);ack;
                    if(pointer!=16'h814e || rx!=0)$fatal(1,"status not cleared");
                    clears=clears+1;
                end
            end else begin
                case(pointer)
                    16'h814e:value=no_touch?8'h80:8'h81;
                    16'h8150:value=8'hf8;
                    16'h8151:value=8'h02;
                    16'h8152:value=8'hef;
                    16'h8153:value=8'h00;
                    default:$fatal(1,"unexpected register %h",pointer);
                endcase
                transmit(value);
            end
            // Ignore the byte engine's stop transition before next START.
            @(posedge sda);while(scl!==1)@(posedge sda);
        end
    end
    always @(negedge clk)if(valid)begin
        samples=samples+1;
        if(!online || clears!=samples)$fatal(1,"sample published before status clear");
        if(!no_touch && (!down || x!=760 || y!=239))$fatal(1,"bad touch coordinates %d %d",x,y);
        if(no_touch && down)$fatal(1,"release not reported");
    end
    initial begin
        repeat(5)@(negedge clk);rst_n=1;
        repeat(10)@(negedge clk);if(reset_out || intr!==0)$fatal(1,"reset sequence");
        wait(reset_out);if(intr!==0)$fatal(1,"GT911 address selection");
        wait(dut.boot_done);if(intr!==1'bz)$fatal(1,"INT not released");
        wait(samples==1);@(negedge clk);no_touch=1;
        wait(samples==2);$display("PASS: GT911 reset/address sequence, real I2C ACK/read/NACK, little-endian coordinates, clear and release");$finish;
    end
    initial begin #1000000;$fatal(1,"touch timeout op=%d state=%d",dut.operation,dut.state);end
endmodule
