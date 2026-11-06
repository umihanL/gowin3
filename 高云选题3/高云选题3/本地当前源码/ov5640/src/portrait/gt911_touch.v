// GT911/GT9147 share the coordinate register map. Boot selects 7-bit 0x5D.
// Uses the project's proven 16-bit-register I2C controller at 100 kHz.
module gt911_touch #(
    parameter CLOCK_HZ=33000000,
    parameter RESET_CYCLES=CLOCK_HZ/50,
    parameter INT_HOLD_CYCLES=CLOCK_HZ/100,
    parameter STARTUP_CYCLES=CLOCK_HZ/10,
    parameter POLL_CYCLES=CLOCK_HZ/200
)(
    input clk, input rst_n,
    inout tp_scl, inout tp_sda, inout tp_int, output tp_rst_n,
    output reg sample_valid, output reg pressed,
    output reg [15:0] x,y, output reg online
);
    reg [31:0] startup;
    wire boot_done=(startup==RESET_CYCLES+INT_HOLD_CYCLES+STARTUP_CYCLES);
    assign tp_rst_n=rst_n && startup>=RESET_CYCLES;
    assign tp_int=(!rst_n || startup<RESET_CYCLES+INT_HOLD_CYCLES)?1'b0:1'bz;
    always @(posedge clk or negedge rst_n)
        if(!rst_n) startup<=0;
        else if(!boot_done) startup<=startup+1'b1;
    wire serial_clk;
    assign tp_scl=serial_clk?1'bz:1'b0;
    reg rd_req,wr_req;
    reg [15:0] reg_addr;
    wire [7:0] rd_data;
    wire done,nack;
    reg [7:0] device_id;
    reg bus_reset_n;
    i2c_control #(.SYS_CLOCK(CLOCK_HZ),.SCL_CLOCK(100000)) bus (
        .Clk(clk),.Rst_n(rst_n && bus_reset_n),
        .wrreg_req(wr_req),.rdreg_req(rd_req),.addr(reg_addr),.addr_mode(1'b1),
        .wrdata(8'd0),.rddata(rd_data),.device_id(device_id),
        .RW_Done(done),.ack(nack),.dly_cnt_max(32'd0),
        .i2c_sclk(serial_clk),.i2c_sdat(tp_sda)
    );
    localparam WAIT_BOOT=0,WAIT_POLL=1,LAUNCH=2,WAIT_BUS=3;
    reg [1:0] state;
    reg [2:0] operation;
    reg [31:0] timer;
    reg [15:0] sample_x,sample_y;
    reg sample_down;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            state<=WAIT_BOOT; operation<=0; timer<=0; bus_reset_n<=0;
            rd_req<=0; wr_req<=0; reg_addr<=16'h814e; device_id<=8'hba;
            sample_valid<=0; pressed<=0; x<=0; y<=0; online<=0;
            sample_x<=0; sample_y<=0; sample_down<=0;
        end else begin
            rd_req<=0; wr_req<=0; sample_valid<=0; bus_reset_n<=1;
            case(state)
                WAIT_BOOT: if(boot_done) begin state<=LAUNCH; operation<=0; end
                WAIT_POLL: begin
                    if(timer==POLL_CYCLES-1) begin timer<=0; operation<=0; state<=LAUNCH; end
                    else timer<=timer+1'b1;
                end
                LAUNCH: begin
                    case(operation)
                        0:reg_addr<=16'h814e;
                        1:reg_addr<=16'h8150;
                        2:reg_addr<=16'h8151;
                        3:reg_addr<=16'h8152;
                        4:reg_addr<=16'h8153;
                        default:reg_addr<=16'h814e;
                    endcase
                    rd_req<=operation!=5; wr_req<=operation==5;
                    timer<=0; state<=WAIT_BUS;
                end
                WAIT_BUS: begin
                    timer<=timer+1'b1;
                    if(done) begin
                        timer<=0;
                        if(nack) begin
                            online<=0; pressed<=0; state<=WAIT_POLL;
                            // Covers a module that retained the alternate address.
                            device_id<=(device_id==8'hba)?8'h28:8'hba;
                        end else begin
                            online<=1;
                            case(operation)
                                0: begin
                                    if(!rd_data[7]) state<=WAIT_POLL;
                                    else begin
                                        sample_down<=rd_data[3:0]!=0 && rd_data[3:0]<=5;
                                        operation<=(rd_data[3:0]!=0 && rd_data[3:0]<=5)?1:5;
                                        state<=LAUNCH;
                                    end
                                end
                                1:begin sample_x[7:0]<=rd_data; operation<=2; state<=LAUNCH; end
                                2:begin sample_x[15:8]<=rd_data; operation<=3; state<=LAUNCH; end
                                3:begin sample_y[7:0]<=rd_data; operation<=4; state<=LAUNCH; end
                                4:begin sample_y[15:8]<=rd_data; operation<=5; state<=LAUNCH; end
                                5:begin
                                    x<=sample_x; y<=sample_y; pressed<=sample_down;
                                    sample_valid<=1; state<=WAIT_POLL;
                                end
                            endcase
                        end
                    end else if(timer>=CLOCK_HZ/50) begin
                        bus_reset_n<=0; online<=0; pressed<=0;
                        timer<=0; state<=WAIT_POLL;
                    end
                end
            endcase
        end
    end
endmodule
