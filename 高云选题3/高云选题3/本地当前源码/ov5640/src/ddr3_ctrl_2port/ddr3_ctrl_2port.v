module ddr3_ctrl_2port(
    input           clk                 ,      //50M时钟信号
    input           pll_lock            ,
    input           clk_200m            ,      //DDR3参考时钟信号
    input           sys_rst_n           ,      //外部复位信号
    output          init_calib_complete ,    //DDR初始化完成信号
    output          pll_stop            ,
    input wr_clk, input cam_rst_n, input wr_load,
    input wfifo_wren, input [15:0] wfifo_din, input camera_line_active,
    input rd_clk, input rd_load, input rfifo_rden,
    output rdfifo_empty, output [15:0] rfifo_dout,
    output frame_available, output capture_overflow,

    //DDR3   
    inout   [15:0]     ddr3_dq             ,   //DDR3 数据
    inout   [1:0]      ddr3_dqs_n          ,   //DDR3 dqs负
    inout   [1:0]      ddr3_dqs_p          ,   //DDR3 dqs正  
    output  [13:0]     ddr3_addr           ,   //DDR3 地址   
    output  [2:0]      ddr3_ba             ,   //DDR3 banck 选择
    output             ddr3_ras_n          ,   //DDR3 行选择
    output             ddr3_cas_n          ,   //DDR3 列选择
    output             ddr3_we_n           ,   //DDR3 读写选择
    output             ddr3_reset_n        ,   //DDR3 复位
    output  [0:0]      ddr3_ck_p           ,   //DDR3 时钟正
    output  [0:0]      ddr3_ck_n           ,   //DDR3 时钟负
    output  [0:0]      ddr3_cke            ,   //DDR3 时钟使能
    output  [0:0]      ddr3_cs_n           ,   //DDR3 片选
    output  [1:0]      ddr3_dm             ,   //DDR3_dm
    output  [0:0]      ddr3_odt                //DDR3_odt   
);
    wire    ui_clk;
    wire    ui_clk_sync_rst;
    wire    app_rdy;
    wire    app_wdf_rdy;
    wire    app_rd_data_valid;
    wire [127:0] app_rd_data;
    wire [27:0] app_addr;
    wire app_en;
    wire app_wdf_wren;
    wire app_wdf_end;
    wire [2:0] app_cmd;
    wire [127:0] app_wdf_data;
    wire app_rd_data_end, app_sr_active, app_ref_ack;
    wire adapter_rst_n;
    reset_sync u_ui_reset (
        .clk(ui_clk), .arst_n(sys_rst_n & ~ui_clk_sync_rst),
        .rst_n(adapter_rst_n)
    );

    portrait_ddr_adapter #(.CHECK_LINES(1)) portrait_ddr_adapter(
        .ui_clk(ui_clk), .rst_n(adapter_rst_n), .init_calib_complete(init_calib_complete),
        .app_rdy(app_rdy), .app_wdf_rdy(app_wdf_rdy),
        .app_rd_data_valid(app_rd_data_valid), .app_rd_data(app_rd_data),
        .app_addr(app_addr), .app_en(app_en), .app_wdf_wren(app_wdf_wren),
        .app_wdf_end(app_wdf_end), .app_cmd(app_cmd), .app_wdf_data(app_wdf_data),
        .cam_clk(wr_clk), .cam_rst_n(cam_rst_n), .frame_start(wr_load),
        .pixel_valid(wfifo_wren), .rgb(wfifo_din), .line_active(camera_line_active),
        .rd_clk(rd_clk), .rd_frame(rd_load), .rfifo_rden(rfifo_rden),
        .rdfifo_empty(rdfifo_empty), .rfifo_dout(rfifo_dout),
        .frame_available(frame_available), .overflow(capture_overflow)
    );

    DDR3_Memory_Interface_Top DDR3_Memory_Interface_Top(
        .clk(clk), //input clk
        .pll_stop(pll_stop), //output pll_stop
        .memory_clk(clk_200m), //input memory_clk
        .pll_lock(pll_lock), //input pll_lock
        .rst_n(sys_rst_n), //input rst_n
        .cmd_ready(app_rdy), //output cmd_ready
        .cmd(app_cmd), //input [2:0] cmd
        .cmd_en(app_en), //input cmd_en
        .addr(app_addr), //input [27:0] addr
        .wr_data_rdy(app_wdf_rdy), //output wr_data_rdy
        .wr_data(app_wdf_data), //input [127:0] wr_data
        .wr_data_en(app_wdf_wren), //input wr_data_en
        .wr_data_end(app_wdf_end), //input wr_data_end
        .wr_data_mask(16'b0), //input [15:0] wr_data_mask
        .rd_data(app_rd_data), //output [127:0] rd_data
        .rd_data_valid(app_rd_data_valid), //output rd_data_valid
        .rd_data_end(app_rd_data_end), //output rd_data_end
        .sr_req(1'b0), //input sr_req
        .ref_req(1'b0), //input ref_req
        .sr_ack(app_sr_active), //output sr_ack
        .ref_ack(app_ref_ack), //output ref_ack
        .init_calib_complete(init_calib_complete), //output init_calib_complete
        .clk_out(ui_clk), //output clk_out
        .ddr_rst(ui_clk_sync_rst), //output ddr_rst
        .burst(1'b1), //input burst
        .O_ddr_addr(ddr3_addr), //output [13:0] O_ddr_addr
        .O_ddr_ba(ddr3_ba), //output [2:0] O_ddr_ba
        .O_ddr_cs_n(ddr3_cs_n), //output O_ddr_cs_n
        .O_ddr_ras_n(ddr3_ras_n), //output O_ddr_ras_n
        .O_ddr_cas_n(ddr3_cas_n), //output O_ddr_cas_n
        .O_ddr_we_n(ddr3_we_n), //output O_ddr_we_n
        .O_ddr_clk(ddr3_ck_p), //output O_ddr_clk
        .O_ddr_clk_n(ddr3_ck_n), //output O_ddr_clk_n
        .O_ddr_cke(ddr3_cke), //output O_ddr_cke
        .O_ddr_odt(ddr3_odt), //output O_ddr_odt
        .O_ddr_reset_n(ddr3_reset_n), //output O_ddr_reset_n
        .O_ddr_dqm(ddr3_dm), //output [1:0] O_ddr_dqm
        .IO_ddr_dq(ddr3_dq), //inout [15:0] IO_ddr_dq
        .IO_ddr_dqs(ddr3_dqs_p), //inout [1:0] IO_ddr_dqs
        .IO_ddr_dqs_n(ddr3_dqs_n) //inout [1:0] IO_ddr_dqs_n
    );

endmodule 
