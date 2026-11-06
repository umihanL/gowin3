// 640x480 native 4:3 image centred in an 800x480 landscape panel.
// Side bars never consume FIFO words; menus still consume the hidden image.
module portrait_display #(
    parameter VIDEO_WIDTH=640, VIDEO_HEIGHT=480, VIDEO_LEFT=80
)(
    input clk, rst_n, frame_begin, data_req,
    input [11:0] x_addr,y_addr,
    input frame_available, fifo_empty, input [15:0] fifo_data,
    output fifo_read, output [15:0] pixel_data,
    output reg underflow
);
    reg ready_meta,ready_sync,frame_bad;
    wire in_video=data_req && x_addr>=VIDEO_LEFT && x_addr<VIDEO_LEFT+VIDEO_WIDTH && y_addr<VIDEO_HEIGHT;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            ready_meta<=0; ready_sync<=0; frame_bad<=0; underflow<=0;
        end else begin
            ready_meta<=frame_available; ready_sync<=ready_meta;
            if(frame_begin) frame_bad<=0;
            else if(in_video && ready_sync && fifo_empty) begin
                frame_bad<=1; underflow<=1;
            end
        end
    end
    assign fifo_read=in_video && ready_sync && !fifo_empty && !frame_bad;
    assign pixel_data=!in_video ? 16'd0 : fifo_read ? fifo_data :
                      (x_addr[5]^y_addr[5]) ? 16'hc618 : 16'had55;
endmodule
