// Keep the panel running even when the camera or DDR is not ready.
// DataReq and H_Addr refer to the pixel sampled at the next ClkDisp edge.
module tft_video_source #(
    parameter IMAGE_WIDTH = 800
) (
    input clk,
    input rst_n,
    input source_ready,
    input frame_begin,
    input data_req,
    input [11:0] x_addr,
    input fifo_empty,
    input [15:0] fifo_data,
    output fifo_read,
    output [15:0] pixel_data
);
    reg [1:0] ready_sync;
    reg use_camera;
    reg [15:0] color_bar;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ready_sync <= 2'b00;
            use_camera <= 1'b0;
        end else begin
            ready_sync <= {ready_sync[0], source_ready};
            if (!ready_sync[1])
                use_camera <= 1'b0;
            else if (frame_begin)
                use_camera <= 1'b1;
        end
    end

    always @(*) begin
        if      (x_addr < IMAGE_WIDTH * 1 / 8) color_bar = 16'hffff;
        else if (x_addr < IMAGE_WIDTH * 2 / 8) color_bar = 16'hffe0;
        else if (x_addr < IMAGE_WIDTH * 3 / 8) color_bar = 16'h07ff;
        else if (x_addr < IMAGE_WIDTH * 4 / 8) color_bar = 16'h07e0;
        else if (x_addr < IMAGE_WIDTH * 5 / 8) color_bar = 16'hf81f;
        else if (x_addr < IMAGE_WIDTH * 6 / 8) color_bar = 16'hf800;
        else if (x_addr < IMAGE_WIDTH * 7 / 8) color_bar = 16'h001f;
        else                                color_bar = 16'h0000;
    end

    // Drain the prefetched FIFO during startup as well, keeping DDR addresses
    // aligned to the display raster when the camera becomes available.
    assign fifo_read = rst_n && data_req && !fifo_empty;
    assign pixel_data = (use_camera && !fifo_empty) ? fifo_data : color_bar;
endmodule
