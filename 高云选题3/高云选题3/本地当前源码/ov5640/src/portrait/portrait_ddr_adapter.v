// Native RGB565, row-major DDR frame buffers. Addresses count 16-bit words.
// The Gowin write FIFO owns byte packing and the camera/DDR clock crossing.
module portrait_ddr_adapter #(
    parameter WIDTH=640,
    parameter [27:0] HEIGHT=480, SCREEN_WIDTH=WIDTH,
    parameter [27:0] FRAME_WORDS=WIDTH*HEIGHT, SLOT_WORDS=524288,
    parameter CHECK_LINES=0
)(
    input ui_clk, input rst_n, input init_calib_complete,
    input app_rdy, input app_wdf_rdy, input app_rd_data_valid,
    input [127:0] app_rd_data,
    output [27:0] app_addr, output app_en, output app_wdf_wren,
    output app_wdf_end, output [2:0] app_cmd, output [127:0] app_wdf_data,
    input cam_clk, input cam_rst_n, input frame_start,
    input pixel_valid, input [15:0] rgb, input line_active,
    input rd_clk, input rd_frame, input rfifo_rden,
    output rdfifo_empty, output [15:0] rfifo_dout,
    output reg frame_available, output reg overflow
);
    reg frame_ack, camera_tag, camera_done, camera_abort;
    reg ack_meta, ack_sync, capture_active, capture_busy, line_d;
    localparam FRAME_BITS=$clog2(FRAME_WORDS+1);
    reg [FRAME_BITS-1:0] pixel_count;
    reg [11:0] line_pixels, line_count;
    wire [127:0] packed_rgb;
    wire write_empty, write_full, write_pop;
    reg [5:0] write_reset_count;
    reg write_fifo_reset;
    always @(posedge ui_clk or negedge rst_n)
        if(!rst_n)write_fifo_reset<=1;else write_fifo_reset<=write_reset_count!=0;
    wire capture_pixel=capture_active && pixel_valid && !frame_start;
    wr_data_fifo camera_fifo (
        .Data(rgb),.Reset(write_fifo_reset),.WrClk(cam_clk),.RdClk(ui_clk),
        .WrEn(capture_pixel && !write_full && pixel_count<FRAME_WORDS),.RdEn(write_pop),
        .Wnum(),.Rnum(),.Almost_Empty(),.Almost_Full(),
        .Q(packed_rgb),.Empty(write_empty),.Full(write_full)
    );
    always @(posedge cam_clk or negedge cam_rst_n) begin
        if (!cam_rst_n) begin
            ack_meta<=0; ack_sync<=0; capture_active<=0; capture_busy<=0;
            camera_tag<=0; camera_done<=0; camera_abort<=0; overflow<=0;
            pixel_count<=0; line_pixels<=0; line_count<=0; line_d<=0;
        end else begin
            ack_meta<=frame_ack; ack_sync<=ack_meta; line_d<=line_active;
            if (frame_start) begin
                if (capture_active) begin
                    capture_active<=0; camera_abort<=~camera_abort; overflow<=1;
                end else if (!capture_busy || ack_sync==camera_tag) begin
                    capture_active<=1; capture_busy<=1; camera_tag<=~camera_tag;
                    pixel_count<=0; line_pixels<=0; line_count<=0;
                end
            end else if (capture_active) begin
                if (pixel_valid) begin
                    if (write_full || pixel_count==FRAME_WORDS ||
                        (CHECK_LINES && (!line_active || line_pixels==WIDTH))) begin
                        capture_active<=0; camera_abort<=~camera_abort; overflow<=1;
                    end else begin
                        pixel_count<=pixel_count+1'b1;
                        line_pixels<=line_pixels+1'b1;
                        if (!CHECK_LINES && pixel_count==FRAME_WORDS-1) begin
                            capture_active<=0; camera_done<=~camera_done;
                        end
                    end
                end
                if (CHECK_LINES && line_d && !line_active) begin
                    line_pixels<=0;
                    if (line_pixels!=WIDTH || line_count>=HEIGHT) begin
                        capture_active<=0; camera_abort<=~camera_abort; overflow<=1;
                    end else begin
                        line_count<=line_count+1'b1;
                        if (line_count==HEIGHT-1) begin
                            capture_active<=0; camera_done<=~camera_done;
                        end
                    end
                end
            end
        end
    end

    // Frame controls cross as toggles; RGB crosses only through vendor FIFOs.
    reg tag_meta,tag_sync,tag_seen,done_meta,done_sync,done_seen;
    reg abort_meta,abort_sync,abort_seen;
    reg rd_toggle,rd_meta,rd_sync,rd_seen;
    always @(posedge rd_clk or negedge rst_n)
        if (!rst_n) rd_toggle<=0;
        else if (rd_frame) rd_toggle<=~rd_toggle;
    always @(posedge ui_clk or negedge rst_n) begin
        if (!rst_n) begin
            tag_meta<=0; tag_sync<=0; done_meta<=0; done_sync<=0;
            abort_meta<=0; abort_sync<=0; rd_meta<=0; rd_sync<=0;
        end else begin
            tag_meta<=camera_tag; tag_sync<=tag_meta;
            done_meta<=camera_done; done_sync<=done_meta;
            abort_meta<=camera_abort; abort_sync<=abort_meta;
            rd_meta<=rd_toggle; rd_sync<=rd_meta;
        end
    end
    reg capture_writing,capture_complete,discard_frame,pending_frame;
    reg [127:0] write_data;
    reg write_valid,load_enable;
    reg write_slot,read_slot;
    reg [FRAME_BITS-1:0] write_offset,read_offset;
    wire [9:0] fifo_count;
    reg [5:0] outstanding;
    reg [1:0] read_credit;
    reg read_space;
    reg flushing;
    reg [4:0] reset_count;
    wire abort_event=abort_sync!=abort_seen;
    reg fifo_reset;
    always @(posedge ui_clk or negedge rst_n)
        if(!rst_n)fifo_reset<=1;else fifo_reset<=reset_count!=0;
    // Register the capacity decision before arbitration.  The 160-beat
    // watermark reserves space for all 16 pending reads and status latency.
    wire need_read=init_calib_complete && frame_available && !flushing
                  && reset_count==0 && read_offset<FRAME_WORDS
                  && outstanding<16 && read_space;
    wire write_wanted=init_calib_complete && capture_writing && !discard_frame
                     && !abort_event && write_reset_count==0 && write_valid;
    // A camera frame is written continuously, but display reads must still
    // receive a bounded share of the DDR command port.  The credit counter
    // gives one read at least every four UI clocks without using FIFO-count
    // feedback in the BSRAM enable path.
    wire read_priority=need_read && (!write_wanted || read_credit==0);
    wire do_read=read_priority && app_rdy;
    wire do_write=!read_priority && write_wanted && app_rdy && app_wdf_rdy;
    assign app_cmd=do_read?3'd1:3'd0;
    assign app_en=do_read || do_write;
    assign app_addr=do_read?((read_slot?SLOT_WORDS:28'd0)+read_offset):
                            ((write_slot?SLOT_WORDS:28'd0)+write_offset);
    assign app_wdf_data=write_data;
    assign app_wdf_wren=do_write;
    assign app_wdf_end=do_write;
    // This one-beat register isolates the FIFO BSRAM enable from arbitration.
    assign write_pop=load_enable && !write_valid && !write_empty;
    rd_data_fifo scan_fifo (
        .Data(app_rd_data),.Reset(fifo_reset),.WrClk(ui_clk),.RdClk(rd_clk),
        .WrEn(app_rd_data_valid && !flushing && reset_count==0),.RdEn(rfifo_rden),
        .Wnum(fifo_count),.Rnum(),.Almost_Empty(),.Almost_Full(),
        .Q(rfifo_dout),.Empty(rdfifo_empty),.Full()
    );
    always @(posedge ui_clk or negedge rst_n) begin
        if (!rst_n) begin
            tag_seen<=0; done_seen<=0; abort_seen<=0; frame_ack<=0;
            write_data<=0; write_valid<=0; load_enable<=0;
            capture_writing<=0; capture_complete<=0; discard_frame<=0;
            pending_frame<=0; frame_available<=0; write_slot<=1; read_slot<=0;
            write_offset<=0; read_offset<=0; outstanding<=0; read_credit<=0; read_space<=0;
            rd_seen<=0; flushing<=1; reset_count<=0; write_reset_count<=0;
        end else begin
            read_space<=fifo_count<10'd160;
            load_enable<=init_calib_complete && capture_writing && !discard_frame
                         && !abort_event && write_reset_count==0;
            if (write_pop) begin write_data<=packed_rgb; write_valid<=1; end
            if (do_write) write_valid<=0;
            case ({do_read,app_rd_data_valid})
                2'b10: outstanding<=outstanding+1'b1;
                2'b01: if(outstanding!=0) outstanding<=outstanding-1'b1;
                default: ;
            endcase
            if (do_read) read_credit<=2'd3;
            else if (!write_wanted) read_credit<=0;
            else if (read_credit!=0) read_credit<=read_credit-1'b1;
            if (do_read) read_offset<=read_offset+28'd8;
            if (rd_sync!=rd_seen) begin rd_seen<=rd_sync; flushing<=1; end
            if (flushing && outstanding==0 && !do_read && reset_count==0) begin
                reset_count<=31; flushing<=0; read_offset<=0;
                if (pending_frame && !discard_frame && !abort_event) begin
                    read_slot<=write_slot; write_slot<=read_slot;
                    pending_frame<=0; frame_available<=1; frame_ack<=tag_seen;
                    capture_complete<=0;
                end
            end else if (reset_count!=0) reset_count<=reset_count-1'b1;

            if (tag_sync!=tag_seen) begin
                tag_seen<=tag_sync; capture_writing<=1; capture_complete<=0;
                write_offset<=0; write_valid<=0;
            end
            if (done_sync!=done_seen) begin
                done_seen<=done_sync; capture_complete<=1;
            end
            if (do_write) begin
                write_offset<=write_offset+28'd8;
                if (write_offset==FRAME_WORDS-8) capture_writing<=0;
            end
            if (capture_complete && write_offset==FRAME_WORDS && !discard_frame && !abort_event && !pending_frame)
                pending_frame<=1;
            // The producer is stopped before an abort crosses this boundary.
            // Reset also removes a partial group of fewer than eight pixels.
            if (abort_event) begin
                abort_seen<=abort_sync; discard_frame<=1; pending_frame<=0;
                capture_writing<=0; capture_complete<=0; write_reset_count<=31; write_valid<=0;
            end else if (write_reset_count!=0) begin
                write_reset_count<=write_reset_count-1'b1; write_valid<=0;
                if (write_reset_count==1) begin
                    discard_frame<=0; frame_ack<=tag_seen; write_offset<=0;
                end
            end
        end
    end
endmodule
