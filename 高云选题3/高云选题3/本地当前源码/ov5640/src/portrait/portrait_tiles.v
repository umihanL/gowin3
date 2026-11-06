// Two 4-line mailboxes cross camera -> DDR clock domains. Each bank is
// immutable between its ready toggle and its acknowledge toggle.
module portrait_tiles #(
    parameter WIDTH=640, HEIGHT=480, TILE_ROWS=4
)(
    input cam_clk, input cam_rst_n, input frame_start,
    input pixel_valid, input [15:0] rgb,
    input ui_clk, input ui_rst_n,
    input frame_ack, output reg frame_tag,
    output reg abort_toggle,
    output reg [1:0] ready_toggle,
    input [1:0] ack_toggle,
    output reg [7:0] meta0, output reg [7:0] meta1,
    input [12:0] read_addr, output reg [15:0] read_data,
    output reg overflow
);
    reg [15:0] mem [0:8191];
    reg [1:0] ack_meta, ack_sync;
    reg frame_ack_meta, frame_ack_sync;
    reg active, busy, bank;
    reg [11:0] addr;
    reg [6:0] tile;
    wire bank_free=(ack_sync[bank]==ready_toggle[bank]);
    always @(posedge ui_clk) read_data<=mem[read_addr];
    always @(posedge cam_clk)
        if (cam_rst_n && active && pixel_valid && bank_free)
            mem[{bank,addr}]<=rgb;
    always @(posedge cam_clk or negedge cam_rst_n) begin
        if (!cam_rst_n) begin
            ack_meta<=0; ack_sync<=0; frame_ack_meta<=0; frame_ack_sync<=0;
            active<=0; busy<=0; bank<=0; addr<=0; tile<=0;
            frame_tag<=0; abort_toggle<=0; ready_toggle<=0;
            meta0<=0; meta1<=0; overflow<=0;
        end else begin
            ack_meta<=ack_toggle; ack_sync<=ack_meta;
            frame_ack_meta<=frame_ack; frame_ack_sync<=frame_ack_meta;
            if (frame_start) begin
                if (active) begin
                    // A malformed/incomplete camera frame must never be shown.
                    active<=0; abort_toggle<=~abort_toggle; overflow<=1;
                end else if (!busy || frame_ack_sync==frame_tag) begin
                    active<=1; busy<=1; frame_tag<=~frame_tag;
                    bank<=0; addr<=0; tile<=0;
                end
            end else if (active && pixel_valid) begin
                if (!bank_free) begin
                    active<=0; abort_toggle<=~abort_toggle; overflow<=1;
                end else if (addr==WIDTH*TILE_ROWS-1) begin
                    if (bank) meta1<={frame_tag,tile};
                    else meta0<={frame_tag,tile};
                    ready_toggle[bank]<=~ready_toggle[bank];
                    addr<=0; bank<=~bank; tile<=tile+1'b1;
                    if (tile==HEIGHT/TILE_ROWS-1) active<=0;
                end else addr<=addr+1'b1;
            end
        end
    end
endmodule
