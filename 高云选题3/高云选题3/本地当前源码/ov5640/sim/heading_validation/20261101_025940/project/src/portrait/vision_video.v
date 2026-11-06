// Display-clock tap: the native DDR FIFO is consumed exactly as before.
// Two 104x78 RGB222 thumbnails, synchronous dual-port BSRAM inference.
// Portrait presentation is address mapping; native FIFO cadence is preserved.
module vision_video(
    input clk,rst_n,frame_begin,data_req,pixel_valid,zebra_enable,
    input [11:0] x,y,input [15:0] rgb,input [7:0] threshold,
    output [15:0] preview,
    output zebra_update,zebra_detected
);
    reg [5:0] bank0[0:8111],bank1[0:8111];
    reg write_bank,read_bank,ready;
    reg [18:0] captured;
    reg [9:0] phase_x,phase_y;
    reg [6:0] sample_x,sample_y;
    wire native=data_req && x>=80 && x<720 && y<480;
    wire pick_x=phase_x>=536,pick_y=phase_y>=402;
    wire [12:0] write_addr=(sample_y<<6)+(sample_y<<5)+(sample_y<<3)+sample_x;
    wire [5:0] packed_rgb={rgb[4:3],rgb[10:9],rgb[15:14]};
    reg [7:0] capture_low,capture_high;
    reg signed [6:0] threshold_bias;
    wire signed [9:0] adjusted_threshold=$signed({2'b0,threshold})+$signed(threshold_bias);
    wire [7:0] effective_threshold=(threshold<32 || threshold>223)?threshold:adjusted_threshold<1?8'd1:
        adjusted_threshold>254?8'd254:adjusted_threshold[7:0];
    wire [8:0] contrast_midpoint=({1'b0,capture_low}+{1'b0,capture_high})>>1;
    wire signed [9:0] requested_bias=$signed({1'b0,contrast_midpoint})-$signed({2'b0,threshold});
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin capture_low<=255;capture_high<=0;threshold_bias<=0;end
        else if(frame_begin)begin
            if(captured==307200 && capture_high>=capture_low+40)
                threshold_bias<=requested_bias>32?7'sd32:requested_bias < -32?-7'sd32:requested_bias;
            else threshold_bias<=0;
            capture_low<=255;capture_high<=0;
        end else if(native && pixel_valid && pick_x && pick_y)begin
            if(luma(packed_rgb)<capture_low)capture_low<=luma(packed_rgb);
            if(luma(packed_rgb)>capture_high)capture_high<=luma(packed_rgb);
        end
    end
    vision_zebra zebra(.clk(clk),.rst_n(rst_n),.enable(zebra_enable),.frame_begin(frame_begin),
        .frame_complete(captured==307200),
        .sample_valid(native && pixel_valid && pick_x && pick_y),
        .sample_x(sample_x),.sample_y(sample_y),
        .centre(7'd52),
        .dark(luma(packed_rgb)<effective_threshold),.update(zebra_update),.detected(zebra_detected));
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin
            write_bank<=0;read_bank<=1;ready<=0;captured<=0;
            phase_x<=0;phase_y<=0;sample_x<=0;sample_y<=0;
        end else if(frame_begin)begin
            if(captured==307200)begin read_bank<=write_bank;write_bank<=!write_bank;ready<=1;end
            captured<=0;phase_x<=0;phase_y<=0;sample_x<=0;sample_y<=0;
        end else if(native)begin
            if(pixel_valid)captured<=captured+1'b1;
            if(x==719)begin
                phase_x<=0;sample_x<=0;
                phase_y<=pick_y?phase_y-402:phase_y+78;
                if(pick_y)sample_y<=sample_y+1'b1;
            end else begin
                phase_x<=pick_x?phase_x-536:phase_x+104;
                if(pick_x)sample_x<=sample_x+1'b1;
            end
        end
    end
    always @(posedge clk)if(native && pixel_valid && pick_x && pick_y)begin
        if(write_bank)bank1[write_addr]<=packed_rgb;
        else bank0[write_addr]<=packed_rgb;
    end
    // Portrait canvas U=479-y (screen horizontal), V=x (screen vertical).
    // Each upright 4:3 picture is 480x360: V=28..387 and 396..755,
    // while U spans the panel's full 480-pixel short edge.
    wire upper=x>=28 && x<388;
    wire lower=x>=396 && x<756;
    wire [8:0] view_u=y<480?12'd479-y:12'd0;
    wire [8:0] view_v=upper?x-28:lower?x-396:0;
    // 104/480=78/360=13/60, no anisotropic stretching.
    // The Gowin constant divider did not preserve RTL semantics here: zero
    // mapped to 127 in the synthesized circuit, reading beyond the 78 rows.
    // Constant-fold the exact rational scale at elaboration; asynchronous
    // LUT reads keep the existing one-clock preview latency.
    reg [6:0] scale_lut[0:511];integer scale_index;
    initial for(scale_index=0;scale_index<512;scale_index=scale_index+1)
        scale_lut[scale_index]=(scale_index*13)/60;
    wire [6:0] source_column=scale_lut[view_u],source_row=scale_lut[view_v];
    wire [12:0] preview_addr=(source_row<<6)+(source_row<<5)+(source_row<<3)+source_column;
    reg [5:0] read0,read1;
    reg in_preview,binary_view,preview_ready;
    always @(posedge clk)begin
        read0<=bank0[preview_addr];read1<=bank1[preview_addr];
        in_preview<=data_req && (upper || lower);binary_view<=lower;preview_ready<=ready;
    end
    wire [5:0] color=read_bank?read1:read0;
    function [7:0] luma;
        input [5:0] c;
        reg [7:0] r,g,b;
        begin
            r={4{c[5:4]}};g={4{c[3:2]}};b={4{c[1:0]}};
            luma=(r>>2)+(g>>1)+(b>>2);
        end
    endfunction
    wire [15:0] expanded={color[1:0],color[1:0],color[1],color[3:2],color[3:2],color[3:2],color[5:4],color[5:4],color[5]};
    assign preview=!(in_preview && preview_ready)?16'h3986:
        binary_view?(luma(color)<effective_threshold?16'h0000:16'hffff):expanded;
endmodule
