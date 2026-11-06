// Four editable, axis-aligned road barriers. Coordinates are raw centimetres.
// Register the overlay exactly like the map ROM: no additional frame RAM.
module nav_barrier_overlay #(parameter DIRECT_COORDINATES=0)(input clk,input [3:0] valid,
    input [35:0] x0,y0,x1,y1,
    input [14:0] plan_addr,display_plan_addr,input [16:0] raw_addr,
    input [17:0] plan_coordinates,display_coordinates,raw_coordinates,
    output reg plan_hit,raw_hit,display_plan_hit);
    function [17:0] coordinates;
        input [16:0] address;input coarse;
        reg [17:0] remainder;reg [8:0] row;integer b;
        begin
            remainder={1'b0,address};row=0;
            for(b=8;b>=0;b=b-1)begin
                if(remainder>=((coarse?18'd176:18'd352)<<b))begin
                    remainder=remainder-((coarse?18'd176:18'd352)<<b);row[b]=1;
                end
            end
            coordinates=coarse?{row[7:0],1'b1,remainder[7:0],1'b1}:{row,remainder[8:0]};
        end
    endfunction
    function occupied;
        input [17:0] xy;input [8:0] margin;
        integer i;reg [9:0] xx,yy;
        begin
            xx={1'b0,xy[8:0]};yy={1'b0,xy[17:9]};occupied=0;
            for(i=0;i<4;i=i+1)
                if(valid[i] && xx+margin>=x0[i*9+:9] && xx<=x1[i*9+:9]+{1'b0,margin} &&
                   yy+margin>=y0[i*9+:9] && yy<=y1[i*9+:9]+{1'b0,margin})occupied=1;
        end
    endfunction
    // Callers already know XY before raster addressing. Avoid an unnecessary
    // multiply followed by a long constant divide in the real-time path.
    wire [17:0] plan_xy=DIRECT_COORDINATES?plan_coordinates:coordinates({2'b0,plan_addr},1'b1);
    wire [17:0] display_xy=DIRECT_COORDINATES?display_coordinates:coordinates({2'b0,display_plan_addr},1'b1);
    wire [17:0] raw_xy=DIRECT_COORDINATES?raw_coordinates:coordinates(raw_addr,1'b0);
    wire plan_next=occupied(plan_xy,9'd11),display_next=occupied(display_xy,9'd11),raw_next=occupied(raw_xy,9'd1);
    always @(posedge clk)begin
        plan_hit<=plan_next;
        display_plan_hit<=display_next;
        raw_hit<=raw_next;
    end
endmodule
