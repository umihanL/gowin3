// Rotation CORDIC. Angle in tenths of a degree, result signed Q15.
// No ROM, RAM or run-time division. Full +/-180 degree input range.
module vehicle_sincos(input clk,rst_n,start,input signed [15:0] angle,
    output reg done,output reg signed [16:0] cosine,sine);
    reg busy;reg [4:0] step;
    reg signed [19:0] x,y;
    // Keep angle in tenths of a degree throughout rotation. This removes
    // signed /10 hardware from the initialization path.
    reg signed [23:0] z;
    function [23:0] atan_q12;
        input [4:0] i;
        begin case(i)
            0:atan_q12=184320;1:atan_q12=108810;2:atan_q12=57492;
            3:atan_q12=29184;4:atan_q12=14649;5:atan_q12=7331;
            6:atan_q12=3667;7:atan_q12=1833;8:atan_q12=917;
            9:atan_q12=458;10:atan_q12=229;11:atan_q12=115;
            12:atan_q12=57;13:atan_q12=29;14:atan_q12=14;
            15:atan_q12=7;default:atan_q12=4;
        endcase end
    endfunction
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin busy<=0;done<=0;step<=0;x<=0;y<=0;z<=0;cosine<=32768;sine<=0;end
        else begin
            done<=0;
            if(start && !busy)begin
                busy<=1;step<=0;y<=0;
                if(angle>900)begin x<=-19898;z<=(angle-16'sd1800)*24'sd4096;end
                else if(angle < -900)begin x<=-19898;z<=(angle+16'sd1800)*24'sd4096;end
                else begin x<=19898;z<=$signed(angle)*24'sd4096;end
            end else if(busy)begin
                if(step==17)begin cosine<=x[16:0];sine<=y[16:0];busy<=0;done<=1;end
                else begin
                    if(z>=0)begin x<=x-(y>>>step);y<=y+(x>>>step);z<=z-$signed(atan_q12(step))*10;end
                    else begin x<=x+(y>>>step);y<=y-(x>>>step);z<=z+$signed(atan_q12(step))*10;end
                    step<=step+1'b1;
                end
            end
        end
    end
endmodule
