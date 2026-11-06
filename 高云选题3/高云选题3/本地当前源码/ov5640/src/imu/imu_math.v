// Unsigned radix-4 restoring square root; 32 clocks for a 64-bit radicand.
module imu_sqrt(input clk,rst_n,start,input [63:0] value,
                output reg done,output reg [31:0] root);
    reg busy; reg [5:0] step; reg [63:0] bits;
    reg [65:0] remnant;
    wire [65:0] trial_rem={remnant[63:0],bits[63:62]};
    wire [65:0] divisor={32'd0,root,2'b01};
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin busy<=0;done<=0;root<=0;step<=0;bits<=0;remnant<=0;end
        else begin
            done<=0;
            if(start && !busy) begin busy<=1;root<=0;step<=0;bits<=value;remnant<=0;end
            else if(busy) begin
                bits<=bits<<2;
                if(trial_rem>=divisor) begin remnant<=trial_rem-divisor;root<={root[30:0],1'b1};end
                else begin remnant<=trial_rem;root<={root[30:0],1'b0};end
                if(step==31) begin busy<=0;done<=1;end else step<=step+1'b1;
            end
        end
    end
endmodule

// Full-quadrant atan2, vectoring CORDIC. Input Q28, output degrees * 65536.
module imu_atan2(input clk,rst_n,start,input signed [31:0] vx,vy,
                 output reg done,output reg signed [31:0] angle);
    reg busy; reg [4:0] step; reg signed [33:0] x,y;
    reg signed [31:0] z;
    function [31:0] atan_table;
        input [4:0] i;
        begin case(i)
            0:atan_table=2949120;1:atan_table=1740967;2:atan_table=919879;
            3:atan_table=466945;4:atan_table=234379;5:atan_table=117304;
            6:atan_table=58666;7:atan_table=29335;8:atan_table=14668;
            9:atan_table=7334;10:atan_table=3667;11:atan_table=1833;
            12:atan_table=917;13:atan_table=458;14:atan_table=229;
            15:atan_table=115;16:atan_table=57;17:atan_table=29;
            18:atan_table=14;19:atan_table=7;default:atan_table=0;
        endcase end
    endfunction
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin busy<=0;done<=0;angle<=0;step<=0;x<=0;y<=0;z<=0;end
        else begin
            done<=0;
            if(start && !busy) begin
                busy<=1;step<=0;
                if(vx<0) begin
                    x<=-$signed({vx[31],vx});y<=-$signed({vy[31],vy});
                    z<=vy<0 ? -32'sd11796480 : 32'sd11796480;
                end else begin x<=vx;y<=vy;z<=0;end
            end else if(busy) begin
                if(step==20) begin busy<=0;done<=1;angle<=z;end
                else begin
                    if(y>0) begin x<=x+(y>>>step);y<=y-(x>>>step);z<=z+$signed(atan_table(step));end
                    else if(y<0) begin x<=x-(y>>>step);y<=y+(x>>>step);z<=z-$signed(atan_table(step));end
                    step<=step+1'b1;
                end
            end
        end
    end
endmodule
