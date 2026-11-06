// Streaming binary detector: three broad white transverse bands separated
// by dark road. Three independent lateral windows must agree. No image RAM.
module vision_zebra(
    input clk,rst_n,enable,frame_begin,frame_complete,sample_valid,dark,
    input [6:0] sample_x,sample_y,centre,
    output reg update,detected
);
    reg [2:0] left_white,middle_white,right_white;
    reg [5:0] run_length;reg have_kind,white_kind;
    reg [3:0] white_bands,dark_bands;reg near_white;
    reg collecting;
    wire [6:0] c=centre<16?7'd16:centre>87?7'd87:centre;
    wire white_row=left_white>=4 && middle_white>=4 && right_white>=4;
    wire dark_row=left_white<=1 && middle_white<=1 && right_white<=1;
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin
            left_white<=0;middle_white<=0;right_white<=0;run_length<=0;
            have_kind<=0;white_kind<=0;white_bands<=0;dark_bands<=0;near_white<=0;
            update<=0;detected<=0;
            collecting<=0;
        end else if(enable!==1'b1)begin
            // Disabling clears the actual detector, not only its UI output.
            left_white<=0;middle_white<=0;right_white<=0;run_length<=0;
            have_kind<=0;white_kind<=0;white_bands<=0;dark_bands<=0;near_white<=0;
            update<=0;detected<=0;collecting<=0;
        end else begin
            update<=0;
            if(frame_begin)begin
                update<=frame_complete && collecting;
                detected<=frame_complete && collecting && white_bands>=3 && dark_bands>=2 && near_white;
                collecting<=1;
                left_white<=0;middle_white<=0;right_white<=0;run_length<=0;
                have_kind<=0;white_kind<=0;white_bands<=0;dark_bands<=0;near_white<=0;
            end else if(sample_valid && collecting)begin
                if(!dark)begin
                    if(sample_x>=c-8 && sample_x<=c-4)left_white<=left_white+1'b1;
                    if(sample_x>=c-2 && sample_x<=c+2)middle_white<=middle_white+1'b1;
                    if(sample_x>=c+4 && sample_x<=c+8)right_white<=right_white+1'b1;
                end
                if(sample_x==103)begin
                    left_white<=0;middle_white<=0;right_white<=0;
                    if(sample_y>=34 && sample_y<=74)begin
                        if(white_row && sample_y>=60)near_white<=1;
                        if(white_row || dark_row)begin
                            if(!have_kind)begin have_kind<=1;white_kind<=white_row;run_length<=1;end
                            else if(white_row==white_kind)begin if(run_length<63)run_length<=run_length+1'b1;end
                            else begin
                                if(run_length>=2 && run_length<=12)begin
                                    if(white_kind && white_bands<15)white_bands<=white_bands+1'b1;
                                    if(!white_kind && dark_bands<15)dark_bands<=dark_bands+1'b1;
                                end
                                white_kind<=white_row;run_length<=1;
                            end
                        end else begin have_kind<=0;run_length<=0;end
                    end
                end
            end
        end
    end
endmodule
