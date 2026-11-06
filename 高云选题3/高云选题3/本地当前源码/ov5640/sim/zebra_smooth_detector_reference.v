// Crosswalk-only streaming detector. Independent vertical strips tolerate
// slanted transverse bars; horizontal runs also cover the longitudinal bars
// printed on this course. No road pose or steering is inferred from video.
module vision_zebra_reference(
    input clk,rst_n,frame_begin,frame_complete,sample_valid,dark,
    input [6:0] sample_x,sample_y,centre,
    output reg update,detected
);
    wire [6:0] c=centre<32?7'd32:centre>71?7'd71:centre;
    wire in_rows=sample_y>=24 && sample_y<=74;
    reg [4:0] row_white[0:2];
    reg [5:0] strip_run[0:2];
    reg [2:0] strip_have,strip_kind;
    reg [3:0] strip_bands[0:2],strip_gaps[0:2];
    wire [2:0] strip_white={row_white[2]>=9,row_white[1]>=9,row_white[0]>=9};
    wire [2:0] strip_dark={row_white[2]<=4,row_white[1]<=4,row_white[0]<=4};
    wire [2:0] strip_match;
    genvar s;
    generate for(s=0;s<3;s=s+1)begin: match_strip
        // Count a final band touching the bottom of the ROI as well.
        assign strip_match[s]=strip_gaps[s]>=2 && (strip_bands[s]>=3 ||
            (strip_bands[s]==2 && strip_have[s] && strip_kind[s] && strip_run[s]>=1 && strip_run[s]<=16));
    end endgenerate
    wire transverse=(strip_match[0] && strip_match[1]) ||
                    (strip_match[1] && strip_match[2]) || (strip_match[0] && strip_match[2]);
    reg [6:0] across_run;
    reg across_have,across_kind;
    reg [3:0] across_bands,across_gaps;
    reg [5:0] longitudinal_rows;
    reg [3:0] longitudinal_run;
    reg longitudinal;
    wire across_match=across_gaps>=2 && (across_bands>=3 ||
        (across_bands==2 && across_have && across_kind && across_run>=2 && across_run<=12));
    integer i;
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin
            strip_have<=0;strip_kind<=0;
            across_run<=0;across_have<=0;across_kind<=0;across_bands<=0;across_gaps<=0;
            longitudinal_rows<=0;longitudinal_run<=0;longitudinal<=0;update<=0;detected<=0;
            for(i=0;i<3;i=i+1)begin row_white[i]<=0;strip_run[i]<=0;strip_bands[i]<=0;strip_gaps[i]<=0;end
        end else begin
            update<=0;
            if(frame_begin)begin
                update<=frame_complete;
                if(frame_complete)detected<=transverse || (longitudinal && longitudinal_rows>=6);
                strip_have<=0;strip_kind<=0;
                across_run<=0;across_have<=0;across_kind<=0;across_bands<=0;across_gaps<=0;
                longitudinal_rows<=0;longitudinal_run<=0;longitudinal<=0;
                for(i=0;i<3;i=i+1)begin row_white[i]<=0;strip_run[i]<=0;strip_bands[i]<=0;strip_gaps[i]<=0;end
            end else if(sample_valid)begin
                if(in_rows && !dark)begin
                    if(sample_x>=c-24 && sample_x<=c-10)row_white[0]<=row_white[0]+1'b1;
                    if(sample_x>=c-7 && sample_x<=c+7)row_white[1]<=row_white[1]+1'b1;
                    if(sample_x>=c+10 && sample_x<=c+24)row_white[2]<=row_white[2]+1'b1;
                end
                if(in_rows && sample_x>=c-32 && sample_x<c+32)begin
                    if(!across_have)begin across_have<=1;across_kind<=!dark;across_run<=1;end
                    else if(across_kind==!dark)begin if(across_run<127)across_run<=across_run+1'b1;end
                    else begin
                        if(across_kind && across_run>=2 && across_run<=12 && across_bands<15)
                            across_bands<=across_bands+1'b1;
                        if(!across_kind && across_run>=1 && across_run<=12 && across_bands!=0 && across_gaps<15)
                            across_gaps<=across_gaps+1'b1;
                        across_kind<=!dark;across_run<=1;
                    end
                end
                if(sample_x==103)begin
                    across_run<=0;across_have<=0;across_bands<=0;across_gaps<=0;
                    for(i=0;i<3;i=i+1)begin
                        row_white[i]<=0;
                        if(in_rows)begin
                            if(strip_white[i] || strip_dark[i])begin
                                if(!strip_have[i])begin strip_have[i]<=1;strip_kind[i]<=strip_white[i];strip_run[i]<=1;end
                                else if(strip_kind[i]==strip_white[i])begin if(strip_run[i]<63)strip_run[i]<=strip_run[i]+1'b1;end
                                else begin
                                    if(strip_kind[i] && strip_run[i]>=1 && strip_run[i]<=16 && strip_bands[i]<15)
                                        strip_bands[i]<=strip_bands[i]+1'b1;
                                    if(!strip_kind[i] && strip_run[i]>=1 && strip_run[i]<=16 && strip_bands[i]!=0 && strip_gaps[i]<15)
                                        strip_gaps[i]<=strip_gaps[i]+1'b1;
                                    strip_kind[i]<=strip_white[i];strip_run[i]<=1;
                                end
                            end
                        end
                    end
                    if(in_rows)begin
                        if(across_match)begin
                            if(longitudinal_rows<63)longitudinal_rows<=longitudinal_rows+1'b1;
                            if(longitudinal_run<15)longitudinal_run<=longitudinal_run+1'b1;
                            if(longitudinal_run>=3)longitudinal<=1;
                        end else longitudinal_run<=0;
                    end
                end
            end
        end
    end
endmodule
