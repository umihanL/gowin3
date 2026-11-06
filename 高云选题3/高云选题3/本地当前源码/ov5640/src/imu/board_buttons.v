// S4 asserts immediately. Release requires 20ms continuously high on clk50m.
// Initial values also provide a power-on reset when no key has been pressed.
module board_reset #(parameter RELEASE_CYCLES=1000000)(
    input clk,key_n,output reg reset_n=0
);
    reg [19:0] count=0;
    reg [1:0] sync=0;
    always @(posedge clk or negedge key_n) begin
        if(!key_n) begin sync<=0;count<=0;reset_n<=0;end
        else begin
            sync<={sync[0],1'b1};
            if(!sync[1]) begin count<=0;reset_n<=0;end
            else if(count<RELEASE_CYCLES-1) count<=count+1'b1;
            else reset_n<=1;
        end
    end
endmodule

module imu_zero_button #(parameter DEBOUNCE_CYCLES=660000)(
    input clk,rst_n,key_n,output reg pressed
);
    reg [1:0] sync; reg stable; reg [19:0] count;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin sync<=3;stable<=1;count<=0;pressed<=0;end
        else begin
            sync<={sync[0],key_n};pressed<=0;
            if(sync[1]==stable) count<=0;
            else if(count==DEBOUNCE_CYCLES-1) begin
                stable<=sync[1];count<=0;pressed<=!sync[1];
            end else count<=count+1'b1;
        end
    end
endmodule
