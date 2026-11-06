// Assert immediately when the clock is unavailable; release after three edges.
module reset_sync (
    input clk,
    input arst_n,
    output rst_n
);
    reg [2:0] reset_pipe = 3'b000;
    always @(posedge clk or negedge arst_n)
        if (!arst_n)
            reset_pipe <= 3'b000;
        else
            reset_pipe <= {reset_pipe[1:0], 1'b1};
    assign rst_n = reset_pipe[2];
endmodule
