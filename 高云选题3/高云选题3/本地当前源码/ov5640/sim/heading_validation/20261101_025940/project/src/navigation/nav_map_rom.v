// Synchronous ROMs infer Gowin BSRAM; their initialization enters the bitstream.
module nav_map_rom(
    input clk,
    input [16:0] display_addr, output display_blocked,
    input [14:0] plan_addr, output plan_blocked,
    output [7:0] plan_directions,
    output plan_central,
    input [14:0] display_plan_addr, output display_plan_blocked,
    input [5:0] metadata_addr, output reg [31:0] metadata
);
    reg [15:0] raw_map[0:7039];
    reg [15:0] plan_map[0:1759];
    reg [79:0] heading_map[0:1759];
    reg [31:0] dimensions[0:63];
    reg [15:0] raw_word,plan_word,display_plan_word;
    reg [3:0] raw_bit,plan_bit,display_plan_bit;
    reg [79:0] heading_word;
    wire [4:0] cell_info=heading_word[plan_bit*5+:5];
    wire [3:0] axes=cell_info[3:0];
    assign plan_central=cell_info[4];
    assign plan_directions={axes[3],axes[3],axes[2],axes[2],axes[1],axes[1],axes[0],axes[0]};
    assign display_blocked=raw_word[raw_bit];
    assign plan_blocked=plan_word[plan_bit];
    assign display_plan_blocked=display_plan_word[display_plan_bit];
    initial begin
        `include "nav_map_init.vh"
    end
    always @(posedge clk) begin
        raw_word<=raw_map[display_addr[16:4]];raw_bit<=display_addr[3:0];
        plan_word<=plan_map[plan_addr[14:4]];plan_bit<=plan_addr[3:0];
        heading_word<=heading_map[plan_addr[14:4]];
        display_plan_word<=plan_map[display_plan_addr[14:4]];display_plan_bit<=display_plan_addr[3:0];
        metadata<=dimensions[metadata_addr];
    end
endmodule
