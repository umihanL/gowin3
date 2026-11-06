`timescale 1ns/1ps
module nav_map_rom_tb;
    reg clk=0;always #5 clk=~clk;
    reg [16:0] ra=0;reg [14:0] pa=0,da=0;
    wire rb,pb,db;wire [31:0] metadata;wire [7:0] directions;wire central;
    reg raw[0:112639],plan[0:28159];
    reg [7:0] heading[0:28159];
    reg centre[0:28159];
    nav_map_rom dut(.clk(clk),.display_addr(ra),.display_blocked(rb),.plan_addr(pa),.plan_blocked(pb),
        .display_plan_addr(da),.display_plan_blocked(db),.metadata_addr(6'd4),.metadata(metadata),.plan_directions(directions),.plan_central(central));
    integer i;
    initial begin
        $readmemb("src/navigation/map_raw_1cm.mem",raw);
        $readmemb("src/navigation/map_plan_2cm.mem",plan);
        $readmemh("src/navigation/map_heading_2cm.mem",heading);
        $readmemb("src/navigation/map_central_2cm.mem",centre);
        for(i=0;i<112640;i=i+1)begin
            @(negedge clk);ra=i;pa=i%28160;da=28159-i%28160;
            @(negedge clk);
            if(rb!==raw[i] || pb!==plan[i%28160] || db!==plan[28159-i%28160])$fatal(1,"map ROM address %d",i);
            if(directions!==heading[i%28160])$fatal(1,"body-heading ROM address %d",i);
            if(central!==centre[i%28160])$fatal(1,"clearance ROM address %d",i);
        end
        if(metadata!=190)$fatal(1,"vehicle width metadata");
        $display("PASS map ROM: every 1cm and 2cm cell, packed bit order, both planning ports and metadata");$finish;
    end
endmodule
