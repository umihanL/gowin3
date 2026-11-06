`timescale 1ns/1ps
// Independent physical I2C slave: no access to DUT internals. Pages and
// factory parameters differ per device, so a shared stop/SPAD value fails.
module vl53l0x_sensor_model #(
    parameter integer SENSOR=0, parameter [15:0] DISTANCE=123
)(
    input rst_n, xs, present, invalid_range, never_ready, nack_config,
    input bad_id, never_spad,
    inout sda, input scl
);
    localparam RX_ADDR=0, RX_REG=1, RX_DATA=2, ACK_SETUP=3,
        ACK_HIGH=4, ACK_RELEASE=5, TX=6, TX_RELEASE=7, MASTER_ACK=8,
        MASTER_ACK_END=9, IGNORE=10;
    localparam [7:0] FACTORY_STOP=8'h35+SENSOR*17;
    localparam [7:0] FACTORY_SPAD=(SENSOR%2 ? 8'h80 : 0)+(10+SENSOR*3);
    localparam [47:0] FACTORY_MAP=48'hDDEF7DBFF6F3;
    integer state=IGNORE, next_state=IGNORE, bits=0;
    reg [6:0] address=7'h29;
    reg [7:0] pointer=0, page=0, shift=0, outgoing=0, memory[0:2047];
    reg selected=0, drive_low=0, master_acked=0;
    reg ranging=0, calibrating=0, spad_requested=0;
    integer vhv_count=0, phase_count=0, clear_count=0, stop_count=0;
    integer ready_reads=0, spad_reads=0, distance_reads=0, repeated_starts=0;
    integer init_writes=0, id_reads=0, i;
    reg [47:0] expected_map;
    integer enabled;
    assign sda=drive_low ? 1'b0 : 1'bz;
    function [7:0] read_register;
        input [7:0] reg_address;
        begin
            read_register=memory[{page[2:0],reg_address}];
            if(page==0) case(reg_address)
                8'hC0: read_register=bad_id ? 8'hEA : 8'hEE;
                8'hC1: read_register=8'hAA;
                8'h13: read_register=(ranging || calibrating) && !never_ready && ready_reads>1 ? 4 : 0;
                8'h14: read_register=invalid_range ? 8'h20 : 8'h58;
                8'h1E: read_register=DISTANCE[15:8];
                8'h1F: read_register=DISTANCE[7:0];
            endcase
            if(page==1 && reg_address==8'h91) read_register=FACTORY_STOP;
            if(page==7 && reg_address==8'h83)
                read_register=spad_requested && spad_reads>1 && !never_spad ? 1 : 0;
            if(page==7 && reg_address==8'h92) read_register=FACTORY_SPAD;
        end
    endfunction
    task write_register;
        input [7:0] reg_address, value;
        begin
            init_writes=init_writes+1;
            if(reg_address==8'hFF) page=value;
            else begin
                memory[{page[2:0],reg_address}]=value;
                if(page==0 && reg_address==8'h8A) address=value[6:0];
                if(page==7 && reg_address==8'h83 && value==0) begin
                    spad_requested=1; spad_reads=0;
                end
                if(page==0 && reg_address==0 && (value==8'h41 || value==1)) begin
                    if(value==8'h41) begin
                        if(memory[1]!=1) $fatal(1,"Device %0d missing VHV sequence",SENSOR);
                        vhv_count=vhv_count+1;
                    end else begin
                        if(memory[1]!=2 || vhv_count!=1 || stop_count!=1)
                            $fatal(1,"Device %0d missing first calibration",SENSOR);
                        phase_count=phase_count+1;
                    end
                    calibrating=1; ready_reads=0;
                end
                if(page==0 && reg_address==0 && value==0 && calibrating) begin
                    if(ready_reads<2 || clear_count<vhv_count+phase_count+1)
                        $fatal(1,"Device %0d calibration missing poll/clear",SENSOR);
                    calibrating=0; stop_count=stop_count+1;
                end
                if(page==0 && reg_address==8'h0B && value==1) clear_count=clear_count+1;
                if(page==0 && reg_address==0 && value==2) begin
                    if(vhv_count!=1 || phase_count!=1 || stop_count!=2 || init_writes<110)
                        $fatal(1,"Device %0d incomplete init/calibrations",SENSOR);
                    if(memory['h191]!=FACTORY_STOP)
                        $fatal(1,"Device %0d stop_variable was not restored independently",SENSOR);
                    if(memory[1]!=8'hE8 || memory['h89]!=8'hA1 || memory['h84]!=8'hA5 ||
                       {memory['h71],memory['h72]}!=16'h02FA ||
                       {memory['h44],memory['h45]}!=16'h0020 ||
                       {memory['h51],memory['h52]}!=16'h0096 ||
                       memory['h50]!=6 || memory['h70]!=4 || memory['h46]!=8'h25 ||
                       memory['h60]!=0 || memory['h0A]!=4 ||
                       memory['h14F]!=0 || memory['h14E]!=8'h2C || memory['hB6]!=8'hB4)
                        $fatal(1,"Device %0d config/read-modify-write mismatch",SENSOR);
                    expected_map=FACTORY_MAP; enabled=0;
                    for(i=0;i<48;i=i+1) begin
                        if((FACTORY_SPAD[7] && i<12) || enabled==FACTORY_SPAD[6:0])
                            expected_map[i]=0;
                        else if(expected_map[i]) enabled=enabled+1;
                    end
                    if({memory['hB5],memory['hB4],memory['hB3],memory['hB2],memory['hB1],memory['hB0]}!=expected_map)
                        $fatal(1,"Device %0d factory SPAD filtering mismatch",SENSOR);
                    if(id_reads<2 || !spad_requested || spad_reads<2)
                        $fatal(1,"Device %0d missing ID/SPAD queries",SENSOR);
                    ranging=1; ready_reads=0;
                end
            end
        end
    endtask
    always @(negedge rst_n or negedge xs) begin
        state=IGNORE; address=7'h29; pointer=0; page=0; drive_low=0;
        selected=0; ranging=0; calibrating=0; spad_requested=0;
        vhv_count=0; phase_count=0; clear_count=0; stop_count=0;
        ready_reads=0; spad_reads=0; distance_reads=0; repeated_starts=0;
        init_writes=0; id_reads=0;
        for(i=0;i<2048;i=i+1) memory[i]=0;
        memory['h89]=8'hA0; memory['h84]=8'hB5; memory['h60]=8'h40;
        memory['h683]=8'hA0;
        for(i=0;i<6;i=i+1) memory['hB0+i]=FACTORY_MAP>>(i*8);
    end
    always @(negedge sda) if(scl===1'b1 && !drive_low && rst_n && xs) begin
        if(selected) repeated_starts=repeated_starts+1;
        state=RX_ADDR; shift=0; bits=0; selected=0;
    end
    always @(posedge sda) if(scl===1'b1 && !drive_low) begin state=IGNORE; selected=0; end
    always @(posedge scl) if(rst_n && xs && present) begin
        if(state==RX_ADDR || state==RX_REG || state==RX_DATA) begin
            shift={shift[6:0],sda}; bits=bits+1;
            if(bits==8) begin
                bits=0;
                case(state)
                    RX_ADDR: begin
                        selected=shift[7:1]==address; next_state=shift[0]?TX:RX_REG;
                        if(selected && shift[0]) begin
                            if(page==0 && pointer==8'hC0) id_reads=id_reads+1;
                            if(page==0 && pointer==8'h13) ready_reads=ready_reads+1;
                            if(page==0 && pointer==8'h1E) distance_reads=distance_reads+1;
                            if(page==7 && pointer==8'h83) spad_reads=spad_reads+1;
                        end
                    end
                    RX_REG: begin pointer=shift; next_state=RX_DATA; end
                    RX_DATA: begin
                        if(selected) begin
                            if(nack_config && page==0 && pointer==8'h47) selected=0;
                            else write_register(pointer,shift);
                        end
                        pointer=pointer+1'b1; next_state=RX_DATA;
                    end
                endcase
                state=ACK_SETUP;
            end
        end else if(state==ACK_HIGH) state=ACK_RELEASE;
        else if(state==TX) begin if(bits==0) state=TX_RELEASE; else bits=bits-1; end
        else if(state==MASTER_ACK) begin master_acked=!sda; state=MASTER_ACK_END; end
    end
    always @(negedge scl) if(rst_n && xs && present) begin
        if(state==ACK_SETUP) begin drive_low=selected; state=ACK_HIGH; end
        else if(state==ACK_RELEASE) begin
            drive_low=0; state=selected?next_state:IGNORE;
            if(state==TX) begin outgoing=read_register(pointer); bits=7; drive_low=!outgoing[7]; end
        end else if(state==TX) drive_low=!outgoing[bits];
        else if(state==TX_RELEASE) begin drive_low=0; state=MASTER_ACK; end
        else if(state==MASTER_ACK_END) begin
            if(master_acked) begin
                pointer=pointer+1'b1; outgoing=read_register(pointer); bits=7;
                drive_low=!outgoing[7]; state=TX;
            end else begin drive_low=0; state=IGNORE; end
        end
    end
endmodule
