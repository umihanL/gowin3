// LSM6DSV16X primary SPI: mode 3, MSB first. One command, 1..12 data bytes.
// rx_data[7:0] is the FIRST received byte (register address); CS stays low
// throughout a burst. HALF_PERIOD=17 at 33 MHz gives 970.6 kHz.
module spi_register #(parameter HALF_PERIOD=17)(
    input clk, rst_n, start, input [7:0] command, write_data,
    input [3:0] byte_count, input miso,
    output reg sclk, mosi, cs_n, busy, done,
    output reg [95:0] rx_data
);
    reg [15:0] divider;
    reg [7:0] tx_shift, rx_shift;
    reg [3:0] byte_index, count;
    reg [2:0] bit_index;
    reg ending;
    reg miso_meta, miso_sync;
    always @(posedge clk) begin miso_meta<=miso; miso_sync<=miso_meta; end
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            divider<=0; tx_shift<=0; rx_shift<=0; byte_index<=0;
            count<=0; bit_index<=7; ending<=0;
            sclk<=1; mosi<=0; cs_n<=1; busy<=0; done<=0; rx_data<=0;
        end else begin
            done<=0;
            if (!busy) begin
                if (start) begin
                    busy<=1; cs_n<=0; sclk<=1; divider<=0; ending<=0;
                    tx_shift<=command; count<=byte_count; byte_index<=0;
                    bit_index<=7; rx_shift<=0; rx_data<=0;
                end
            end else if (divider==HALF_PERIOD-1) begin
                divider<=0;
                if (ending) begin
                    cs_n<=1; busy<=0; done<=1; ending<=0;
                end else if (sclk) begin
                    sclk<=0; mosi<=tx_shift[7]; tx_shift<={tx_shift[6:0],1'b0};
                end else begin
                    sclk<=1; rx_shift<={rx_shift[6:0],miso_sync};
                    if (bit_index==0) begin
                        bit_index<=7;
                        if (byte_index!=0)
                            rx_data[(byte_index-1)*8 +: 8]<={rx_shift[6:0],miso_sync};
                        if (byte_index==count) ending<=1;
                        else begin
                            byte_index<=byte_index+1'b1;
                            tx_shift<=write_data;
                        end
                    end else bit_index<=bit_index-1'b1;
                end
            end else divider<=divider+1'b1;
        end
    end
endmodule
