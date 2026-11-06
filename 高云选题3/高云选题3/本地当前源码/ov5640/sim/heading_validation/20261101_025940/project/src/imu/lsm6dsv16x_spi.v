// DS13510 rev4: primary 4-wire SPI, WHO_AM_I=70h.
// SFLP fuses accel + gyro internally, including continuous gyro-bias estimation.
// FIFO tag 13h contains quaternion X,Y,Z as IEEE754 binary16, little endian.
module lsm6dsv16x_spi #(
    parameter CLK_HZ=33000000, SPI_HALF=66,
    parameter BOOT_CYCLES=CLK_HZ/10, POLL_CYCLES=CLK_HZ/1000,
    parameter TIMEOUT_CYCLES=CLK_HZ*2, STARTUP_CYCLES=CLK_HZ*5,
    parameter CONFIG_WAIT_CYCLES=CLK_HZ/10000,
    parameter BANK_WAIT_CYCLES=CLK_HZ/1000,
    parameter TRANSACTION_TIMEOUT_CYCLES=CLK_HZ/100+SPI_HALF*16*14
)(
    input clk, rst_n, input miso, output sclk, mosi, cs_n,
    output reg online, sample_valid,
    // Pulse only when the embedded fusion engine is actually reset.
    output reg fusion_reset,
    // 0 initializing, 1 awaiting data, 2 streaming, 3 ID, 4 config, 5 timeout, 6 FIFO
    output reg [2:0] status,
    // Latched together on a failed verification; retained throughout retry.
    output reg [7:0] error_reg,error_expected,error_actual,error_mask,error_step,
    output reg [15:0] qx_half, qy_half, qz_half,
    output reg signed [15:0] gx,gy,gz,ax,ay,az
);
    reg start;
    reg [7:0] command, write_data;
    reg [3:0] byte_count;
    wire busy,done; wire [95:0] rx;
    spi_register #(.HALF_PERIOD(SPI_HALF)) u_spi(
        .clk(clk),.rst_n(rst_n && !spi_abort),.start(start),.command(command),.write_data(write_data),
        .byte_count(byte_count),.miso(miso),.sclk(sclk),.mosi(mosi),.cs_n(cs_n),
        .busy(busy),.done(done),.rx_data(rx));
    reg [5:0] state, config_index;
    reg [31:0] timer, watchdog;
    reg configured,have_sample;
    reg [1:0] stream_retries;
    reg [1:0] recovery_attempts;
    reg spi_abort;
    localparam TRANSACTION_W=$clog2(TRANSACTION_TIMEOUT_CYCLES+1);
    reg [TRANSACTION_W-1:0] transaction_age;
    wire [31:0] timeout_limit=have_sample ? TIMEOUT_CYCLES : STARTUP_CYCLES;
    reg [15:0] hx,hy,hz;
    reg [15:0] config_word;
    reg [7:0] config_mask;
    reg [1:0] verify_retries;
    reg [1:0] bank_retries;
    reg [7:0] bank_write_value, bank_echo;
    wire bank_step=(config_word[15:8]==8'h01);
    wire bank_embedded=config_word[7];
    // After POR SFLP_ODR is 120 Hz (bits 5:3 = 011). On the second
    // visit it must retain the 60 Hz setting written at step 8.
    wire [7:0] bank_odr_expected=(config_index==7) ? 8'h18 : 8'h10;
    always @* begin
        case(config_index)
            0:config_word=16'h0301; // four-wire SPI; disable I2C/I3C
            1:config_word=16'h1244; // BDU + address increment
            2:config_word=16'h1504; // +/-2000 dps
            3:config_word=16'h1701; // +/-4 g
            4:config_word=16'h0a00; // flush FIFO before starting
            5:config_word=16'h1005; // accel 60 Hz, high performance
            6:config_word=16'h1105; // gyro 60 Hz, high performance
            7:config_word=16'h0180; // embedded register bank
            8:config_word=16'h5e53; // SFLP 60 Hz, matched to both sensors
            9:config_word=16'h4402; // batch quaternion
            10:config_word=16'h0100; // user bank
            11:config_word=16'h0180;
            12:config_word=16'h0402; // enable SFLP AFTER sensors are running
            13:config_word=16'h0100;
            14:config_word=16'h0a06; // continuous FIFO
            default:config_word=0;
        endcase
        // Compare only persistent fields required by this application.
        // Values written above still include the datasheet's required reserved bits.
        case(config_index)
            0:config_mask=8'h05; // IF_CFG: SIM, I2C_I3C_disable
            1:config_mask=8'h44; // CTRL3: BDU, IF_INC (not reset/boot)
            2:config_mask=8'h0f; // CTRL6: gyro full scale
            3:config_mask=8'h0b; // CTRL8: accel full scale, dual channel
            4,14:config_mask=8'h07; // FIFO mode
            5,6:config_mask=8'h7f; // ODR and operating mode
            7,10,11,13:config_mask=8'hc0; // bank select
            8:config_mask=8'h38; // SFLP ODR bits 5:3
            9:config_mask=8'hf2; // embedded FIFO enables
            12:config_mask=8'hba; // embedded algorithm enables
            default:config_mask=8'hff;
        endcase
    end
    // SPI launches are separated from completion by a whole state.
    task transaction;
        input [7:0] cmd, data; input [3:0] n; input [5:0] next_state;
        begin command<=cmd; write_data<=data; byte_count<=n; start<=1; state<=next_state; end
    endtask
    task config_advance;
        begin
            if(config_index==14) state<=12;
            else if(config_index==6) begin state<=23;timer<=0;end
            else begin config_index<=config_index+1'b1;state<=10;end
        end
    endtask
    task bank_failed;
        input [7:0] probe_reg,probe_expected,probe_actual,probe_mask;
        begin
            if(bank_retries!=2) begin
                // FUNC_CFG_ACCESS is accessible from either bank. Reissue
                // only this bank selection, without resetting the sensor.
                bank_retries<=bank_retries+1'b1;state<=25;
            end else begin
                online<=0;configured<=0;status<=4;state<=0;timer<=0;
                error_step<={2'd0,config_index};
                if((bank_echo&8'hc0)!=(config_word[7:0]&8'hc0)) begin
                    error_reg<=8'h01;error_expected<=config_word[7:0];
                    error_actual<=bank_echo;error_mask<=8'hc0;
                end else begin
                    error_reg<=probe_reg;error_expected<=probe_expected;
                    error_actual<=probe_actual;error_mask<=probe_mask;
                end
            end
        end
    endtask
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state<=0; timer<=0; watchdog<=0; config_index<=0; start<=0;
            command<=0; write_data<=0; byte_count<=1; online<=0; sample_valid<=0;
            configured<=0;have_sample<=0;status<=0;fusion_reset<=0;stream_retries<=0;
            recovery_attempts<=0;spi_abort<=0;transaction_age<=0;
            error_reg<=0;error_expected<=0;error_actual<=0;error_mask<=0;error_step<=0;verify_retries<=0;
            bank_retries<=0;bank_write_value<=0;bank_echo<=0;
            hx<=0;hy<=0;hz<=0;qx_half<=0;qy_half<=0;qz_half<=0;
            gx<=0;gy<=0;gz<=0;ax<=0;ay<=0;az<=0;
        end else begin
            start<=0; sample_valid<=0;fusion_reset<=0;spi_abort<=0;
            if(busy)begin
                if(transaction_age<TRANSACTION_TIMEOUT_CYCLES)transaction_age<=transaction_age+1'b1;
            end else transaction_age<=0;
            if (configured) begin
                if(watchdog<timeout_limit) watchdog<=watchdog+1'b1;
            end else watchdog<=0;
            case(state)
                0: if(timer==BOOT_CYCLES-1) begin timer<=0;state<=1;end else timer<=timer+1'b1;
                1: transaction(8'h01,8'h00,1,2); // recover bank after FPGA-only reset
                2: if(done) state<=3;
                3: transaction(8'h8f,0,1,4);
                4: if(done) begin
                    if(rx[7:0]==8'h70) state<=5; else begin state<=0;timer<=0;status<=3;end
                end
                5: begin
                    transaction(8'h01,8'h04,1,6); // Boot/configuration recovery only, before first stream.
                    fusion_reset<=1;stream_retries<=0;
                end
                6: if(done) begin state<=7;timer<=0;end
                7: if(timer==BOOT_CYCLES-1) begin state<=8;timer<=0;end else timer<=timer+1'b1;
                8: transaction(8'h8f,0,1,9);
                9: if(done) begin
                    if(rx[7:0]!=8'h70) begin state<=0;timer<=0;status<=3;end
                    else begin state<=10;config_index<=0;end
                end
                10:if(bank_step) begin bank_retries<=0;state<=25;end
                   else transaction(config_word[15:8],config_word[7:0],1,11);
                11:if(done) begin state<=24;timer<=0;verify_retries<=0;end
                24:if(timer==CONFIG_WAIT_CYCLES-1) begin state<=21;timer<=0;end
                   else timer<=timer+1'b1;
                21:transaction(config_word[15:8]|8'h80,0,1,22);
                22:if(done) begin // bank selections use the separate checks below
                    if((rx[7:0]&config_mask)!=(config_word[7:0]&config_mask)) begin
                        if(verify_retries!=2) begin
                            // Re-read after settling without rewriting an uncertain bank.
                            verify_retries<=verify_retries+1'b1;state<=24;timer<=0;
                        end else begin
                            online<=0;configured<=0;status<=4;state<=0;timer<=0;
                            error_reg<=config_word[15:8];error_expected<=config_word[7:0];
                            error_actual<=rx[7:0];error_mask<=config_mask;error_step<={2'd0,config_index};
                        end
                    end else config_advance;
                end
                // Match ST's read/modify/write bank selection, preserving
                // unrelated persistent control bits, never replaying SW_POR.
                25:transaction(8'h81,0,1,26);
                26:if(done) begin
                    bank_write_value<=(rx[7:0]&8'h0b)|(config_word[7:0]&8'hc0);
                    state<=27;
                end
                27:transaction(8'h01,bank_write_value,1,28);
                28:if(done) begin state<=29;timer<=0;end
                29:if(timer==BANK_WAIT_CYCLES-1) begin state<=30;timer<=0;end
                   else timer<=timer+1'b1;
                30:transaction(8'h81,0,1,31);
                31:if(done) begin bank_echo<=rx[7:0];state<=32;end
                // Check the actual register map, not only the bank-select
                // echo. A bad echo cannot bypass either page-specific check.
                // These probes use defined fields, not reserved-bit values.
                32:if(bank_embedded) transaction(8'hde,0,1,33);
                   else transaction(8'h8f,0,1,33);
                33:if(done) begin
                    if(bank_embedded && ((rx[7:0]&8'h38)!=bank_odr_expected))
                        bank_failed(8'h5e,bank_odr_expected,rx[7:0],8'h38);
                    else if(!bank_embedded && rx[7:0]!=8'h70)
                        bank_failed(8'h0f,8'h70,rx[7:0],8'hff);
                    else state<=34;
                end
                // 0F is FSM_INT2 in embedded bank (we leave every route off),
                // but WHO_AM_I=70 in main bank. Main CTRL3 retains BDU/IF_INC.
                34:if(bank_embedded) transaction(8'h8f,0,1,35);
                   else transaction(8'h92,0,1,35);
                35:if(done) begin
                    if(bank_embedded && rx[7:0]!=0)
                        bank_failed(8'h0f,8'h00,rx[7:0],8'hff);
                    else if(!bank_embedded && ((rx[7:0]&8'h44)!=8'h44))
                        bank_failed(8'h12,8'h44,rx[7:0],8'h44);
                    else config_advance;
                end
                23:if(timer==BOOT_CYCLES-1) begin
                    // Settle both sensors BEFORE the first embedded access.
                    config_index<=7;state<=10;timer<=0;
                end else timer<=timer+1'b1;
                12:transaction(8'h8f,0,1,13);
                13:if(done) begin
                    if(rx[7:0]==8'h70) begin
                        configured<=1;have_sample<=0;online<=0;
                        if(status<3)status<=1;
                        watchdog<=0;state<=14;timer<=0;
                    end else begin online<=0;configured<=0;status<=3;state<=0;timer<=0;end
                end
                14:if(timer==POLL_CYCLES-1) begin timer<=0;state<=15;end else timer<=timer+1'b1;
                15:transaction(8'h9b,0,2,16); // FIFO_STATUS1/2, count[8:0]
                16:if(done) begin
                    if(rx[15:0]==16'hffff) begin
                        // A short burst of bad SPI words is not a fusion reset.
                        // Retry with poll spacing until the bounded no-sample
                        // watchdog expires; then recover the bank/FIFO locally.
                        if(stream_retries<3)stream_retries<=stream_retries+1'b1;
                        state<=14;timer<=0;
                    end
                    else if(rx[14] || rx[13] || rx[11]) begin
                        // FIFO recovery must not reset the SFLP orientation.
                        status<=6;state<=36;stream_retries<=0;
                    end
                    else if(rx[8:0]!=0)begin state<=17;stream_retries<=0;end
                    else begin state<=14;stream_retries<=0;end
                end
                17:transaction(8'hf8,0,7,18); // tag + XYZ, single CS assertion
                18:if(done) begin
                    if(rx[7:3]==5'h13 && rx[22:18]<16 && rx[38:34]<16 && rx[54:50]<16) begin
                        hx<=rx[23:8];hy<=rx[39:24];hz<=rx[55:40];state<=19;
                    end else state<=15; // consume other FIFO tags
                end
                19:transaction(8'ha2,0,12,20); // raw gyro+accel, stationarity only
                20:if(done) begin
                    gx<=rx[15:0];gy<=rx[31:16];gz<=rx[47:32];
                    ax<=rx[63:48];ay<=rx[79:64];az<=rx[95:80];
                    qx_half<=hx;qy_half<=hy;qz_half<=hz;sample_valid<=1;
                    online<=1;have_sample<=1;status<=2;stream_retries<=0;
                    watchdog<=0;recovery_attempts<=0;state<=15; // drain backlog before next poll delay
                end
                36:transaction(8'h0a,8'h00,1,37);
                37:if(done)state<=38;
                38:transaction(8'h0a,8'h06,1,39);
                39:if(done)begin state<=14;timer<=0;end
                // Once streaming has succeeded, a transport gap must not reset
                // SFLP and change its coordinate system. Recover only bank/FIFO.
                40:transaction(8'h01,8'h00,1,41);
                41:if(done)state<=42;
                42:transaction(8'h8f,0,1,43);
                43:if(done)begin
                    if(rx[7:0]==8'h70)state<=36;
                    else begin status<=3;state<=14;timer<=0;end
                end
                default:state<=0;
            endcase
            if(configured && watchdog>=timeout_limit-1 && !busy && !start) begin
                online<=0;status<=5;timer<=0;watchdog<=0;start<=0;
                if(have_sample && recovery_attempts<2)begin
                    recovery_attempts<=recovery_attempts+1'b1;state<=40;
                end else begin
                    // Two bounded local bank/FIFO recoveries failed. A dead
                    // SFLP/configuration requires the same verified complete
                    // initialization as board reset, without resetting the FPGA
                    // or requesting a new user zero. fusion_reset marks the
                    // sensor coordinate change for the attitude bridge.
                    configured<=0;have_sample<=0;state<=0;
                end
            end
            if(busy && transaction_age>=TRANSACTION_TIMEOUT_CYCLES-1)begin
                spi_abort<=1;transaction_age<=0;start<=0;configured<=0;
                have_sample<=0;online<=0;status<=5;state<=0;timer<=0;watchdog<=0;
            end
        end
    end
endmodule
