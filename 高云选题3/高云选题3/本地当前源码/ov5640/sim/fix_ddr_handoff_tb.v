`timescale 1ns/1ps
// Exercise the real PLL_INIT startup FSM and DDR PLL wrapper. The analog PLL
// is replaced by a pin-level model, so this checks ownership and reset logic,
// not board-level DDR calibration or analog lock behavior.
module fix_ddr_handoff_tb;
    reg clk = 0;
    always #10 clk = ~clk;
    reg reset = 1;
    reg [1:0] op = 2'b00;
    reg inc = 0;
    reg [7:0] data = 0;
    wire lock;
    wire [7:0] readback;
    ddr_pll dut (.clkin(clk), .clkout0(), .clkout1(), .clkout2(),
        .lock(lock), .mdopc(op), .mdainc(inc), .mdwdi(data),
        .mdrdo(readback), .pll_init_bypass(1'b0), .mdclk(clk), .reset(reset));

    task check_pass;
        begin
            #2;
            if (dut.u_pll.mdopc !== op || dut.u_pll.mdainc !== inc ||
                dut.u_pll.mdwdi !== data || readback !== 8'ha5)
                $fatal(1, "DDR calibration mDRP requests/readback are blocked");
        end
    endtask

    task check_startup;
        begin
            // Give PLL_INIT several clocks in reset, then release the board.
            repeat(5) @(negedge clk);
            reset=0;
            // Analog PLL may lock during the tuning trials. The wrapper must
            // keep its filtered lock low until the actual startup FSM finishes.
            wait(dut.u_pll.lock === 1'b1);
            #2;
            if (lock !== 0 || readback !== 0)
                $fatal(1,"PLL startup was bypassed before initialization");
            wait(lock === 1'b1);
            repeat(4) @(negedge clk);
            op=2'b10; inc=1; data=8'h3c; check_pass;
            @(negedge clk); op=2'b01; inc=0; data=8'h81; check_pass;
            @(negedge clk); op=0; data=0; check_pass;
        end
    endtask

    initial begin
        check_startup;
        // Stopping/changing a clock may briefly drop LOCK. Ownership must stay
        // with DDR so its restart command can still reach the PLL.
        dut.u_pll.lock_allowed=0;
        repeat(8) @(negedge clk);
        if (lock !== 0) $fatal(1,"raw lock loss not reported");
        op=2'b01; data=8'h42; check_pass;
        dut.u_pll.lock_allowed=1;
        repeat(4) @(negedge clk); check_pass;
        reset=1; op=0; data=0;
        check_startup;
        $display("PASS: PLL startup, DDR mDRP handoff/readback, lock loss, board reset and reinitialization");
        $finish;
    end
    initial begin #500000; $fatal(1,"PLL initialization/handoff timeout"); end
endmodule

module ddr_pll_MOD (lock, clkout0, clkout1, clkout2, mdrdo,
                    clkin, reset, mdclk, mdopc, mdainc, mdwdi);
    output lock, clkout0, clkout1, clkout2;
    output [7:0] mdrdo;
    input clkin, reset, mdclk, mdainc;
    input [1:0] mdopc;
    input [7:0] mdwdi;
    reg lock_allowed=1;
    assign lock = !reset && lock_allowed;
    assign mdrdo = 8'ha5;
    assign clkout0=clkin;
    assign clkout1=clkin;
    assign clkout2=clkin;
endmodule
