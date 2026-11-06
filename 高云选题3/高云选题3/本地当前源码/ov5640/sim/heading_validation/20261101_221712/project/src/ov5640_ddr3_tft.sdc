# Explicit clocks for the system, display, camera and DDR PHY.
create_clock -name clk50m -period 20 [get_ports {clk50m}]
# Conservative 100 MHz limit for the external OV5640 pixel clock.
create_clock -name camera_pclk -period 10 [get_ports {camera_pclk}]
create_generated_clock -name sys50 -source [get_ports {clk50m}] -divide_by 1 [get_pins {Gowin_PLL/u_pll/PLLA_inst/CLKOUT0}]
create_generated_clock -name pixel33 -source [get_ports {clk50m}] -multiply_by 33 -divide_by 50 [get_pins {Gowin_PLL/u_pll/PLLA_inst/CLKOUT2}]
create_generated_clock -name camera25 -source [get_ports {clk50m}] -divide_by 2 [get_pins {Gowin_PLL/u_pll/PLLA_inst/CLKOUT3}]
create_generated_clock -name ddr400 -source [get_ports {clk50m}] -multiply_by 8 [get_pins {ddr_pll/u_pll/PLLA_inst/CLKOUT2}]
create_generated_clock -name ddr_ui100 -source [get_pins {ddr_pll/u_pll/PLLA_inst/CLKOUT2}] -divide_by 4 [get_pins {ddr3_ctrl_2port/DDR3_Memory_Interface_Top/gw3_top/u_ddr_phy_top/fclkdiv/CLKOUT}]
# Data crosses these boundaries through the vendor asynchronous FIFOs;
# status/frame controls use synchronizers.
# The board's chapter 36 reference (project/src/ddr3_1v4_hs.sdc) also
# separates the PHY 400 MHz and user 100 MHz clocks for this DDR IP.
# Although physically divided, their DLL/DQS calibration control paths
# use the IP's calibration protocol instead of cycle-to-cycle STA.
set_clock_groups -asynchronous -group [get_clocks {clk50m sys50 pixel33 camera25}] -group [get_clocks {ddr400}] -group [get_clocks {ddr_ui100}] -group [get_clocks {camera_pclk}]
# PLL lock asynchronously asserts the reset synchronizer. Only its CLEAR
# pins are asynchronous; all three data stages and the released reset remain
# timed in the display domain.
set_false_path -to [get_pins {u_disp_reset/reset_pipe_0_s0/CLEAR u_disp_reset/reset_pipe_1_s0/CLEAR u_disp_reset/reset_pipe_2_s0/CLEAR}]

# Imported MOTOR PI holds captured operands and PI state for four clocks.
# Match only the two PI state banks; enables, PWM and other control remain timed.
set_multicycle_path -from [get_regs {u_vehicle/motor1/pi_target_counts* u_vehicle/motor1/pi_measured_counts* u_vehicle/motor1/u_pi/* u_vehicle/motor2/pi_target_counts* u_vehicle/motor2/pi_measured_counts* u_vehicle/motor2/u_pi/*}] -to [get_regs {u_vehicle/motor1/u_pi/* u_vehicle/motor2/u_pi/*}] -setup -end 4
set_multicycle_path -from [get_regs {u_vehicle/motor1/pi_target_counts* u_vehicle/motor1/pi_measured_counts* u_vehicle/motor1/u_pi/* u_vehicle/motor2/pi_target_counts* u_vehicle/motor2/pi_measured_counts* u_vehicle/motor2/u_pi/*}] -to [get_regs {u_vehicle/motor1/u_pi/* u_vehicle/motor2/u_pi/*}] -hold -end 3
# Board inputs are asynchronous to the FPGA. Internal synchronizer stages remain timed.
set_false_path -from [get_ports {encoder_A encoder_B encoder2_A encoder2_B ps2_miso host_rx}]
