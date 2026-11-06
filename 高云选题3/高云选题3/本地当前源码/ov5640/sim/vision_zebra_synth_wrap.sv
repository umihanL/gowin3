`timescale 1ns/1ps
// Boundary wrapper for the 2026-10-29 full-project synthesized netlist.
// Synthesis moved ROI and register feedback gates across module boundaries.
// The LUT INITs below are copied from vision_video in that same netlist;
// n15 feedback implements the extracted portrait_display LUT3 INIT=8'h0E.
// centre is the production constant 52. Inputs are binary dark/light samples;
// RGB565 exposure arithmetic is covered separately by the RTL exposure test.
module vision_zebra(
 input clk,rst_n,frame_begin,frame_complete,sample_valid,dark,
 input [6:0] sample_x,sample_y,centre,
 output update,detected
);
 wire frame_complete_i=frame_begin && frame_complete;
 wire [2:0] strip_have;
 wire strip_have_1,strip_have_0,strip_have_2,longitudinal, longitudinal_15;
 wire n1078_13,n1078_14;
 wire zebra_low_6_11,zebra_low_6_13;
 LUT4 zebra_roi_x(.F(zebra_low_6_11),.I0(sample_x[3]),.I1(sample_x[2]),.I2(sample_x[4]),.I3(sample_x[5]));
 defparam zebra_roi_x.INIT=16'h001F;
 LUT4 zebra_roi(.F(zebra_low_6_13),.I0(n1078_13),.I1(n1078_14),.I2(sample_x[6]),.I3(zebra_low_6_11));
 defparam zebra_roi.INIT=16'hE00E;
 wire n15_13=(!frame_begin) && (strip_have[1] || strip_have_1);
 wire n15_17=(!frame_begin) && (strip_have[0] || strip_have_0);
 wire n15_20=(!frame_begin) && (longitudinal || longitudinal_15);
 wire n15_23=(!frame_begin) && (strip_have[2] || strip_have_2);
 wire [7:7] zebra_luma=dark?1'b0:1'b1;
 wire [7:7] zebra_threshold=1'b1;
 vision_zebra_synth dut(
   .TFT_clk_d(clk), .n564_6(!rst_n), .n1460_11(frame_complete_i),
   .zebra_low_6_15(sample_valid && zebra_low_6_13), .Frame_Begin_Z(frame_begin),
   .capture_low_7_9(sample_valid), .n469_33(dark),
   .zebra_low_6_13(zebra_low_6_13), .n783_8(&sample_y[1:0]),
   .n15_13(n15_13), .n15_17(n15_17), .n15_20(n15_20), .n15_23(n15_23),
   .zebra_luma(zebra_luma), .zebra_threshold(zebra_threshold),
   .sample_x(sample_x), .sample_y(sample_y[6:2]),
   .zebra_update(update), .zebra_detected(detected),
   .n1078_13(n1078_13), .n1078_14(n1078_14), .longitudinal(longitudinal),
   .strip_have_1_16(strip_have_1), .strip_have_0_16(strip_have_0),
   .strip_have_2_14(strip_have_2), .longitudinal_15(longitudinal_15), .strip_have(strip_have)
 );
endmodule
