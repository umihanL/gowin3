module vision_zebra (
  TFT_clk_d,
  n564_6,
  n1460_11,
  zebra_low_6_15,
  Frame_Begin_Z,
  capture_low_7_9,
  n469_33,
  zebra_low_6_13,
  n783_8,
  n15_13,
  n15_17,
  n15_20,
  n15_23,
  zebra_luma,
  zebra_threshold,
  sample_x,
  sample_y,
  zebra_update,
  zebra_detected,
  n1078_13,
  n1078_14,
  longitudinal,
  strip_have_1_16,
  strip_have_0_16,
  strip_have_2_14,
  longitudinal_15,
  strip_have
)
;
input TFT_clk_d;
input n564_6;
input n1460_11;
input zebra_low_6_15;
input Frame_Begin_Z;
input capture_low_7_9;
input n469_33;
input zebra_low_6_13;
input n783_8;
input n15_13;
input n15_17;
input n15_20;
input n15_23;
input [7:7] zebra_luma;
input [7:7] zebra_threshold;
input [6:0] sample_x;
input [6:2] sample_y;
output zebra_update;
output zebra_detected;
output n1078_13;
output n1078_14;
output longitudinal;
output strip_have_1_16;
output strip_have_0_16;
output strip_have_2_14;
output longitudinal_15;
output [2:0] strip_have;
wire n87_3;
wire across_kind_8;
wire \strip_gaps[0]_3_8 ;
wire \strip_gaps[1]_3_8 ;
wire \strip_gaps[2]_3_8 ;
wire n1032_8;
wire n1031_7;
wire n1030_7;
wire n1029_7;
wire n1028_8;
wire n1027_7;
wire n1026_7;
wire n1025_7;
wire n1024_8;
wire n1023_7;
wire n1022_7;
wire n1021_7;
wire n1019_7;
wire n1018_7;
wire n1017_7;
wire n1015_7;
wire n1014_7;
wire n1013_7;
wire n1011_7;
wire n1010_7;
wire n1009_7;
wire n1038_8;
wire n1037_7;
wire n1036_7;
wire n1035_7;
wire n1034_7;
wire n990_7;
wire n989_7;
wire n988_7;
wire n1144_7;
wire n1143_7;
wire n1142_7;
wire n1141_7;
wire n1140_7;
wire n1139_7;
wire n1138_7;
wire n1137_7;
wire n1136_7;
wire n1135_7;
wire n1134_7;
wire n1133_7;
wire n1132_7;
wire n1131_7;
wire n1130_7;
wire n1129_7;
wire n1128_7;
wire n1127_7;
wire n1126_7;
wire n1125_7;
wire n1124_7;
wire n1123_7;
wire n1122_7;
wire n1121_7;
wire n1120_7;
wire n1119_7;
wire n1118_7;
wire n1117_7;
wire n1116_7;
wire n1115_7;
wire n1114_7;
wire n1113_7;
wire n1112_7;
wire n1078_7;
wire n1077_7;
wire n1076_7;
wire n1075_7;
wire n1068_7;
wire n1067_7;
wire n1066_7;
wire n1065_7;
wire n1064_7;
wire n1063_7;
wire n1062_7;
wire n1059_7;
wire n1058_7;
wire n1057_7;
wire n1056_7;
wire n1055_7;
wire n1054_7;
wire n1053_7;
wire n1052_7;
wire n87_4;
wire n87_5;
wire n87_6;
wire n87_7;
wire strip_have_1_9;
wire strip_have_0_9;
wire strip_kind_2_10;
wire across_kind_9;
wire longitudinal_rows_5_9;
wire longitudinal_rows_5_10;
wire longitudinal_9;
wire \strip_bands[0]_3_10 ;
wire \strip_bands[1]_3_10 ;
wire \strip_bands[2]_3_10 ;
wire \strip_gaps[0]_3_9 ;
wire \strip_gaps[0]_3_10 ;
wire \strip_gaps[1]_3_9 ;
wire \strip_gaps[1]_3_10 ;
wire \strip_gaps[2]_3_9 ;
wire \strip_gaps[2]_3_10 ;
wire n1029_8;
wire n1025_8;
wire n1021_8;
wire n1017_8;
wire n1013_8;
wire n1009_8;
wire n1035_8;
wire n937_9;
wire n990_8;
wire n989_8;
wire n988_8;
wire n1143_9;
wire n1141_8;
wire n1140_8;
wire n1139_8;
wire n1137_9;
wire n1135_8;
wire n1134_8;
wire n1133_8;
wire n1131_9;
wire n1129_8;
wire n1128_8;
wire n1127_8;
wire n1126_8;
wire n1125_8;
wire n1124_8;
wire n1123_8;
wire n1122_8;
wire n1121_8;
wire n1120_8;
wire n1119_8;
wire n1118_8;
wire n1117_8;
wire n1116_8;
wire n1115_8;
wire n1114_8;
wire n1113_8;
wire n1112_8;
wire n1078_8;
wire n1077_8;
wire n1076_8;
wire n1076_9;
wire n1075_8;
wire n1075_9;
wire n1068_8;
wire n1067_8;
wire n1066_8;
wire n1065_8;
wire n1065_9;
wire n1064_8;
wire n1063_8;
wire n1062_8;
wire n1061_8;
wire n1058_8;
wire n1057_8;
wire n1056_8;
wire n1055_8;
wire n1054_8;
wire n1053_8;
wire n1053_9;
wire n1052_8;
wire n1052_9;
wire n87_9;
wire n87_11;
wire n87_13;
wire n87_14;
wire strip_have_1_10;
wire strip_have_0_10;
wire strip_kind_2_12;
wire \strip_bands[0]_3_11 ;
wire \strip_bands[1]_3_11 ;
wire \strip_bands[2]_3_11 ;
wire \strip_gaps[0]_3_11 ;
wire \strip_gaps[0]_3_12 ;
wire \strip_gaps[1]_3_11 ;
wire \strip_gaps[1]_3_12 ;
wire \strip_gaps[2]_3_11 ;
wire \strip_gaps[2]_3_12 ;
wire n990_9;
wire n989_9;
wire n988_9;
wire n1143_10;
wire n1141_9;
wire n1140_10;
wire n1137_10;
wire n1135_9;
wire n1134_10;
wire n1131_10;
wire n1129_9;
wire n1128_10;
wire n1126_9;
wire n1125_9;
wire n1124_9;
wire n1123_9;
wire n1122_9;
wire n1121_9;
wire n1120_9;
wire n1117_9;
wire n1116_9;
wire n1114_9;
wire n1113_9;
wire n1112_9;
wire n1076_10;
wire n1075_10;
wire n1068_9;
wire n1066_9;
wire n1064_9;
wire n1063_9;
wire n1062_9;
wire n1057_9;
wire n1056_9;
wire n1055_9;
wire n1055_10;
wire n1054_9;
wire n1053_10;
wire n1053_11;
wire n1126_11;
wire n1126_12;
wire n1121_10;
wire n1121_11;
wire n1121_12;
wire n1116_10;
wire n1075_11;
wire n1075_12;
wire n1068_10;
wire n1068_11;
wire n1064_10;
wire n1056_10;
wire n1053_12;
wire n1121_13;
wire n1116_11;
wire n1078_15;
wire n1075_13;
wire n1068_12;
wire n1068_13;
wire n1068_14;
wire n1078_17;
wire n1034_10;
wire n1068_17;
wire n1119_11;
wire n1126_16;
wire n87_16;
wire n87_18;
wire n87_20;
wire \strip_bands[0]_3_13 ;
wire strip_kind_0_12;
wire \strip_bands[1]_3_13 ;
wire strip_kind_1_12;
wire \strip_bands[2]_3_13 ;
wire strip_kind_2_14;
wire n1054_12;
wire n1061_10;
wire n1067_11;
wire n1126_18;
wire n1078_19;
wire n1058_12;
wire n1132_10;
wire n1138_10;
wire n1144_10;
wire n937_11;
wire n1058_14;
wire n1128_12;
wire n1131_12;
wire strip_kind_0_14;
wire n1134_12;
wire n1137_12;
wire strip_kind_1_14;
wire n1140_12;
wire n1143_12;
wire strip_kind_2_16;
wire n1033_10;
wire longitudinal_rows_5_14;
wire \strip_bands[0]_3_15 ;
wire \strip_bands[1]_3_15 ;
wire \strip_bands[2]_3_15 ;
wire n1012_15;
wire n1016_15;
wire n1020_15;
wire n1033_14;
wire n1115_11;
wire n1118_11;
wire n1126_20;
wire across_have;
wire across_kind;
wire [6:0] across_run;
wire [3:0] across_bands;
wire [3:0] across_gaps;
wire [3:0] longitudinal_run;
wire [4:0] \row_white[0] ;
wire [4:0] \row_white[1] ;
wire [4:0] \row_white[2] ;
wire [5:0] \strip_run[0] ;
wire [5:0] \strip_run[1] ;
wire [5:0] \strip_run[2] ;
wire [2:0] strip_kind;
wire [5:0] longitudinal_rows;
wire [3:0] \strip_bands[0] ;
wire [3:0] \strip_bands[1] ;
wire [3:0] \strip_bands[2] ;
wire [3:0] \strip_gaps[0] ;
wire [3:0] \strip_gaps[1] ;
wire [3:0] \strip_gaps[2] ;
wire VCC;
wire GND;
  LUT4 n87_s0 (
    .F(n87_3),
    .I0(n87_4),
    .I1(n87_5),
    .I2(n87_6),
    .I3(n87_7) 
);
defparam n87_s0.INIT=16'hFFE8;
  LUT3 across_kind_s3 (
    .F(across_kind_8),
    .I0(across_kind_9),
    .I1(zebra_low_6_15),
    .I2(Frame_Begin_Z) 
);
defparam across_kind_s3.INIT=8'hF4;
  LUT4 \strip_gaps[0]_3_s3  (
    .F(\strip_gaps[0]_3_8 ),
    .I0(\strip_gaps[0]_3_9 ),
    .I1(\strip_bands[0]_3_13 ),
    .I2(\strip_gaps[0]_3_10 ),
    .I3(Frame_Begin_Z) 
);
defparam \strip_gaps[0]_3_s3 .INIT=16'hFF80;
  LUT4 \strip_gaps[1]_3_s3  (
    .F(\strip_gaps[1]_3_8 ),
    .I0(\strip_gaps[1]_3_9 ),
    .I1(\strip_bands[1]_3_13 ),
    .I2(\strip_gaps[1]_3_10 ),
    .I3(Frame_Begin_Z) 
);
defparam \strip_gaps[1]_3_s3 .INIT=16'hFF80;
  LUT4 \strip_gaps[2]_3_s3  (
    .F(\strip_gaps[2]_3_8 ),
    .I0(\strip_gaps[2]_3_9 ),
    .I1(\strip_bands[2]_3_13 ),
    .I2(\strip_gaps[2]_3_10 ),
    .I3(Frame_Begin_Z) 
);
defparam \strip_gaps[2]_3_s3 .INIT=16'hFF80;
  LUT2 n1032_s3 (
    .F(n1032_8),
    .I0(Frame_Begin_Z),
    .I1(\strip_gaps[2] [0]) 
);
defparam n1032_s3.INIT=4'h1;
  LUT3 n1031_s2 (
    .F(n1031_7),
    .I0(Frame_Begin_Z),
    .I1(\strip_gaps[2] [1]),
    .I2(\strip_gaps[2] [0]) 
);
defparam n1031_s2.INIT=8'h14;
  LUT4 n1030_s2 (
    .F(n1030_7),
    .I0(\strip_gaps[2] [1]),
    .I1(\strip_gaps[2] [0]),
    .I2(Frame_Begin_Z),
    .I3(\strip_gaps[2] [2]) 
);
defparam n1030_s2.INIT=16'h0708;
  LUT3 n1029_s2 (
    .F(n1029_7),
    .I0(Frame_Begin_Z),
    .I1(\strip_gaps[2] [3]),
    .I2(n1029_8) 
);
defparam n1029_s2.INIT=8'h14;
  LUT2 n1028_s3 (
    .F(n1028_8),
    .I0(Frame_Begin_Z),
    .I1(\strip_gaps[1] [0]) 
);
defparam n1028_s3.INIT=4'h1;
  LUT3 n1027_s2 (
    .F(n1027_7),
    .I0(Frame_Begin_Z),
    .I1(\strip_gaps[1] [1]),
    .I2(\strip_gaps[1] [0]) 
);
defparam n1027_s2.INIT=8'h14;
  LUT4 n1026_s2 (
    .F(n1026_7),
    .I0(\strip_gaps[1] [1]),
    .I1(\strip_gaps[1] [0]),
    .I2(Frame_Begin_Z),
    .I3(\strip_gaps[1] [2]) 
);
defparam n1026_s2.INIT=16'h0708;
  LUT3 n1025_s2 (
    .F(n1025_7),
    .I0(Frame_Begin_Z),
    .I1(\strip_gaps[1] [3]),
    .I2(n1025_8) 
);
defparam n1025_s2.INIT=8'h14;
  LUT2 n1024_s3 (
    .F(n1024_8),
    .I0(Frame_Begin_Z),
    .I1(\strip_gaps[0] [0]) 
);
defparam n1024_s3.INIT=4'h1;
  LUT3 n1023_s2 (
    .F(n1023_7),
    .I0(Frame_Begin_Z),
    .I1(\strip_gaps[0] [1]),
    .I2(\strip_gaps[0] [0]) 
);
defparam n1023_s2.INIT=8'h14;
  LUT4 n1022_s2 (
    .F(n1022_7),
    .I0(\strip_gaps[0] [1]),
    .I1(\strip_gaps[0] [0]),
    .I2(Frame_Begin_Z),
    .I3(\strip_gaps[0] [2]) 
);
defparam n1022_s2.INIT=16'h0708;
  LUT3 n1021_s2 (
    .F(n1021_7),
    .I0(Frame_Begin_Z),
    .I1(\strip_gaps[0] [3]),
    .I2(n1021_8) 
);
defparam n1021_s2.INIT=8'h14;
  LUT3 n1019_s2 (
    .F(n1019_7),
    .I0(Frame_Begin_Z),
    .I1(\strip_bands[2] [0]),
    .I2(\strip_bands[2] [1]) 
);
defparam n1019_s2.INIT=8'h14;
  LUT4 n1018_s2 (
    .F(n1018_7),
    .I0(\strip_bands[2] [0]),
    .I1(\strip_bands[2] [1]),
    .I2(Frame_Begin_Z),
    .I3(\strip_bands[2] [2]) 
);
defparam n1018_s2.INIT=16'h0708;
  LUT3 n1017_s2 (
    .F(n1017_7),
    .I0(Frame_Begin_Z),
    .I1(\strip_bands[2] [3]),
    .I2(n1017_8) 
);
defparam n1017_s2.INIT=8'h14;
  LUT3 n1015_s2 (
    .F(n1015_7),
    .I0(Frame_Begin_Z),
    .I1(\strip_bands[1] [0]),
    .I2(\strip_bands[1] [1]) 
);
defparam n1015_s2.INIT=8'h14;
  LUT4 n1014_s2 (
    .F(n1014_7),
    .I0(\strip_bands[1] [0]),
    .I1(\strip_bands[1] [1]),
    .I2(Frame_Begin_Z),
    .I3(\strip_bands[1] [2]) 
);
defparam n1014_s2.INIT=16'h0708;
  LUT3 n1013_s2 (
    .F(n1013_7),
    .I0(Frame_Begin_Z),
    .I1(\strip_bands[1] [3]),
    .I2(n1013_8) 
);
defparam n1013_s2.INIT=8'h14;
  LUT3 n1011_s2 (
    .F(n1011_7),
    .I0(Frame_Begin_Z),
    .I1(\strip_bands[0] [0]),
    .I2(\strip_bands[0] [1]) 
);
defparam n1011_s2.INIT=8'h14;
  LUT4 n1010_s2 (
    .F(n1010_7),
    .I0(\strip_bands[0] [0]),
    .I1(\strip_bands[0] [1]),
    .I2(Frame_Begin_Z),
    .I3(\strip_bands[0] [2]) 
);
defparam n1010_s2.INIT=16'h0708;
  LUT3 n1009_s2 (
    .F(n1009_7),
    .I0(Frame_Begin_Z),
    .I1(\strip_bands[0] [3]),
    .I2(n1009_8) 
);
defparam n1009_s2.INIT=8'h14;
  LUT2 n1038_s3 (
    .F(n1038_8),
    .I0(Frame_Begin_Z),
    .I1(longitudinal_rows[0]) 
);
defparam n1038_s3.INIT=4'h1;
  LUT3 n1037_s2 (
    .F(n1037_7),
    .I0(Frame_Begin_Z),
    .I1(longitudinal_rows[1]),
    .I2(longitudinal_rows[0]) 
);
defparam n1037_s2.INIT=8'h14;
  LUT4 n1036_s2 (
    .F(n1036_7),
    .I0(longitudinal_rows[1]),
    .I1(longitudinal_rows[0]),
    .I2(Frame_Begin_Z),
    .I3(longitudinal_rows[2]) 
);
defparam n1036_s2.INIT=16'h0708;
  LUT3 n1035_s2 (
    .F(n1035_7),
    .I0(Frame_Begin_Z),
    .I1(longitudinal_rows[3]),
    .I2(n1035_8) 
);
defparam n1035_s2.INIT=8'h14;
  LUT3 n1034_s2 (
    .F(n1034_7),
    .I0(Frame_Begin_Z),
    .I1(longitudinal_rows[4]),
    .I2(n1034_10) 
);
defparam n1034_s2.INIT=8'h14;
  LUT2 n990_s2 (
    .F(n990_7),
    .I0(Frame_Begin_Z),
    .I1(n990_8) 
);
defparam n990_s2.INIT=4'h1;
  LUT2 n989_s2 (
    .F(n989_7),
    .I0(Frame_Begin_Z),
    .I1(n989_8) 
);
defparam n989_s2.INIT=4'h1;
  LUT2 n988_s2 (
    .F(n988_7),
    .I0(Frame_Begin_Z),
    .I1(n988_8) 
);
defparam n988_s2.INIT=4'h1;
  LUT4 n1144_s2 (
    .F(n1144_7),
    .I0(n1144_10),
    .I1(\strip_run[2] [0]),
    .I2(Frame_Begin_Z),
    .I3(strip_kind_2_10) 
);
defparam n1144_s2.INIT=16'h070C;
  LUT4 n1143_s2 (
    .F(n1143_7),
    .I0(n1143_12),
    .I1(\strip_run[2] [1]),
    .I2(n1143_9),
    .I3(Frame_Begin_Z) 
);
defparam n1143_s2.INIT=16'h00F4;
  LUT4 n1142_s2 (
    .F(n1142_7),
    .I0(strip_kind_2_14),
    .I1(Frame_Begin_Z),
    .I2(\strip_run[2] [2]),
    .I3(n1143_12) 
);
defparam n1142_s2.INIT=16'h0230;
  LUT2 n1141_s2 (
    .F(n1141_7),
    .I0(Frame_Begin_Z),
    .I1(n1141_8) 
);
defparam n1141_s2.INIT=4'h1;
  LUT3 n1140_s2 (
    .F(n1140_7),
    .I0(Frame_Begin_Z),
    .I1(n1140_8),
    .I2(n1140_12) 
);
defparam n1140_s2.INIT=8'h01;
  LUT4 n1139_s2 (
    .F(n1139_7),
    .I0(strip_kind_2_10),
    .I1(n1139_8),
    .I2(\strip_run[2] [5]),
    .I3(strip_kind_2_16) 
);
defparam n1139_s2.INIT=16'h00F8;
  LUT4 n1138_s2 (
    .F(n1138_7),
    .I0(n1138_10),
    .I1(\strip_run[1] [0]),
    .I2(Frame_Begin_Z),
    .I3(strip_have_1_9) 
);
defparam n1138_s2.INIT=16'h070C;
  LUT4 n1137_s2 (
    .F(n1137_7),
    .I0(n1137_12),
    .I1(\strip_run[1] [1]),
    .I2(n1137_9),
    .I3(Frame_Begin_Z) 
);
defparam n1137_s2.INIT=16'h00F4;
  LUT4 n1136_s2 (
    .F(n1136_7),
    .I0(strip_kind_1_12),
    .I1(Frame_Begin_Z),
    .I2(\strip_run[1] [2]),
    .I3(n1137_12) 
);
defparam n1136_s2.INIT=16'h0230;
  LUT2 n1135_s2 (
    .F(n1135_7),
    .I0(Frame_Begin_Z),
    .I1(n1135_8) 
);
defparam n1135_s2.INIT=4'h1;
  LUT3 n1134_s2 (
    .F(n1134_7),
    .I0(Frame_Begin_Z),
    .I1(n1134_8),
    .I2(n1134_12) 
);
defparam n1134_s2.INIT=8'h01;
  LUT4 n1133_s2 (
    .F(n1133_7),
    .I0(strip_have_1_9),
    .I1(n1133_8),
    .I2(\strip_run[1] [5]),
    .I3(strip_kind_1_14) 
);
defparam n1133_s2.INIT=16'h00F8;
  LUT4 n1132_s2 (
    .F(n1132_7),
    .I0(n1132_10),
    .I1(\strip_run[0] [0]),
    .I2(Frame_Begin_Z),
    .I3(strip_have_0_9) 
);
defparam n1132_s2.INIT=16'h070C;
  LUT4 n1131_s2 (
    .F(n1131_7),
    .I0(n1131_12),
    .I1(\strip_run[0] [1]),
    .I2(n1131_9),
    .I3(Frame_Begin_Z) 
);
defparam n1131_s2.INIT=16'h00F4;
  LUT4 n1130_s2 (
    .F(n1130_7),
    .I0(strip_kind_0_12),
    .I1(Frame_Begin_Z),
    .I2(\strip_run[0] [2]),
    .I3(n1131_12) 
);
defparam n1130_s2.INIT=16'h0230;
  LUT2 n1129_s2 (
    .F(n1129_7),
    .I0(Frame_Begin_Z),
    .I1(n1129_8) 
);
defparam n1129_s2.INIT=4'h1;
  LUT3 n1128_s2 (
    .F(n1128_7),
    .I0(Frame_Begin_Z),
    .I1(n1128_8),
    .I2(n1128_12) 
);
defparam n1128_s2.INIT=8'h01;
  LUT4 n1127_s2 (
    .F(n1127_7),
    .I0(strip_have_0_9),
    .I1(n1127_8),
    .I2(\strip_run[0] [5]),
    .I3(strip_kind_0_14) 
);
defparam n1127_s2.INIT=16'h00F8;
  LUT2 n1126_s2 (
    .F(n1126_7),
    .I0(Frame_Begin_Z),
    .I1(n1126_8) 
);
defparam n1126_s2.INIT=4'h1;
  LUT2 n1125_s2 (
    .F(n1125_7),
    .I0(Frame_Begin_Z),
    .I1(n1125_8) 
);
defparam n1125_s2.INIT=4'h1;
  LUT2 n1124_s2 (
    .F(n1124_7),
    .I0(Frame_Begin_Z),
    .I1(n1124_8) 
);
defparam n1124_s2.INIT=4'h1;
  LUT2 n1123_s2 (
    .F(n1123_7),
    .I0(Frame_Begin_Z),
    .I1(n1123_8) 
);
defparam n1123_s2.INIT=4'h1;
  LUT2 n1122_s2 (
    .F(n1122_7),
    .I0(Frame_Begin_Z),
    .I1(n1122_8) 
);
defparam n1122_s2.INIT=4'h1;
  LUT2 n1121_s2 (
    .F(n1121_7),
    .I0(Frame_Begin_Z),
    .I1(n1121_8) 
);
defparam n1121_s2.INIT=4'h1;
  LUT2 n1120_s2 (
    .F(n1120_7),
    .I0(Frame_Begin_Z),
    .I1(n1120_8) 
);
defparam n1120_s2.INIT=4'h1;
  LUT2 n1119_s2 (
    .F(n1119_7),
    .I0(Frame_Begin_Z),
    .I1(n1119_8) 
);
defparam n1119_s2.INIT=4'h1;
  LUT2 n1118_s2 (
    .F(n1118_7),
    .I0(Frame_Begin_Z),
    .I1(n1118_8) 
);
defparam n1118_s2.INIT=4'h1;
  LUT2 n1117_s2 (
    .F(n1117_7),
    .I0(Frame_Begin_Z),
    .I1(n1117_8) 
);
defparam n1117_s2.INIT=4'h1;
  LUT2 n1116_s2 (
    .F(n1116_7),
    .I0(Frame_Begin_Z),
    .I1(n1116_8) 
);
defparam n1116_s2.INIT=4'h1;
  LUT2 n1115_s2 (
    .F(n1115_7),
    .I0(Frame_Begin_Z),
    .I1(n1115_8) 
);
defparam n1115_s2.INIT=4'h1;
  LUT2 n1114_s2 (
    .F(n1114_7),
    .I0(Frame_Begin_Z),
    .I1(n1114_8) 
);
defparam n1114_s2.INIT=4'h1;
  LUT2 n1113_s2 (
    .F(n1113_7),
    .I0(Frame_Begin_Z),
    .I1(n1113_8) 
);
defparam n1113_s2.INIT=4'h1;
  LUT2 n1112_s2 (
    .F(n1112_7),
    .I0(Frame_Begin_Z),
    .I1(n1112_8) 
);
defparam n1112_s2.INIT=4'h1;
  LUT4 n1078_s2 (
    .F(n1078_7),
    .I0(n1078_8),
    .I1(longitudinal_run[0]),
    .I2(Frame_Begin_Z),
    .I3(n1078_19) 
);
defparam n1078_s2.INIT=16'h0A0C;
  LUT2 n1077_s2 (
    .F(n1077_7),
    .I0(Frame_Begin_Z),
    .I1(n1077_8) 
);
defparam n1077_s2.INIT=4'h1;
  LUT3 n1076_s2 (
    .F(n1076_7),
    .I0(Frame_Begin_Z),
    .I1(n1076_8),
    .I2(n1076_9) 
);
defparam n1076_s2.INIT=8'h01;
  LUT4 n1075_s2 (
    .F(n1075_7),
    .I0(n1075_8),
    .I1(n1078_19),
    .I2(Frame_Begin_Z),
    .I3(n1075_9) 
);
defparam n1075_s2.INIT=16'h000B;
  LUT2 n1068_s2 (
    .F(n1068_7),
    .I0(Frame_Begin_Z),
    .I1(n1068_8) 
);
defparam n1068_s2.INIT=4'h1;
  LUT2 n1067_s2 (
    .F(n1067_7),
    .I0(Frame_Begin_Z),
    .I1(n1067_8) 
);
defparam n1067_s2.INIT=4'h1;
  LUT2 n1066_s2 (
    .F(n1066_7),
    .I0(Frame_Begin_Z),
    .I1(n1066_8) 
);
defparam n1066_s2.INIT=4'h1;
  LUT4 n1065_s2 (
    .F(n1065_7),
    .I0(capture_low_7_9),
    .I1(n1065_8),
    .I2(across_gaps[3]),
    .I3(n1065_9) 
);
defparam n1065_s2.INIT=16'hF800;
  LUT2 n1064_s2 (
    .F(n1064_7),
    .I0(Frame_Begin_Z),
    .I1(n1064_8) 
);
defparam n1064_s2.INIT=4'h1;
  LUT2 n1063_s2 (
    .F(n1063_7),
    .I0(Frame_Begin_Z),
    .I1(n1063_8) 
);
defparam n1063_s2.INIT=4'h1;
  LUT2 n1062_s2 (
    .F(n1062_7),
    .I0(Frame_Begin_Z),
    .I1(n1062_8) 
);
defparam n1062_s2.INIT=4'h1;
  LUT3 n1059_s2 (
    .F(n1059_7),
    .I0(zebra_low_6_15),
    .I1(across_have),
    .I2(n1065_9) 
);
defparam n1059_s2.INIT=8'hE0;
  LUT4 n1058_s2 (
    .F(n1058_7),
    .I0(zebra_low_6_15),
    .I1(across_run[0]),
    .I2(Frame_Begin_Z),
    .I3(n1058_8) 
);
defparam n1058_s2.INIT=16'h000E;
  LUT4 n1057_s2 (
    .F(n1057_7),
    .I0(n1057_8),
    .I1(across_run[1]),
    .I2(Frame_Begin_Z),
    .I3(capture_low_7_9) 
);
defparam n1057_s2.INIT=16'h0A0C;
  LUT4 n1056_s2 (
    .F(n1056_7),
    .I0(n1056_8),
    .I1(across_run[2]),
    .I2(Frame_Begin_Z),
    .I3(capture_low_7_9) 
);
defparam n1056_s2.INIT=16'h0A0C;
  LUT2 n1055_s2 (
    .F(n1055_7),
    .I0(Frame_Begin_Z),
    .I1(n1055_8) 
);
defparam n1055_s2.INIT=4'h1;
  LUT4 n1054_s2 (
    .F(n1054_7),
    .I0(n1054_8),
    .I1(across_run[4]),
    .I2(Frame_Begin_Z),
    .I3(capture_low_7_9) 
);
defparam n1054_s2.INIT=16'h0A0C;
  LUT3 n1053_s2 (
    .F(n1053_7),
    .I0(Frame_Begin_Z),
    .I1(n1053_8),
    .I2(n1053_9) 
);
defparam n1053_s2.INIT=8'h01;
  LUT4 n1052_s2 (
    .F(n1052_7),
    .I0(zebra_low_6_15),
    .I1(n1052_8),
    .I2(across_run[6]),
    .I3(n1052_9) 
);
defparam n1052_s2.INIT=16'hF800;
  LUT4 n87_s1 (
    .F(n87_4),
    .I0(\strip_bands[2] [3]),
    .I1(n87_20),
    .I2(\strip_bands[2] [2]),
    .I3(n87_9) 
);
defparam n87_s1.INIT=16'h00FE;
  LUT4 n87_s2 (
    .F(n87_5),
    .I0(\strip_bands[0] [3]),
    .I1(n87_16),
    .I2(\strip_bands[0] [2]),
    .I3(n87_11) 
);
defparam n87_s2.INIT=16'h00FE;
  LUT4 n87_s3 (
    .F(n87_6),
    .I0(\strip_bands[1] [3]),
    .I1(n87_18),
    .I2(\strip_bands[1] [2]),
    .I3(n87_13) 
);
defparam n87_s3.INIT=16'h00FE;
  LUT3 n87_s4 (
    .F(n87_7),
    .I0(n87_14),
    .I1(longitudinal_rows[3]),
    .I2(longitudinal) 
);
defparam n87_s4.INIT=8'hD0;
  LUT2 strip_have_1_s4 (
    .F(strip_have_1_9),
    .I0(strip_have_1_10),
    .I1(n1078_19) 
);
defparam strip_have_1_s4.INIT=4'h4;
  LUT2 strip_have_0_s4 (
    .F(strip_have_0_9),
    .I0(strip_have_0_10),
    .I1(n1078_19) 
);
defparam strip_have_0_s4.INIT=4'h4;
  LUT2 strip_kind_2_s5 (
    .F(strip_kind_2_10),
    .I0(strip_kind_2_12),
    .I1(n1078_19) 
);
defparam strip_kind_2_s5.INIT=4'h4;
  LUT3 across_kind_s4 (
    .F(across_kind_9),
    .I0(across_kind),
    .I1(n937_9),
    .I2(across_have) 
);
defparam across_kind_s4.INIT=8'h90;
  LUT2 longitudinal_rows_5_s4 (
    .F(longitudinal_rows_5_9),
    .I0(longitudinal_rows[4]),
    .I1(n1034_10) 
);
defparam longitudinal_rows_5_s4.INIT=4'h8;
  LUT2 longitudinal_rows_5_s5 (
    .F(longitudinal_rows_5_10),
    .I0(n1078_19),
    .I1(n1075_8) 
);
defparam longitudinal_rows_5_s5.INIT=4'h8;
  LUT4 longitudinal_s4 (
    .F(longitudinal_9),
    .I0(longitudinal_run[0]),
    .I1(longitudinal_run[1]),
    .I2(longitudinal_run[3]),
    .I3(longitudinal_run[2]) 
);
defparam longitudinal_s4.INIT=16'h0007;
  LUT4 \strip_bands[0]_3_s5  (
    .F(\strip_bands[0]_3_10 ),
    .I0(n1009_8),
    .I1(\strip_bands[0] [3]),
    .I2(strip_have_0_10),
    .I3(\strip_bands[0]_3_11 ) 
);
defparam \strip_bands[0]_3_s5 .INIT=16'h0700;
  LUT4 \strip_bands[1]_3_s5  (
    .F(\strip_bands[1]_3_10 ),
    .I0(n1013_8),
    .I1(\strip_bands[1] [3]),
    .I2(strip_have_1_10),
    .I3(\strip_bands[1]_3_11 ) 
);
defparam \strip_bands[1]_3_s5 .INIT=16'h0700;
  LUT4 \strip_bands[2]_3_s5  (
    .F(\strip_bands[2]_3_10 ),
    .I0(n1017_8),
    .I1(\strip_bands[2] [3]),
    .I2(strip_kind_2_12),
    .I3(\strip_bands[2]_3_11 ) 
);
defparam \strip_bands[2]_3_s5 .INIT=16'h0700;
  LUT4 \strip_gaps[0]_3_s4  (
    .F(\strip_gaps[0]_3_9 ),
    .I0(\strip_run[0] [5]),
    .I1(\strip_run[0] [4]),
    .I2(\strip_gaps[0]_3_11 ),
    .I3(strip_have[0]) 
);
defparam \strip_gaps[0]_3_s4 .INIT=16'h4100;
  LUT4 \strip_gaps[0]_3_s5  (
    .F(\strip_gaps[0]_3_10 ),
    .I0(n1021_8),
    .I1(\strip_gaps[0] [3]),
    .I2(strip_kind[0]),
    .I3(\strip_gaps[0]_3_12 ) 
);
defparam \strip_gaps[0]_3_s5 .INIT=16'h0007;
  LUT4 \strip_gaps[1]_3_s4  (
    .F(\strip_gaps[1]_3_9 ),
    .I0(\strip_run[1] [5]),
    .I1(\strip_run[1] [4]),
    .I2(\strip_gaps[1]_3_11 ),
    .I3(strip_have[1]) 
);
defparam \strip_gaps[1]_3_s4 .INIT=16'h4100;
  LUT4 \strip_gaps[1]_3_s5  (
    .F(\strip_gaps[1]_3_10 ),
    .I0(n1025_8),
    .I1(\strip_gaps[1] [3]),
    .I2(strip_kind[1]),
    .I3(\strip_gaps[1]_3_12 ) 
);
defparam \strip_gaps[1]_3_s5 .INIT=16'h0007;
  LUT4 \strip_gaps[2]_3_s4  (
    .F(\strip_gaps[2]_3_9 ),
    .I0(\strip_run[2] [5]),
    .I1(\strip_run[2] [4]),
    .I2(\strip_gaps[2]_3_11 ),
    .I3(strip_have[2]) 
);
defparam \strip_gaps[2]_3_s4 .INIT=16'h4100;
  LUT4 \strip_gaps[2]_3_s5  (
    .F(\strip_gaps[2]_3_10 ),
    .I0(n1029_8),
    .I1(\strip_gaps[2] [3]),
    .I2(strip_kind[2]),
    .I3(\strip_gaps[2]_3_12 ) 
);
defparam \strip_gaps[2]_3_s5 .INIT=16'h0007;
  LUT3 n1029_s3 (
    .F(n1029_8),
    .I0(\strip_gaps[2] [1]),
    .I1(\strip_gaps[2] [2]),
    .I2(\strip_gaps[2] [0]) 
);
defparam n1029_s3.INIT=8'h80;
  LUT3 n1025_s3 (
    .F(n1025_8),
    .I0(\strip_gaps[1] [1]),
    .I1(\strip_gaps[1] [2]),
    .I2(\strip_gaps[1] [0]) 
);
defparam n1025_s3.INIT=8'h80;
  LUT3 n1021_s3 (
    .F(n1021_8),
    .I0(\strip_gaps[0] [1]),
    .I1(\strip_gaps[0] [2]),
    .I2(\strip_gaps[0] [0]) 
);
defparam n1021_s3.INIT=8'h80;
  LUT3 n1017_s3 (
    .F(n1017_8),
    .I0(\strip_bands[2] [0]),
    .I1(\strip_bands[2] [1]),
    .I2(\strip_bands[2] [2]) 
);
defparam n1017_s3.INIT=8'h80;
  LUT3 n1013_s3 (
    .F(n1013_8),
    .I0(\strip_bands[1] [0]),
    .I1(\strip_bands[1] [1]),
    .I2(\strip_bands[1] [2]) 
);
defparam n1013_s3.INIT=8'h80;
  LUT3 n1009_s3 (
    .F(n1009_8),
    .I0(\strip_bands[0] [0]),
    .I1(\strip_bands[0] [1]),
    .I2(\strip_bands[0] [2]) 
);
defparam n1009_s3.INIT=8'h80;
  LUT3 n1035_s3 (
    .F(n1035_8),
    .I0(longitudinal_rows[2]),
    .I1(longitudinal_rows[1]),
    .I2(longitudinal_rows[0]) 
);
defparam n1035_s3.INIT=8'h80;
  LUT3 n937_s4 (
    .F(n937_9),
    .I0(n469_33),
    .I1(zebra_luma[7]),
    .I2(zebra_threshold[7]) 
);
defparam n937_s4.INIT=8'h4D;
  LUT4 n990_s3 (
    .F(n990_8),
    .I0(\row_white[0] [2]),
    .I1(n990_9),
    .I2(\row_white[0] [3]),
    .I3(\row_white[0] [4]) 
);
defparam n990_s3.INIT=16'h004F;
  LUT4 n989_s3 (
    .F(n989_8),
    .I0(\row_white[1] [2]),
    .I1(n989_9),
    .I2(\row_white[1] [3]),
    .I3(\row_white[1] [4]) 
);
defparam n989_s3.INIT=16'h004F;
  LUT4 n988_s3 (
    .F(n988_8),
    .I0(\row_white[2] [2]),
    .I1(n988_9),
    .I2(\row_white[2] [3]),
    .I3(\row_white[2] [4]) 
);
defparam n988_s3.INIT=16'h004F;
  LUT4 n1143_s4 (
    .F(n1143_9),
    .I0(\strip_run[2] [1]),
    .I1(\strip_run[2] [0]),
    .I2(strip_kind_2_14),
    .I3(strip_kind_2_10) 
);
defparam n1143_s4.INIT=16'h6000;
  LUT4 n1141_s3 (
    .F(n1141_8),
    .I0(strip_kind_2_10),
    .I1(n1141_9),
    .I2(strip_kind_2_14),
    .I3(\strip_run[2] [3]) 
);
defparam n1141_s3.INIT=16'h8A7F;
  LUT4 n1140_s3 (
    .F(n1140_8),
    .I0(\strip_run[2] [5]),
    .I1(n1139_8),
    .I2(strip_kind_2_14),
    .I3(strip_kind_2_10) 
);
defparam n1140_s3.INIT=16'h4F00;
  LUT2 n1139_s3 (
    .F(n1139_8),
    .I0(\strip_run[2] [4]),
    .I1(n1140_10) 
);
defparam n1139_s3.INIT=4'h8;
  LUT4 n1137_s4 (
    .F(n1137_9),
    .I0(\strip_run[1] [1]),
    .I1(\strip_run[1] [0]),
    .I2(strip_have_1_9),
    .I3(strip_kind_1_12) 
);
defparam n1137_s4.INIT=16'h6000;
  LUT4 n1135_s3 (
    .F(n1135_8),
    .I0(strip_have_1_9),
    .I1(n1135_9),
    .I2(strip_kind_1_12),
    .I3(\strip_run[1] [3]) 
);
defparam n1135_s3.INIT=16'h8A7F;
  LUT4 n1134_s3 (
    .F(n1134_8),
    .I0(\strip_run[1] [5]),
    .I1(n1133_8),
    .I2(strip_kind_1_12),
    .I3(strip_have_1_9) 
);
defparam n1134_s3.INIT=16'h4F00;
  LUT2 n1133_s3 (
    .F(n1133_8),
    .I0(\strip_run[1] [4]),
    .I1(n1134_10) 
);
defparam n1133_s3.INIT=4'h8;
  LUT4 n1131_s4 (
    .F(n1131_9),
    .I0(\strip_run[0] [1]),
    .I1(\strip_run[0] [0]),
    .I2(strip_have_0_9),
    .I3(strip_kind_0_12) 
);
defparam n1131_s4.INIT=16'h6000;
  LUT4 n1129_s3 (
    .F(n1129_8),
    .I0(strip_have_0_9),
    .I1(n1129_9),
    .I2(strip_kind_0_12),
    .I3(\strip_run[0] [3]) 
);
defparam n1129_s3.INIT=16'h8A7F;
  LUT4 n1128_s3 (
    .F(n1128_8),
    .I0(\strip_run[0] [5]),
    .I1(n1127_8),
    .I2(strip_kind_0_12),
    .I3(strip_have_0_9) 
);
defparam n1128_s3.INIT=16'h4F00;
  LUT2 n1127_s3 (
    .F(n1127_8),
    .I0(\strip_run[0] [4]),
    .I1(n1128_10) 
);
defparam n1127_s3.INIT=4'h8;
  LUT4 n1126_s3 (
    .F(n1126_8),
    .I0(n1126_9),
    .I1(capture_low_7_9),
    .I2(n1126_20),
    .I3(\row_white[2] [0]) 
);
defparam n1126_s3.INIT=16'hC8BF;
  LUT4 n1125_s3 (
    .F(n1125_8),
    .I0(n1126_9),
    .I1(capture_low_7_9),
    .I2(n1125_9),
    .I3(\row_white[2] [1]) 
);
defparam n1125_s3.INIT=16'hC8BF;
  LUT4 n1124_s3 (
    .F(n1124_8),
    .I0(n1126_9),
    .I1(capture_low_7_9),
    .I2(n1124_9),
    .I3(\row_white[2] [2]) 
);
defparam n1124_s3.INIT=16'hC8BF;
  LUT4 n1123_s3 (
    .F(n1123_8),
    .I0(n1126_9),
    .I1(capture_low_7_9),
    .I2(n1123_9),
    .I3(\row_white[2] [3]) 
);
defparam n1123_s3.INIT=16'hC8BF;
  LUT4 n1122_s3 (
    .F(n1122_8),
    .I0(n1126_9),
    .I1(capture_low_7_9),
    .I2(n1122_9),
    .I3(\row_white[2] [4]) 
);
defparam n1122_s3.INIT=16'hC8BF;
  LUT4 n1121_s3 (
    .F(n1121_8),
    .I0(n1126_9),
    .I1(capture_low_7_9),
    .I2(n1121_9),
    .I3(\row_white[1] [0]) 
);
defparam n1121_s3.INIT=16'hC8BF;
  LUT4 n1120_s3 (
    .F(n1120_8),
    .I0(n1126_9),
    .I1(capture_low_7_9),
    .I2(n1120_9),
    .I3(\row_white[1] [1]) 
);
defparam n1120_s3.INIT=16'hC8BF;
  LUT4 n1119_s3 (
    .F(n1119_8),
    .I0(n1126_9),
    .I1(capture_low_7_9),
    .I2(n1119_11),
    .I3(\row_white[1] [2]) 
);
defparam n1119_s3.INIT=16'hC8BF;
  LUT4 n1118_s3 (
    .F(n1118_8),
    .I0(n1126_9),
    .I1(capture_low_7_9),
    .I2(n1118_11),
    .I3(\row_white[1] [3]) 
);
defparam n1118_s3.INIT=16'hC8BF;
  LUT4 n1117_s3 (
    .F(n1117_8),
    .I0(n1126_9),
    .I1(capture_low_7_9),
    .I2(n1117_9),
    .I3(\row_white[1] [4]) 
);
defparam n1117_s3.INIT=16'hC8BF;
  LUT4 n1116_s3 (
    .F(n1116_8),
    .I0(n1126_9),
    .I1(capture_low_7_9),
    .I2(n1116_9),
    .I3(\row_white[0] [0]) 
);
defparam n1116_s3.INIT=16'hC8BF;
  LUT4 n1115_s3 (
    .F(n1115_8),
    .I0(n1126_9),
    .I1(capture_low_7_9),
    .I2(n1115_11),
    .I3(\row_white[0] [1]) 
);
defparam n1115_s3.INIT=16'hC8BF;
  LUT4 n1114_s3 (
    .F(n1114_8),
    .I0(n1126_9),
    .I1(capture_low_7_9),
    .I2(n1114_9),
    .I3(\row_white[0] [2]) 
);
defparam n1114_s3.INIT=16'hC8BF;
  LUT4 n1113_s3 (
    .F(n1113_8),
    .I0(n1126_9),
    .I1(capture_low_7_9),
    .I2(n1113_9),
    .I3(\row_white[0] [3]) 
);
defparam n1113_s3.INIT=16'hC8BF;
  LUT4 n1112_s3 (
    .F(n1112_8),
    .I0(n1126_9),
    .I1(capture_low_7_9),
    .I2(n1112_9),
    .I3(\row_white[0] [4]) 
);
defparam n1112_s3.INIT=16'hC8BF;
  LUT2 n1078_s3 (
    .F(n1078_8),
    .I0(n1078_17),
    .I1(n1075_8) 
);
defparam n1078_s3.INIT=4'h4;
  LUT4 n1077_s3 (
    .F(n1077_8),
    .I0(n1078_19),
    .I1(n1078_17),
    .I2(n1075_8),
    .I3(longitudinal_run[1]) 
);
defparam n1077_s3.INIT=16'h8A7F;
  LUT4 n1076_s3 (
    .F(n1076_8),
    .I0(longitudinal_run[3]),
    .I1(n1076_10),
    .I2(n1075_8),
    .I3(n1078_19) 
);
defparam n1076_s3.INIT=16'h4F00;
  LUT4 n1076_s4 (
    .F(n1076_9),
    .I0(longitudinal_run[1]),
    .I1(longitudinal_run[0]),
    .I2(n1078_19),
    .I3(longitudinal_run[2]) 
);
defparam n1076_s4.INIT=16'h007F;
  LUT4 n1075_s3 (
    .F(n1075_8),
    .I0(across_gaps[2]),
    .I1(across_gaps[1]),
    .I2(across_gaps[3]),
    .I3(n1075_10) 
);
defparam n1075_s3.INIT=16'h00FE;
  LUT3 n1075_s4 (
    .F(n1075_9),
    .I0(n1076_10),
    .I1(n1078_19),
    .I2(longitudinal_run[3]) 
);
defparam n1075_s4.INIT=8'h07;
  LUT4 n1068_s3 (
    .F(n1068_8),
    .I0(n1126_9),
    .I1(capture_low_7_9),
    .I2(n1068_9),
    .I3(across_gaps[0]) 
);
defparam n1068_s3.INIT=16'hC8BF;
  LUT4 n1067_s3 (
    .F(n1067_8),
    .I0(n1126_9),
    .I1(capture_low_7_9),
    .I2(n1067_11),
    .I3(across_gaps[1]) 
);
defparam n1067_s3.INIT=16'hC8BF;
  LUT4 n1066_s3 (
    .F(n1066_8),
    .I0(n1126_9),
    .I1(capture_low_7_9),
    .I2(n1066_9),
    .I3(across_gaps[2]) 
);
defparam n1066_s3.INIT=16'hC8BF;
  LUT4 n1065_s3 (
    .F(n1065_8),
    .I0(across_gaps[2]),
    .I1(across_gaps[1]),
    .I2(across_gaps[0]),
    .I3(n1068_9) 
);
defparam n1065_s3.INIT=16'h8000;
  LUT3 n1065_s4 (
    .F(n1065_9),
    .I0(n1126_9),
    .I1(capture_low_7_9),
    .I2(Frame_Begin_Z) 
);
defparam n1065_s4.INIT=8'h07;
  LUT4 n1064_s3 (
    .F(n1064_8),
    .I0(n1126_9),
    .I1(capture_low_7_9),
    .I2(n1064_9),
    .I3(across_bands[0]) 
);
defparam n1064_s3.INIT=16'hC8BF;
  LUT4 n1063_s3 (
    .F(n1063_8),
    .I0(n1126_9),
    .I1(capture_low_7_9),
    .I2(n1063_9),
    .I3(across_bands[1]) 
);
defparam n1063_s3.INIT=16'hC8BF;
  LUT4 n1062_s3 (
    .F(n1062_8),
    .I0(n1126_9),
    .I1(capture_low_7_9),
    .I2(n1062_9),
    .I3(across_bands[2]) 
);
defparam n1062_s3.INIT=16'hC8BF;
  LUT4 n1061_s3 (
    .F(n1061_8),
    .I0(across_bands[2]),
    .I1(capture_low_7_9),
    .I2(n1062_9),
    .I3(across_bands[3]) 
);
defparam n1061_s3.INIT=16'h007F;
  LUT4 n1058_s3 (
    .F(n1058_8),
    .I0(n1058_12),
    .I1(n1058_14),
    .I2(n1126_9),
    .I3(capture_low_7_9) 
);
defparam n1058_s3.INIT=16'hF800;
  LUT2 n1057_s3 (
    .F(n1057_8),
    .I0(n1126_9),
    .I1(n1057_9) 
);
defparam n1057_s3.INIT=4'h1;
  LUT2 n1056_s3 (
    .F(n1056_8),
    .I0(n1126_9),
    .I1(n1056_9) 
);
defparam n1056_s3.INIT=4'h1;
  LUT4 n1055_s3 (
    .F(n1055_8),
    .I0(n1055_9),
    .I1(capture_low_7_9),
    .I2(n1055_10),
    .I3(across_run[3]) 
);
defparam n1055_s3.INIT=16'hC43F;
  LUT2 n1054_s3 (
    .F(n1054_8),
    .I0(n1126_9),
    .I1(n1054_9) 
);
defparam n1054_s3.INIT=4'h1;
  LUT3 n1053_s3 (
    .F(n1053_8),
    .I0(n1053_10),
    .I1(zebra_low_6_13),
    .I2(across_run[5]) 
);
defparam n1053_s3.INIT=8'h07;
  LUT4 n1053_s4 (
    .F(n1053_9),
    .I0(n1126_9),
    .I1(n1053_11),
    .I2(across_run[5]),
    .I3(capture_low_7_9) 
);
defparam n1053_s4.INIT=16'hEE0F;
  LUT2 n1052_s3 (
    .F(n1052_8),
    .I0(across_run[5]),
    .I1(n1053_10) 
);
defparam n1052_s3.INIT=4'h8;
  LUT3 n1052_s4 (
    .F(n1052_9),
    .I0(n1055_9),
    .I1(capture_low_7_9),
    .I2(Frame_Begin_Z) 
);
defparam n1052_s4.INIT=8'h0B;
  LUT3 n87_s6 (
    .F(n87_9),
    .I0(\strip_gaps[2] [1]),
    .I1(\strip_gaps[2] [2]),
    .I2(\strip_gaps[2] [3]) 
);
defparam n87_s6.INIT=8'h01;
  LUT3 n87_s8 (
    .F(n87_11),
    .I0(\strip_gaps[0] [1]),
    .I1(\strip_gaps[0] [2]),
    .I2(\strip_gaps[0] [3]) 
);
defparam n87_s8.INIT=8'h01;
  LUT3 n87_s10 (
    .F(n87_13),
    .I0(\strip_gaps[1] [1]),
    .I1(\strip_gaps[1] [2]),
    .I2(\strip_gaps[1] [3]) 
);
defparam n87_s10.INIT=8'h01;
  LUT4 n87_s11 (
    .F(n87_14),
    .I0(longitudinal_rows[1]),
    .I1(longitudinal_rows[2]),
    .I2(longitudinal_rows[4]),
    .I3(longitudinal_rows[5]) 
);
defparam n87_s11.INIT=16'h0007;
  LUT4 strip_have_1_s5 (
    .F(strip_have_1_10),
    .I0(\row_white[1] [4]),
    .I1(\row_white[1] [3]),
    .I2(n989_9),
    .I3(\row_white[1] [2]) 
);
defparam strip_have_1_s5.INIT=16'h0140;
  LUT4 strip_have_0_s5 (
    .F(strip_have_0_10),
    .I0(\row_white[0] [4]),
    .I1(\row_white[0] [3]),
    .I2(n990_9),
    .I3(\row_white[0] [2]) 
);
defparam strip_have_0_s5.INIT=16'h0140;
  LUT4 strip_kind_2_s7 (
    .F(strip_kind_2_12),
    .I0(\row_white[2] [4]),
    .I1(\row_white[2] [3]),
    .I2(n988_9),
    .I3(\row_white[2] [2]) 
);
defparam strip_kind_2_s7.INIT=16'h0140;
  LUT2 \strip_bands[0]_3_s6  (
    .F(\strip_bands[0]_3_11 ),
    .I0(strip_kind[0]),
    .I1(\strip_gaps[0]_3_9 ) 
);
defparam \strip_bands[0]_3_s6 .INIT=4'h8;
  LUT2 \strip_bands[1]_3_s6  (
    .F(\strip_bands[1]_3_11 ),
    .I0(strip_kind[1]),
    .I1(\strip_gaps[1]_3_9 ) 
);
defparam \strip_bands[1]_3_s6 .INIT=4'h8;
  LUT2 \strip_bands[2]_3_s6  (
    .F(\strip_bands[2]_3_11 ),
    .I0(strip_kind[2]),
    .I1(\strip_gaps[2]_3_9 ) 
);
defparam \strip_bands[2]_3_s6 .INIT=4'h8;
  LUT4 \strip_gaps[0]_3_s6  (
    .F(\strip_gaps[0]_3_11 ),
    .I0(\strip_run[0] [3]),
    .I1(\strip_run[0] [2]),
    .I2(\strip_run[0] [1]),
    .I3(\strip_run[0] [0]) 
);
defparam \strip_gaps[0]_3_s6 .INIT=16'h0001;
  LUT4 \strip_gaps[0]_3_s7  (
    .F(\strip_gaps[0]_3_12 ),
    .I0(\strip_bands[0] [0]),
    .I1(\strip_bands[0] [1]),
    .I2(\strip_bands[0] [2]),
    .I3(\strip_bands[0] [3]) 
);
defparam \strip_gaps[0]_3_s7 .INIT=16'h0001;
  LUT4 \strip_gaps[1]_3_s6  (
    .F(\strip_gaps[1]_3_11 ),
    .I0(\strip_run[1] [3]),
    .I1(\strip_run[1] [2]),
    .I2(\strip_run[1] [1]),
    .I3(\strip_run[1] [0]) 
);
defparam \strip_gaps[1]_3_s6 .INIT=16'h0001;
  LUT4 \strip_gaps[1]_3_s7  (
    .F(\strip_gaps[1]_3_12 ),
    .I0(\strip_bands[1] [0]),
    .I1(\strip_bands[1] [1]),
    .I2(\strip_bands[1] [2]),
    .I3(\strip_bands[1] [3]) 
);
defparam \strip_gaps[1]_3_s7 .INIT=16'h0001;
  LUT4 \strip_gaps[2]_3_s6  (
    .F(\strip_gaps[2]_3_11 ),
    .I0(\strip_run[2] [3]),
    .I1(\strip_run[2] [2]),
    .I2(\strip_run[2] [1]),
    .I3(\strip_run[2] [0]) 
);
defparam \strip_gaps[2]_3_s6 .INIT=16'h0001;
  LUT4 \strip_gaps[2]_3_s7  (
    .F(\strip_gaps[2]_3_12 ),
    .I0(\strip_bands[2] [0]),
    .I1(\strip_bands[2] [1]),
    .I2(\strip_bands[2] [2]),
    .I3(\strip_bands[2] [3]) 
);
defparam \strip_gaps[2]_3_s7 .INIT=16'h0001;
  LUT2 n990_s4 (
    .F(n990_9),
    .I0(\row_white[0] [1]),
    .I1(\row_white[0] [0]) 
);
defparam n990_s4.INIT=4'h1;
  LUT2 n989_s4 (
    .F(n989_9),
    .I0(\row_white[1] [1]),
    .I1(\row_white[1] [0]) 
);
defparam n989_s4.INIT=4'h1;
  LUT2 n988_s4 (
    .F(n988_9),
    .I0(\row_white[2] [1]),
    .I1(\row_white[2] [0]) 
);
defparam n988_s4.INIT=4'h1;
  LUT4 n1143_s5 (
    .F(n1143_10),
    .I0(n1139_8),
    .I1(\strip_run[2] [5]),
    .I2(\strip_run[2] [0]),
    .I3(\strip_run[2] [1]) 
);
defparam n1143_s5.INIT=16'h7000;
  LUT2 n1141_s4 (
    .F(n1141_9),
    .I0(\strip_run[2] [2]),
    .I1(n1143_10) 
);
defparam n1141_s4.INIT=4'h8;
  LUT4 n1140_s5 (
    .F(n1140_10),
    .I0(\strip_run[2] [3]),
    .I1(\strip_run[2] [2]),
    .I2(\strip_run[2] [1]),
    .I3(\strip_run[2] [0]) 
);
defparam n1140_s5.INIT=16'h8000;
  LUT4 n1137_s5 (
    .F(n1137_10),
    .I0(n1133_8),
    .I1(\strip_run[1] [5]),
    .I2(\strip_run[1] [0]),
    .I3(\strip_run[1] [1]) 
);
defparam n1137_s5.INIT=16'h7000;
  LUT2 n1135_s4 (
    .F(n1135_9),
    .I0(\strip_run[1] [2]),
    .I1(n1137_10) 
);
defparam n1135_s4.INIT=4'h8;
  LUT4 n1134_s5 (
    .F(n1134_10),
    .I0(\strip_run[1] [3]),
    .I1(\strip_run[1] [2]),
    .I2(\strip_run[1] [1]),
    .I3(\strip_run[1] [0]) 
);
defparam n1134_s5.INIT=16'h8000;
  LUT4 n1131_s5 (
    .F(n1131_10),
    .I0(n1127_8),
    .I1(\strip_run[0] [5]),
    .I2(\strip_run[0] [0]),
    .I3(\strip_run[0] [1]) 
);
defparam n1131_s5.INIT=16'h7000;
  LUT2 n1129_s4 (
    .F(n1129_9),
    .I0(\strip_run[0] [2]),
    .I1(n1131_10) 
);
defparam n1129_s4.INIT=4'h8;
  LUT4 n1128_s5 (
    .F(n1128_10),
    .I0(\strip_run[0] [3]),
    .I1(\strip_run[0] [2]),
    .I2(\strip_run[0] [1]),
    .I3(\strip_run[0] [0]) 
);
defparam n1128_s5.INIT=16'h8000;
  LUT4 n1126_s4 (
    .F(n1126_9),
    .I0(sample_x[2]),
    .I1(sample_x[1]),
    .I2(sample_x[0]),
    .I3(n1126_11) 
);
defparam n1126_s4.INIT=16'h8000;
  LUT2 n1125_s4 (
    .F(n1125_9),
    .I0(\row_white[2] [0]),
    .I1(n1126_20) 
);
defparam n1125_s4.INIT=4'h8;
  LUT2 n1124_s4 (
    .F(n1124_9),
    .I0(\row_white[2] [1]),
    .I1(n1125_9) 
);
defparam n1124_s4.INIT=4'h8;
  LUT2 n1123_s4 (
    .F(n1123_9),
    .I0(\row_white[2] [2]),
    .I1(n1124_9) 
);
defparam n1123_s4.INIT=4'h8;
  LUT2 n1122_s4 (
    .F(n1122_9),
    .I0(\row_white[2] [3]),
    .I1(n1123_9) 
);
defparam n1122_s4.INIT=4'h8;
  LUT4 n1121_s4 (
    .F(n1121_9),
    .I0(n1121_10),
    .I1(n1121_11),
    .I2(n1121_12),
    .I3(n1126_18) 
);
defparam n1121_s4.INIT=16'h1000;
  LUT2 n1120_s4 (
    .F(n1120_9),
    .I0(\row_white[1] [0]),
    .I1(n1121_9) 
);
defparam n1120_s4.INIT=4'h8;
  LUT2 n1117_s4 (
    .F(n1117_9),
    .I0(\row_white[1] [3]),
    .I1(n1118_11) 
);
defparam n1117_s4.INIT=4'h8;
  LUT3 n1116_s4 (
    .F(n1116_9),
    .I0(sample_x[6]),
    .I1(n1116_10),
    .I2(n1126_18) 
);
defparam n1116_s4.INIT=8'h10;
  LUT2 n1114_s4 (
    .F(n1114_9),
    .I0(\row_white[0] [1]),
    .I1(n1115_11) 
);
defparam n1114_s4.INIT=4'h8;
  LUT2 n1113_s4 (
    .F(n1113_9),
    .I0(\row_white[0] [2]),
    .I1(n1114_9) 
);
defparam n1113_s4.INIT=4'h8;
  LUT2 n1112_s4 (
    .F(n1112_9),
    .I0(\row_white[0] [3]),
    .I1(n1113_9) 
);
defparam n1112_s4.INIT=4'h8;
  LUT3 n1076_s5 (
    .F(n1076_10),
    .I0(longitudinal_run[2]),
    .I1(longitudinal_run[1]),
    .I2(longitudinal_run[0]) 
);
defparam n1076_s5.INIT=8'h80;
  LUT4 n1075_s5 (
    .F(n1075_10),
    .I0(across_bands[0]),
    .I1(n1075_11),
    .I2(across_bands[1]),
    .I3(n1075_12) 
);
defparam n1075_s5.INIT=16'h1F00;
  LUT3 n1068_s4 (
    .F(n1068_9),
    .I0(zebra_low_6_13),
    .I1(n1068_10),
    .I2(n1068_11) 
);
defparam n1068_s4.INIT=8'h80;
  LUT2 n1066_s4 (
    .F(n1066_9),
    .I0(across_gaps[1]),
    .I1(n1067_11) 
);
defparam n1066_s4.INIT=4'h8;
  LUT4 n1064_s4 (
    .F(n1064_9),
    .I0(n1064_10),
    .I1(n937_9),
    .I2(n1075_11),
    .I3(zebra_low_6_13) 
);
defparam n1064_s4.INIT=16'h1000;
  LUT2 n1063_s4 (
    .F(n1063_9),
    .I0(across_bands[0]),
    .I1(n1064_9) 
);
defparam n1063_s4.INIT=4'h8;
  LUT3 n1062_s4 (
    .F(n1062_9),
    .I0(across_bands[0]),
    .I1(across_bands[1]),
    .I2(n1064_9) 
);
defparam n1062_s4.INIT=8'h80;
  LUT4 n1057_s4 (
    .F(n1057_9),
    .I0(zebra_low_6_13),
    .I1(n1058_12),
    .I2(across_kind_9),
    .I3(across_run[1]) 
);
defparam n1057_s4.INIT=16'h8A7F;
  LUT4 n1056_s4 (
    .F(n1056_9),
    .I0(zebra_low_6_13),
    .I1(n1056_10),
    .I2(across_kind_9),
    .I3(across_run[2]) 
);
defparam n1056_s4.INIT=16'h8A7F;
  LUT3 n1055_s4 (
    .F(n1055_9),
    .I0(across_kind_9),
    .I1(zebra_low_6_13),
    .I2(n1126_9) 
);
defparam n1055_s4.INIT=8'h0B;
  LUT3 n1055_s5 (
    .F(n1055_10),
    .I0(across_run[2]),
    .I1(n1058_14),
    .I2(n1056_10) 
);
defparam n1055_s5.INIT=8'h80;
  LUT4 n1054_s4 (
    .F(n1054_9),
    .I0(zebra_low_6_13),
    .I1(n1054_12),
    .I2(across_kind_9),
    .I3(across_run[4]) 
);
defparam n1054_s4.INIT=16'h8A7F;
  LUT4 n1053_s5 (
    .F(n1053_10),
    .I0(across_run[4]),
    .I1(across_run[1]),
    .I2(across_run[0]),
    .I3(n1053_12) 
);
defparam n1053_s5.INIT=16'h8000;
  LUT4 n1053_s6 (
    .F(n1053_11),
    .I0(across_run[6]),
    .I1(n1052_8),
    .I2(across_kind_9),
    .I3(zebra_low_6_13) 
);
defparam n1053_s6.INIT=16'h4F00;
  LUT4 n1126_s6 (
    .F(n1126_11),
    .I0(sample_x[4]),
    .I1(sample_x[3]),
    .I2(sample_x[6]),
    .I3(sample_x[5]) 
);
defparam n1126_s6.INIT=16'h1000;
  LUT4 n1126_s7 (
    .F(n1126_12),
    .I0(sample_x[5]),
    .I1(n1121_10),
    .I2(sample_x[6]),
    .I3(n1126_16) 
);
defparam n1126_s7.INIT=16'h00BF;
  LUT2 n1121_s5 (
    .F(n1121_10),
    .I0(sample_x[4]),
    .I1(n1121_13) 
);
defparam n1121_s5.INIT=4'h1;
  LUT3 n1121_s6 (
    .F(n1121_11),
    .I0(sample_x[4]),
    .I1(sample_x[3]),
    .I2(sample_x[2]) 
);
defparam n1121_s6.INIT=8'h80;
  LUT2 n1121_s7 (
    .F(n1121_12),
    .I0(sample_x[6]),
    .I1(sample_x[5]) 
);
defparam n1121_s7.INIT=4'h4;
  LUT4 n1116_s5 (
    .F(n1116_10),
    .I0(sample_x[4]),
    .I1(n1116_11),
    .I2(n1121_11),
    .I3(sample_x[5]) 
);
defparam n1116_s5.INIT=16'hEE0F;
  LUT4 n1078_s8 (
    .F(n1078_13),
    .I0(sample_y[2]),
    .I1(n783_8),
    .I2(sample_y[3]),
    .I3(n1078_15) 
);
defparam n1078_s8.INIT=16'h1F00;
  LUT4 n1078_s9 (
    .F(n1078_14),
    .I0(sample_y[3]),
    .I1(sample_y[4]),
    .I2(sample_y[5]),
    .I3(sample_y[6]) 
);
defparam n1078_s9.INIT=16'h00F8;
  LUT4 n1075_s6 (
    .F(n1075_11),
    .I0(n1075_13),
    .I1(across_run[1]),
    .I2(across_kind),
    .I3(n1068_10) 
);
defparam n1075_s6.INIT=16'hD000;
  LUT2 n1075_s7 (
    .F(n1075_12),
    .I0(across_bands[2]),
    .I1(across_bands[3]) 
);
defparam n1075_s7.INIT=4'h1;
  LUT4 n1068_s5 (
    .F(n1068_10),
    .I0(across_run[0]),
    .I1(across_run[1]),
    .I2(n1053_12),
    .I3(n1068_12) 
);
defparam n1068_s5.INIT=16'h1F00;
  LUT4 n1068_s6 (
    .F(n1068_11),
    .I0(across_kind),
    .I1(n937_9),
    .I2(n1068_13),
    .I3(n1068_14) 
);
defparam n1068_s6.INIT=16'h0400;
  LUT4 n1064_s5 (
    .F(n1064_10),
    .I0(across_bands[0]),
    .I1(across_bands[1]),
    .I2(across_bands[2]),
    .I3(across_bands[3]) 
);
defparam n1064_s5.INIT=16'h8000;
  LUT4 n1056_s5 (
    .F(n1056_10),
    .I0(n1052_8),
    .I1(across_run[6]),
    .I2(across_run[0]),
    .I3(across_run[1]) 
);
defparam n1056_s5.INIT=16'h7000;
  LUT2 n1053_s7 (
    .F(n1053_12),
    .I0(across_run[3]),
    .I1(across_run[2]) 
);
defparam n1053_s7.INIT=4'h8;
  LUT4 n1121_s8 (
    .F(n1121_13),
    .I0(sample_x[0]),
    .I1(sample_x[1]),
    .I2(sample_x[3]),
    .I3(sample_x[2]) 
);
defparam n1121_s8.INIT=16'hE000;
  LUT4 n1116_s6 (
    .F(n1116_11),
    .I0(sample_x[1]),
    .I1(sample_x[0]),
    .I2(sample_x[2]),
    .I3(sample_x[3]) 
);
defparam n1116_s6.INIT=16'hF800;
  LUT3 n1078_s10 (
    .F(n1078_15),
    .I0(sample_y[4]),
    .I1(sample_y[5]),
    .I2(sample_y[6]) 
);
defparam n1078_s10.INIT=8'h10;
  LUT2 n1075_s8 (
    .F(n1075_13),
    .I0(across_run[3]),
    .I1(across_run[2]) 
);
defparam n1075_s8.INIT=4'h1;
  LUT4 n1068_s7 (
    .F(n1068_12),
    .I0(across_run[6]),
    .I1(across_run[5]),
    .I2(across_run[4]),
    .I3(across_have) 
);
defparam n1068_s7.INIT=16'h0100;
  LUT4 n1068_s8 (
    .F(n1068_13),
    .I0(across_gaps[3]),
    .I1(across_gaps[2]),
    .I2(across_gaps[1]),
    .I3(across_gaps[0]) 
);
defparam n1068_s8.INIT=16'h8000;
  LUT4 n1068_s9 (
    .F(n1068_14),
    .I0(across_run[0]),
    .I1(across_run[1]),
    .I2(n1075_13),
    .I3(n1068_17) 
);
defparam n1068_s9.INIT=16'h00EF;
  LUT4 n1078_s11 (
    .F(n1078_17),
    .I0(longitudinal_run[3]),
    .I1(longitudinal_run[2]),
    .I2(longitudinal_run[1]),
    .I3(longitudinal_run[0]) 
);
defparam n1078_s11.INIT=16'h7F00;
  LUT4 n1034_s4 (
    .F(n1034_10),
    .I0(longitudinal_rows[3]),
    .I1(longitudinal_rows[2]),
    .I2(longitudinal_rows[1]),
    .I3(longitudinal_rows[0]) 
);
defparam n1034_s4.INIT=16'h8000;
  LUT4 n1068_s11 (
    .F(n1068_17),
    .I0(across_bands[0]),
    .I1(across_bands[1]),
    .I2(across_bands[2]),
    .I3(across_bands[3]) 
);
defparam n1068_s11.INIT=16'h0001;
  LUT3 n1119_s5 (
    .F(n1119_11),
    .I0(\row_white[1] [1]),
    .I1(\row_white[1] [0]),
    .I2(n1121_9) 
);
defparam n1119_s5.INIT=8'h80;
  LUT4 n1126_s10 (
    .F(n1126_16),
    .I0(sample_x[1]),
    .I1(n1121_11),
    .I2(sample_x[6]),
    .I3(sample_x[5]) 
);
defparam n1126_s10.INIT=16'h0800;
  LUT4 n87_s12 (
    .F(n87_16),
    .I0(strip_kind[0]),
    .I1(\strip_gaps[0]_3_9 ),
    .I2(\strip_bands[0] [0]),
    .I3(\strip_bands[0] [1]) 
);
defparam n87_s12.INIT=16'hF800;
  LUT4 n87_s13 (
    .F(n87_18),
    .I0(strip_kind[1]),
    .I1(\strip_gaps[1]_3_9 ),
    .I2(\strip_bands[1] [0]),
    .I3(\strip_bands[1] [1]) 
);
defparam n87_s13.INIT=16'hF800;
  LUT4 n87_s14 (
    .F(n87_20),
    .I0(strip_kind[2]),
    .I1(\strip_gaps[2]_3_9 ),
    .I2(\strip_bands[2] [0]),
    .I3(\strip_bands[2] [1]) 
);
defparam n87_s14.INIT=16'hF800;
  LUT3 \strip_bands[0]_3_s7  (
    .F(\strip_bands[0]_3_13 ),
    .I0(n1078_19),
    .I1(strip_kind[0]),
    .I2(n990_8) 
);
defparam \strip_bands[0]_3_s7 .INIT=8'h82;
  LUT3 strip_kind_0_s6 (
    .F(strip_kind_0_12),
    .I0(strip_kind[0]),
    .I1(n990_8),
    .I2(strip_have[0]) 
);
defparam strip_kind_0_s6.INIT=8'h60;
  LUT3 \strip_bands[1]_3_s7  (
    .F(\strip_bands[1]_3_13 ),
    .I0(n1078_19),
    .I1(strip_kind[1]),
    .I2(n989_8) 
);
defparam \strip_bands[1]_3_s7 .INIT=8'h82;
  LUT3 strip_kind_1_s6 (
    .F(strip_kind_1_12),
    .I0(strip_kind[1]),
    .I1(n989_8),
    .I2(strip_have[1]) 
);
defparam strip_kind_1_s6.INIT=8'h60;
  LUT3 \strip_bands[2]_3_s7  (
    .F(\strip_bands[2]_3_13 ),
    .I0(n1078_19),
    .I1(strip_kind[2]),
    .I2(n988_8) 
);
defparam \strip_bands[2]_3_s7 .INIT=8'h82;
  LUT3 strip_kind_2_s8 (
    .F(strip_kind_2_14),
    .I0(strip_kind[2]),
    .I1(n988_8),
    .I2(strip_have[2]) 
);
defparam strip_kind_2_s8.INIT=8'h60;
  LUT3 n1054_s6 (
    .F(n1054_12),
    .I0(across_run[3]),
    .I1(across_run[2]),
    .I2(n1056_10) 
);
defparam n1054_s6.INIT=8'h80;
  LUT4 n1061_s4 (
    .F(n1061_10),
    .I0(n1061_8),
    .I1(n1126_9),
    .I2(capture_low_7_9),
    .I3(Frame_Begin_Z) 
);
defparam n1061_s4.INIT=16'h0015;
  LUT4 n1067_s5 (
    .F(n1067_11),
    .I0(across_gaps[0]),
    .I1(zebra_low_6_13),
    .I2(n1068_10),
    .I3(n1068_11) 
);
defparam n1067_s5.INIT=16'h8000;
  LUT3 n1126_s11 (
    .F(n1126_18),
    .I0(n1078_13),
    .I1(n1078_14),
    .I2(n937_9) 
);
defparam n1126_s11.INIT=8'hE0;
  LUT4 n1078_s12 (
    .F(n1078_19),
    .I0(n1078_13),
    .I1(n1078_14),
    .I2(capture_low_7_9),
    .I3(n1126_9) 
);
defparam n1078_s12.INIT=16'hE000;
  LUT4 n1058_s6 (
    .F(n1058_12),
    .I0(across_run[5]),
    .I1(n1053_10),
    .I2(across_run[6]),
    .I3(across_run[0]) 
);
defparam n1058_s6.INIT=16'h7F00;
  LUT4 n1132_s4 (
    .F(n1132_10),
    .I0(\strip_run[0] [4]),
    .I1(n1128_10),
    .I2(\strip_run[0] [5]),
    .I3(strip_kind_0_12) 
);
defparam n1132_s4.INIT=16'h7F00;
  LUT4 n1138_s4 (
    .F(n1138_10),
    .I0(\strip_run[1] [4]),
    .I1(n1134_10),
    .I2(\strip_run[1] [5]),
    .I3(strip_kind_1_12) 
);
defparam n1138_s4.INIT=16'h7F00;
  LUT4 n1144_s4 (
    .F(n1144_10),
    .I0(\strip_run[2] [4]),
    .I1(n1140_10),
    .I2(\strip_run[2] [5]),
    .I3(strip_kind_2_14) 
);
defparam n1144_s4.INIT=16'h7F00;
  LUT4 n937_s5 (
    .F(n937_11),
    .I0(Frame_Begin_Z),
    .I1(n469_33),
    .I2(zebra_luma[7]),
    .I3(zebra_threshold[7]) 
);
defparam n937_s5.INIT=16'h1051;
  LUT4 n1058_s7 (
    .F(n1058_14),
    .I0(zebra_low_6_13),
    .I1(across_kind),
    .I2(n937_9),
    .I3(across_have) 
);
defparam n1058_s7.INIT=16'h8200;
  LUT4 n1128_s6 (
    .F(n1128_12),
    .I0(n1128_10),
    .I1(strip_have_0_10),
    .I2(n1078_19),
    .I3(\strip_run[0] [4]) 
);
defparam n1128_s6.INIT=16'h00DF;
  LUT4 n1131_s6 (
    .F(n1131_12),
    .I0(n1131_10),
    .I1(strip_kind_0_12),
    .I2(strip_have_0_10),
    .I3(n1078_19) 
);
defparam n1131_s6.INIT=16'h0B00;
  LUT4 strip_kind_0_s7 (
    .F(strip_kind_0_14),
    .I0(strip_kind_0_12),
    .I1(strip_have_0_10),
    .I2(n1078_19),
    .I3(Frame_Begin_Z) 
);
defparam strip_kind_0_s7.INIT=16'hFF10;
  LUT4 n1134_s6 (
    .F(n1134_12),
    .I0(n1134_10),
    .I1(strip_have_1_10),
    .I2(n1078_19),
    .I3(\strip_run[1] [4]) 
);
defparam n1134_s6.INIT=16'h00DF;
  LUT4 n1137_s6 (
    .F(n1137_12),
    .I0(n1137_10),
    .I1(strip_kind_1_12),
    .I2(strip_have_1_10),
    .I3(n1078_19) 
);
defparam n1137_s6.INIT=16'h0B00;
  LUT4 strip_kind_1_s7 (
    .F(strip_kind_1_14),
    .I0(strip_kind_1_12),
    .I1(strip_have_1_10),
    .I2(n1078_19),
    .I3(Frame_Begin_Z) 
);
defparam strip_kind_1_s7.INIT=16'hFF10;
  LUT4 n1140_s6 (
    .F(n1140_12),
    .I0(n1140_10),
    .I1(strip_kind_2_12),
    .I2(n1078_19),
    .I3(\strip_run[2] [4]) 
);
defparam n1140_s6.INIT=16'h00DF;
  LUT4 n1143_s6 (
    .F(n1143_12),
    .I0(n1143_10),
    .I1(strip_kind_2_14),
    .I2(strip_kind_2_12),
    .I3(n1078_19) 
);
defparam n1143_s6.INIT=16'h0B00;
  LUT4 strip_kind_2_s9 (
    .F(strip_kind_2_16),
    .I0(strip_kind_2_14),
    .I1(strip_kind_2_12),
    .I2(n1078_19),
    .I3(Frame_Begin_Z) 
);
defparam strip_kind_2_s9.INIT=16'hFF10;
  LUT3 strip_have_1_s8 (
    .F(strip_have_1_16),
    .I0(strip_have_1_10),
    .I1(n1078_19),
    .I2(Frame_Begin_Z) 
);
defparam strip_have_1_s8.INIT=8'hF4;
  LUT3 strip_have_0_s8 (
    .F(strip_have_0_16),
    .I0(strip_have_0_10),
    .I1(n1078_19),
    .I2(Frame_Begin_Z) 
);
defparam strip_have_0_s8.INIT=8'hF4;
  LUT3 n1033_s4 (
    .F(n1033_10),
    .I0(longitudinal_rows[5]),
    .I1(n1033_14),
    .I2(longitudinal_rows_5_14) 
);
defparam n1033_s4.INIT=8'hCA;
  LUT4 longitudinal_rows_5_s7 (
    .F(longitudinal_rows_5_14),
    .I0(longitudinal_rows[5]),
    .I1(longitudinal_rows_5_9),
    .I2(longitudinal_rows_5_10),
    .I3(Frame_Begin_Z) 
);
defparam longitudinal_rows_5_s7.INIT=16'hFF70;
  LUT3 \strip_bands[0]_3_s8  (
    .F(\strip_bands[0]_3_15 ),
    .I0(\strip_bands[0]_3_13 ),
    .I1(\strip_bands[0]_3_10 ),
    .I2(Frame_Begin_Z) 
);
defparam \strip_bands[0]_3_s8 .INIT=8'hF8;
  LUT3 \strip_bands[1]_3_s8  (
    .F(\strip_bands[1]_3_15 ),
    .I0(\strip_bands[1]_3_13 ),
    .I1(\strip_bands[1]_3_10 ),
    .I2(Frame_Begin_Z) 
);
defparam \strip_bands[1]_3_s8 .INIT=8'hF8;
  LUT3 \strip_bands[2]_3_s8  (
    .F(\strip_bands[2]_3_15 ),
    .I0(\strip_bands[2]_3_13 ),
    .I1(\strip_bands[2]_3_10 ),
    .I2(Frame_Begin_Z) 
);
defparam \strip_bands[2]_3_s8 .INIT=8'hF8;
  LUT3 strip_have_2_s6 (
    .F(strip_have_2_14),
    .I0(strip_kind_2_12),
    .I1(n1078_19),
    .I2(Frame_Begin_Z) 
);
defparam strip_have_2_s6.INIT=8'hF4;
  LUT3 n1012_s7 (
    .F(n1012_15),
    .I0(Frame_Begin_Z),
    .I1(\strip_bands[0] [0]),
    .I2(\strip_bands[0]_3_15 ) 
);
defparam n1012_s7.INIT=8'h1E;
  LUT3 n1016_s7 (
    .F(n1016_15),
    .I0(Frame_Begin_Z),
    .I1(\strip_bands[1] [0]),
    .I2(\strip_bands[1]_3_15 ) 
);
defparam n1016_s7.INIT=8'h1E;
  LUT3 n1020_s7 (
    .F(n1020_15),
    .I0(Frame_Begin_Z),
    .I1(\strip_bands[2] [0]),
    .I2(\strip_bands[2]_3_15 ) 
);
defparam n1020_s7.INIT=8'h1E;
  LUT4 n1033_s6 (
    .F(n1033_14),
    .I0(Frame_Begin_Z),
    .I1(longitudinal_rows[5]),
    .I2(longitudinal_rows[4]),
    .I3(n1034_10) 
);
defparam n1033_s6.INIT=16'h1444;
  LUT4 longitudinal_s7 (
    .F(longitudinal_15),
    .I0(longitudinal_9),
    .I1(n1078_19),
    .I2(n1075_8),
    .I3(Frame_Begin_Z) 
);
defparam longitudinal_s7.INIT=16'hFF40;
  LUT4 n1115_s5 (
    .F(n1115_11),
    .I0(\row_white[0] [0]),
    .I1(sample_x[6]),
    .I2(n1116_10),
    .I3(n1126_18) 
);
defparam n1115_s5.INIT=16'h0200;
  LUT4 n1118_s5 (
    .F(n1118_11),
    .I0(\row_white[1] [2]),
    .I1(\row_white[1] [1]),
    .I2(\row_white[1] [0]),
    .I3(n1121_9) 
);
defparam n1118_s5.INIT=16'h8000;
  LUT4 n1126_s12 (
    .F(n1126_20),
    .I0(n1126_12),
    .I1(n1078_13),
    .I2(n1078_14),
    .I3(n937_9) 
);
defparam n1126_s12.INIT=16'h5400;
  DFFCE across_run_6_s0 (
    .Q(across_run[6]),
    .D(n1052_7),
    .CLK(TFT_clk_d),
    .CLEAR(n564_6),
    .CE(VCC) 
);
defparam across_run_6_s0.INIT=1'b0;
  DFFCE across_run_5_s0 (
    .Q(across_run[5]),
    .D(n1053_7),
    .CLK(TFT_clk_d),
    .CLEAR(n564_6),
    .CE(VCC) 
);
defparam across_run_5_s0.INIT=1'b0;
  DFFCE across_run_4_s0 (
    .Q(across_run[4]),
    .D(n1054_7),
    .CLK(TFT_clk_d),
    .CLEAR(n564_6),
    .CE(VCC) 
);
defparam across_run_4_s0.INIT=1'b0;
  DFFCE across_run_3_s0 (
    .Q(across_run[3]),
    .D(n1055_7),
    .CLK(TFT_clk_d),
    .CLEAR(n564_6),
    .CE(VCC) 
);
defparam across_run_3_s0.INIT=1'b0;
  DFFCE across_run_2_s0 (
    .Q(across_run[2]),
    .D(n1056_7),
    .CLK(TFT_clk_d),
    .CLEAR(n564_6),
    .CE(VCC) 
);
defparam across_run_2_s0.INIT=1'b0;
  DFFCE across_run_1_s0 (
    .Q(across_run[1]),
    .D(n1057_7),
    .CLK(TFT_clk_d),
    .CLEAR(n564_6),
    .CE(VCC) 
);
defparam across_run_1_s0.INIT=1'b0;
  DFFCE across_run_0_s0 (
    .Q(across_run[0]),
    .D(n1058_7),
    .CLK(TFT_clk_d),
    .CLEAR(n564_6),
    .CE(VCC) 
);
defparam across_run_0_s0.INIT=1'b0;
  DFFCE across_have_s0 (
    .Q(across_have),
    .D(n1059_7),
    .CLK(TFT_clk_d),
    .CLEAR(n564_6),
    .CE(VCC) 
);
defparam across_have_s0.INIT=1'b0;
  DFFCE across_bands_3_s0 (
    .Q(across_bands[3]),
    .D(n1061_10),
    .CLK(TFT_clk_d),
    .CLEAR(n564_6),
    .CE(VCC) 
);
defparam across_bands_3_s0.INIT=1'b0;
  DFFCE across_bands_2_s0 (
    .Q(across_bands[2]),
    .D(n1062_7),
    .CLK(TFT_clk_d),
    .CLEAR(n564_6),
    .CE(VCC) 
);
defparam across_bands_2_s0.INIT=1'b0;
  DFFCE across_bands_1_s0 (
    .Q(across_bands[1]),
    .D(n1063_7),
    .CLK(TFT_clk_d),
    .CLEAR(n564_6),
    .CE(VCC) 
);
defparam across_bands_1_s0.INIT=1'b0;
  DFFCE across_bands_0_s0 (
    .Q(across_bands[0]),
    .D(n1064_7),
    .CLK(TFT_clk_d),
    .CLEAR(n564_6),
    .CE(VCC) 
);
defparam across_bands_0_s0.INIT=1'b0;
  DFFCE across_gaps_3_s0 (
    .Q(across_gaps[3]),
    .D(n1065_7),
    .CLK(TFT_clk_d),
    .CLEAR(n564_6),
    .CE(VCC) 
);
defparam across_gaps_3_s0.INIT=1'b0;
  DFFCE across_gaps_2_s0 (
    .Q(across_gaps[2]),
    .D(n1066_7),
    .CLK(TFT_clk_d),
    .CLEAR(n564_6),
    .CE(VCC) 
);
defparam across_gaps_2_s0.INIT=1'b0;
  DFFCE across_gaps_1_s0 (
    .Q(across_gaps[1]),
    .D(n1067_7),
    .CLK(TFT_clk_d),
    .CLEAR(n564_6),
    .CE(VCC) 
);
defparam across_gaps_1_s0.INIT=1'b0;
  DFFCE across_gaps_0_s0 (
    .Q(across_gaps[0]),
    .D(n1068_7),
    .CLK(TFT_clk_d),
    .CLEAR(n564_6),
    .CE(VCC) 
);
defparam across_gaps_0_s0.INIT=1'b0;
  DFFCE longitudinal_run_3_s0 (
    .Q(longitudinal_run[3]),
    .D(n1075_7),
    .CLK(TFT_clk_d),
    .CLEAR(n564_6),
    .CE(VCC) 
);
defparam longitudinal_run_3_s0.INIT=1'b0;
  DFFCE longitudinal_run_2_s0 (
    .Q(longitudinal_run[2]),
    .D(n1076_7),
    .CLK(TFT_clk_d),
    .CLEAR(n564_6),
    .CE(VCC) 
);
defparam longitudinal_run_2_s0.INIT=1'b0;
  DFFCE longitudinal_run_1_s0 (
    .Q(longitudinal_run[1]),
    .D(n1077_7),
    .CLK(TFT_clk_d),
    .CLEAR(n564_6),
    .CE(VCC) 
);
defparam longitudinal_run_1_s0.INIT=1'b0;
  DFFCE longitudinal_run_0_s0 (
    .Q(longitudinal_run[0]),
    .D(n1078_7),
    .CLK(TFT_clk_d),
    .CLEAR(n564_6),
    .CE(VCC) 
);
defparam longitudinal_run_0_s0.INIT=1'b0;
  DFFCE update_s0 (
    .Q(zebra_update),
    .D(n1460_11),
    .CLK(TFT_clk_d),
    .CLEAR(n564_6),
    .CE(VCC) 
);
defparam update_s0.INIT=1'b0;
  DFFCE detected_s0 (
    .Q(zebra_detected),
    .D(n87_3),
    .CLK(TFT_clk_d),
    .CLEAR(n564_6),
    .CE(n1460_11) 
);
  DFFCE \row_white[0]_4_s0  (
    .Q(\row_white[0] [4]),
    .D(n1112_7),
    .CLK(TFT_clk_d),
    .CLEAR(n564_6),
    .CE(VCC) 
);
defparam \row_white[0]_4_s0 .INIT=1'b0;
  DFFCE \row_white[0]_3_s0  (
    .Q(\row_white[0] [3]),
    .D(n1113_7),
    .CLK(TFT_clk_d),
    .CLEAR(n564_6),
    .CE(VCC) 
);
defparam \row_white[0]_3_s0 .INIT=1'b0;
  DFFCE \row_white[0]_2_s0  (
    .Q(\row_white[0] [2]),
    .D(n1114_7),
    .CLK(TFT_clk_d),
    .CLEAR(n564_6),
    .CE(VCC) 
);
defparam \row_white[0]_2_s0 .INIT=1'b0;
  DFFCE \row_white[0]_1_s0  (
    .Q(\row_white[0] [1]),
    .D(n1115_7),
    .CLK(TFT_clk_d),
    .CLEAR(n564_6),
    .CE(VCC) 
);
defparam \row_white[0]_1_s0 .INIT=1'b0;
  DFFCE \row_white[0]_0_s0  (
    .Q(\row_white[0] [0]),
    .D(n1116_7),
    .CLK(TFT_clk_d),
    .CLEAR(n564_6),
    .CE(VCC) 
);
defparam \row_white[0]_0_s0 .INIT=1'b0;
  DFFCE \row_white[1]_4_s0  (
    .Q(\row_white[1] [4]),
    .D(n1117_7),
    .CLK(TFT_clk_d),
    .CLEAR(n564_6),
    .CE(VCC) 
);
defparam \row_white[1]_4_s0 .INIT=1'b0;
  DFFCE \row_white[1]_3_s0  (
    .Q(\row_white[1] [3]),
    .D(n1118_7),
    .CLK(TFT_clk_d),
    .CLEAR(n564_6),
    .CE(VCC) 
);
defparam \row_white[1]_3_s0 .INIT=1'b0;
  DFFCE \row_white[1]_2_s0  (
    .Q(\row_white[1] [2]),
    .D(n1119_7),
    .CLK(TFT_clk_d),
    .CLEAR(n564_6),
    .CE(VCC) 
);
defparam \row_white[1]_2_s0 .INIT=1'b0;
  DFFCE \row_white[1]_1_s0  (
    .Q(\row_white[1] [1]),
    .D(n1120_7),
    .CLK(TFT_clk_d),
    .CLEAR(n564_6),
    .CE(VCC) 
);
defparam \row_white[1]_1_s0 .INIT=1'b0;
  DFFCE \row_white[1]_0_s0  (
    .Q(\row_white[1] [0]),
    .D(n1121_7),
    .CLK(TFT_clk_d),
    .CLEAR(n564_6),
    .CE(VCC) 
);
defparam \row_white[1]_0_s0 .INIT=1'b0;
  DFFCE \row_white[2]_4_s0  (
    .Q(\row_white[2] [4]),
    .D(n1122_7),
    .CLK(TFT_clk_d),
    .CLEAR(n564_6),
    .CE(VCC) 
);
defparam \row_white[2]_4_s0 .INIT=1'b0;
  DFFCE \row_white[2]_3_s0  (
    .Q(\row_white[2] [3]),
    .D(n1123_7),
    .CLK(TFT_clk_d),
    .CLEAR(n564_6),
    .CE(VCC) 
);
defparam \row_white[2]_3_s0 .INIT=1'b0;
  DFFCE \row_white[2]_2_s0  (
    .Q(\row_white[2] [2]),
    .D(n1124_7),
    .CLK(TFT_clk_d),
    .CLEAR(n564_6),
    .CE(VCC) 
);
defparam \row_white[2]_2_s0 .INIT=1'b0;
  DFFCE \row_white[2]_1_s0  (
    .Q(\row_white[2] [1]),
    .D(n1125_7),
    .CLK(TFT_clk_d),
    .CLEAR(n564_6),
    .CE(VCC) 
);
defparam \row_white[2]_1_s0 .INIT=1'b0;
  DFFCE \row_white[2]_0_s0  (
    .Q(\row_white[2] [0]),
    .D(n1126_7),
    .CLK(TFT_clk_d),
    .CLEAR(n564_6),
    .CE(VCC) 
);
defparam \row_white[2]_0_s0 .INIT=1'b0;
  DFFCE \strip_run[0]_5_s0  (
    .Q(\strip_run[0] [5]),
    .D(n1127_7),
    .CLK(TFT_clk_d),
    .CLEAR(n564_6),
    .CE(VCC) 
);
defparam \strip_run[0]_5_s0 .INIT=1'b0;
  DFFCE \strip_run[0]_4_s0  (
    .Q(\strip_run[0] [4]),
    .D(n1128_7),
    .CLK(TFT_clk_d),
    .CLEAR(n564_6),
    .CE(VCC) 
);
defparam \strip_run[0]_4_s0 .INIT=1'b0;
  DFFCE \strip_run[0]_3_s0  (
    .Q(\strip_run[0] [3]),
    .D(n1129_7),
    .CLK(TFT_clk_d),
    .CLEAR(n564_6),
    .CE(VCC) 
);
defparam \strip_run[0]_3_s0 .INIT=1'b0;
  DFFCE \strip_run[0]_2_s0  (
    .Q(\strip_run[0] [2]),
    .D(n1130_7),
    .CLK(TFT_clk_d),
    .CLEAR(n564_6),
    .CE(VCC) 
);
defparam \strip_run[0]_2_s0 .INIT=1'b0;
  DFFCE \strip_run[0]_1_s0  (
    .Q(\strip_run[0] [1]),
    .D(n1131_7),
    .CLK(TFT_clk_d),
    .CLEAR(n564_6),
    .CE(VCC) 
);
defparam \strip_run[0]_1_s0 .INIT=1'b0;
  DFFCE \strip_run[0]_0_s0  (
    .Q(\strip_run[0] [0]),
    .D(n1132_7),
    .CLK(TFT_clk_d),
    .CLEAR(n564_6),
    .CE(VCC) 
);
defparam \strip_run[0]_0_s0 .INIT=1'b0;
  DFFCE \strip_run[1]_5_s0  (
    .Q(\strip_run[1] [5]),
    .D(n1133_7),
    .CLK(TFT_clk_d),
    .CLEAR(n564_6),
    .CE(VCC) 
);
defparam \strip_run[1]_5_s0 .INIT=1'b0;
  DFFCE \strip_run[1]_4_s0  (
    .Q(\strip_run[1] [4]),
    .D(n1134_7),
    .CLK(TFT_clk_d),
    .CLEAR(n564_6),
    .CE(VCC) 
);
defparam \strip_run[1]_4_s0 .INIT=1'b0;
  DFFCE \strip_run[1]_3_s0  (
    .Q(\strip_run[1] [3]),
    .D(n1135_7),
    .CLK(TFT_clk_d),
    .CLEAR(n564_6),
    .CE(VCC) 
);
defparam \strip_run[1]_3_s0 .INIT=1'b0;
  DFFCE \strip_run[1]_2_s0  (
    .Q(\strip_run[1] [2]),
    .D(n1136_7),
    .CLK(TFT_clk_d),
    .CLEAR(n564_6),
    .CE(VCC) 
);
defparam \strip_run[1]_2_s0 .INIT=1'b0;
  DFFCE \strip_run[1]_1_s0  (
    .Q(\strip_run[1] [1]),
    .D(n1137_7),
    .CLK(TFT_clk_d),
    .CLEAR(n564_6),
    .CE(VCC) 
);
defparam \strip_run[1]_1_s0 .INIT=1'b0;
  DFFCE \strip_run[1]_0_s0  (
    .Q(\strip_run[1] [0]),
    .D(n1138_7),
    .CLK(TFT_clk_d),
    .CLEAR(n564_6),
    .CE(VCC) 
);
defparam \strip_run[1]_0_s0 .INIT=1'b0;
  DFFCE \strip_run[2]_5_s0  (
    .Q(\strip_run[2] [5]),
    .D(n1139_7),
    .CLK(TFT_clk_d),
    .CLEAR(n564_6),
    .CE(VCC) 
);
defparam \strip_run[2]_5_s0 .INIT=1'b0;
  DFFCE \strip_run[2]_4_s0  (
    .Q(\strip_run[2] [4]),
    .D(n1140_7),
    .CLK(TFT_clk_d),
    .CLEAR(n564_6),
    .CE(VCC) 
);
defparam \strip_run[2]_4_s0 .INIT=1'b0;
  DFFCE \strip_run[2]_3_s0  (
    .Q(\strip_run[2] [3]),
    .D(n1141_7),
    .CLK(TFT_clk_d),
    .CLEAR(n564_6),
    .CE(VCC) 
);
defparam \strip_run[2]_3_s0 .INIT=1'b0;
  DFFCE \strip_run[2]_2_s0  (
    .Q(\strip_run[2] [2]),
    .D(n1142_7),
    .CLK(TFT_clk_d),
    .CLEAR(n564_6),
    .CE(VCC) 
);
defparam \strip_run[2]_2_s0 .INIT=1'b0;
  DFFCE \strip_run[2]_1_s0  (
    .Q(\strip_run[2] [1]),
    .D(n1143_7),
    .CLK(TFT_clk_d),
    .CLEAR(n564_6),
    .CE(VCC) 
);
defparam \strip_run[2]_1_s0 .INIT=1'b0;
  DFFCE \strip_run[2]_0_s0  (
    .Q(\strip_run[2] [0]),
    .D(n1144_7),
    .CLK(TFT_clk_d),
    .CLEAR(n564_6),
    .CE(VCC) 
);
defparam \strip_run[2]_0_s0 .INIT=1'b0;
  DFFCE strip_kind_2_s1 (
    .Q(strip_kind[2]),
    .D(n988_7),
    .CLK(TFT_clk_d),
    .CLEAR(n564_6),
    .CE(strip_kind_2_16) 
);
defparam strip_kind_2_s1.INIT=1'b0;
  DFFCE strip_kind_1_s1 (
    .Q(strip_kind[1]),
    .D(n989_7),
    .CLK(TFT_clk_d),
    .CLEAR(n564_6),
    .CE(strip_kind_1_14) 
);
defparam strip_kind_1_s1.INIT=1'b0;
  DFFCE strip_kind_0_s1 (
    .Q(strip_kind[0]),
    .D(n990_7),
    .CLK(TFT_clk_d),
    .CLEAR(n564_6),
    .CE(strip_kind_0_14) 
);
defparam strip_kind_0_s1.INIT=1'b0;
  DFFCE across_kind_s1 (
    .Q(across_kind),
    .D(n937_11),
    .CLK(TFT_clk_d),
    .CLEAR(n564_6),
    .CE(across_kind_8) 
);
defparam across_kind_s1.INIT=1'b0;
  DFFCE longitudinal_rows_4_s1 (
    .Q(longitudinal_rows[4]),
    .D(n1034_7),
    .CLK(TFT_clk_d),
    .CLEAR(n564_6),
    .CE(longitudinal_rows_5_14) 
);
defparam longitudinal_rows_4_s1.INIT=1'b0;
  DFFCE longitudinal_rows_3_s1 (
    .Q(longitudinal_rows[3]),
    .D(n1035_7),
    .CLK(TFT_clk_d),
    .CLEAR(n564_6),
    .CE(longitudinal_rows_5_14) 
);
defparam longitudinal_rows_3_s1.INIT=1'b0;
  DFFCE longitudinal_rows_2_s1 (
    .Q(longitudinal_rows[2]),
    .D(n1036_7),
    .CLK(TFT_clk_d),
    .CLEAR(n564_6),
    .CE(longitudinal_rows_5_14) 
);
defparam longitudinal_rows_2_s1.INIT=1'b0;
  DFFCE longitudinal_rows_1_s1 (
    .Q(longitudinal_rows[1]),
    .D(n1037_7),
    .CLK(TFT_clk_d),
    .CLEAR(n564_6),
    .CE(longitudinal_rows_5_14) 
);
defparam longitudinal_rows_1_s1.INIT=1'b0;
  DFFCE longitudinal_rows_0_s1 (
    .Q(longitudinal_rows[0]),
    .D(n1038_8),
    .CLK(TFT_clk_d),
    .CLEAR(n564_6),
    .CE(longitudinal_rows_5_14) 
);
defparam longitudinal_rows_0_s1.INIT=1'b0;
  DFFCE \strip_bands[0]_3_s1  (
    .Q(\strip_bands[0] [3]),
    .D(n1009_7),
    .CLK(TFT_clk_d),
    .CLEAR(n564_6),
    .CE(\strip_bands[0]_3_15 ) 
);
defparam \strip_bands[0]_3_s1 .INIT=1'b0;
  DFFCE \strip_bands[0]_2_s1  (
    .Q(\strip_bands[0] [2]),
    .D(n1010_7),
    .CLK(TFT_clk_d),
    .CLEAR(n564_6),
    .CE(\strip_bands[0]_3_15 ) 
);
defparam \strip_bands[0]_2_s1 .INIT=1'b0;
  DFFCE \strip_bands[0]_1_s1  (
    .Q(\strip_bands[0] [1]),
    .D(n1011_7),
    .CLK(TFT_clk_d),
    .CLEAR(n564_6),
    .CE(\strip_bands[0]_3_15 ) 
);
defparam \strip_bands[0]_1_s1 .INIT=1'b0;
  DFFCE \strip_bands[1]_3_s1  (
    .Q(\strip_bands[1] [3]),
    .D(n1013_7),
    .CLK(TFT_clk_d),
    .CLEAR(n564_6),
    .CE(\strip_bands[1]_3_15 ) 
);
defparam \strip_bands[1]_3_s1 .INIT=1'b0;
  DFFCE \strip_bands[1]_2_s1  (
    .Q(\strip_bands[1] [2]),
    .D(n1014_7),
    .CLK(TFT_clk_d),
    .CLEAR(n564_6),
    .CE(\strip_bands[1]_3_15 ) 
);
defparam \strip_bands[1]_2_s1 .INIT=1'b0;
  DFFCE \strip_bands[1]_1_s1  (
    .Q(\strip_bands[1] [1]),
    .D(n1015_7),
    .CLK(TFT_clk_d),
    .CLEAR(n564_6),
    .CE(\strip_bands[1]_3_15 ) 
);
defparam \strip_bands[1]_1_s1 .INIT=1'b0;
  DFFCE \strip_bands[2]_3_s1  (
    .Q(\strip_bands[2] [3]),
    .D(n1017_7),
    .CLK(TFT_clk_d),
    .CLEAR(n564_6),
    .CE(\strip_bands[2]_3_15 ) 
);
defparam \strip_bands[2]_3_s1 .INIT=1'b0;
  DFFCE \strip_bands[2]_2_s1  (
    .Q(\strip_bands[2] [2]),
    .D(n1018_7),
    .CLK(TFT_clk_d),
    .CLEAR(n564_6),
    .CE(\strip_bands[2]_3_15 ) 
);
defparam \strip_bands[2]_2_s1 .INIT=1'b0;
  DFFCE \strip_bands[2]_1_s1  (
    .Q(\strip_bands[2] [1]),
    .D(n1019_7),
    .CLK(TFT_clk_d),
    .CLEAR(n564_6),
    .CE(\strip_bands[2]_3_15 ) 
);
defparam \strip_bands[2]_1_s1 .INIT=1'b0;
  DFFCE \strip_gaps[0]_3_s1  (
    .Q(\strip_gaps[0] [3]),
    .D(n1021_7),
    .CLK(TFT_clk_d),
    .CLEAR(n564_6),
    .CE(\strip_gaps[0]_3_8 ) 
);
defparam \strip_gaps[0]_3_s1 .INIT=1'b0;
  DFFCE \strip_gaps[0]_2_s1  (
    .Q(\strip_gaps[0] [2]),
    .D(n1022_7),
    .CLK(TFT_clk_d),
    .CLEAR(n564_6),
    .CE(\strip_gaps[0]_3_8 ) 
);
defparam \strip_gaps[0]_2_s1 .INIT=1'b0;
  DFFCE \strip_gaps[0]_1_s1  (
    .Q(\strip_gaps[0] [1]),
    .D(n1023_7),
    .CLK(TFT_clk_d),
    .CLEAR(n564_6),
    .CE(\strip_gaps[0]_3_8 ) 
);
defparam \strip_gaps[0]_1_s1 .INIT=1'b0;
  DFFCE \strip_gaps[0]_0_s1  (
    .Q(\strip_gaps[0] [0]),
    .D(n1024_8),
    .CLK(TFT_clk_d),
    .CLEAR(n564_6),
    .CE(\strip_gaps[0]_3_8 ) 
);
defparam \strip_gaps[0]_0_s1 .INIT=1'b0;
  DFFCE \strip_gaps[1]_3_s1  (
    .Q(\strip_gaps[1] [3]),
    .D(n1025_7),
    .CLK(TFT_clk_d),
    .CLEAR(n564_6),
    .CE(\strip_gaps[1]_3_8 ) 
);
defparam \strip_gaps[1]_3_s1 .INIT=1'b0;
  DFFCE \strip_gaps[1]_2_s1  (
    .Q(\strip_gaps[1] [2]),
    .D(n1026_7),
    .CLK(TFT_clk_d),
    .CLEAR(n564_6),
    .CE(\strip_gaps[1]_3_8 ) 
);
defparam \strip_gaps[1]_2_s1 .INIT=1'b0;
  DFFCE \strip_gaps[1]_1_s1  (
    .Q(\strip_gaps[1] [1]),
    .D(n1027_7),
    .CLK(TFT_clk_d),
    .CLEAR(n564_6),
    .CE(\strip_gaps[1]_3_8 ) 
);
defparam \strip_gaps[1]_1_s1 .INIT=1'b0;
  DFFCE \strip_gaps[1]_0_s1  (
    .Q(\strip_gaps[1] [0]),
    .D(n1028_8),
    .CLK(TFT_clk_d),
    .CLEAR(n564_6),
    .CE(\strip_gaps[1]_3_8 ) 
);
defparam \strip_gaps[1]_0_s1 .INIT=1'b0;
  DFFCE \strip_gaps[2]_3_s1  (
    .Q(\strip_gaps[2] [3]),
    .D(n1029_7),
    .CLK(TFT_clk_d),
    .CLEAR(n564_6),
    .CE(\strip_gaps[2]_3_8 ) 
);
defparam \strip_gaps[2]_3_s1 .INIT=1'b0;
  DFFCE \strip_gaps[2]_2_s1  (
    .Q(\strip_gaps[2] [2]),
    .D(n1030_7),
    .CLK(TFT_clk_d),
    .CLEAR(n564_6),
    .CE(\strip_gaps[2]_3_8 ) 
);
defparam \strip_gaps[2]_2_s1 .INIT=1'b0;
  DFFCE \strip_gaps[2]_1_s1  (
    .Q(\strip_gaps[2] [1]),
    .D(n1031_7),
    .CLK(TFT_clk_d),
    .CLEAR(n564_6),
    .CE(\strip_gaps[2]_3_8 ) 
);
defparam \strip_gaps[2]_1_s1 .INIT=1'b0;
  DFFCE \strip_gaps[2]_0_s1  (
    .Q(\strip_gaps[2] [0]),
    .D(n1032_8),
    .CLK(TFT_clk_d),
    .CLEAR(n564_6),
    .CE(\strip_gaps[2]_3_8 ) 
);
defparam \strip_gaps[2]_0_s1 .INIT=1'b0;
  DFFCE strip_have_1_s7 (
    .Q(strip_have[1]),
    .D(n15_13),
    .CLK(TFT_clk_d),
    .CLEAR(n564_6),
    .CE(VCC) 
);
defparam strip_have_1_s7.INIT=1'b0;
  DFFCE strip_have_0_s7 (
    .Q(strip_have[0]),
    .D(n15_17),
    .CLK(TFT_clk_d),
    .CLEAR(n564_6),
    .CE(VCC) 
);
defparam strip_have_0_s7.INIT=1'b0;
  DFFCE longitudinal_rows_5_s6 (
    .Q(longitudinal_rows[5]),
    .D(n1033_10),
    .CLK(TFT_clk_d),
    .CLEAR(n564_6),
    .CE(VCC) 
);
defparam longitudinal_rows_5_s6.INIT=1'b0;
  DFFCE longitudinal_s5 (
    .Q(longitudinal),
    .D(n15_20),
    .CLK(TFT_clk_d),
    .CLEAR(n564_6),
    .CE(VCC) 
);
defparam longitudinal_s5.INIT=1'b0;
  DFFCE \strip_bands[0]_0_s3  (
    .Q(\strip_bands[0] [0]),
    .D(n1012_15),
    .CLK(TFT_clk_d),
    .CLEAR(n564_6),
    .CE(VCC) 
);
defparam \strip_bands[0]_0_s3 .INIT=1'b0;
  DFFCE \strip_bands[1]_0_s3  (
    .Q(\strip_bands[1] [0]),
    .D(n1016_15),
    .CLK(TFT_clk_d),
    .CLEAR(n564_6),
    .CE(VCC) 
);
defparam \strip_bands[1]_0_s3 .INIT=1'b0;
  DFFCE \strip_bands[2]_0_s3  (
    .Q(\strip_bands[2] [0]),
    .D(n1020_15),
    .CLK(TFT_clk_d),
    .CLEAR(n564_6),
    .CE(VCC) 
);
defparam \strip_bands[2]_0_s3 .INIT=1'b0;
  DFFCE strip_have_2_s5 (
    .Q(strip_have[2]),
    .D(n15_23),
    .CLK(TFT_clk_d),
    .CLEAR(n564_6),
    .CE(VCC) 
);
defparam strip_have_2_s5.INIT=1'b0;
  VCC VCC_cZ (
    .V(VCC)
);
  GND GND_cZ (
    .G(GND)
);
endmodule
