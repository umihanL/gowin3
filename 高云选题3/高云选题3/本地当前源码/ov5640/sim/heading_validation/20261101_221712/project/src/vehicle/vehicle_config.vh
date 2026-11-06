`ifndef VEHICLE_CONFIG_VH
`define VEHICLE_CONFIG_VH
// User body measurements and measured steering linkage, 2026-10-24.
`define VEHICLE_WHEELBASE_MM 142
`define VEHICLE_REAR_TRACK_MM 168
`define VEHICLE_FRONT_TRACK_MM 178
`define VEHICLE_WHEEL_WIDTH_MM 26
`define VEHICLE_BODY_WIDTH_MM 200
`define VEHICLE_WHEEL_DIAMETER_MM 66
`define VEHICLE_COUNTS_PER_REV 1560
// Nose is the map/touch reference; bicycle kinematics use the rear axle.
// 320 - 77 - 142 = 101 mm rear overhang; nose is 219 mm ahead of rear axle.
`define VEHICLE_BODY_LENGTH_MM 320
`define VEHICLE_REAR_OVERHANG_MM 101
`define VEHICLE_NOSE_FROM_REAR_MM 219
`define VEHICLE_FRONT_OVERHANG_MM 77
`define VEHICLE_FRONT_LASER_FROM_NOSE_MM 30
`define VEHICLE_FRONT_LASER_TRACK_MM 112
`define VEHICLE_SIDE_FROM_NOSE_MM 110
`define VEHICLE_SIDE_LASER_TRACK_MM 144
// Camera measured 2026-10-25: 185 mm above ground, 15 mm behind the nose,
// 102 mm from the right body edge (about +2 mm left of the vehicle centre).
// The screen-centre ground mark is 380 mm ahead of the nose.  The mechanical
// pitch remains the existing 30 degree setting; the IPM uses the measured
// centre mark as its effective vertical calibration anchor.
`define VEHICLE_CAMERA_HEIGHT_MM 185
`define VEHICLE_CAMERA_FROM_NOSE_MM 15
`define VEHICLE_CAMERA_LEFT_MM 2
`define VEHICLE_CAMERA_PITCH_TENTHS 300
`define VEHICLE_CAMERA_CENTRE_GROUND_MM 380
// Measured nonlinear linkage: servo 40..150, centre 90, wheels +/-28 deg.
`define VEHICLE_SERVO_CENTER_DEG 90
`define VEHICLE_SERVO_MIN_DEG 40
`define VEHICLE_SERVO_MAX_DEG 150
// Keep the measured asymmetric offsets; translate them around the live trim.
// All live angles use tenths of a degree. Absolute servo commands are 0..180.
`define VEHICLE_SERVO_RIGHT_OFFSET_TENTHS ((`VEHICLE_SERVO_CENTER_DEG-`VEHICLE_SERVO_MIN_DEG)*10)
`define VEHICLE_SERVO_LEFT_OFFSET_TENTHS ((`VEHICLE_SERVO_MAX_DEG-`VEHICLE_SERVO_CENTER_DEG)*10)
`define VEHICLE_SERVO_CENTER_MIN_TENTHS `VEHICLE_SERVO_RIGHT_OFFSET_TENTHS
`define VEHICLE_SERVO_CENTER_MAX_TENTHS (1800-`VEHICLE_SERVO_LEFT_OFFSET_TENTHS)
// Left/CCW is positive and increases the servo command on this installation.
`define VEHICLE_SERVO_SIGN 1
`define VEHICLE_LEFT_STEER_DEG 28
`define VEHICLE_RIGHT_STEER_DEG 28
`define VEHICLE_MAX_SPEED_MM_S 1000
`define VEHICLE_MOTOR1_IS_LEFT 1
`define VEHICLE_ENCODER1_INVERT 0
// M2 bridge polarity is reversed in vehicle_subsystem; reverse feedback too.
`define VEHICLE_ENCODER2_INVERT 0
`define VEHICLE_IMU_YAW_SIGN 1
// Planning uses the smaller measured steering limit for both directions.
// R=142/tan(28)=267.06 mm; inner rear wheel keeps the driving direction.
`define VEHICLE_MAX_STEER_DEG 28
`define VEHICLE_MIN_RADIUS_MM 268
`endif
