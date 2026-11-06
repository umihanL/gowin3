#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "$0")/.."
# All generated images and fake actuator tests stay outside the live domain.
export ROS_DOMAIN_ID=83 ROS_LOCALHOST_ONLY=1 RESCUE_RUN_ROS_TEST=1
source ./env.sh
python -m compileall -q src/rescue_vision/rescue_vision
python -c 'import sys,unittest; sys.path.insert(0,"tests"); names=["test_stereo","test_stereo_detection_geometry","test_depth_frame_join","test_nozzle_geometry","test_fire_aim_geometry","test_fire_aim_compensation","test_local_actuator_guard","test_local_actuator_io","test_stereo_depth_board","test_fast_mipi_board","test_local_actuator_board"]; result=unittest.TextTestRunner(verbosity=2).run(unittest.defaultTestLoader.loadTestsFromNames(names)); sys.exit(not result.wasSuccessful())'
