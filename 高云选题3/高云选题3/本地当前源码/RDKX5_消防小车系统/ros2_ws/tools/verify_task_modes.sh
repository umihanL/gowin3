#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "$0")/.."
export ROS_DOMAIN_ID=83 ROS_LOCALHOST_ONLY=1 RESCUE_RUN_ROS_TEST=1
source ./env.sh
python -m compileall -q src tools/task_command.py
python -c 'import sys,unittest;sys.path.insert(0,"tests");names=["test_vision_task_gate","test_task_drive","test_crosswalk","test_traffic","test_fire_targets","test_fire_aim_geometry","test_fire_aim_compensation","test_task_interaction","test_task_scene","test_voice","test_interaction","test_local_actuator_guard","test_local_actuator_io","test_fast_task_gate_board","test_fast_mipi_board","test_road_vision_board","test_task_mission_board","test_selected_pump_board","test_task_dialogue_board","test_fire_event_rate_board"];r=unittest.TextTestRunner(verbosity=2).run(unittest.defaultTestLoader.loadTestsFromNames(names));sys.exit(not r.wasSuccessful())'
