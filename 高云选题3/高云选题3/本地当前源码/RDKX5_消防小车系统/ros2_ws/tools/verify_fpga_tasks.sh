#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "$0")/.."
export ROS_DOMAIN_ID=83 ROS_LOCALHOST_ONLY=1 RESCUE_RUN_ROS_TEST=1
source ./env.sh
python -m compileall -q src/rescue_vision tools/task_command.py
python - <<'PY'
import sys, unittest
sys.path.insert(0,'tests')
names=['test_task_interaction','test_task_scene','test_voice','test_edge_dialogue',
       'test_mipi_building_ocr','test_ocr_semantics','test_ocr_workload','test_fpga_navigation','test_fpga_task_logic',
       'test_protocol','test_crosswalk','test_task_drive','test_vision_task_gate',
       'test_traffic','test_fire_targets','test_auto_fire','test_local_actuator_guard',
       'test_fpga_task_mission_board','test_auto_fire_board','test_selected_pump_board','test_task_dialogue_board']
result=unittest.TextTestRunner(verbosity=2).run(unittest.defaultTestLoader.loadTestsFromNames(names))
sys.exit(not result.wasSuccessful())
PY
