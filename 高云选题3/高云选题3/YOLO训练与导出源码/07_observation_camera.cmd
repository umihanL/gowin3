@echo off
chcp 65001 >nul
cd /d "%~dp0"
set "PYTHONUTF8=1"
set "PYTHONUNBUFFERED=1"
echo This opens camera 0 with saved YOLO weights. It does not start training or drive actuators.
echo Without fresh vehicle telemetry it displays detections and remains in observation hold.
echo See OBSERVATION_GUIDE.md for GS130W SBS, calibrated depth and telemetry options.
set "OBS_RUN=%RANDOM%_%RANDOM%"
"%~dp0.venv\Scripts\python.exe" runtime/run_observation.py --source 0 --device cpu --display --output "runs/observation_camera_%OBS_RUN%"
pause
