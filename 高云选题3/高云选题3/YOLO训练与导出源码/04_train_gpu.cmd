@echo off
chcp 65001 >nul
cd /d "%~dp0"
if not exist "%~dp0.venv\Scripts\python.exe" (
  echo Run 01_setup_environment.cmd with CUDA first.
  pause
  exit /b 1
)
"%~dp0.venv\Scripts\python.exe" "%~dp0tools\check_environment.py" --require-training --quiet
if errorlevel 1 (
  pause
  exit /b 1
)
echo Starting full training with local weights\yolo11n.pt when available. These are COCO initialization weights, not a trained fire detector.
"%~dp0.venv\Scripts\python.exe" "%~dp0train.py" --mode train --device 0 %*
set "TASK_EXIT=%errorlevel%"
pause
exit /b %TASK_EXIT%
