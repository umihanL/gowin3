@echo off
chcp 65001 >nul
cd /d "%~dp0"
set "PYTHONUTF8=1"
set "PYTHONUNBUFFERED=1"
if not exist "%~dp0.venv\Scripts\python.exe" (
  echo Run 01_setup_environment.cmd with CUDA first.
  pause
  exit /b 1
)
if not exist "%~dp0dataset_people_v2\independent_validation.json" (
  echo Expanded dataset is not validated yet. Read TRAINING_STATUS.md.
  pause
  exit /b 1
)
"%~dp0.venv\Scripts\python.exe" "%~dp0tools\check_environment.py" --require-training --require-cuda --quiet
if errorlevel 1 (
  pause
  exit /b 1
)
echo New fine-tune from the preserved best model. Existing job names cannot be overwritten or launched twice.
"%~dp0.venv\Scripts\python.exe" "%~dp0tools\run_training_job.py" --name wallpaper_yolo11n_people_v2_20261008 --data dataset_people_v2/data.yaml --model runs/wallpaper_yolo11n_gpu_20261008/stopped_for_data_expansion/best.pt --config config/yolo11n_people_finetune.yaml --epochs 80 --patience 20 --batch 8
set "TASK_EXIT=%errorlevel%"
pause
exit /b %TASK_EXIT%
