@echo off
chcp 65001 >nul
cd /d "%~dp0"
set "PYTHONUTF8=1"
set "PYTHONUNBUFFERED=1"
"%~dp0.venv\Scripts\python.exe" "%~dp0tools\check_environment.py" --require-training --require-cuda --quiet
if errorlevel 1 (
  pause
  exit /b 1
)
echo New fine-tune with 96 additional firefighter-scene captures. Existing jobs cannot be overwritten.
"%~dp0.venv\Scripts\python.exe" "%~dp0tools\run_training_job.py" --name wallpaper_yolo11n_firefighter_v4_20261008 --data dataset_firefighter_v4/data.yaml --model runs/wallpaper_yolo11n_observation_v3_20261008/paused_by_user_20261008_220602/weights/best.pt --config config/yolo11n_people_finetune.yaml --epochs 80 --patience 20 --batch 8
set "FIREFIGHTER_TRAIN_EXIT=%errorlevel%"
pause
exit /b %FIREFIGHTER_TRAIN_EXIT%
