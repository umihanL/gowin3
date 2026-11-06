@echo off
chcp 65001 >nul
cd /d "%~dp0"
set "PYTHONUTF8=1"
set "PYTHONUNBUFFERED=1"
if not exist "%~dp0.venv\Scripts\python.exe" (
  echo Project Python environment is missing.
  pause
  exit /b 1
)
"%~dp0.venv\Scripts\python.exe" "%~dp0tools\check_environment.py" --require-training --require-cuda --quiet
if errorlevel 1 (
  pause
  exit /b 1
)
echo V8 merged data fine-tune. Existing run names cannot be overwritten or started twice.
"%~dp0.venv\Scripts\python.exe" "%~dp0tools\run_training_job.py" --name wallpaper_yolo11n_observation_v3_20261008 --data dataset_observation_v3/data.yaml --model runs/wallpaper_yolo11n_people_v2_20261008/paused_for_review_20261008_193528/weights/best.pt --config config/yolo11n_people_finetune.yaml --epochs 80 --patience 20 --batch 8
set "OBSERVATION_TRAIN_EXIT=%errorlevel%"
pause
exit /b %OBSERVATION_TRAIN_EXIT%
