@echo off
chcp 65001 >nul
cd /d "%~dp0"
if not exist "%~dp0.venv\Scripts\python.exe" (
  echo Run 01_setup_environment.cmd with CPU or CUDA first.
  pause
  exit /b 1
)
"%~dp0.venv\Scripts\python.exe" "%~dp0tools\check_environment.py" --require-training --quiet
if errorlevel 1 (
  pause
  exit /b 1
)
"%~dp0.venv\Scripts\python.exe" "%~dp0train.py" --mode train --smoke --from-scratch --device cpu %*
set "TASK_EXIT=%errorlevel%"
pause
exit /b %TASK_EXIT%
