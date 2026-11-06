@echo off
chcp 65001 >nul
cd /d "%~dp0"
set "PYTHONUTF8=1"
if "%~1"=="" (
  echo No camera will open without explicit device, format and capture parameters.
  echo Read real_capture_validation/采集说明.md first.
  "%~dp0.venv\Scripts\python.exe" tools/capture_real_validation.py --help
  pause
  exit /b 2
)
"%~dp0.venv\Scripts\python.exe" tools/capture_real_validation.py %*
exit /b %errorlevel%
