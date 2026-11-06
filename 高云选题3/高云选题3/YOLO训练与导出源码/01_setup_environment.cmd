@echo off
chcp 65001 >nul
cd /d "%~dp0"
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0scripts\setup_environment.ps1" %*
set "TASK_EXIT=%errorlevel%"
pause
exit /b %TASK_EXIT%
