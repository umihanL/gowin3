@echo off
powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%~dp0sync_rdkx5_time.ps1"
if errorlevel 1 echo Clock check failed. VS Code will still open; verify the X5 date before downloading extensions.
call "D:\Microsoft VS Code\bin\code.cmd" --new-window --folder-uri vscode-remote://ssh-remote+rdkx5/root/rescue/ros2_ws
