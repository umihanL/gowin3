$ErrorActionPreference='Stop'
$projectRoot=Split-Path -Parent $PSScriptRoot
$application=Join-Path $projectRoot 'desktop/dist/消防救援演示台.exe'
if (-not (Test-Path -LiteralPath $application -PathType Leaf)) {
    throw '缺少上位机程序，请先运行 desktop/build-app.ps1。'
}
# Keep this historical filename so existing shortcuts open the live client.
Write-Host '打开实车控制上位机。在窗口中连接 X5 的 IP:8090；所有任务指令发往真实小车。'
Start-Process -FilePath $application -WorkingDirectory (Split-Path -Parent $application) -WindowStyle Normal
