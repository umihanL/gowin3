param(
    [string]$Board='rdkx5',
    [string]$Python='python',
    [switch]$Deploy
)
$ErrorActionPreference='Stop'
if ($Board -notmatch '^[A-Za-z0-9_.@-]+$' -or $Board.StartsWith('-')) {
    throw '请输入 SSH 主机别名或地址，例如 rdkx5。'
}
$prefix='task_console_update_20261031'
& $Python (Join-Path $PSScriptRoot ($prefix+'_build.py'))
if ($LASTEXITCODE -ne 0) { throw '本地更新包生成失败，尚未连接 X5。' }
$archive=Join-Path $PSScriptRoot ($prefix+'.tar.gz')
if (-not $Deploy) {
    Write-Host "已生成本地完整更新包：$archive"
    Write-Host '没有连接板子。需要部署时，先按同目录说明停止板端任务，再执行本脚本并添加 -Deploy。'
    return
}
$stage='/root/rescue/.task-console-upload-'+[Guid]::NewGuid().ToString('N')
& ssh -o BatchMode=yes -o ConnectTimeout=8 $Board 'test -d /root/rescue/ros2_ws/tools && test -x /root/rescue/ros2_ws/.venv/bin/python'
if ($LASTEXITCODE -ne 0) { throw '连接/现有工作区检查失败，活动程序未更新。' }
& ssh $Board "mkdir -p '$stage'"
if ($LASTEXITCODE -ne 0) { throw '无法创建上传暂存目录。' }
& scp $archive "${Board}:$stage/update.tar.gz"
if ($LASTEXITCODE -ne 0) { throw '上传失败，活动程序未更新。' }
$expected=(Get-FileHash -LiteralPath $archive -Algorithm SHA256).Hash.ToLowerInvariant()
$answer=& ssh $Board "sha256sum '$stage/update.tar.gz'"
if ($LASTEXITCODE -ne 0 -or ($answer -split '\s+')[0] -ne $expected) {
    throw '更新包校验失败，活动程序未更新。'
}
& ssh $Board "tar -xzf '$stage/update.tar.gz' -C '$stage' && python3 '$stage/install_update.py' --workspace /root/rescue/ros2_ws"
if ($LASTEXITCODE -ne 0) {
    throw '板端预检查未通过，活动文件未更新。请阅读输出，先停止相关服务/手动任务，不要重复启动节点。'
}
& ssh $Board "python3 '$stage/install_update.py' --workspace /root/rescue/ros2_ws --apply"
if ($LASTEXITCODE -ne 0) {
    throw '更新或板端构建失败，服务保持停止。请根据输出检查备份目录和 build.log。'
}
Write-Host '完整源码已更新并在 X5 构建；未启动服务、未修改 UART/电机/水泵启用配置。'
Write-Host '先核对原有板端配置，再按 task_console_update_20261031_说明.md 启动并检查。'
