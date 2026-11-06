# Compatibility entry point. A three-file UI copy is incomplete for this release.
param([string]$Board='rdkx5', [string]$Python='python', [switch]$Deploy)
$ErrorActionPreference='Stop'
& (Join-Path $PSScriptRoot 'task_console_update_20261031.ps1') -Board $Board -Python $Python -Deploy:$Deploy
