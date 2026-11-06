param(
    [string]$Gowin = '',
    [string]$Python = '',
    [string]$WorkRoot = 'D:\FPGA\codex_builds'
)

$ErrorActionPreference = 'Stop'
if (-not $Gowin) {
    $candidates = @(
        'D:\FPGA\Gowin\Gowin_V1.9.12.03_x64\IDE\bin\gw_sh.exe',
        'C:\Gowin\Gowin_V1.9.12.03_x64\IDE\bin\gw_sh.exe'
    )
    $Gowin = $candidates | Where-Object { Test-Path -LiteralPath $_ -PathType Leaf } | Select-Object -First 1
}
if (-not $Gowin) { throw 'Gowin not found. Pass -Gowin with the path to gw_sh.exe.' }
if (-not (Test-Path -LiteralPath $Gowin -PathType Leaf)) {
    throw "Gowin not found: $Gowin. Pass -Gowin with the path to gw_sh.exe."
}

if (-not $Python) {
    $Python = @('D:\Program Files\Python310\python.exe', 'C:\Python310\python.exe') |
        Where-Object { Test-Path -LiteralPath $_ -PathType Leaf } | Select-Object -First 1
    if (-not $Python) {
        $command = Get-Command python -ErrorAction SilentlyContinue
        if ($command) { $Python = $command.Source }
    }
}
if (-not $Python) { throw 'Python 3.10+ not found. Pass -Python with its executable path.' }

# Gowin corrupts generated project paths containing Chinese characters.
# The helper freezes sources in an ASCII directory and verifies all outputs
# before replacing the project artifacts; the prior artifacts are backed up.
& $Python (Join-Path $PSScriptRoot 'tools\build_project.py') --project $PSScriptRoot --gowin $Gowin --work-root $WorkRoot
if ($LASTEXITCODE -ne 0) { throw 'Build or validation failed. See the ASCII workspace/log path printed above.' }
