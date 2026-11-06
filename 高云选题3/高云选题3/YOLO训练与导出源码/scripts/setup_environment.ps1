param(
    [ValidateSet('Ask','Check','CPU','CUDA','Existing')][string]$TorchMode = 'Ask',
    [string]$CudaWheelTag = '',
    [string]$PythonExe = ''
)
$ErrorActionPreference = 'Stop'
$ProjectRoot = Split-Path -Parent $PSScriptRoot
$VenvRoot = Join-Path $ProjectRoot '.venv'
$VenvPython = Join-Path $VenvRoot 'Scripts\python.exe'
$EnvironmentCheck = Join-Path $ProjectRoot 'tools\check_environment.py'

function Invoke-CheckedPython([string[]]$Arguments) {
    & $VenvPython @Arguments
    if ($LASTEXITCODE -ne 0) { throw "Python command failed with exit code $LASTEXITCODE" }
}

if (-not (Test-Path -LiteralPath $VenvPython)) {
    if ($PythonExe) {
        & $PythonExe -m venv $VenvRoot
    } elseif (Get-Command py -ErrorAction SilentlyContinue) {
        & py -3.10 -m venv $VenvRoot
        if ($LASTEXITCODE -ne 0) { & py -3.11 -m venv $VenvRoot }
        if ($LASTEXITCODE -ne 0) { & py -3.12 -m venv $VenvRoot }
    } else {
        & python -m venv $VenvRoot
    }
    if ($LASTEXITCODE -ne 0) { throw 'Create .venv failed. Install Python 3.10/3.11 or pass -PythonExe with its full path.' }
}
Invoke-CheckedPython @($EnvironmentCheck,'--require-python','--quiet')
Invoke-CheckedPython @('-m','pip','install','-r',(Join-Path $ProjectRoot 'requirements-check.txt'))
if ($TorchMode -eq 'Ask') {
    Write-Host 'Select installation: Check = small data-check environment; CPU = CPU training; CUDA = NVIDIA training; Existing = keep installed torch.'
    $TorchMode = Read-Host 'Type Check, CPU, CUDA, or Existing (default Check)'
    if (-not $TorchMode) { $TorchMode = 'Check' }
    if ($TorchMode -notin @('Check','CPU','CUDA','Existing')) { throw 'Unknown installation mode.' }
}
if ($TorchMode -eq 'CPU') {
    Invoke-CheckedPython @('-m','pip','install','torch','torchvision','--index-url','https://download.pytorch.org/whl/cpu')
} elseif ($TorchMode -eq 'CUDA') {
    Write-Host 'Choose a CUDA wheel tag supported by your NVIDIA driver and the current PyTorch release: https://pytorch.org/get-started/locally/'
    if (-not $CudaWheelTag) { $CudaWheelTag = Read-Host 'CUDA wheel tag from PyTorch selector, for example cu128 (not automatically selected)' }
    if ($CudaWheelTag -notmatch '^cu[0-9]{3}$') { throw 'Invalid CUDA wheel tag. Use the exact cuNNN tag offered by PyTorch.' }
    Invoke-CheckedPython @('-m','pip','install','torch','torchvision','--index-url',("https://download.pytorch.org/whl/" + $CudaWheelTag))
}
if ($TorchMode -ne 'Check') {
    Invoke-CheckedPython @($EnvironmentCheck,'--require-torch','--quiet')
    if ($TorchMode -eq 'CUDA') {
        Invoke-CheckedPython @($EnvironmentCheck,'--require-cuda','--quiet')
    }
    Invoke-CheckedPython @('-m','pip','install','-r',(Join-Path $ProjectRoot 'requirements-train.txt'))
    Invoke-CheckedPython @('-m','pip','check')
    Invoke-CheckedPython @($EnvironmentCheck,'--require-training','--quiet')
}
Invoke-CheckedPython @($EnvironmentCheck,'--output',(Join-Path $ProjectRoot 'environment.local.json'))
& $VenvPython -m pip freeze | Set-Content -LiteralPath (Join-Path $ProjectRoot 'requirements.installed.txt') -Encoding utf8
if ($LASTEXITCODE -ne 0) { throw 'Could not record installed package versions.' }
Write-Host 'Ready. Data check: .venv\Scripts\python.exe train.py --check-only'
Write-Host 'No model has been downloaded or trained by this setup script.'
