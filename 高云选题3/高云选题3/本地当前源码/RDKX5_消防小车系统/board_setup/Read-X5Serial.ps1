param(
    [string]$Port = 'COM36',
    [Parameter(Mandatory=$true)][string]$Command,
    [string]$Report,
    [ValidateRange(1,60)][int]$TimeoutSeconds = 20
)
$ErrorActionPreference = 'Stop'
# For the verified X5 debug console, never the FPGA or motor serial port.
# MobaXterm must release this console first. Opening does not assert DTR/RTS.
$console = [System.IO.Ports.SerialPort]::new($Port,115200,[System.IO.Ports.Parity]::None,8,[System.IO.Ports.StopBits]::One)
$console.Handshake = [System.IO.Ports.Handshake]::None
$console.DtrEnable = $false
$console.RtsEnable = $false
$console.Encoding = [System.Text.UTF8Encoding]::new($false)
$console.WriteTimeout = 3000
$buffer = ''
try {
    $console.Open()
    $console.Write("`r")
    $timer = [Diagnostics.Stopwatch]::StartNew()
    while ($timer.Elapsed.TotalSeconds -lt 3) {
        $buffer += $console.ReadExisting()
        if ($buffer -match 'root@ubuntu:[^\r\n]*#') { break }
        Start-Sleep -Milliseconds 80
    }
    if ($buffer -notmatch 'root@ubuntu:[^\r\n]*#') {
        throw "No authenticated X5 root shell observed; no command sent. Console: $buffer"
    }
    if ($Command.Contains("`r") -or $Command.Contains("`n")) { throw 'Use a single shell command line.' }
    $marker = 'RESCUE_READ_DONE_' + [Guid]::NewGuid().ToString('N')
    $console.DiscardInBuffer()
    $console.Write($Command + "; printf '\n" + $marker + "\n'`r")
    $buffer = ''
    $timer.Restart()
    while ($timer.Elapsed.TotalSeconds -lt $TimeoutSeconds) {
        $buffer += $console.ReadExisting()
        if ($buffer -match "[\r\n]$marker[\r\n]") { break }
        Start-Sleep -Milliseconds 80
    }
    if ($Report) { [IO.File]::WriteAllText($Report,$buffer,[Text.UTF8Encoding]::new($false)) }
    Write-Output $buffer
    if ($buffer -notmatch "[\r\n]$marker[\r\n]") { throw 'Console response timed out; inspect before retrying.' }
} finally {
    if ($console.IsOpen) { $console.Close() }
    $console.Dispose()
}
