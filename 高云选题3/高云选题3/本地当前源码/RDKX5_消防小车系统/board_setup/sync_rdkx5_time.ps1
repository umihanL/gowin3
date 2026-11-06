param([string]$SshTarget = 'rdkx5')

$ErrorActionPreference = 'Stop'
try {
    $x5Output = & ssh -T -o BatchMode=yes -o ConnectTimeout=6 $SshTarget 'date +%s'
    if ($LASTEXITCODE -ne 0) { throw 'Cannot read the board clock over SSH.' }
    $x5Epoch = [long](($x5Output -join '').Trim())
    $pcEpoch = [DateTimeOffset]::UtcNow.ToUnixTimeSeconds()
    $deltaSeconds = [Math]::Abs($pcEpoch - $x5Epoch)
    if ($deltaSeconds -le 10) {
        Write-Host "RDK X5 clock OK (difference ${deltaSeconds}s)."
        exit 0
    }

    # Use the PC's current UTC time. Keep TLS certificate verification enabled.
    # Only adjust a clock with a material offset, before opening development tools.
    $pcEpoch = [DateTimeOffset]::UtcNow.ToUnixTimeSeconds()
    $remoteCommand = "date -u -s '@$pcEpoch' && hwclock --systohc --utc && date -Is"
    & ssh -T -o BatchMode=yes -o ConnectTimeout=6 $SshTarget $remoteCommand
    if ($LASTEXITCODE -ne 0) { throw 'Clock or RTC update failed; see the SSH output.' }

    $x5Output = & ssh -T -o BatchMode=yes -o ConnectTimeout=6 $SshTarget 'date +%s'
    if ($LASTEXITCODE -ne 0) { throw 'Cannot verify the updated board clock.' }
    $x5Epoch = [long](($x5Output -join '').Trim())
    $deltaSeconds = [Math]::Abs([DateTimeOffset]::UtcNow.ToUnixTimeSeconds() - $x5Epoch)
    if ($deltaSeconds -gt 10) { throw "Clock verification failed: ${deltaSeconds}s difference." }
    Write-Host "RDK X5 clock synchronized from this PC; RTC written (difference ${deltaSeconds}s)."
    exit 0
} catch {
    Write-Host "RDK X5 clock check failed: $($_.Exception.Message)" -ForegroundColor Yellow
    exit 1
}
