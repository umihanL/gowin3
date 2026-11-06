param([Parameter(Mandatory=$true)][string]$Address)
$ErrorActionPreference = 'Stop'
$parsed = $null
if (-not [System.Net.IPAddress]::TryParse($Address, [ref]$parsed) -or
    $parsed.AddressFamily -ne [System.Net.Sockets.AddressFamily]::InterNetwork) {
    throw 'Enter the X5 IPv4 address, without http:// or a port.'
}
foreach ($port in @(22, 8090, 8086)) {
    $client = [System.Net.Sockets.TcpClient]::new()
    try {
        $pending = $client.ConnectAsync($Address, $port)
        if (-not $pending.Wait(2000) -or -not $client.Connected) {
            Write-Output "${Address}:$port unreachable"
        } else {
            Write-Output "${Address}:$port reachable"
        }
    } catch { Write-Output "${Address}:$port unreachable" }
    finally { $client.Dispose() }
}
Write-Output "Desktop console address: ${Address}:8090"
Write-Output 'This checks TCP reachability only; it does not execute robot commands.'
