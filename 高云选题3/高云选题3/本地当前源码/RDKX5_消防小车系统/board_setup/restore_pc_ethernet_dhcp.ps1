# Run elevated only when deliberately returning the dedicated Ethernet adapter to DHCP.
$ErrorActionPreference = 'Stop'
$adapter = @(Get-NetAdapter | Where-Object { $_.InterfaceGuid -eq '{71BE5E92-F39D-4BB5-B6C7-DADD61391FD4}' })
if ($adapter.Count -ne 1) { throw 'Original Ethernet adapter not found uniquely.' }
$before = Get-Content -LiteralPath (Join-Path $PSScriptRoot 'pc_ethernet_before.json') -Raw | ConvertFrom-Json
if ($before.Dhcp -ne 'Enabled') { throw 'Original adapter was not DHCP; review saved configuration before restoring.' }
Get-NetIPAddress -InterfaceIndex $adapter[0].ifIndex -AddressFamily IPv4 | Where-Object IPAddress -eq '192.168.127.1' | Remove-NetIPAddress -Confirm:$false
Set-NetIPInterface -InterfaceIndex $adapter[0].ifIndex -AddressFamily IPv4 -Dhcp Enabled
Write-Output 'Dedicated Ethernet restored to DHCP. Wi-Fi was not modified.'
