$ErrorActionPreference = 'Stop'
$setupPath = $PSScriptRoot
try {
  $adapter = Get-NetAdapter | Where-Object { $_.InterfaceGuid -eq '{71BE5E92-F39D-4BB5-B6C7-DADD61391FD4}' }
  if (@($adapter).Count -ne 1 -or $adapter.InterfaceDescription -ne 'Realtek PCIe GbE Family Controller') { throw 'Expected wired adapter was not found uniquely.' }
  $idx = $adapter.ifIndex
  $current = @(Get-NetIPAddress -InterfaceIndex $idx -AddressFamily IPv4)
  if ($current | Where-Object { $_.IPAddress -notlike '169.254.*' -and $_.IPAddress -ne '192.168.127.1' }) { throw 'Unexpected existing static IP; stopped without replacing it.' }
  $backupPath = Join-Path $setupPath 'pc_ethernet_before.json'
  if (-not (Test-Path -LiteralPath $backupPath)) {
    [pscustomobject]@{Time=(Get-Date -Format o);Adapter=$adapter.Name;Guid=$adapter.InterfaceGuid;InterfaceIndex=$idx;Dhcp=(Get-NetIPInterface -InterfaceIndex $idx -AddressFamily IPv4).Dhcp.ToString();Addresses=@($current|Select-Object IPAddress,PrefixLength);Routes=@(Get-NetRoute -InterfaceIndex $idx -AddressFamily IPv4|Select-Object DestinationPrefix,NextHop,RouteMetric)} | ConvertTo-Json -Depth 6 | Set-Content -LiteralPath $backupPath -Encoding UTF8
  }
  if (-not ($current | Where-Object IPAddress -eq '192.168.127.1')) {
    New-NetIPAddress -InterfaceIndex $idx -IPAddress '192.168.127.1' -PrefixLength 24 -AddressFamily IPv4 -ErrorAction Stop | Out-Null
  }
  [pscustomobject]@{Success=$true;Adapter=$adapter.Name;IPv4=@(Get-NetIPAddress -InterfaceIndex $idx -AddressFamily IPv4|Select-Object IPAddress,PrefixLength,AddressState)} | ConvertTo-Json -Depth 5 | Set-Content -LiteralPath (Join-Path $setupPath 'pc_ethernet_result.json') -Encoding UTF8
} catch {
  [pscustomobject]@{Success=$false;Error=$_.Exception.Message} | ConvertTo-Json | Set-Content -LiteralPath (Join-Path $setupPath 'pc_ethernet_result.json') -Encoding UTF8
  exit 1
}
