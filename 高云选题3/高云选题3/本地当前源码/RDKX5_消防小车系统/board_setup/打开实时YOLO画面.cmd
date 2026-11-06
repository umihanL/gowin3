@echo off
powershell.exe -NoProfile -Command "$saved = Join-Path $env:LOCALAPPDATA 'TreeBird\RescuePresentation\connection.txt'; $address = '10.250.137.126'; if (Test-Path -LiteralPath $saved) { $value = (Get-Content -LiteralPath $saved -Raw).Trim(); if ($value -match '^(?:http://)?((?:\d{1,3}\.){3}\d{1,3}):8090/?$') { $address = $Matches[1] } }; Start-Process ('http://' + $address + ':8086/')"
