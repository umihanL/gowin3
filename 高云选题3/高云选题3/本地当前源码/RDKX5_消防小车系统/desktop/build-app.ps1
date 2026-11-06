param([switch]$Check, [string]$OutputDirectory)
$ErrorActionPreference = 'Stop'
$taskDesktop = $PSScriptRoot
$version = '1.0.4258.31'
$sdkDigest = '56F7F4B8BF9AEE4B8EFEFBBDD4F67D5F74EBD1B100ED0806DA71BF76AF481AA9'
$sdkUrl = "https://api.nuget.org/v3-flatcontainer/microsoft.web.webview2/$version/microsoft.web.webview2.$version.nupkg"
$package = Join-Path $taskDesktop "vendor\microsoft.web.webview2.$version.nupkg"
$sdk = Join-Path $taskDesktop "vendor\webview2-$version"
$dist = if ($OutputDirectory) { [System.IO.Path]::GetFullPath($OutputDirectory) } else { Join-Path $taskDesktop 'dist' }
$compiler = Join-Path $env:WINDIR 'Microsoft.NET\Framework64\v4.0.30319\csc.exe'
New-Item -ItemType Directory -Force -Path (Join-Path $taskDesktop 'vendor'), $dist | Out-Null
if (!(Test-Path -LiteralPath $package)) {
    $ProgressPreference = 'SilentlyContinue'
    Invoke-WebRequest -Uri $sdkUrl -OutFile $package
}
if ((Get-FileHash -LiteralPath $package -Algorithm SHA256).Hash -ne $sdkDigest) { throw 'WebView2 SDK SHA256 mismatch.' }
if (!(Test-Path -LiteralPath $sdk)) {
    Add-Type -AssemblyName System.IO.Compression.FileSystem
    [System.IO.Compression.ZipFile]::ExtractToDirectory($package, $sdk)
}
if (!(Test-Path -LiteralPath $compiler)) { throw '.NET Framework C# compiler is unavailable.' }
$core = Join-Path $sdk 'lib\net462\Microsoft.Web.WebView2.Core.dll'
$forms = Join-Path $sdk 'lib\net462\Microsoft.Web.WebView2.WinForms.dll'
$output = Join-Path $dist '消防救援演示台.exe'
$arguments = @('/nologo', '/target:winexe', '/platform:x64', '/optimize+', '/utf8output',
    "/out:$output", "/win32manifest:$(Join-Path $taskDesktop 'src\app.manifest')",
    "/win32icon:$(Join-Path $taskDesktop 'src\app.ico')",
    '/reference:System.dll', '/reference:System.Core.dll', '/reference:System.Windows.Forms.dll',
    '/reference:System.Drawing.dll', '/reference:System.Net.Http.dll', '/reference:System.Web.Extensions.dll',
    "/reference:$core", "/reference:$forms", (Join-Path $taskDesktop 'src\App.cs'), (Join-Path $taskDesktop 'src\AssemblyInfo.cs'))
& $compiler @arguments
if ($LASTEXITCODE -ne 0) { throw "C# build failed: $LASTEXITCODE" }
Copy-Item -LiteralPath $core, $forms -Destination $dist -Force
Copy-Item -LiteralPath (Join-Path $sdk 'runtimes\win-x64\native\WebView2Loader.dll') -Destination $dist -Force
Copy-Item -LiteralPath (Join-Path $sdk 'LICENSE.txt') -Destination (Join-Path $dist 'WebView2-LICENSE.txt') -Force
Copy-Item -LiteralPath (Join-Path $sdk 'NOTICE.txt') -Destination (Join-Path $dist 'WebView2-NOTICE.txt') -Force
$runtimeConfig = @'
<?xml version="1.0" encoding="utf-8"?>
<configuration><startup><supportedRuntime version="v4.0" sku=".NETFramework,Version=v4.6.2"/></startup><runtime><AppContextSwitchOverrides value="Switch.System.Windows.Forms.DoNotSupportSelectAllShortcutInMultilineTextBox=false"/></runtime></configuration>
'@
[System.IO.File]::WriteAllText("$output.config", $runtimeConfig, [System.Text.UTF8Encoding]::new($false))
$record = [ordered]@{
    product = 'TreeBird Rescue Control'; built_at = (Get-Date -Format o); platform = 'Windows x64';
    app_sha256 = (Get-FileHash -LiteralPath $output -Algorithm SHA256).Hash;
    compiler = $compiler; webview2_sdk = $version; sdk_url = $sdkUrl; sdk_sha256 = $sdkDigest;
    external_runtime = 'Microsoft Edge WebView2 Evergreen Runtime';
    embedded_backend = $false;
    privileges = 'asInvoker'; default_mode = 'live X5 connection';
    default_endpoint = 'http://192.168.127.10:8090'; startup_commands_sent = $false;
    credentials_persisted = $false
}
$record | ConvertTo-Json -Depth 4 | Set-Content -LiteralPath (Join-Path $dist 'BUILD.json') -Encoding UTF8
Write-Output "Built $output"
if ($Check) {
    $checkOutput = Join-Path $taskDesktop 'vendor\ShellChecks.exe'
    & $compiler '/nologo' '/target:exe' '/platform:x64' "/out:$checkOutput" (Join-Path $taskDesktop 'src\ShellChecks.cs')
    if ($LASTEXITCODE -ne 0) { throw 'Shell checks failed to compile.' }
    & $checkOutput $output
    if ($LASTEXITCODE -ne 0) { throw 'Shell endpoint checks failed.' }
}
