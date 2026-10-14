param([string]$BinDir = 'build/live/Release')
$ErrorActionPreference = 'Stop'
Push-Location (Split-Path $PSScriptRoot -Parent)
try {
    $c2PackageDir = Join-Path (Get-Location) 'dist/windows'
    New-Item -ItemType Directory -Force $c2PackageDir | Out-Null
    foreach ($c2Executable in @('c2_dashboard.exe', 'c2_server.exe', 'dummy_observation.exe', 'dummy_effector.exe')) {
        $c2Source = Join-Path $BinDir $c2Executable
        if (-not (Test-Path -LiteralPath $c2Source)) { throw "Build first: missing $c2Source" }
        Copy-Item -LiteralPath $c2Source -Destination $c2PackageDir -Force
    }
    Copy-Item -LiteralPath docs/asset-integration.md -Destination $c2PackageDir -Force
    Copy-Item -LiteralPath external/warwarden-protocol/proto/mfs/icd/v3/mfs.proto -Destination $c2PackageDir -Force
    Copy-Item -LiteralPath runtime/LICENSE -Destination (Join-Path $c2PackageDir 'LICENSE-runtime.txt') -Force
    Copy-Item -LiteralPath external/warwarden-protocol/LICENSE -Destination (Join-Path $c2PackageDir 'LICENSE-protocol.txt') -Force
    $c2Revision = git rev-parse HEAD
    @"
AI Generated C2 / Windows x64 / revision $c2Revision

Run from PowerShell in this extracted folder:
  .\c2_dashboard.exe --bind 0.0.0.0 --asset-port 5000

Start peers in separate terminals (replace 127.0.0.1 with C2 PC IPv4 for remote peers):
  .\dummy_observation.exe --asset-id 101 --listen-port 5101 --c2-address 127.0.0.1 --c2-port 5000
  .\dummy_effector.exe --asset-id 201 --listen-port 6001 --c2-address 127.0.0.1 --c2-port 5000

Read asset-integration.md before connecting a real board. mfs.proto is the pinned wire contract.
May require the Microsoft Visual C++ x64 runtime installed with Visual Studio or its redistributable.
No firewall configuration is changed by this package.
"@ | Set-Content -LiteralPath (Join-Path $c2PackageDir 'RUNME.txt') -Encoding utf8
    $c2PackageFiles = @(Get-ChildItem -LiteralPath $c2PackageDir -File | ForEach-Object FullName)
    Compress-Archive -LiteralPath $c2PackageFiles -DestinationPath dist/ai-generated-c2-windows-x64.zip -Force
    Write-Output (Join-Path (Get-Location) 'dist/ai-generated-c2-windows-x64.zip')
} finally { Pop-Location }
