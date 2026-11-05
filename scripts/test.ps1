param([ValidateSet('Debug','Release')][string]$Configuration='Debug')
$ErrorActionPreference='Stop'
$root=Split-Path $PSScriptRoot -Parent
Push-Location $root
try {
    & "./x64/$Configuration/C2.Tests.exe"
    if($LASTEXITCODE){throw 'Binary ICD unit tests failed.'}
    python ./C2.Tests/Integration/test_rdr_live_diagnostics.py --console "./x64/$Configuration/C2.Console.exe"
    if($LASTEXITCODE){throw 'Read-only lidar diagnostic scenarios failed.'}
    python ./C2.Tests/Integration/test_station.py --console "./x64/$Configuration/C2.Console.exe"
    if($LASTEXITCODE){throw 'Binary UDP integration test failed.'}
    python ./C2.Tests/Integration/test_simulator.py --console "./x64/$Configuration/C2.Console.exe"
    if($LASTEXITCODE){throw 'Persistent simulator scenarios failed.'}
    python ./C2.Tests/Integration/test_target_dummy.py --console "./x64/$Configuration/C2.Console.exe"
    if($LASTEXITCODE){throw 'Custom target dummy scenarios failed.'}
    python ./C2.Tests/Integration/test_target_dummy.py --console "./x64/$Configuration/C2.Console.exe" --external-fcs
    if($LASTEXITCODE){throw 'Lidar-only dummy with external FCS scenarios failed.'}
} finally {Pop-Location}
