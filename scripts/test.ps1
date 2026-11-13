param([ValidateSet('Debug','Release')][string]$Configuration='Debug')
$ErrorActionPreference='Stop'
$root=Split-Path $PSScriptRoot -Parent
Push-Location $root
try {
    & "./x64/$Configuration/C2.Tests.exe"
    if($LASTEXITCODE){throw 'Binary ICD unit tests failed.'}
    python ./C2.Tests/Integration/test_observation_retention.py --console "./x64/$Configuration/C2.Console.exe"
    if($LASTEXITCODE){throw 'Observation retention scenarios failed.'}
    python ./C2.Tests/Integration/test_lidar_interlock.py --console "./x64/$Configuration/C2.Console.exe"
    if($LASTEXITCODE){throw 'Lidar optical interlock scenarios failed.'}
    python ./C2.Tests/Integration/test_camera_settings.py --console "./x64/$Configuration/C2.Console.exe"
    if($LASTEXITCODE){throw 'Camera settings persistence scenarios failed.'}
    python ./C2.Tests/Integration/test_installation_persistence.py --console "./x64/$Configuration/C2.Console.exe"
    if($LASTEXITCODE){throw 'Installation position persistence checks failed.'}
    python ./C2.Tests/Integration/test_rdr_recording_analysis.py
    if($LASTEXITCODE){throw 'Coordinate recording statistics checks failed.'}
    python ./C2.Tests/Integration/test_rdr_live_diagnostics.py --console "./x64/$Configuration/C2.Console.exe"
    if($LASTEXITCODE){throw 'Read-only lidar diagnostic scenarios failed.'}
    python ./C2.Tests/Integration/test_rdr_live_diagnostics.py --console "./x64/$Configuration/C2.Console.exe" --filtered
    if($LASTEXITCODE){throw 'GUI Kalman filter scenarios failed.'}
    python ./C2.Tests/Integration/test_rdr_live_diagnostics.py --console "./x64/$Configuration/C2.Console.exe" --filtered --adaptive
    if($LASTEXITCODE){throw 'Adaptive GUI Kalman filter scenarios failed.'}
    python ./C2.Tests/Integration/test_rdr_live_diagnostics.py --console "./x64/$Configuration/C2.Console.exe" --filtered --adaptive --axes
    if($LASTEXITCODE){throw 'Independent XYZ GUI Kalman filter scenarios failed.'}
    python ./C2.Tests/Integration/test_station.py --console "./x64/$Configuration/C2.Console.exe"
    if($LASTEXITCODE){throw 'Binary UDP integration test failed.'}
    python ./C2.Tests/Integration/test_station.py --console "./x64/$Configuration/C2.Console.exe" --filtered
    if($LASTEXITCODE){throw 'Shared filtered GUI/FCS coordinate scenarios failed.'}
    python ./C2.Tests/Integration/test_station.py --console "./x64/$Configuration/C2.Console.exe" --jitter
    if($LASTEXITCODE){throw 'Assignment jitter and sustained movement scenarios failed.'}
    python ./C2.Tests/Integration/test_simulator.py --console "./x64/$Configuration/C2.Console.exe"
    if($LASTEXITCODE){throw 'Persistent simulator scenarios failed.'}
    python ./C2.Tests/Integration/test_target_dummy.py --console "./x64/$Configuration/C2.Console.exe"
    if($LASTEXITCODE){throw 'Custom target dummy scenarios failed.'}
    python ./C2.Tests/Integration/test_target_dummy.py --console "./x64/$Configuration/C2.Console.exe" --external-fcs
    if($LASTEXITCODE){throw 'Lidar-only dummy with external FCS scenarios failed.'}
} finally {Pop-Location}
