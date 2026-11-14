param([switch]$NoGui)
$ErrorActionPreference = 'Stop'
$root = Split-Path $PSScriptRoot -Parent
$gui = Join-Path (Split-Path (Split-Path $root -Parent) -Parent) 'c2-GUI/bin/Debug/net10.0-windows/AadcCommandCenter.exe'
$ecs = Join-Path $root 'x64/Debug/C2.Console.exe'
$run = Join-Path $root 'logs/rdr-live-fcs-dummy'
$config = Join-Path $root 'configs/rdr-live-fcs-dummy.json'
if (Get-Process -Name C2.Console -ErrorAction SilentlyContinue) { throw 'ECS is already running. Stop it before launching this environment.' }
if (!(Test-Path -LiteralPath $ecs)) { throw 'Build ECS Debug x64 first.' }
if (!$NoGui -and !(Test-Path -LiteralPath $gui)) { throw 'Build projects/c2-GUI Debug first.' }
New-Item -ItemType Directory -Path $run -Force | Out-Null
if (!(Test-Path -LiteralPath $config)) {
    $settings = Get-Content (Join-Path $root 'configs/rdr-live.json') -Raw | ConvertFrom-Json
    $settings.fcs.local.ip = '127.0.0.1'; $settings.fcs.local.port = 0
    $settings.fcs.peer.ip = '127.0.0.1'; $settings.fcs.peer.port = 22000
    $settings.fcs.enabled = $true
    $settings | ConvertTo-Json -Depth 30 | Set-Content -LiteralPath $config -Encoding utf8
}
$settings = Get-Content -LiteralPath $config -Raw | ConvertFrom-Json
if ($settings.fcs.peer.ip -ne '127.0.0.1' -or $settings.fcs.peer.port -ne 22000) { throw 'Dummy configuration must point FCS at 127.0.0.1:22000.' }
foreach ($name in @('ecs.stop','dummy.stop')) {
    $flag = Join-Path $run $name
    if (Test-Path -LiteralPath $flag) { Remove-Item -LiteralPath $flag }
}
$python = (Get-Command python -ErrorAction Stop).Source
$dummy = Start-Process -FilePath $python -ArgumentList @('-u','simulators/fcs_dummy.py','--port','22000','--stop-file','logs/rdr-live-fcs-dummy/dummy.stop','--status-file','logs/rdr-live-fcs-dummy/dummy-status.json') -WorkingDirectory $root -WindowStyle Hidden -RedirectStandardOutput (Join-Path $run 'dummy.stdout.log') -RedirectStandardError (Join-Path $run 'dummy.stderr.log') -PassThru
Start-Sleep -Milliseconds 500
if ($dummy.HasExited) { throw 'FCS dummy failed to start. See dummy.stderr.log.' }
$station = Start-Process -FilePath $ecs -ArgumentList @('--config','configs/rdr-live-fcs-dummy.json','--headless','--stop-file','logs/rdr-live-fcs-dummy/ecs.stop','--status-file','logs/rdr-live-fcs-dummy/ecs-status.json') -WorkingDirectory $root -WindowStyle Hidden -RedirectStandardOutput (Join-Path $run 'ecs.stdout.log') -RedirectStandardError (Join-Path $run 'ecs.stderr.log') -PassThru
@{ecs_pid=$station.Id;dummy_pid=$dummy.Id;config=$config} | ConvertTo-Json | Set-Content (Join-Path $run 'processes.json')
if (!$NoGui -and !(Get-Process -Name AadcCommandCenter -ErrorAction SilentlyContinue)) { Start-Process -FilePath $gui -WorkingDirectory (Split-Path $gui -Parent) }
Write-Output "Started ECS + loopback FCS dummy. Configuration: $config"
