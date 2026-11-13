$ErrorActionPreference = 'Stop'
$root = Split-Path $PSScriptRoot -Parent
$run = Join-Path $root 'logs/rdr-live-fcs-dummy'
New-Item -ItemType File -Path (Join-Path $run 'ecs.stop') -Force | Out-Null
Start-Sleep -Seconds 3
New-Item -ItemType File -Path (Join-Path $run 'dummy.stop') -Force | Out-Null
Write-Output 'Requested graceful ECS and dummy shutdown. GUI can be closed separately.'
