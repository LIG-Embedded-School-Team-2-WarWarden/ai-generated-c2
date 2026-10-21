param([string]$BinDir = 'build/live/Release')
$ErrorActionPreference = 'Stop'
Push-Location (Split-Path $PSScriptRoot -Parent)
try {
    # A new folder prevents obsolete private codecs from leaking into a new package.
    $c2PackageName = 'team-mvp-' + (Get-Date -Format 'yyyyMMdd-HHmmss') + '-' + [guid]::NewGuid().ToString('N').Substring(0,6)
    $c2PackageDir = Join-Path (Get-Location) ('dist/' + $c2PackageName)
    New-Item -ItemType Directory -Path $c2PackageDir | Out-Null
    foreach ($c2Executable in @('c2_mvp_console.exe')) {
        $c2Source = Join-Path $BinDir $c2Executable
        if (-not (Test-Path -LiteralPath $c2Source)) { throw "Build first: missing $c2Source" }
        Copy-Item -LiteralPath $c2Source -Destination $c2PackageDir
    }
    foreach ($c2Subdir in @('tests/dummies','configs','scripts','external/warwarden-protocol/python/mfs/icd/v1','external/warwarden-protocol/proto/mfs/icd/v1')) {
        New-Item -ItemType Directory -Force (Join-Path $c2PackageDir $c2Subdir) | Out-Null
    }
    foreach ($c2File in Get-ChildItem tests/dummies -File -Filter '*.py') {
        Copy-Item -LiteralPath $c2File.FullName -Destination (Join-Path $c2PackageDir 'tests/dummies')
    }
    foreach ($c2File in @('scripts/setup-team.ps1','tests/__init__.py','scripts/team_dummy.py','configs/team-demo.json','requirements-team.txt','external/warwarden-protocol/python/mfs/icd/v1/mfs_pb2.py','external/warwarden-protocol/proto/mfs/icd/v1/mfs.proto','external/warwarden-protocol/TEAM_ICD.md')) {
        Copy-Item -LiteralPath $c2File -Destination (Join-Path $c2PackageDir $c2File)
    }
    Copy-Item docs/console-mvp.md (Join-Path $c2PackageDir 'console-mvp.md')
    Copy-Item vendor/lidar/ORIGIN.md (Join-Path $c2PackageDir 'ORIGIN-lidar.md')
    Copy-Item external/warwarden-protocol/LICENSE (Join-Path $c2PackageDir 'LICENSE-protocol.txt')
    $c2Revision = git rev-parse HEAD
    $c2ProtocolRevision = git -C external/warwarden-protocol rev-parse HEAD
    @"
Team ICD 1.0 educational pointer MVP. C2 revision $c2Revision.
Shared protocol revision $c2ProtocolRevision.
The C2 executable needs only the Microsoft Visual C++ x64 runtime. Python 3.10+ is for dummy equipment tests only.
For optional dummy tests, run once:
  ./scripts/setup-team.ps1
Then in three terminals:
  ./c2_mvp_console.exe
  ./.venv-team/Scripts/python.exe scripts/team_dummy.py OBS
  ./.venv-team/Scripts/python.exe scripts/team_dummy.py EFF
Console: status, targets, aim 201, status, stop 201, quit.
Default configuration is synthetic loopback only. See console-mvp.md for real device prerequisites.
The shared schema is pinned from the public protobuf repository.
"@ | Set-Content (Join-Path $c2PackageDir 'RUNME.txt') -Encoding utf8
    $c2Zip = Join-Path (Get-Location) ('dist/' + $c2PackageName + '.zip')
    Compress-Archive -Path (Join-Path $c2PackageDir '*') -DestinationPath $c2Zip
    Write-Output $c2Zip
} finally { Pop-Location }
