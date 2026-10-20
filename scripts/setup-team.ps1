param()
$ErrorActionPreference = 'Stop'
Push-Location (Split-Path $PSScriptRoot -Parent)
try {
    if (-not (Test-Path .venv-team/Scripts/python.exe)) {
        python -m venv .venv-team
        if ($LASTEXITCODE) { throw 'Python 3.10+ is required.' }
    }
    & ./.venv-team/Scripts/python.exe -m pip install -r requirements-team.txt
    if ($LASTEXITCODE) { throw 'Dependency installation failed.' }
    & ./.venv-team/Scripts/python.exe -c 'from team_mvp.contract import pb; print(pb.DESCRIPTOR.name)'
    if ($LASTEXITCODE) { throw 'Shared generated protocol missing. See external/warwarden-protocol/TEAM_ICD.md.' }
} finally { Pop-Location }
