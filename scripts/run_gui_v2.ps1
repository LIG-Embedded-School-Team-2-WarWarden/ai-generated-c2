param(
    [string]$GuiDir = '',
    [string]$BinDir = 'build/gui-v2/Release',
    [string]$PythonExe = '',
    [switch]$FixedObserver,
    [switch]$SkipGuiBuild
)
$ErrorActionPreference = 'Stop'
$guiV2Root = Split-Path $PSScriptRoot -Parent
if (-not $GuiDir) { $GuiDir = Join-Path (Split-Path $guiV2Root -Parent) 'c2-GUI' }
if (-not [IO.Path]::IsPathRooted($BinDir)) { $BinDir = Join-Path $guiV2Root $BinDir }
if (-not $PythonExe) { $PythonExe = Join-Path $guiV2Root '.venv-team/Scripts/python.exe' }
$guiV2Contract = Get-Content -LiteralPath (Join-Path $GuiDir 'SharedMemoryFrame.cs') -Raw
if ($guiV2Contract -notmatch 'Version\s*=\s*2\s*;' -or $guiV2Contract -notmatch 'PayloadSize\s*=\s*452\s*;') {
    throw 'Use c2-GUI fe22d07 (shared memory v2 / 484 bytes); see docs/gui-v2.md.'
}
$env:DOTNET_CLI_HOME = Join-Path $guiV2Root 'build/dotnet-home'
$env:DOTNET_CLI_TELEMETRY_OPTOUT = '1'
$guiV2Output = Join-Path $guiV2Root 'build/gui-reference'
if (-not $SkipGuiBuild) {
    & dotnet build (Join-Path $GuiDir 'AadcCommandCenter.csproj') -c Release -o $guiV2Output "-p:BaseIntermediateOutputPath=$guiV2Output/obj/" --ignore-failed-sources
    if ($LASTEXITCODE) { throw 'GUI build failed' }
}
$guiV2Key = [guid]::NewGuid().ToString('N')
$guiV2Stop = Join-Path $guiV2Root "build/gui-v2-$guiV2Key.stop"
$guiV2Backend = $null
$guiV2Fakes = @()
try {
    $guiV2Backend = Start-Process -FilePath (Join-Path $BinDir 'c2_mvp_console.exe') -ArgumentList @('--gui','--headless','--stop-file',('"'+$guiV2Stop+'"')) -WorkingDirectory $guiV2Root -WindowStyle Hidden -PassThru -RedirectStandardOutput (Join-Path $guiV2Root "build/gui-v2-$guiV2Key.log") -RedirectStandardError (Join-Path $guiV2Root "build/gui-v2-$guiV2Key.stderr.log")
    foreach ($guiV2Role in @('OBS','EFF')) {
        $guiV2Args = @(('"'+(Join-Path $PSScriptRoot 'gui_fake.py')+'"'), $guiV2Role)
        if ($FixedObserver -and $guiV2Role -eq 'OBS') { $guiV2Args += '--fixed-view' }
        $guiV2Fakes += Start-Process -FilePath $PythonExe -ArgumentList $guiV2Args -WorkingDirectory $guiV2Root -WindowStyle Hidden -PassThru
    }
    $guiV2Deadline = (Get-Date).AddSeconds(10)
    do {
        if ($guiV2Backend.HasExited) { throw 'C++ backend exited; see the GUI v2 stderr log.' }
        foreach ($guiV2Fake in $guiV2Fakes) { if ($guiV2Fake.HasExited) { throw 'Fake exited; run scripts/setup-team.ps1 first.' } }
        if ((Get-Content -LiteralPath (Join-Path $guiV2Root "build/gui-v2-$guiV2Key.log") -Raw -ErrorAction SilentlyContinue) -match '"ready":true') { break }
        if ((Get-Date) -gt $guiV2Deadline) { throw 'C++ backend startup timed out' }
        Start-Sleep -Milliseconds 100
    } while ($true)
    Write-Host 'C++ C2 + loopback fake OBS/EFF + GUI v2. Closing the GUI stops this demo.'
    $guiV2Window = Start-Process -FilePath (Join-Path $guiV2Output 'AadcCommandCenter.exe') -WorkingDirectory $guiV2Root -PassThru
    $guiV2Window.WaitForExit()
} finally {
    if ($guiV2Backend -and -not $guiV2Backend.HasExited) {
        [IO.File]::WriteAllText($guiV2Stop, 'stop')
        if (-not $guiV2Backend.WaitForExit(7000)) { Stop-Process -Id $guiV2Backend.Id -ErrorAction SilentlyContinue }
    }
    foreach ($guiV2Fake in $guiV2Fakes) {
        if (-not $guiV2Fake.HasExited) { Stop-Process -Id $guiV2Fake.Id -ErrorAction SilentlyContinue }
    }
    if (Test-Path -LiteralPath $guiV2Stop) { Remove-Item -LiteralPath $guiV2Stop }
}
