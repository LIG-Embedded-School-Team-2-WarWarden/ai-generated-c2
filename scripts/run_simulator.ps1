param(
    [switch]$Gui,
    [switch]$Headless,
    [switch]$NoBuild,
    [string]$GuiDir = '',
    [ValidateSet('Debug','Release')][string]$Configuration = 'Release'
)
$ErrorActionPreference = 'Stop'
$root = Split-Path $PSScriptRoot -Parent
if (-not $GuiDir) { $GuiDir = Join-Path (Split-Path $root -Parent) 'c2-GUI' }
if (-not $NoBuild) { & (Join-Path $PSScriptRoot 'build.ps1') -Configuration $Configuration }
$simArgs = @((Join-Path $root 'simulators/run.py'), '--console', (Join-Path $root "x64/$Configuration/C2.Console.exe"))
if ($Headless) { $simArgs += '--headless' }
if ($Gui) {
    $project = Join-Path $GuiDir 'AadcCommandCenter.csproj'
    if (-not (Test-Path -LiteralPath $project)) { throw "WPF source missing: $project. Specify -GuiDir with the c2-GUI v3 source." }
    if (-not (Select-String -LiteralPath (Join-Path $GuiDir 'PipeCommandClient.cs') -SimpleMatch 'AADC9_PIPE_NAME' -Quiet)) {
        throw 'This GUI needs the optional AADC9_PIPE_NAME override for isolated simulation. See docs/simulator.md.'
    }
    $outputDir = Join-Path $GuiDir 'build/Release'
    if (-not $NoBuild) {
        & dotnet build $project -c Release -o $outputDir
        if ($LASTEXITCODE) { throw 'WPF build failed.' }
    }
    $simArgs += @('--gui', (Join-Path $outputDir 'AadcCommandCenter.exe'))
}
& python @simArgs
if ($LASTEXITCODE) { throw 'Simulator stopped with an error. Check logs/simulator.' }
