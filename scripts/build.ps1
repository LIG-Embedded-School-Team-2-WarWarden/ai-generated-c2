param([ValidateSet('Debug','Release')][string]$Configuration = 'Debug')
$ErrorActionPreference = 'Stop'
$root = Split-Path $PSScriptRoot -Parent
function Resolve-MSBuildPath {
    if (-not ${env:ProgramFiles(x86)}) {
        throw 'Visual Studio discovery requires ProgramFiles(x86). Run this script on Windows.'
    }
    $vswhere = Join-Path ${env:ProgramFiles(x86)} 'Microsoft Visual Studio/Installer/vswhere.exe'
    if (-not (Test-Path -LiteralPath $vswhere -PathType Leaf)) {
        throw "Visual Studio Installer discovery tool not found: $vswhere"
    }
    $installation = & $vswhere -latest -products '*' -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath
    if ($LASTEXITCODE) { throw "Visual Studio discovery failed (exit code $LASTEXITCODE)." }
    if (-not $installation) { throw 'Install Visual Studio Desktop development with C++.' }
    $path = Join-Path $installation 'MSBuild/Current/Bin/amd64/MSBuild.exe'
    if (-not (Test-Path -LiteralPath $path -PathType Leaf)) {
        throw "MSBuild not found: $path. Repair the Visual Studio C++ installation."
    }
    return $path
}

$msbuild = Resolve-MSBuildPath
& $msbuild (Join-Path $root 'ai-generated-c2.sln') /t:Build "/p:Configuration=$Configuration" /p:Platform=x64 /m:1 /nr:false /v:minimal
if ($LASTEXITCODE) { throw "Visual Studio build failed (exit code $LASTEXITCODE)." }
