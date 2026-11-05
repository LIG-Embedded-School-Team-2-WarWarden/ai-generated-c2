param([ValidateSet('Debug','Release')][string]$Configuration = 'Debug')
$ErrorActionPreference = 'Stop'
$root = Split-Path $PSScriptRoot -Parent
$vswhere = Join-Path ${env:ProgramFiles(x86)} 'Microsoft Visual Studio/Installer/vswhere.exe'
$installation = & $vswhere -latest -products '*' -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath
if (-not $installation) { throw 'Install Visual Studio Desktop development with C++.' }
$msbuild = Join-Path $installation 'MSBuild/Current/Bin/amd64/MSBuild.exe'
& $msbuild (Join-Path $root 'ai-generated-c2.sln') /t:Build "/p:Configuration=$Configuration" /p:Platform=x64 /m:1 /nr:false /v:minimal
if ($LASTEXITCODE) { throw 'Visual Studio build failed.' }
