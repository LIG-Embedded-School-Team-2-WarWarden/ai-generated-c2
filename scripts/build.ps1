param([string]$BuildDir = 'build/live')
$ErrorActionPreference = 'Stop'
Push-Location (Split-Path $PSScriptRoot -Parent)
try {
    $c2CMakeCommand = Get-Command cmake -ErrorAction SilentlyContinue
    if ($c2CMakeCommand) { $c2CMake = $c2CMakeCommand.Source }
    else {
        $c2VSWhere = Join-Path ${env:ProgramFiles(x86)} 'Microsoft Visual Studio/Installer/vswhere.exe'
        if (-not (Test-Path -LiteralPath $c2VSWhere)) { throw 'Install CMake or Visual Studio C++ tools.' }
        $c2VSRoot = & $c2VSWhere -latest -products '*' -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath
        $c2CMake = Join-Path $c2VSRoot 'Common7/IDE/CommonExtensions/Microsoft/CMake/CMake/bin/cmake.exe'
    }
    if (-not (Test-Path external/warwarden-protocol/CMakeLists.txt)) {
        throw 'Initialize the pinned protocol: git submodule update --init --recursive'
    }
    $c2ConfigureArguments = @('-S', '.', '-B', $BuildDir)
    if (-not (Test-Path -LiteralPath (Join-Path $BuildDir 'CMakeCache.txt'))) {
        $c2ConfigureArguments += @('-A', 'x64')
    }
    & $c2CMake @c2ConfigureArguments
    if ($LASTEXITCODE) { throw 'CMake configure failed.' }
    & $c2CMake --build $BuildDir --config Release -- /m:1 /nr:false
    if ($LASTEXITCODE) { throw 'Build failed.' }
    $c2CTest = Join-Path (Split-Path $c2CMake -Parent) 'ctest.exe'
    & $c2CTest --test-dir $BuildDir -C Release --output-on-failure
    if ($LASTEXITCODE) { throw 'Tests failed.' }
} finally { Pop-Location }
