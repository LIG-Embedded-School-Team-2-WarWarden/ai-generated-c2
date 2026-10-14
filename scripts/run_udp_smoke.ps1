param(
    [Parameter(Mandatory = $true)]
    [string]$BinDir
)

$ErrorActionPreference = 'Stop'

function Find-Executable {
    param([string[]]$Names)
    foreach ($name in $Names) {
        $candidate = Join-Path $BinDir $name
        if (Test-Path -LiteralPath $candidate) {
            return (Resolve-Path -LiteralPath $candidate).Path
        }
    }
    throw "Executable not found in ${BinDir}: $($Names -join ', ')"
}

function Start-RedirectedProcess {
    param(
        [string]$Path,
        [string[]]$ArgumentList = @()
    )
    $info = [System.Diagnostics.ProcessStartInfo]::new()
    $info.FileName = $Path
    foreach ($argument in $ArgumentList) {
        $null = $info.ArgumentList.Add($argument)
    }
    $info.UseShellExecute = $false
    $info.CreateNoWindow = $true
    $info.RedirectStandardInput = $true
    $info.RedirectStandardOutput = $true
    $info.RedirectStandardError = $true
    $process = [System.Diagnostics.Process]::Start($info)
    $outputLines = [System.Collections.Concurrent.ConcurrentQueue[string]]::new()
    $errorLines = [System.Collections.Concurrent.ConcurrentQueue[string]]::new()
    $process | Add-Member -NotePropertyName OutputLines -NotePropertyValue $outputLines
    $process | Add-Member -NotePropertyName ErrorLines -NotePropertyValue $errorLines
    $process | Add-Member -NotePropertyName OutputReadTask -NotePropertyValue $process.StandardOutput.ReadLineAsync()
    $process | Add-Member -NotePropertyName ErrorReadTask -NotePropertyValue $process.StandardError.ReadLineAsync()
    return $process
}

function Update-ProcessOutput {
    param([System.Diagnostics.Process]$Process)
    foreach ($stream in @(
        @{ Task = 'OutputReadTask'; Lines = 'OutputLines'; Reader = 'StandardOutput' },
        @{ Task = 'ErrorReadTask'; Lines = 'ErrorLines'; Reader = 'StandardError' }
    )) {
        while ($null -ne $Process.($stream.Task) -and
               $Process.($stream.Task).IsCompleted) {
            $task = $Process.($stream.Task)
            if ($task.IsFaulted) {
                throw "Failed to read process output: $($task.Exception)"
            }
            $line = $task.GetAwaiter().GetResult()
            if ($null -eq $line) {
                $Process.($stream.Task) = $null
                break
            }
            $Process.($stream.Lines).Enqueue($line)
            $Process.($stream.Task) = $Process.($stream.Reader).ReadLineAsync()
        }
    }
}

function Get-ProcessText {
    param(
        [System.Diagnostics.Process]$Process,
        [switch]$ErrorStream
    )
    Update-ProcessOutput $Process
    if ($ErrorStream) { return (@($Process.ErrorLines) -join "`n") }
    return (@($Process.OutputLines) -join "`n")
}

function Wait-OutputPattern {
    param(
        [System.Diagnostics.Process]$Process,
        [string]$Pattern,
        [int]$TimeoutMs = 10000,
        [string]$ProbeCommand = ''
    )
    $timer = [System.Diagnostics.Stopwatch]::StartNew()
    $nextProbe = 0
    while ($timer.ElapsedMilliseconds -lt $TimeoutMs) {
        if ($Process.HasExited) {
            throw "Process exited before output '${Pattern}'. Exit=$($Process.ExitCode), stderr=$(Get-ProcessText $Process -ErrorStream)"
        }
        if ((Get-ProcessText $Process) -match $Pattern) { return }
        if ($ProbeCommand -and $timer.ElapsedMilliseconds -ge $nextProbe) {
            $Process.StandardInput.WriteLine($ProbeCommand)
            $nextProbe = $timer.ElapsedMilliseconds + 200
        }
        Start-Sleep -Milliseconds 20
    }
    throw "Timed out waiting for output '${Pattern}'. stdout=$(Get-ProcessText $Process), stderr=$(Get-ProcessText $Process -ErrorStream)"
}

function Wait-SentCount {
    param(
        [System.Diagnostics.Process]$Process,
        [int]$Expected,
        [int]$TimeoutMs = 5000
    )
    $timer = [System.Diagnostics.Stopwatch]::StartNew()
    while ($timer.ElapsedMilliseconds -lt $TimeoutMs) {
        $count = ([regex]::Matches((Get-ProcessText $Process), '> sent')).Count
        if ($count -ge $Expected) { return }
        if ($Process.HasExited) { break }
        Start-Sleep -Milliseconds 20
    }
    throw "Timed out waiting for ${Expected} successful commands. stdout=$(Get-ProcessText $Process), stderr=$(Get-ProcessText $Process -ErrorStream)"
}

function Wait-OutputCountPattern {
    param(
        [System.Diagnostics.Process]$Process,
        [string]$Pattern,
        [int]$Expected,
        [int]$TimeoutMs = 5000
    )
    $timer = [System.Diagnostics.Stopwatch]::StartNew()
    while ($timer.ElapsedMilliseconds -lt $TimeoutMs) {
        $count = ([regex]::Matches((Get-ProcessText $Process), $Pattern)).Count
        if ($count -ge $Expected) { return }
        if ($Process.HasExited) { break }
        Start-Sleep -Milliseconds 20
    }
    throw "Timed out waiting for ${Expected} matches of '${Pattern}'. stdout=$(Get-ProcessText $Process), stderr=$(Get-ProcessText $Process -ErrorStream)"
}

function Wait-AnyOutputPattern {
    param(
        [System.Diagnostics.Process[]]$Processes,
        [string]$Pattern,
        [int]$TimeoutMs = 10000
    )
    $timer = [System.Diagnostics.Stopwatch]::StartNew()
    while ($timer.ElapsedMilliseconds -lt $TimeoutMs) {
        foreach ($process in $Processes) {
            if (-not $process.HasExited -and (Get-ProcessText $process) -match $Pattern) {
                return ,$process
            }
        }
        Start-Sleep -Milliseconds 20
    }
    $outputs = $Processes | ForEach-Object { Get-ProcessText $_ }
    throw "Timed out waiting for any process output '${Pattern}'. stdout=$($outputs -join "`n---`n")"
}

function Assert-InvalidConfiguration {
    param(
        [string]$Path,
        [string[]]$ArgumentList,
        [string]$ExpectedError
    )
    $process = Start-RedirectedProcess $Path $ArgumentList
    try {
        if (-not $process.WaitForExit(5000)) {
            throw "Invalid-configuration process did not exit: $Path"
        }
        $errorOutput = Get-ProcessText $process -ErrorStream
        if ($process.ExitCode -eq 0 -or $errorOutput -notmatch [regex]::Escape($ExpectedError)) {
            throw "Expected configuration error '${ExpectedError}'. Exit=$($process.ExitCode), stderr=${errorOutput}"
        }
    } finally {
        if (-not $process.HasExited) { $process.Kill($true) }
        $process.Dispose()
    }
}

$serverPath = Find-Executable @('c2_server.exe', 'C2.Server.exe')
$observationPath = Find-Executable @('dummy_observation.exe', 'Dummy.Observation.exe')
$effectorPath = Find-Executable @('dummy_effector.exe', 'Dummy.Effector.exe')
$processes = @()

try {
    $server = Start-RedirectedProcess $serverPath @(
        '--asset-port', '15000',
        '--observation-status-port', '15001',
        '--target-port', '15002',
        '--observation-command-port', '15101',
        '--effector-command-port', '16001',
        '--effector-status-port', '16002',
        '--target-validity-ms', '3000',
        '--heartbeat-interval-ms', '700',
        '--heartbeat-timeout-ms', '4000',
        '--command-validity-ms', '700',
        '--ack-timeout-ms', '300',
        '--command-attempts', '4',
        '--emergency-stop-repetitions', '4'
    )
    Wait-OutputPattern $server 'C2 server started'
    $observation1 = Start-RedirectedProcess $observationPath @(
        '--asset-id', '101', '--listen-port', '0', '--c2-port', '15000',
        '--status-interval-ms', '80',
        '--heartbeat-interval-ms', '300', '--registration-interval-ms', '500',
        '--target-interval-ms', '300', '--vx', '5',
        '--watchdog-timeout-ms', '4000'
    )
    $observation2 = Start-RedirectedProcess $observationPath @(
        '--asset-id', '102', '--listen-port', '0', '--c2-port', '15000',
        '--status-interval-ms', '80',
        '--heartbeat-interval-ms', '300', '--registration-interval-ms', '500',
        '--target-interval-ms', '300', '--y', '20',
        '--target-x', '2', '--target-y', '3', '--target-z', '4',
        '--watchdog-timeout-ms', '4000'
    )
    $effector1 = Start-RedirectedProcess $effectorPath @(
        '--asset-id', '201', '--listen-port', '0', '--c2-port', '15000',
        '--status-interval-ms', '80',
        '--heartbeat-interval-ms', '300', '--registration-interval-ms', '500',
        '--watchdog-timeout-ms', '4000'
    )
    $effector2 = Start-RedirectedProcess $effectorPath @(
        '--asset-id', '202', '--listen-port', '0', '--c2-port', '15000',
        '--status-interval-ms', '80',
        '--heartbeat-interval-ms', '300', '--registration-interval-ms', '500',
        '--watchdog-timeout-ms', '4000', '--x', '50'
    )
    $effector3 = Start-RedirectedProcess $effectorPath @(
        '--asset-id', '203', '--listen-port', '0', '--c2-port', '15000',
        '--status-interval-ms', '80',
        '--heartbeat-interval-ms', '300', '--registration-interval-ms', '500',
        '--watchdog-timeout-ms', '4000', '--x', '90'
    )
    $assets = @($observation1, $observation2, $effector1, $effector2, $effector3)
    $processes = @($server) + $assets

    Wait-OutputPattern $server 'assets=5 tracks=2' 10000 'status'
    Wait-OutputPattern $server 'metrics rx_datagrams=[1-9][0-9]* .*queue_processed=[1-9][0-9]*' 10000 'metrics'
    $server.StandardInput.WriteLine('targets')
    Wait-OutputPattern $server 'observer=102 detection=1 xyz=\(2,3,4\)' 10000 'targets'
    $sent = 0
    Wait-OutputPattern $server '(?s)asset=102[^\r\n]*connection=CONNECTED.*asset=203[^\r\n]*connection=CONNECTED' 10000 'assets'
    foreach ($poseCase in @(
        @{ Command = 'dev-pose 102 -25 30 2 90'; Pattern = 'asset=102[^\r\n]*xyz=\(-25,30,2\) azimuth_deg=90' },
        @{ Command = 'dev-pose 203 95 5 2 45'; Pattern = 'asset=203[^\r\n]*xyz=\(95,5,2\) azimuth_deg=45' },
        @{ Command = 'dev-pose 102 0 20 1.5 0'; Pattern = 'asset=102[^\r\n]*xyz=\(0,20,1\.5\) azimuth_deg=0' },
        @{ Command = 'dev-pose 203 90 0 1.5 0'; Pattern = 'asset=203[^\r\n]*xyz=\(90,0,1\.5\) azimuth_deg=0' }
    )) {
        $server.StandardInput.WriteLine($poseCase.Command)
        ++$sent
        Wait-SentCount $server $sent
        Wait-OutputPattern $server $poseCase.Pattern 10000 'assets'
    }
    foreach ($command in @(
        'scan 101 10 5',
        'obs-stop 101',
        'obs-home 101',
        'point 1',
        'arm 1',
        'start 1 100'
    )) {
        if ($command -eq 'arm 1') {
            # A manual angle command is not proof of a tracked output target.
            # Wait for a new automatic-track report before requesting ARM.
            Wait-OutputPattern $server 'tracking_track=1[^\r\n]*status_current=yes' 10000 'assets'
        }
        $server.StandardInput.WriteLine($command)
        ++$sent
        Wait-SentCount $server $sent
    }
    $trackingEffector = Wait-AnyOutputPattern @($effector1, $effector2, $effector3) 'tracking=1'
    $trackingCount = ([regex]::Matches((Get-ProcessText $trackingEffector), 'tracking=1')).Count
    Wait-OutputCountPattern $trackingEffector 'tracking=1' ($trackingCount + 2) 5000
    $server.StandardInput.WriteLine('assets')
    Wait-OutputPattern $server 'asset=201 role=2 session=\d+'
    $effector1.StandardInput.WriteLine('quit')
    if (-not $effector1.WaitForExit(5000)) {
        throw 'Original effector 201 did not exit for session replacement'
    }
    $effector1Restart = Start-RedirectedProcess $effectorPath @(
        '--asset-id', '201', '--listen-port', '0', '--c2-port', '15000',
        '--status-interval-ms', '80',
        '--heartbeat-interval-ms', '300', '--registration-interval-ms', '500',
        '--watchdog-timeout-ms', '4000'
    )
    $assets += $effector1Restart
    $processes += $effector1Restart
    $originalSessionMatches = [regex]::Matches(
        (Get-ProcessText $server), 'asset=201 role=2 session=(\d+)')
    $originalSession = $originalSessionMatches[0].Groups[1].Value
    Wait-OutputPattern $server "asset=201 role=2 session=(?!${originalSession}\b)\d+" 10000 'assets'
    $server.StandardInput.WriteLine('status')
    Wait-OutputPattern $server 'assets=5 tracks=2 assignments=1 pending_commands=0' 10000 'status'
    $server.StandardInput.WriteLine('estop-all')
    Wait-OutputPattern $server 'estop assets=3 datagrams=12'
    Wait-OutputPattern $server '(?s)estop assets=3 datagrams=12.*asset=201.*effector_state=3.*asset=202.*effector_state=3.*asset=203.*effector_state=3' 10000 'assets'
    $server.StandardInput.WriteLine('quit')
    foreach ($asset in $assets) {
        if (-not $asset.HasExited) { $asset.StandardInput.WriteLine('quit') }
    }

    foreach ($process in $processes) {
        if (-not $process.WaitForExit(5000)) {
            throw "Process did not exit within five seconds: $($process.StartInfo.FileName)"
        }
    }

    $serverOutput = Get-ProcessText $server
    $allErrors = ($processes | ForEach-Object {
        Get-ProcessText $_ -ErrorStream
    }) -join "`n"
    $sentCount = ([regex]::Matches($serverOutput, '> sent')).Count
    if ($sentCount -ne 10) {
        throw "Expected ten successful commands but observed ${sentCount}.`n${serverOutput}`n${allErrors}"
    }
    if ($serverOutput -notmatch 'assets=5 tracks=2 assignments=1 pending_commands=0' -or
        $serverOutput -notmatch 'estop assets=3 datagrams=12') {
        throw "Dynamic assets or emergency stop summary was incorrect.`n${serverOutput}`n${allErrors}"
    }
    $sessions = [regex]::Matches(
        $serverOutput, 'asset=201 role=2 session=(\d+)') |
        ForEach-Object { $_.Groups[1].Value } | Select-Object -Unique
    if ($sessions.Count -lt 2) {
        throw "Effector 201 did not register a new session after restart.`n${serverOutput}`n${allErrors}"
    }
    foreach ($process in $processes) {
        if ($process.ExitCode -ne 0) {
            throw "Process failed with exit code $($process.ExitCode): $($process.StartInfo.FileName)"
        }
    }
    Assert-InvalidConfiguration $serverPath @('--heartbeat-interval-ms', '0') 'invalid heartbeat interval'
    Assert-InvalidConfiguration $observationPath @('--target-interval-ms', '0') 'invalid target interval'
    Assert-InvalidConfiguration $effectorPath @('--status-interval-ms', '0') 'invalid status interval'
    Write-Host 'UDP process smoke test passed: 2 observations, 3 effectors, development pose commands and reports, moving-target routing, continuous tracking beyond output duration, session replacement, automatic assignment, attack flow, estop-all safe states.'
} finally {
    foreach ($process in $processes) {
        if ($null -ne $process -and -not $process.HasExited) {
            $process.Kill($true)
            $process.WaitForExit()
        }
        if ($null -ne $process) { $process.Dispose() }
    }
}
