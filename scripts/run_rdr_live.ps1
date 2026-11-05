param([string]$ConfigFile='configs/rdr-live.json',[ValidateSet('Debug','Release')][string]$Configuration='Release',[switch]$Headless,[string]$StopFile='',[switch]$PrintPlan)
$ErrorActionPreference='Stop'
$root=Split-Path $PSScriptRoot -Parent
if(-not [IO.Path]::IsPathRooted($ConfigFile)){$ConfigFile=Join-Path $root $ConfigFile}
$config=Get-Content -LiteralPath $ConfigFile -Raw|ConvertFrom-Json
if($config.rdr.protocol -ne 'edde24' -or $config.fcs.protocol -ne 'effe'){throw 'Configuration must use RDR edde24 and FCS effe.'}
$executable=Join-Path $root "x64/$Configuration/C2.Console.exe"
$arguments=@('--config',$ConfigFile)
if($Headless){$arguments+='--headless'}
if($StopFile){$arguments+=@('--stop-file',[IO.Path]::GetFullPath($StopFile))}
if($PrintPlan){[ordered]@{executable=$executable;arguments=$arguments;rdr=$config.rdr;fcs=$config.fcs}|ConvertTo-Json -Depth 8;return}
Push-Location $root
try{& $executable @arguments;if($LASTEXITCODE){throw 'C2 exited with an error.'}}finally{Pop-Location}
