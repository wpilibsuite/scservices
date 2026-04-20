Set-StrictMode -Version 'Latest'
$PSDefaultParameterValues['*:ErrorAction'] = 'Stop'

$SrcRoot = Split-Path $PSScriptRoot -Parent

function Log($msg) {
    Write-Host "[$(Get-Date)] $msg"
}

# Executes cmake with the given arguments.
function Invoke-Cmake([String]$BuildDir, [String]$Arguments) {
    Log "cmake $($Arguments)"
    $process = Start-Process cmake $Arguments -PassThru -NoNewWindow -WorkingDirectory $BuildDir
    $handle = $process.Handle # Magic work around. Don't remove this line.
    $process.WaitForExit();

    if ($process.ExitCode -ne 0) {
        Write-Error "[$(Get-Date)] CMake exited with status code $($process.ExitCode)"
    }
}

Invoke-Cmake -BuildDir $SrcRoot -Arguments "--build --preset mrc"

ssh systemcore@robot.local "sudo systemctl stop motioncoreodometrydaemon"

scp -s "$SrcRoot/buildmrc/bin/MotionCoreOdometryDaemon" systemcore@robot.local:~/MotionCoreOdometryDaemon

ssh systemcore@robot.local "sudo chmod +x ~/MotionCoreOdometryDaemon"
