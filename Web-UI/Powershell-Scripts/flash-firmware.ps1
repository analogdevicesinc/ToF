$FirmwarePath = Read-Host "Give the Firmware Path"

# Check if the firmware file exists
if (-Not (Test-Path $FirmwarePath)) {
    Write-Error "Firmware file not found: $FirmwarePath"
    exit 1
}

$hashFilePath = "$FirmwarePath.sha256"

if ((Test-Path $FirmwarePath) -and (Test-Path $hashFilePath)) {
    $expectedHash = Get-Content $hashFilePath | Select-Object -First 1
    $actualHash = (Get-FileHash -Path $FirmwarePath -Algorithm SHA256).Hash

    if ($actualHash -eq $expectedHash) {
        Write-Host "File integrity verified." -ForegroundColor Green
    } else {
        Write-Host "File integrity check failed!" -ForegroundColor Red
        Write-Host "Expected SHA-256: $expectedHash"
        Write-Host "Actual SHA-256:   $actualHash"
        exit 1
    }
} else {
    Write-Host "File or hash file not found." -ForegroundColor Red
    exit 1
}

# Remote connection details
$remoteUser = "analog"
$remoteHost = "192.168.56.1"
$remotePassword = "analog"
$remoteDest = "/home/analog/Workspace/" + [System.IO.Path]::GetFileName($FirmwarePath)

# Copy firmware to remote device
Write-Host "Copying firmware to remote device..."
scp $FirmwarePath "${remoteUser}@${remoteHost}:${remoteDest}"

# Construct the remote command
$remoteCommand = @"
echo $remotePassword | sudo -S /home/analog/Workspace/Tools/Firmware_update_utility/Firmware_Update $remoteDest && echo $remotePassword |sudo -S rm -f $remoteDest
"@.Trim()


& ssh "$remoteUser@$remoteHost" "echo" 2>&1
if ($LASTEXITCODE -eq 0) {
	# Flash firmware on remote device
	Write-Host "`nFlashing firmware..."
    ssh "$remoteUser@$remoteHost" "$remoteCommand"
} else {
    Write-Host "SSH connection failed." -ForegroundColor Red
}



