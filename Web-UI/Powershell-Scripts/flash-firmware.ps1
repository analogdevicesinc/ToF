$FirmwarePath = Read-Host "Give the Firmware Path"

# Check if the firmware file exists
if (-Not (Test-Path $FirmwarePath)) {
    Write-Error "Firmware file not found: $FirmwarePath"
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


# Flash firmware on remote device
Write-Host "`nFlashing firmware..."
ssh "$remoteUser@$remoteHost" "$remoteCommand"


