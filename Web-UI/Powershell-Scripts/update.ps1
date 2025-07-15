$Updatefile = Read-Host "Give the File Path"

# Check if the firmware file exists
if (-Not (Test-Path $Updatefile)) {
    Write-Error "file not found: $Updatefile"
    exit 1
}

# Remote connection details
$RemoteUser = "analog"
$RemoteHost = "192.168.56.1"
$RemotePassword = "analog"
$remoteDest = "/home/analog/" + [System.IO.Path]::GetFileName($Updatefile)
# Check if scp is available
if (-not (Get-Command scp -ErrorAction SilentlyContinue)) {
    Write-Error "scp command not found. Please ensure OpenSSH is installed and available in PATH."
    exit 1
}

# Copy firmware to remote device
Write-Host "Copying file to remote device..."
scp $Updatefile "${remoteUser}@${remoteHost}:${remoteDest}"

# Construct the remote command
$remoteCommand = @"
tar xzf $remoteDest && echo $remotePassword |sudo -S rm -f $remoteDest
"@.Trim()


# Flash firmware on remote device
Write-Host "`nUnzipping ..."
ssh "$remoteUser@$remoteHost" "$remoteCommand"
