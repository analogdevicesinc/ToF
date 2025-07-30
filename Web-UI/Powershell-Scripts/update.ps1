$Updatefile = Read-Host "Give the File Path"

# Check if the firmware file exists
if (-Not (Test-Path $Updatefile)) {
    Write-Error "file not found: $Updatefile"
    exit 1
}

# check for file system Read-Write Permission
$output = powershell -Command "& { .\permission-ops.ps1 -value 'view' }"

$output = $output -join "`n"
$output = $output.Trim()

if ($output -match "Permission is\s*:\s*(RO|RW)") {
    $permission = $matches[1]
	if($permission -eq "RO"){
		Write-Host "File System Permission is : $permission. Make Sure File System Permission is RW." -ForegroundColor Red
		exit 1
	}
} else {
    Write-Host "Permission not found in output."
    exit 1
}




$hashFilePath = "$Updatefile.sha256"

if ((Test-Path $Updatefile) -and (Test-Path $hashFilePath)) {
    $expectedHash = Get-Content $hashFilePath | Select-Object -First 1
    $actualHash = (Get-FileHash -Path $Updatefile -Algorithm SHA256).Hash

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

$sshOutput = & ssh "$remoteUser@$remoteHost" "$remoteCommand" 2>&1

if($LASTEXITCODE -eq 0){
	Write-Host "`nWorkspace has been Successfully updated !!" -ForegroundColor Green
	exit 0
} else {
	Write-Host "`nSSH Connection failed. Please Check your SSH setup." -ForegroundColor Red
	exit 1
}



