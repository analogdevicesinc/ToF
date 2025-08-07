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


Write-Host "`nWarning : This Process Will Reboot the system." -ForegroundColor Red

Write-Host "Press 'y' to continue (any other key to cancel)..."
$key = [System.Console]::ReadKey($true).KeyChar

if($key -eq 'y' -or $key -eq 'Y'){

ssh -tt analog@192.168.56.1 "/home/analog/web/remove-workspace.sh y"

}



