# Define the Flask server base URL
$baseUrl = "http://192.168.56.1:8000"

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
Write-Host ""
Write-Host "Press 'y' to continue (any other key to cancel)..."
$key = [System.Console]::ReadKey($true).KeyChar

if ($key -eq 'y' -or $key -eq 'Y') {
    # Prompt user for WiFi credentials
	$username = Read-Host "`nSSID"
	$securePassword = Read-Host "`npassword" -AsSecureString
	$password = [System.Net.NetworkCredential]::new("",$securePassword).Password

	try {
		$postBody = @{
			username = $username
			password = $password
		} | ConvertTo-Json -Depth 3

		$response = Invoke-RestMethod -Uri "$baseUrl/setup-wifi" -Method Post -Body $postBody -ContentType "application/json"
		Write-Host ""
		Write-Host $response.message -ForegroundColor Yellow
	}
	catch {
		Write-Host "An error occurred while trying to set up WiFi:"
		Write-Host $_
	}
} else {
    Write-Host "`nCancelled." -ForegroundColor Yellow
}

