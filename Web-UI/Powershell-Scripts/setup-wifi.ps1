# Define the Flask server base URL
$baseUrl = "http://192.168.56.1:8000"

Write-Host "`nNote : This Setup requires RW access." -ForegroundColor Cyan

Write-Host "`nWarning : This Process Will Reboot the system." -ForegroundColor Red
Write-Host ""
Write-Host "Press 'y' to continue (any other key to cancel)..."
$key = [System.Console]::ReadKey($true).KeyChar

if ($key -eq 'y' -or $key -eq 'Y') {
    # Prompt user for WiFi credentials
	$username = Read-Host "`nSSID"
	$securePassword = Read-Host "`npassword" -AsSecureString
	$password = [Runtime.InteropServices.Marshal]::PtrToStringAuto(
		[Runtime.InteropServices.Marshal]::SecureStringToBSTR($securePassword)
	)

	try {
		$postBody = @{
			username = $username
			password = $password
		} | ConvertTo-Json

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

