param (
    [string]$choice
)

# Now the rest of your script
if (-not $choice) {
	Write-Host "`nChoose an option:"
	Write-Host "`n1. Check current firmware version "
	Write-Host "2. List firmware versions "
	$choice = Read-Host "`nEnter 1 or 2"
}


# Define the base URL
$baseUrl = "http://192.168.56.1:8000"

try {
    
    if ($choice -eq "1") {
        $response = Invoke-RestMethod -Uri "$baseUrl/current_firmware" -Method Get
        if ($response.current_firmware) {
            Write-Host "`nCurrent Firmware Version: $($response.current_firmware)" -ForegroundColor Yellow
        } elseif ($response.error) {
            Write-Host "`nError from server: $($response.error)" -ForegroundColor Red
        } else {
            Write-Host "`nUnexpected response format:"
            $response | ConvertTo-Json -Depth 5 | Write-Host -ForegroundColor Red
        }
    } elseif ($choice -eq "2") {
        $response = Invoke-RestMethod -Uri "$baseUrl/list_firmware_versions" -Method Get
        if ($response) {
            Write-Host "`nFirmware Versions:"
            $response | ForEach-Object { Write-Host "- $_" } 
        } else {
            Write-Host "`nNo response received or unexpected format." -ForegroundColor Red
        }
    } else {
        Write-Host "`nInvalid choice. Please enter 1 or 2."
    }
} catch {
    Write-Host "`nAn error occurred:"
    Write-Host $_
}
