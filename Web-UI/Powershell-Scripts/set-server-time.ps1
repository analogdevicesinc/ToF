# Flask endpoint URL
$flaskUrl = "http://192.168.56.1:8000/set-server-time"

# Prepare the data
$systemTime = (Get-Date).ToString("yyyy-MM-ddTHH:mm:ss")  # ISO format
$timeZone = (Get-TimeZone).Id

# Try to detect Windows
try {
    $os = (Get-CimInstance Win32_OperatingSystem -ErrorAction Stop).Caption
    if ($os -like "*Windows*") {
		$timeZoneMap = @{
		"India Standard Time" = "Asia/Kolkata"
		"Pacific Standard Time" = "America/Los_Angeles"
		"Eastern Standard Time" = "America/New_York"
		"UTC" = "UTC"
		# Add more mappings as needed
		}
		
		$ianaTimeZone = $timeZoneMap[$timeZone]
    }
}
catch {
    # If not Windows, try to detect Ubuntu from /etc/os-release
    if (Test-Path "/etc/os-release") {
        $osRelease = Get-Content "/etc/os-release"
        if ($osRelease -match "ID=ubuntu") {
			$ianaTimeZone = $timeZone
        }
    }
}




if (-not $ianaTimeZone) {
    Write-Host "`nTime zone '$timeZone' not mapped. Defaulting to UTC."
    $ianaTimeZone = "UTC"
}


# Convert to JSON
$body = @{
    browser_time = $systemTime
    time_zone_name = $ianaTimeZone
} | ConvertTo-Json

# Send POST request
for($i = 0; $i -lt 3; $i++){
	try {
	    $response = Invoke-RestMethod -Uri $flaskUrl -Method POST -Body $body -ContentType "application/json"
	    Write-Host ""
	    Write-Host $response.message -ForegroundColor Yellow
	    break
	} catch {
	    Write-Host "Attempt $($i + 1) failed: $_"
	    Start-Sleep 1
	}
}
