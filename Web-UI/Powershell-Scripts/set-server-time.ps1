# Flask endpoint URL
$flaskUrl = "http://192.168.56.1:8000/set-server-time"

# Prepare the data
$systemTime = (Get-Date).ToString("yyyy-MM-ddTHH:mm:ss")  # ISO format
$timeZone = (Get-TimeZone).Id

$timeZoneMap = @{
    "India Standard Time" = "Asia/Kolkata"
    "Pacific Standard Time" = "America/Los_Angeles"
    "Eastern Standard Time" = "America/New_York"
    "UTC" = "UTC"
    # Add more mappings as needed
}

$ianaTimeZone = $timeZoneMap[$timeZone]
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
try {
    $response = Invoke-RestMethod -Uri $flaskUrl -Method POST -Body $body -ContentType "application/json"
    Write-Host ""
    Write-Host $response.message -ForegroundColor Yellow
} catch {
    Write-Host "`nFailed to send request. Error:"
    Write-Output $_.Exception.Message
}
