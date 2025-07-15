# Get-ServerTime-Local.ps1

$apiUrl = "http://192.168.56.1:8000/server-time"

try {
    $response = Invoke-RestMethod -Uri $apiUrl -Method Get

    if ($response.server_time_utc) {
        $utcTime = [datetime]::Parse($response.server_time_utc)
        $localTime = $utcTime.ToLocalTime()

        Write-Host ""
        Write-Host "Server Time (Local): $localTime" -ForegroundColor Green
    } else {
        Write-Host "Server time not found in response." -ForegroundColor Yellow
    }
}
catch {
    Write-Host "Failed to contact server or parse response: $_" -ForegroundColor Red
}
