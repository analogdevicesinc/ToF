$apiUrl = "http://192.168.56.1:8000/network-status"

try {
    $response = Invoke-RestMethod -Uri $apiUrl -Method Get

    if ($response.status -eq $true) {
        Write-Host "`nConnected" -ForegroundColor Green
    } else {
        Write-Host "`nDisconnected" -ForegroundColor Red
    }
}
catch {
    Write-Host "`nDisconnected" -ForegroundColor Red
}
