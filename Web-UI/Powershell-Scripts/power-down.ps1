# Define the Flask server base URL

$baseUrl = "http://192.168.56.1:8000"

try {
    $response = Invoke-RestMethod -Uri "$baseUrl/power-down-system" -Method Post -ContentType "application/json"
    Write-Host ""
    Write-Host $response.message -ForegroundColor Yellow
}
catch {
    Write-Host "An error occurred while trying to system power down:"
    Write-Host $_
}