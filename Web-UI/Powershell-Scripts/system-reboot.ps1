# Define the Flask server base URL
$baseUrl = "http://192.168.56.1:8000"


try {
    $response = Invoke-RestMethod -Uri "$baseUrl/reboot-system" -Method Post -ContentType "application/json"
    Write-Host ""
    Write-Host $response.message -ForegroundColor Yellow
}
catch {
    Write-Host "An error occurred while trying to reboot the system:"
    Write-Host $_
}