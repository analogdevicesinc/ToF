# Define the Flask server base URL
$baseUrl = "http://192.168.56.1:8000"

# Define the network mode value to send (choose from 'windows', 'ubuntu', 'check')
$os = (Get-CimInstance Win32_OperatingSystem).Caption

# Determine the value based on OS
if ($os -like "*Windows*") {
    $value = "windows"
} elseif ($os -like "*Ubuntu*") {
    $value = "ubuntu"
} else {
    $value = "check"
}


Write-Host "Starting network change script with value: $value"

try {
    # Step 1: Send POST request to /Change-Network
    $postBody = @{ value = $value } | ConvertTo-Json
    Invoke-RestMethod -Uri "$baseUrl/Change-Network" -Method Post -Body $postBody -ContentType "application/json"

    # Step 2: Send GET request to /Change-Network-GET
    $response = Invoke-RestMethod -Uri "$baseUrl/Change-Network-GET"
	Write-Host ""
    Write-Host $response.message -ForegroundColor Yellow
}
catch {
    Write-Host "An error occurred: $_"
}
