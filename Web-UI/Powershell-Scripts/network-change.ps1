# Define the Flask server base URL

$baseUrl = "http://192.168.56.1:8000"

# Default value
$value = "check"

# Try to detect Windows
try {
    $os = (Get-CimInstance Win32_OperatingSystem -ErrorAction Stop).Caption
    if ($os -like "*Windows*") {
        $value = "windows"
    }
}
catch {
    # If not Windows, try to detect Ubuntu from /etc/os-release
    if (Test-Path "/etc/os-release") {
        $osRelease = Get-Content "/etc/os-release"
        if ($osRelease -match "ID=ubuntu") {
            $value = "ubuntu"
        }
    }
}

Write-Output "Network mode value: $value"




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
