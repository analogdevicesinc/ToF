param (
    [Parameter(Mandatory=$true)]
    [string]$value
)

# Define the Flask server base URL
$baseUrl = "http://192.168.56.1:8000"


try {
    # Step 1: Send POST request to /Change-Permission
    $postBody = @{ value = $value } | ConvertTo-Json
    Invoke-RestMethod -Uri "$baseUrl/Change-Permission" -Method Post -Body $postBody -ContentType "application/json"

    # Step 2: Send GET request to /Change-Permission-Events
    $response = Invoke-RestMethod -Uri "$baseUrl/Change-Permission-Events"
	Write-Host ""
	Write-Host $response.output -ForegroundColor Yellow
}
catch {
    Write-Host "An error occurred: $_"
}
