param (
    [Parameter(Mandatory=$true)]
    [string]$fileName
)

# Define the Flask server base URL
$baseUrl = "http://192.168.56.1:8000"

# Define the file name and path to request
$path = "Tools/host_boot_tools/NVM_Utils"

# Step 1: Send POST request to /download
$postBody = @{
    filename = $filename
    path = $path
} | ConvertTo-Json

Invoke-RestMethod -Uri "$baseUrl/download" -Method Post -Body $postBody -ContentType "application/json"

# Step 2: Send GET request to /download-event to download the file
Invoke-WebRequest -Uri "$baseUrl/download-event" -OutFile $filename

Write-Host "File downloaded as $filename"
