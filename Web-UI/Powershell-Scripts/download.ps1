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
$downloadUrl = "$baseUrl/download-event"

try {
    Invoke-WebRequest -Uri $downloadUrl -OutFile $filename -ErrorAction Stop
    $fileInfo = Get-Item $filename

    if ($fileInfo.Length -eq 0) {
        Remove-Item $filename
        Write-Host "Invalid file. Read failed !"
    } else {
        Write-Host "File downloaded successfully: $filename ($($fileInfo.Length) bytes)"
    }
} catch {
    Write-Host "Download failed: $($_.Exception.Message)"
}

