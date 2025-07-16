param (
    [string]$choice,
    [string]$fileName
)

# Now the rest of your script
if (-not $choice) {
    Write-Host "`nChoose an option:"
    Write-Host "1. NVM Read"
    Write-Host "2. CCB Read"
    $choice = Read-Host "`nEnter 1 or 2"
}

switch ($choice) {
    "1" { $script = "./nvm-read.sh" }
    "2" { $script = "./ccb-read.sh" }
    default {
        Write-Host "Invalid Choice" -ForegroundColor Red
        exit 1
    }
}

if (-not $fileName) {
    $fileName = Read-Host "`nEnter the file name"
}


switch ($choice) {
    "1" { $fileName = "$fileName.nvm" }
    "2" { $fileName = "$fileName.ccb" }
}

# Define the server URL
$baseUri = "http://192.168.56.1:8000"

# Define the JSON payload for the POST request
$payload = @{
    version = $fileName
    script  = "./$script"
} | ConvertTo-Json

# Send the POST request to set the script and version
try {
    Invoke-RestMethod -Uri "$baseUri/run_shell_script" -Method Post -Body $payload -ContentType "application/json"
    Write-Host "`nScript is running. Please Wait .." -ForegroundColor Green
} catch {
    Write-Error "Failed to send POST request: $_"
    exit 1
}

# Ensure logs directory exists
$logDir = "logs"
if (-not (Test-Path $logDir)) {
    New-Item -ItemType Directory -Path $logDir | Out-Null
}

# Generate a unique log filename
$timestamp = Get-Date -Format "yyyyMMdd_HHmmss"
if ($choice -eq "1"){
	$logFile = "$logDir\\nvm-read_log_$timestamp.txt"
}elseif($choice -eq "2"){
	$logFile = "$logDir\\ccb-read_log_$timestamp.txt"
}



# Connect to the SSE endpoint and stream the output
try {
    $request = [System.Net.WebRequest]::Create("$baseUri/run-script")
    $request.Method = "GET"
    $request.Accept = "text/event-stream"

    $response = $request.GetResponse()
    $stream = $response.GetResponseStream()
    $reader = New-Object System.IO.StreamReader($stream)

    while (($line = $reader.ReadLine()) -ne $null) {
        if ($line.StartsWith("data:")) {
            $data = $line.Substring(5).Trim()
			Write-Host $data
			Add-Content -Path $logFile -Value $data
        }
    }
	
	Write-Host "`nCheck $logFile" -ForegroundColor Green

    $reader.Close()
    $response.Close()
} catch {
    Write-Error "Error connecting to the SSE stream: $_" -ForegroundColor Red
    exit 1
}

# download the file
& ".\\download.ps1" -fileName $fileName


