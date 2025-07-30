param (
    [Parameter(Mandatory=$true)]
    [string]$value
)
if($value -like "modify"){
Write-Host "Warning : This process will reboot the system." -ForegroundColor Red
}

if($value -like "modify"){
Write-Host "`nPress 'y' to continue (any other key to cancel)..."
$key = [System.Console]::ReadKey($true).KeyChar
	if($key -eq 'y' -or $key -eq 'Y'){}
	else{
		Write-Host "`nCancelled." -ForegroundColor Yellow
		exit 1
	}
}

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

if($value -like "modify"){
	./system-reboot.ps1
}
