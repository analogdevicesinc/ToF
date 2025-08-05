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

$value = $value.Trim()
Write-Output "Network mode value: $value"

Write-Host "`nWarning : This Process Will Reboot the system." -ForegroundColor Red
Write-Host ""


Write-Host "Press 'y' to continue (any other key to cancel)..."
$key = [System.Console]::ReadKey($true).KeyChar

if($key -eq 'y' -or $key -eq 'Y'){

	Write-Host "Starting network change script with value: $value"

	for($i = 0; $i -lt 3; $i++){
	   try{
		# Step 1: Send POST request to /Change-Network
		$postBody = @{ value = $value } | ConvertTo-Json -Depth 2
		Invoke-RestMethod -Uri "$baseUrl/Change-Network" -Method Post -Body $postBody -ContentType "application/json"

	    }
	    catch{
		Write-Host "Attempt $($i + 1) failed: $_"
		Start-Sleep 1
	    }
	}


	try {
		# Step 2: Send GET request to /Change-Network-GET
		$response = Invoke-RestMethod -Uri "$baseUrl/Change-Network-GET"
		Write-Host ""
		Write-Host $response.message -ForegroundColor Yellow
	}
	catch {
		Write-Host "An error occurred: $_"
		exit 1
	}
	
	# reboot the system
	./system-reboot.ps1
}else{
	Write-Host "`nCancelled." -ForegroundColor Yellow
}

