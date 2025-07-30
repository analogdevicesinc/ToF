param (
    [string]$choice
)

# Now the rest of your script
if (-not $choice) {
	Write-Host "`nChoose an option:"
	Write-Host "`n1. Get Workspace "
	Write-Host "2. Switch Workspace "

	$choice = Read-Host "`nEnter 1 or 2"
}

# Define the flask server url
$baseUrL = "http://192.168.56.1:8000"



try{
	if($choice -eq "1"){
		$response = Invoke-RestMethod -Uri "$baseUrl/get-workspace" -Method Get
		Write-Host $response.workspace
	
	} elseif($choice -eq "2"){
		Write-Host "`nWarning : This Process Will Reboot the system." -ForegroundColor Red
		# Get the list of Available workspaces
		$workspaceList = Invoke-RestMethod -Uri "$baseUrL/execute-post-script" -Method Get
		if(-not $workspaceList){
			Write-Host "No Workspace found" -ForegroundColor Yellow
			exit
		}
		Write-Host "`nAvailable Workspaces:" -ForegroundColor Cyan
		for ($i = 0; $i -lt $workspaceList.Count; $i++) {
			Write-Host "$($i + 1). $($workspaceList[$i])"
		}

		# Step 2: Prompt user to select a workspace
		$selection = Read-Host "`nChoose the option (ex. 1, 2) (or press Enter to cancel)"
		if ([string]::IsNullOrWhiteSpace($selection)) {
			Write-Host "Cancelled." -ForegroundColor Yellow
			exit
		}

		$index = [int]$selection - 1
		if ($index -lt 0 -or $index -ge $workspaceList.Count) {
			Write-Host "Invalid selection." -ForegroundColor Red
			exit
		}

		$selectedWorkspace = $workspaceList[$index]

		# Step 3: Send POST request to switch workspace
		$body = @{ entry = $selectedWorkspace } | ConvertTo-Json
		$response = Invoke-RestMethod -Uri "$baseUrl/switch-workspace" -Method Post -Body $body -ContentType "application/json"

		Write-Host $response.message -ForegroundColor Green

	}
}catch{
	Write-Host "An error occured while getting workspace"
	Write-Host $_ -ForegroundColor Red
	exit 1
}

# reboot the system
if($choice -eq "2"){
./system-reboot.ps1
}
