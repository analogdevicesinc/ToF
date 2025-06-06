# Target: ADIToFGUI.exe
# Tests startup of viewer tool

$AppName = "ADIToFGUI.exe"
$AppPath = "ADIToFGUI.exe"
$Max = 100
$Log = "log.txt"

"=== Log start $(Get-Date) ===" | Set-Content $Log

for ($counter = 1; $counter -le $Max; $counter++) {
    $msg = "Starting cycle $counter of $Max"
    Write-Host $msg
    $msg | Add-Content $Log

    Start-Process $AppPath
    Start-Sleep -Seconds 5

    $process = Get-Process -Name ($AppName -replace ".exe","") -ErrorAction SilentlyContinue
    if ($process) {
        $msg = "App is running, killing process..."
        Write-Host $msg -ForegroundColor Green
        $msg | Add-Content $Log
        try {
            Stop-Process -Name ($AppName -replace ".exe","") -Force -ErrorAction Stop
        } catch {
            if ($_.Exception.Message -like "*Access is denied*") {
                $errMsg = "ERROR: Access is denied when trying to kill $AppName. Stopping script."
                Write-Host $errMsg -ForegroundColor Red
                $errMsg | Add-Content $Log
                exit 1
            } else {
                $errMsg = "ERROR: Unexpected error: $($_.Exception.Message)"
                Write-Host $errMsg -ForegroundColor Red
                $errMsg | Add-Content $Log
                exit 1
            }
        }
    } else {
        $msg = "App is NOT running, skipping kill."
        Write-Host $msg -ForegroundColor Red
        $msg | Add-Content $Log
    }
}
Write-Host "Done!"
"Done!" | Add-Content $Log