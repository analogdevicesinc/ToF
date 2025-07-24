# Master PowerShell Script: master-menu.ps1

function Show-Menu {
    Clear-Host
    Write-Host "===================================================================" -ForegroundColor Cyan
    Write-Host "           Welcome to the Eval Kit Configuration Utility           " -ForegroundColor Green
    Write-Host "===================================================================" -ForegroundColor Cyan
    Write-Host ""
    Write-Host "1.  ADSD3500 Reset                  10. Check Firmware Version"
    Write-Host "2.  Reboot                          11. Update Firmware"
	Write-Host "3.  Power Down                      12. Get SDK Version"
    Write-Host "4.  Get File System Permissions     13. Update SDK Version"
    Write-Host "5.  Modify File System Permissions  14. Switch SDK Version"
    Write-Host "6.  Get Date & Time                 15. Network Switch"
    Write-Host "7.  Set Date & Time                 16. Exit"
    Write-Host "8.  Check WiFi Connection"
    Write-Host "9.  Setup WiFi"
    Write-Host ""
}


do {
    Show-Menu
    $choice = Read-Host "Enter your choice (1-16)"

    switch ($choice) {
        "1" {
			Write-Host "Resetting ADSD3500..."
            & .\adsd3500-reset.ps1
        }
        "2" {
	        Write-Host "Rebooting system..."
            & .\system-reboot.ps1
        }
		"3" {
			Write-Host "Powering Down system.."
			& .\power-down.ps1
		}
        "4" {
            Write-Host "Checking File System Permission..."
            & .\permission-ops.ps1 -value "view"
        }
        "5" {
            Write-Host "Modifying File System Permission..."
            & .\permission-ops.ps1 -value "modify"
        }
        "6" {
			Write-Host "Getting Date & Time..."
			& .\get-servertime.ps1
        }
		"7"{
			Write-Host "Setting Date & Time..."
            $response =& .\set-server-time.ps1
			Write-Host $response.message
		}
		"8"{
			Write-Host "Checking WiFi Connection..."
			& .\network-status.ps1
		}
		"9"{
			Write-Host "Setting up Wifi ..." 
			& .\setup-wifi.ps1
		}
		"10"{
			Write-Host "Checking Firmware version..."
			& .\firmware-ops.ps1 -choice 1
		}
		"11"{
			Write-Host "Flashing Firmware..."
			& .\flash-firmware.ps1
		}
		"12"{
			Write-Host "Getting SDK Version.."
			& .\get-workspace.ps1 -choice 1
		}
		"13"{
			Write-Host "Updating the SDK Version..."
			& .\update.ps1
		}
		"14"{
			Write-Host "Switching SDK..."
			& .\get-workspace.ps1 -choice 2
		}
		"15"{
			Write-Host "Switching Network..."
            .\network-change.ps1
		}
		"16"{
			Write-Host "`nExiting. Goodbye!" -ForegroundColor Yellow
		}
        default {
            Write-Host "Invalid choice. Please select a valid option." -ForegroundColor Red
        }
    }

    if ($choice -ne "16") {
        Write-Host ""
        Read-Host "Press Enter to return to the menu"
    }

} while ($choice -ne "16")
