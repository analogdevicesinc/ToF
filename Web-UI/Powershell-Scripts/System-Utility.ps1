# Master PowerShell Script: master-menu.ps1

function Show-Menu {
    Clear-Host
    Write-Host "===============================================" -ForegroundColor Cyan
    Write-Host "           Welcome to System Utility           " -ForegroundColor Green
    Write-Host "===============================================" -ForegroundColor Cyan
    Write-Host ""
    Write-Host "1.  Reboot System         	10. Get Server date & time"
    Write-Host "2.  Reset ADSD3500       	11. Setup WiFi "
	Write-Host "3.  Power Down			12. Check WiFi Connection "
    Write-Host "4.  Check Permission	 	13. Check Firmware Version"
    Write-Host "5.  Modify Permission	 	14. Flash Firmware"
    Write-Host "6.  Network Switch       	15. Get Workspace"
    Write-Host "7.  NVM Read             	16. Switch Workspace"
    Write-Host "8.  CCB Read             	17. Update Workspace"
    Write-Host "9.  Set Server date & time 	18. Exit "
    Write-Host ""
}


do {
    Show-Menu
    $choice = Read-Host "Enter your choice (1-18)"

    switch ($choice) {
        "1" {
            Write-Host "Rebooting system..."
            & .\system-reboot.ps1
        }
        "2" {
            Write-Host "Resetting ADSD3500..."
            & .\adsd3500-reset.ps1
        }
		"3" {
			Write-Host "Powering Down system.."
			& .\power-down.ps1
		}
        "4" {
            Write-Host "Checking Permission..."
            & .\permission-ops.ps1 -value "view"
        }
        "5" {
            Write-Host "Modifying Permission..."
            & .\permission-ops.ps1 -value "modify"
        }
        "6" {
            Write-Host "Switching Network..."
            .\network-change.ps1
        }
        "7" {
			Write-Host "Reading NVM..."
			& .\read-ops.ps1 -choice 1
        }
		"8"{
			Write-Host "Reading CCB..."
			& .\read-ops.ps1 -choice 2
		}
		"9"{
			Write-Host "Setting Server Date & Time..."
            $response =& .\set-server-time.ps1
			Write-Host $response.message
		}
		"10"{
			Write-Host "Getting Server Date & Time..."
			& .\get-servertime.ps1
		}
		"11"{
			Write-Host "Setting up Wifi ..." 
			& .\setup-wifi.ps1
		}
		"12"{
			Write-Host "Checking WiFi Connection..."
			& .\network-status.ps1
		}
		"13"{
			Write-Host "Checking Firmware version..."
			& .\firmware-ops.ps1 -choice 1
		}
		"14"{
			Write-Host "Flashing Firmware..."
			& .\flash-firmware.ps1
		}
		"15"{
			Write-Host "Getting Workspace.."
			& .\get-workspace.ps1 -choice 1
		}
		"16"{
			Write-Host "Switching Workspace..."
			& .\get-workspace.ps1 -choice 2
		}
		"17"{
			Write-Host "Updating the Workspace"
			& .\update.ps1
		}
		"18"{
			Write-Host "`nExiting. Goodbye!" -ForegroundColor Yellow
		}
        default {
            Write-Host "Invalid choice. Please select a valid option." -ForegroundColor Red
        }
    }

    if ($choice -ne "18") {
        Write-Host ""
        Read-Host "Press Enter to return to the menu"
    }

} while ($choice -ne "18")
