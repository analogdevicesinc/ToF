param (
    [Parameter(Mandatory = $false)]
    [string]$infile,

    [Parameter(Mandatory = $false)]
    [string]$outfile,

    [switch]$help
)

function Show-Help {
    Write-Host "Usage: ./extract-after-atat.ps1 -infile <input.txt> -outfile <output.txt>"
    Write-Host ""
    Write-Host "Options:"
    Write-Host "  -infile     Path to input file to process (required)"
    Write-Host "  -outfile    Path to output file to write (required)"
    Write-Host "  -help       Show this help message"
    exit
}

# Show help if requested or missing required args
if ($help -or !$infile -or !$outfile) {
    Show-Help
}

# Clear or create output file
'' | Set-Content $outfile

# Read all lines and process them directly
Get-Content $infile | ForEach-Object {
    if ($_ -match "@@,(.*)") {
        $Matches[1] | Add-Content $outfile
    }
}
