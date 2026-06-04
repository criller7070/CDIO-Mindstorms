param(
    [string]$Ev3Host = "robot@ev3dev.local",
    [string]$Ev3Path = "/home/robot"
)

$ErrorActionPreference = "Stop"

$repoRoot = Split-Path -Parent $PSScriptRoot
$robotDir = Join-Path $repoRoot "robot"

if (-not (Test-Path $robotDir)) {
    throw "robot directory not found: $robotDir"
}

$files = Get-ChildItem -Path $robotDir -File
if (-not $files) {
    throw "No files found in $robotDir"
}

Write-Host "Deploying robot-only files to $Ev3Host:$Ev3Path" -ForegroundColor Cyan
Write-Host "Source: $robotDir" -ForegroundColor Cyan

foreach ($file in $files) {
    Write-Host "Uploading $($file.Name)..."
    & scp $file.FullName "$Ev3Host`:$Ev3Path/$($file.Name)"
    if ($LASTEXITCODE -ne 0) {
        throw "Failed to upload $($file.Name)"
    }
}

Write-Host "Done. Uploaded $($files.Count) file(s) from robot/ only." -ForegroundColor Green