# start_car.ps1 — Launch the TORCS SCR client (snakeoil3_v6)
# ---------------------------------------------------------------
# Usage:
#   .\start_car.ps1                          # default options
#   .\start_car.ps1 -Track forza -Car trb1  # with labels
#   .\start_car.ps1 -Port 3001 -Debug       # extra flags
#
# Before running:
#   1. Start TORCS
#   2. Race > Quick Race (or Practice)
#   3. Select the SCR robot for your car slot
#   4. Click New Race / Start
#   5. Run this script — it will wait up to 30 s for TORCS
# ---------------------------------------------------------------

param(
    [string]  $Track      = "unknown",
    [string]  $Car        = "unknown",
    [int]     $Port       = 3001,
    [string]  $TorcsHost  = "localhost",
    [int]     $Steps      = 100000,
    [int]     $Stage      = 3,
    [switch]  $Debug
)

# Resolve paths relative to this script's directory
$ScriptDir  = Split-Path -Parent $MyInvocation.MyCommand.Definition
$DriverScript = Join-Path $ScriptDir "snakeoil3_v6.py"

# Verify the driver script exists
if (-not (Test-Path $DriverScript)) {
    Write-Error "Driver script not found: $DriverScript"
    exit 1
}

# Find Python (prefer 'python', fall back to 'python3')
$Python = $null
foreach ($candidate in @("python", "python3")) {
    try {
        $ver = & $candidate --version 2>&1
        if ($ver -match "Python") {
            $Python = $candidate
            break
        }
    } catch {}
}

if ($null -eq $Python) {
    Write-Error "Python not found in PATH. Install Python 3 and ensure it is on your PATH."
    exit 1
}

Write-Host ""
Write-Host "=========================================" -ForegroundColor Cyan
Write-Host "  TORCS SCR Client  -  snakeoil3_v6"      -ForegroundColor Cyan
Write-Host "=========================================" -ForegroundColor Cyan
Write-Host "  Python  : $Python"
Write-Host "  Script  : $DriverScript"
Write-Host "  Host    : $TorcsHost"
Write-Host "  Port    : $Port"
Write-Host "  Track   : $Track"
Write-Host "  Car     : $Car"
Write-Host "  Steps   : $Steps"
Write-Host "  Stage   : $Stage"
if ($Debug) { Write-Host "  Debug   : ON" }
Write-Host ""
Write-Host "Waiting for TORCS to be ready..." -ForegroundColor Yellow
Write-Host "(Start TORCS -> Race -> New Race with SCR robot, then press any key here or just wait)"
Write-Host ""

# Build argument list
$PyArgs = @(
    $DriverScript,
    "--host",  $TorcsHost,
    "--port",  $Port,
    "--track", $Track,
    "--car",   $Car,
    "--steps", $Steps,
    "--stage", $Stage
)
if ($Debug) { $PyArgs += "--debug" }

# Run — blocking until the race ends or Ctrl+C
try {
    & $Python @PyArgs
    $exitCode = $LASTEXITCODE
} catch {
    Write-Error "Failed to launch Python: $_"
    exit 1
}

Write-Host ""
if ($exitCode -eq 0) {
    Write-Host "Race session ended cleanly (exit 0)." -ForegroundColor Green
} else {
    Write-Host "Script exited with code $exitCode." -ForegroundColor Red
}

Write-Host ""
Write-Host "Results written to: $(Join-Path $ScriptDir 'results')" -ForegroundColor Cyan
Write-Host "  lap_data.csv   - lap times, damage"
Write-Host "  track_data.csv - per-step telemetry (~5 Hz)"
Write-Host ""
