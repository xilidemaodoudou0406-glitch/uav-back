param(
    [ValidateSet('normal', 'stale', 'reconnect')]
    [string]$Scenario,
    [string]$Backend = 'http://127.0.0.1:8000'
)
$ErrorActionPreference = 'Stop'
$backendRoot = Split-Path -Parent $PSScriptRoot
$pythonPath = Join-Path $backendRoot 'venv\Scripts\python.exe'
$gatewayPath = Join-Path $PSScriptRoot 'mock_drone_gateway.py'
if (-not (Test-Path -LiteralPath $pythonPath)) { throw 'Missing backend venv. Install requirements first.' }
if (-not $Scenario) {
    Write-Host '1: Normal telemetry   2: Pause telemetry   3: Disconnect and reconnect'
    $choice = Read-Host 'Choose scenario (default 1)'
    $Scenario = switch ($choice) { '2' { 'stale' } '3' { 'reconnect' } default { 'normal' } }
}
& $pythonPath $gatewayPath --backend $Backend --scenario $Scenario
exit $LASTEXITCODE
