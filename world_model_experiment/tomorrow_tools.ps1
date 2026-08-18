param(
    [Parameter(Mandatory = $true)]
    [ValidateSet("preflight", "camera", "robot")]
    [string]$Mode,

    [string]$CommandText = "抓取红色杯子",
    [string]$RobotIp = $env:LEBAI_ROBOT_IP
)

$ErrorActionPreference = "Stop"
$ExperimentRoot = Split-Path -Parent $MyInvocation.MyCommand.Path
$ProjectRoot = Split-Path -Parent $ExperimentRoot
$PythonExe = Join-Path $ProjectRoot ".venv\Scripts\python.exe"
$SitePackages = Join-Path $ProjectRoot ".venv\Lib\site-packages"

if (-not (Test-Path -LiteralPath $PythonExe)) {
    throw "Project Python environment not found. Create .venv and install dependencies first."
}
if (-not (Test-Path -LiteralPath $SitePackages)) {
    throw "Hardware SDK site-packages not found: $SitePackages"
}

$env:PYTHONPATH = $SitePackages
$env:LEBAI_DRY_RUN = "1"
Remove-Item Env:LEBAI_ALLOW_REAL_MOTION -ErrorAction SilentlyContinue
Set-Location -LiteralPath $ProjectRoot

switch ($Mode) {
    "preflight" {
        & $PythonExe .\world_model_experiment\run_preflight.py `
            --report .\world_model_experiment\reports\python311_preflight.json
    }
    "camera" {
        if (-not $env:QWEN_API_KEY -and -not $env:DASHSCOPE_API_KEY) {
            throw "Set QWEN_API_KEY or DASHSCOPE_API_KEY in this PowerShell session first."
        }
        & $PythonExe .\world_model_experiment\capture_camera_only.py `
            --command $CommandText `
            --capture
    }
    "robot" {
        if (-not $RobotIp) {
            throw "Set LEBAI_ROBOT_IP or pass -RobotIp before running the read-only robot check."
        }
        & $PythonExe .\world_model_experiment\robot_readonly_check.py `
            --robot-ip $RobotIp `
            --confirm READ_ONLY
    }
}

if ($LASTEXITCODE -ne 0) {
    throw "Tool failed with exit code $LASTEXITCODE."
}
