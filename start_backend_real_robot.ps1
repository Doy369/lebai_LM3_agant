$ErrorActionPreference = "Stop"

$ProjectRoot = Split-Path -Parent $MyInvocation.MyCommand.Path
$PythonCandidates = @(
    (Join-Path $ProjectRoot ".venv\Scripts\python.exe"),
    "python"
)

$PythonExe = $null
foreach ($Candidate in $PythonCandidates) {
    if ($Candidate -ne "python" -and -not (Test-Path -LiteralPath $Candidate)) {
        continue
    }
    try {
        $null = & $Candidate --version 2>&1
        if ($LASTEXITCODE -eq 0) {
            $PythonExe = $Candidate
            break
        }
    } catch {
        continue
    }
}

if (-not $PythonExe) {
    throw "No working Python interpreter found. Please install Python or repair .venv."
}

$env:LEBAI_DRY_RUN = "0"
$env:LEBAI_ALLOW_REAL_MOTION = "YES"
$env:LEBAI_WEB_HOST = "127.0.0.1"
$env:LEBAI_WEB_PORT = "8001"

if (-not $env:LEBAI_ROBOT_IP) {
    throw "Set LEBAI_ROBOT_IP in the current PowerShell session before starting real-robot mode."
}
if (-not $env:LEBAI_WEB_TOKEN) {
    throw "Set LEBAI_WEB_TOKEN to a strong random value in the current PowerShell session."
}
Write-Host "Robot address and Web control token loaded from environment variables." -ForegroundColor Yellow

# 在这里填入阿里云百炼 / DashScope API Key，用于“当前画面抓取规划”。
# 建议在当前 PowerShell 会话或系统环境变量中预先设置：
# $env:DASHSCOPE_API_KEY = 'your-key'
# 或：
# $env:QWEN_API_KEY = 'your-key'

$env:QWEN_MODEL = "qwen3.5-plus"

if (-not $env:QWEN_API_KEY -and -not $env:DASHSCOPE_API_KEY) {
    Write-Host "QWEN_API_KEY / DASHSCOPE_API_KEY is not set. Camera planning will not call Qwen." -ForegroundColor Yellow
} else {
    $qwenKeyLength = if ($env:QWEN_API_KEY) { $env:QWEN_API_KEY.Length } else { 0 }
    $dashscopeKeyLength = if ($env:DASHSCOPE_API_KEY) { $env:DASHSCOPE_API_KEY.Length } else { 0 }
    Write-Host "Qwen API key configured. QWEN_API_KEY length=$qwenKeyLength, DASHSCOPE_API_KEY length=$dashscopeKeyLength." -ForegroundColor Green
}

Set-Location -LiteralPath $ProjectRoot
& $PythonExe .\09_module4_fastapi_backend.py
