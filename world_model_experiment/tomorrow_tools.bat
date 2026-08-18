@echo off
setlocal EnableDelayedExpansion

set "MODE=%~1"
if "%MODE%"=="" goto :usage

if /I "%MODE%"=="preflight" (
    powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%~dp0tomorrow_tools.ps1" -Mode preflight
    exit /b !ERRORLEVEL!
)

if /I "%MODE%"=="camera" (
    set "COMMAND_TEXT=%~2"
    if not defined COMMAND_TEXT set "COMMAND_TEXT=抓取红色杯子"
    powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%~dp0tomorrow_tools.ps1" -Mode camera -CommandText "%COMMAND_TEXT%"
    exit /b !ERRORLEVEL!
)

if /I "%MODE%"=="robot" (
    set "ROBOT_IP=%~2"
    if not defined ROBOT_IP set "ROBOT_IP=%LEBAI_ROBOT_IP%"
    if not defined ROBOT_IP (
        echo Set LEBAI_ROBOT_IP or pass the robot address as the second argument.
        exit /b 2
    )
    powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%~dp0tomorrow_tools.ps1" -Mode robot -RobotIp "%ROBOT_IP%"
    exit /b !ERRORLEVEL!
)

:usage
echo Usage:
echo   tomorrow_tools.bat preflight
echo   tomorrow_tools.bat camera "pick command"
echo   tomorrow_tools.bat robot ^<robot-ip^>
exit /b 2
