@echo off
chcp 65001 >nul
cd /d "%~dp0"
echo ===============================================
echo  Drill flow console - SENSOR mode
echo ===============================================
python pc\full_flow.py
echo.
pause
