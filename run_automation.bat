@echo off
chcp 65001 >nul
cd /d "%~dp0"
echo ===============================================
echo  Electrode pickup and vision positioning
echo ===============================================
python code\pc\automation_console.py
echo.
pause
