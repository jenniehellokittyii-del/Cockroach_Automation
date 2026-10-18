@echo off
chcp 65001 >nul
cd /d "%~dp0\..\.."
echo ===============================================
echo  Drill console (real mode, sensor required)
echo ===============================================
python tests\manual_drill\drill_console.py
echo.
pause
