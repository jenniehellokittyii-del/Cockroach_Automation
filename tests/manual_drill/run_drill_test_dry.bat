@echo off
chcp 65001 >nul
cd /d "%~dp0\..\.."
echo ===============================================
echo  Drill console (DRY mode, no sensor)
echo ===============================================
python tests\manual_drill\drill_console.py --dry
echo.
pause
