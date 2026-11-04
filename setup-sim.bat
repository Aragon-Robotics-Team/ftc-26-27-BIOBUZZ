@echo off
rem Sets up this laptop for the sim and AdvantageScope (tools\advantagescope\setup_windows.ps1). Double-click it.
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0tools\advantagescope\setup_windows.ps1"
pause
