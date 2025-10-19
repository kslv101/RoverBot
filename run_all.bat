@echo off
start cmd /k call run_robot.bat
timeout /t 3 /nobreak >nul
start cmd /k call run_gui.bat
echo.
echo Open in Browser: http://localhost:5000
pause