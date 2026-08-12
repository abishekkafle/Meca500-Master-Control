@echo off
title Launch MECA 500 Master Control Suite
echo ========================================================
echo   Launching MECA 500 Master Control Platform
echo ========================================================
echo.

echo Starting Python Backend Server (FastAPI on Port 8000)...
start "MECA500 Backend API" cmd /k "cd /d %~dp0backend && pip install -r requirements.txt && python server.py"

echo.
echo Starting React Frontend Dashboard (Port 3000)...
start "MECA500 React App" cmd /k "cd /d %~dp0frontend && npm install && set BROWSER=none && npm start"

echo.
echo Both services are initializing!
echo - API Docs: http://localhost:8000/docs
echo - Web App:  http://localhost:3000
echo.
pause
