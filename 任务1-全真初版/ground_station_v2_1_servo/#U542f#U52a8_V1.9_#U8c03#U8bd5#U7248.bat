@echo off
setlocal EnableExtensions
cd /d "%~dp0"
title Ground Station V1.9 DEBUG - WEB 5001

set "PYTHON_EXE="
if exist "D:\anaconda\python.exe" set "PYTHON_EXE=D:\anaconda\python.exe"
if not defined PYTHON_EXE if exist "%USERPROFILE%\anaconda3\python.exe" set "PYTHON_EXE=%USERPROFILE%\anaconda3\python.exe"
if not defined PYTHON_EXE if exist "%USERPROFILE%\miniconda3\python.exe" set "PYTHON_EXE=%USERPROFILE%\miniconda3\python.exe"
if not defined PYTHON_EXE for /f "delims=" %%I in ('where python 2^>nul') do if not defined PYTHON_EXE set "PYTHON_EXE=%%I"

if not defined PYTHON_EXE (
  echo [ERROR] Python not found.
  pause
  exit /b 1
)

set "PYTHONUTF8=1"
echo =====================================================
echo   GROUND STATION V1.9 DEBUG BUILD
echo   WEB: http://127.0.0.1:5001
echo   UDP GS PORT: 8889
echo =====================================================
start "" http://127.0.0.1:5001
"%PYTHON_EXE%" -u "%~dp0app.py" --config "%~dp0config.json"

if errorlevel 1 (
  echo.
  echo [ERROR] V1.9 ground station exited. Check whether port 5001 or UDP 8889 is occupied.
  pause
)
endlocal
