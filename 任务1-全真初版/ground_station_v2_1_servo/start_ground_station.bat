@echo off
setlocal EnableExtensions
cd /d "%~dp0"
title Land-Air Ground Station

set "PYTHON_EXE="
if exist "D:\anaconda\python.exe" set "PYTHON_EXE=D:\anaconda\python.exe"
if not defined PYTHON_EXE if exist "%USERPROFILE%\anaconda3\python.exe" set "PYTHON_EXE=%USERPROFILE%\anaconda3\python.exe"
if not defined PYTHON_EXE if exist "%USERPROFILE%\miniconda3\python.exe" set "PYTHON_EXE=%USERPROFILE%\miniconda3\python.exe"
if not defined PYTHON_EXE for /f "delims=" %%I in ('where python 2^>nul') do if not defined PYTHON_EXE set "PYTHON_EXE=%%I"

if not defined PYTHON_EXE (
    echo [ERROR] Python was not found.
    pause
    exit /b 1
)

if not exist "%~dp0app.py" (
    echo [ERROR] app.py was not found.
    pause
    exit /b 1
)
if not exist "%~dp0config.json" (
    echo [ERROR] config.json was not found.
    pause
    exit /b 1
)

set "PYTHONUTF8=1"
echo Python: "%PYTHON_EXE%"
echo Starting ground station with config.json...
"%PYTHON_EXE%" -u "%~dp0app.py" --config "%~dp0config.json"

if errorlevel 1 (
    echo.
    echo [ERROR] Ground station exited with code %errorlevel%.
    pause
)
endlocal
