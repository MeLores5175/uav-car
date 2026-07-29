@echo off
setlocal
cd /d "%~dp0"
title UAV + CAR MOCK
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
if not exist "%~dp0mock_devices.py" (
    echo [ERROR] mock_devices.py was not found.
    pause
    exit /b 1
)
echo Python: "%PYTHON_EXE%"
echo Starting exclusive UAV + CAR Mock...
"%PYTHON_EXE%" -u "%~dp0mock_devices.py"
echo.
echo [INFO] Mock process exited.
pause
endlocal
