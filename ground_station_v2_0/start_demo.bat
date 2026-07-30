@echo off
setlocal EnableExtensions
cd /d "%~dp0"
title Land-Air Ground Station - Mock Demo

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

for %%F in (app.py config.mock.json mock_devices.py) do (
    if not exist "%~dp0%%F" (
        echo [ERROR] %%F was not found in this folder.
        pause
        exit /b 1
    )
)

set "PYTHONUTF8=1"

rem Stop only the previous Mock PID recorded by this project.
if exist "%~dp0mock_instance.pid" (
    set /p OLD_MOCK_PID=<"%~dp0mock_instance.pid"
    if defined OLD_MOCK_PID (
        echo Stopping previous Mock PID %OLD_MOCK_PID%...
        taskkill /PID %OLD_MOCK_PID% /T /F >nul 2>&1
    )
)

rem Also close an old console created by this launcher. No PowerShell is used.
taskkill /FI "WINDOWTITLE eq UAV-CAR-MOCK*" /T /F >nul 2>&1
del /q "%~dp0mock_instance.pid" >nul 2>&1
del /q "%~dp0mock_ready.flag" >nul 2>&1

echo Python: "%PYTHON_EXE%"
echo Starting UAV and CAR Mock...
start "UAV-CAR-MOCK" /D "%~dp0" "%PYTHON_EXE%" -u "%~dp0mock_devices.py"

timeout /t 2 /nobreak >nul

echo Probing UAV and CAR command channels...
"%PYTHON_EXE%" -u "%~dp0mock_devices.py" --probe --probe-timeout 2
if errorlevel 1 (
    echo.
    echo [ERROR] Mock command self-test failed.
    echo Check the UAV-CAR-MOCK window. Ports 8888 and 8890 may be occupied.
    pause
    exit /b 2
)

echo Mock command channels are OK.
echo Starting ground station with config.mock.json...
"%PYTHON_EXE%" -u "%~dp0app.py" --config "%~dp0config.mock.json"

if errorlevel 1 (
    echo.
    echo [ERROR] Ground station exited with code %errorlevel%.
    pause
)
endlocal
