@echo off
REM ============================================================================
REM  WeldSense one-command launcher (Windows)
REM
REM    Double-click run.bat, or from a terminal:  run.bat
REM
REM  First run: creates a local Python virtual-env and installs dependencies.
REM  Every run: starts the host and opens the dashboard in your browser.
REM  The XIAO must be plugged in via a DATA-capable USB-C cable.
REM ============================================================================
setlocal
cd /d "%~dp0"

set "VENV=.venv"
set "PY=python"

REM ---- Check Python ----
where %PY% >nul 2>&1
if errorlevel 1 (
  echo ERROR: python not found on PATH.
  echo Install Python 3 from https://www.python.org/downloads/windows/
  echo and tick "Add python.exe to PATH" during setup.
  pause
  exit /b 1
)

REM ---- First-run setup ----
if not exist "%VENV%" (
  echo [setup] creating virtual environment...
  %PY% -m venv "%VENV%"
)

REM Install deps only when requirements.txt is newer than the marker file.
set "STAMP=%VENV%\deps-installed.txt"
set "NEEDINSTALL=0"
if not exist "%STAMP%" set "NEEDINSTALL=1"
for /f %%i in ('dir /b /o-d "host\requirements.txt" "%STAMP%" 2^>nul') do (
  if "%%i"=="requirements.txt" set "NEEDINSTALL=1"
  goto :afterstampcheck
)
:afterstampcheck
if "%NEEDINSTALL%"=="1" (
  echo [setup] installing dependencies...
  "%VENV%\Scripts\python.exe" -m pip install --quiet --upgrade pip
  "%VENV%\Scripts\python.exe" -m pip install --quiet -r "host\requirements.txt"
  echo installed > "%STAMP%"
)

echo [run] starting WeldSense host -^> http://127.0.0.1:8765   (close this window to stop)
REM --open makes the Python host open the browser once it is actually serving.
"%VENV%\Scripts\python.exe" host\weldsense_host.py --open %*
