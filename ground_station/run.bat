@echo off
rem Launch the Ground Station from source on Windows.
rem Requires Python 3.10 (3.10.12) and Bun (first run only, to build the map frontend).
setlocal

cd /d "%~dp0"

if not exist ".venv\Scripts\python.exe" (
    echo Creating virtual environment in .venv ...
    py -3 -m venv .venv || python -m venv .venv
)

set "PYBIN=.venv\Scripts\python.exe"
"%PYBIN%" -m pip install --upgrade pip >nul
"%PYBIN%" -m pip install -r "..\.devcontainer\groundstation_required_pip_packages.txt"

if not exist "src\widgets\map_widget\dist\index.html" (
    echo Built frontend not found. Building with bun ...
    where bun >nul 2>nul || (
        echo Bun is not installed. Install it from https://bun.sh or use a packaged release.
        exit /b 1
    )
    bun install
    bun run build
)

rem Clean up any stale state from a crashed previous run.
if exist "app_data\git_ignore\app_state.json" del /f "app_data\git_ignore\app_state.json"

"%PYBIN%" src\main.py
endlocal
