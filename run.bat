@echo off
rem Build (if needed) and launch the FCCU simulator, then open the dashboard.
cd /d "%~dp0"

if not exist build\fccu_sim.exe (
    cmake -S . -B build -G "MinGW Makefiles" -DCMAKE_BUILD_TYPE=Release || exit /b 1
    cmake --build build -j || exit /b 1
)

start "" http://localhost:8000
build\fccu_sim.exe
