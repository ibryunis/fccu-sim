#!/bin/sh
# Build (if needed) and launch the FCCU simulator. Dashboard: http://localhost:8000
cd "$(dirname "$0")" || exit 1

if [ ! -x build/fccu_sim ]; then
    cmake -S . -B build -DCMAKE_BUILD_TYPE=Release || exit 1
    cmake --build build -j "$(nproc)" || exit 1
fi

xdg-open http://localhost:8000 2>/dev/null &
exec build/fccu_sim
