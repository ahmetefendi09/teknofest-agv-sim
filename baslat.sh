#!/usr/bin/env bash
cd "$(dirname "$0")"
chmod +x ./agv_sim 2>/dev/null
if [ ! -f ./agv_sim ]; then
    echo "Derleniyor..."
    make
fi
./agv_sim
