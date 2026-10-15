#!/bin/sh
# PC regression for the K922 sentence parser + protocol sample frames.
# Needs gcc (CC=... to override) and python3.
set -e
cd "$(dirname "$0")"
CC=${CC:-gcc}

echo "== building parser regression =="
$CC -std=c99 -Wall -Wextra -I../Template -o test_info_bsp_test test_info_bsp.c ../Template/info_bsp.c -lm

echo "== running parser regression =="
./test_info_bsp_test

echo "== checking protocol sample frames =="
if command -v python3 >/dev/null 2>&1; then
    python3 tools/check_frames.py
else
    python tools/check_frames.py
fi

echo "[ok] all PC tests passed"
