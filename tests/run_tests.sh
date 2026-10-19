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
    gcc -std=c99 -Wall -Wextra -I../Template -o bin_cmd_test.exe test_bin_cmd.c ../Template/bin_cmd.c
./bin_cmd_test.exe
gcc -std=c99 -Wall -Wextra -I../Template -o cli_bridge_test.exe test_cli_bridge.c ../Template/cli.c
./cli_bridge_test.exe
python tools/check_frames.py
python tools/check_doc.py "../Template/时变TXP系列定位终端通信协议集V1.0.0_20261016.md"
fi

echo "[ok] all PC tests passed"
