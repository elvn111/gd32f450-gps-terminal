@echo off
rem PC regression for the K922 sentence parser + protocol sample frames.
rem Needs gcc (e.g. MinGW) in PATH; the Keil/ARM toolchain is not used here.
setlocal
chcp 65001 >nul
cd /d "%~dp0"

set CC=gcc
where %CC% >nul 2>nul || (echo [x] gcc not found in PATH & exit /b 1)

echo == building parser regression ==
%CC% -std=c99 -Wall -Wextra -I..\Template -o test_info_bsp.exe test_info_bsp.c ..\Template\info_bsp.c -lm
if errorlevel 1 (echo [x] build failed & exit /b 1)

echo == running parser regression ==
test_info_bsp.exe
if errorlevel 1 (echo [x] tests failed & exit /b 1)

echo == checking protocol sample frames ==
python tools\check_frames.py
if errorlevel 1 (echo [x] frame check failed & exit /b 1)

echo [ok] all PC tests passed
