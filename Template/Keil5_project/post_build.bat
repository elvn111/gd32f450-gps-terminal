@echo off
setlocal enabledelayedexpansion

set "TARGET=%~1"

rem ---------- Step 1: axf -> bin ----------
set "FROMELF=%KEIL_TOOL_PATH%\fromelf.exe"
if not defined KEIL_TOOL_PATH set "FROMELF=C:\Keil_v5\ARM\ARMCC\Bin\fromelf.exe"
if not exist "%FROMELF%" (
    echo [POSTBUILD] fromelf not found at "%FROMELF%"!
    exit /b 1
)
"%FROMELF%" --bin ".\output\%TARGET%.axf" --output ".\obj_bin\%TARGET%.bin"
if errorlevel 1 (
    echo [POSTBUILD] fromelf failed!
    exit /b 1
)

rem ---------- Step 2: parse AppVer.h ----------
set "MAJ_H="
set "MAJ_L="
set "MINOR="

for /f "tokens=3" %%v in ('findstr /c:"APP_MAJOR_VER_H" "..\inc\AppVer.h"') do set "MAJ_H=%%v"
for /f "tokens=3" %%v in ('findstr /c:"APP_MAJOR_VER_L" "..\inc\AppVer.h"') do set "MAJ_L=%%v"
for /f "tokens=3" %%v in ('findstr /c:"APP_MINOR_VER"  "..\inc\AppVer.h"') do set "MINOR=%%v"

if not defined MAJ_H (
    echo [POSTBUILD] failed to parse AppVer.h!
    exit /b 1
)

rem ---------- Step 3: hex to decimal ----------
set /a V1=%MAJ_H%
set /a V2=%MAJ_L%
set /a V3=%MINOR%

rem ---------- Step 4: rename ----------
set "OUTNAME=%TARGET%_%V1%.%V2%.%V3%.bin"
if exist ".\obj_bin\%OUTNAME%" del ".\obj_bin\%OUTNAME%"
move ".\obj_bin\%TARGET%.bin" ".\obj_bin\%OUTNAME%" >nul

if errorlevel 1 (
    echo [POSTBUILD] rename failed!
    exit /b 1
)

echo [POSTBUILD] %OUTNAME%

rem ---------- Step 5: merge bootloader + app ----------
rem  output: .\%TARGET%_merge.hex   (override the name with env var MERGE_NAME)
if not defined MERGE_NAME set "MERGE_NAME=%TARGET%_merge.hex"
if not defined SREC_CAT   set "SREC_CAT=.\srec_cat.exe"
set "BOOT_HEX=.\XP260304_bootloader.hex"
set "APP_HEX=.\output\%TARGET%.hex"
set "MERGE_HEX=.\%MERGE_NAME%"

if not exist "%BOOT_HEX%" (
    echo [POSTBUILD] bootloader hex not found: %BOOT_HEX%
    exit /b 1
)
if not exist "%APP_HEX%" (
    echo [POSTBUILD] app hex not found: %APP_HEX%
    exit /b 1
)
if not exist "%SREC_CAT%" (
    echo [POSTBUILD] srec_cat not found: %SREC_CAT%
    exit /b 1
)

rem --- overlap guard: an app linked at 0x08000000 would overwrite the bootloader ---
set "APP_BASE="
for /f "usebackq delims=" %%L in ("%APP_HEX%") do (
    set "APP_BASE=%%L"
    goto :app_base_done
)
:app_base_done
echo [POSTBUILD] app hex first record: %APP_BASE%
if /i "%APP_BASE%"==":020000040800F2" (
    echo [POSTBUILD] *** merge aborted: app is linked at 0x08000000 and overlaps the bootloader ***
    echo [POSTBUILD]     set Options for Target - Target - IROM1 start address to 0x08020000 and rebuild
    exit /b 1
)

"%SREC_CAT%" "%BOOT_HEX%" -Intel "%APP_HEX%" -Intel -o "%MERGE_HEX%" -Intel
if errorlevel 1 (
    echo [POSTBUILD] srec_cat merge FAILED
    exit /b 1
)
if not exist "%MERGE_HEX%" (
    echo [POSTBUILD] merge output missing: %MERGE_HEX%
    exit /b 1
)
echo [POSTBUILD] %MERGE_HEX%

exit /b 0
