@echo off
setlocal EnableDelayedExpansion

SET DRIVE_LETTER=
if "%PLATFORM%"=="" SET PLATFORM=rp2350
if "%BOARD%"=="" SET BOARD=pico2
if "%RESOLUTION_MODE%"=="" SET RESOLUTION_MODE=2
if "%USE_NES_CLASSIC_OR_WII%"=="" SET USE_NES_CLASSIC_OR_WII=1

IF /I "%~1"=="rp2040" (
    SET PLATFORM=rp2040
    SET BOARD=pico
)
IF /I "%~1"=="rp2350" (
    SET PLATFORM=rp2350
    SET BOARD=pico2
)

IF "%~2"=="0" SET RESOLUTION_MODE=0
IF "%~2"=="1" SET RESOLUTION_MODE=1
IF "%~2"=="2" SET RESOLUTION_MODE=2
IF /I "%~3"=="nes" SET USE_NES_CLASSIC_OR_WII=0
IF /I "%~3"=="wii" SET USE_NES_CLASSIC_OR_WII=1

IF NOT "%~1"=="" (
    IF /I NOT "%~1"=="rp2040" IF /I NOT "%~1"=="rp2350" (
        echo Usage: %~n0 [rp2040^|rp2350] [0^|1^|2] [nes^|wii]
        echo.
        echo Examples:
        echo   %~n0
        echo   %~n0 rp2350
        echo   %~n0 rp2350 2 wii
        echo   %~n0 rp2040 0 nes
        exit /b 1
    )
)

IF NOT "%~2"=="" (
    IF NOT "%~2"=="0" IF NOT "%~2"=="1" IF NOT "%~2"=="2" (
        echo Usage: %~n0 [rp2040^|rp2350] [0^|1^|2] [nes^|wii]
        echo.
        echo Examples:
        echo   %~n0
        echo   %~n0 rp2350
        echo   %~n0 rp2350 2 wii
        echo   %~n0 rp2040 0 nes
        exit /b 1
    )
)

IF NOT "%~3"=="" (
    IF /I NOT "%~3"=="nes" IF /I NOT "%~3"=="wii" (
        echo Usage: %~n0 [rp2040^|rp2350] [0^|1^|2] [nes^|wii]
        echo.
        echo Examples:
        echo   %~n0
        echo   %~n0 rp2350
        echo   %~n0 rp2350 2 wii
        echo   %~n0 rp2040 0 nes
        exit /b 1
    )
)

SET PICOTOOL_CMAKE_ARG=
SET CONTROLLER_SUFFIX=nes
IF "%USE_NES_CLASSIC_OR_WII%"=="1" SET CONTROLLER_SUFFIX=wii
SET OUTPUT_NAME=dmg_dvi_%PLATFORM%_res%RESOLUTION_MODE%_%CONTROLLER_SUFFIX%

echo Configuration:
echo   PLATFORM=%PLATFORM%
echo   BOARD=%BOARD%
echo   RESOLUTION_MODE=%RESOLUTION_MODE%
echo   USE_NES_CLASSIC_OR_WII=%USE_NES_CLASSIC_OR_WII%
echo.

call "%~dp0..\find_picotool.cmd" gb_dvi

cd %~dp0

if exist build\apps\dmg_dvi\CMakeFiles\dmg_dvi.dir\main.c.obj del /q build\apps\dmg_dvi\CMakeFiles\dmg_dvi.dir\main.c.obj
if exist build\apps\dmg_dvi\CMakeFiles\dmg_dvi.dir\main.c.obj.d del /q build\apps\dmg_dvi\CMakeFiles\dmg_dvi.dir\main.c.obj.d
if exist build\elf2uf2\CMakeFiles\elf2uf2.dir\main.cpp.obj del /q build\elf2uf2\CMakeFiles\elf2uf2.dir\main.cpp.obj
if exist build\elf2uf2\CMakeFiles\elf2uf2.dir\main.cpp.obj.d del /q build\elf2uf2\CMakeFiles\elf2uf2.dir\main.cpp.obj.d
if exist build\pioasm\CMakeFiles\pioasm.dir\main.cpp.obj del /q build\pioasm\CMakeFiles\pioasm.dir\main.cpp.obj
if exist build\pioasm\CMakeFiles\pioasm.dir\main.cpp.obj.d del /q build\pioasm\CMakeFiles\pioasm.dir\main.cpp.obj.d

set "NEEDS_CONFIGURE=0"
set "CACHE_MISMATCH=0"
if not exist build\CMakeCache.txt set "NEEDS_CONFIGURE=1"
if exist build\CMakeCache.txt (
    findstr /B /C:"PICO_PLATFORM:STRING=%PLATFORM%" build\CMakeCache.txt >nul || (
        set "NEEDS_CONFIGURE=1"
        set "CACHE_MISMATCH=1"
    )
    findstr /B /C:"PICO_BOARD:STRING=%BOARD%" build\CMakeCache.txt >nul || (
        set "NEEDS_CONFIGURE=1"
        set "CACHE_MISMATCH=1"
    )
    findstr /B /C:"RESOLUTION_MODE:STRING=%RESOLUTION_MODE%" build\CMakeCache.txt >nul || (
        set "NEEDS_CONFIGURE=1"
        set "CACHE_MISMATCH=1"
    )
    findstr /B /C:"USE_NES_CLASSIC_OR_WII:STRING=%USE_NES_CLASSIC_OR_WII%" build\CMakeCache.txt >nul || (
        set "NEEDS_CONFIGURE=1"
        set "CACHE_MISMATCH=1"
    )
)

if "%CACHE_MISMATCH%"=="1" (
    echo.
    echo ===== Build Settings Changed: Recreating Build Folder =====
    rmdir /s /q build
)

if "%NEEDS_CONFIGURE%"=="1" (
    echo.
    echo ===== Configuring Build Folder =====
    cmake -S . -B build -G "MinGW Makefiles" -DPICO_COPY_TO_RAM=1 -DPICO_PLATFORM=%PLATFORM% -DPICO_BOARD=%BOARD% -DRESOLUTION_MODE=%RESOLUTION_MODE% -DUSE_NES_CLASSIC_OR_WII=%USE_NES_CLASSIC_OR_WII% -DCMAKE_EXPORT_COMPILE_COMMANDS=ON %PICOTOOL_CMAKE_ARG%
    if %errorlevel% neq 0 (
        echo.
        echo *** CMAKE CONFIGURATION FAILED ***
        goto :build_fail
    )
)


echo.
echo ===== Building Project =====
cmake --build build --parallel 4
if %errorlevel% neq 0 (
    echo.
    echo *** BUILD FAILED ***
    goto :build_fail
)

echo.
echo ===== Build Successful =====
if exist build\apps\dmg_dvi\dmg_dvi.elf (
    echo Binary size:
    dir build\apps\dmg_dvi\dmg_dvi.elf | find "dmg_dvi.elf"
)

REM Auto-detect RPI-RP2 drive for easy copying of UF2 file after build
set "DRIVE_LETTER="
call "%~dp0..\find_rpi_rp2_drive.cmd"
if defined DRIVE_LETTER (
    echo.
    echo ===== Copying UF2 to %DRIVE_LETTER%\ =====
    copy /y build\apps\dmg_dvi\dmg_dvi.uf2 %DRIVE_LETTER%\ >nul
    if %errorlevel% neq 0 (
        echo *** COPY FAILED ***
        goto :build_fail
    )
    echo Copy successful!
) else (
    echo RPI-RP2 drive not found, skipping copy
)

copy /y build\apps\dmg_dvi\dmg_dvi.uf2 .\%OUTPUT_NAME%.uf2 >nul

echo.
echo Output: %OUTPUT_NAME%.uf2

goto :eof

:build_fail
echo Build or copy failed.
exit /b 1