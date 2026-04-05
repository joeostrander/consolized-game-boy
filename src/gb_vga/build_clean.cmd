@ECHO OFF
SETLOCAL EnableDelayedExpansion

SET PLATFORM=rp2040
SET BOARD=pico
SET USE_RGB332_PCB=0
SET USE_NES_CLASSIC_OR_WII=0
SET PICOTOOL_CMAKE_ARG=

IF /I "%~1"=="rp2040" (
    SET PLATFORM=rp2040
    SET BOARD=pico
)
IF /I "%~1"=="rp2350" (
    SET PLATFORM=rp2350
    SET BOARD=pico2
)

IF /I "%~2"=="rgb222" SET USE_RGB332_PCB=0
IF /I "%~2"=="rgb332" SET USE_RGB332_PCB=1
IF /I "%~3"=="nes" SET USE_NES_CLASSIC_OR_WII=0
IF /I "%~3"=="wii" SET USE_NES_CLASSIC_OR_WII=1

IF NOT "%~1"=="" (
    IF /I NOT "%~1"=="rp2040" IF /I NOT "%~1"=="rp2350" (
        echo Usage: %~n0 [rp2040^|rp2350] [rgb222^|rgb332] [nes^|wii]
        echo.
        echo Examples:
        echo   %~n0
        echo   %~n0 rp2040
        echo   %~n0 rp2040 rgb222
        echo   %~n0 rp2040 rgb332
        echo   %~n0 rp2040 rgb222 nes
        echo   %~n0 rp2040 rgb332 wii
        echo   %~n0 rp2350 rgb222 nes
        echo   %~n0 rp2350 rgb332 wii
        exit /b 1
    )
)

IF NOT "%~2"=="" (
    IF /I NOT "%~2"=="rgb222" IF /I NOT "%~2"=="rgb332" (
        echo Usage: %~n0 [rp2040^|rp2350] [rgb222^|rgb332] [nes^|wii]
        echo.
        echo Examples:
        echo   %~n0
        echo   %~n0 rp2040
        echo   %~n0 rp2040 rgb222
        echo   %~n0 rp2040 rgb332
        echo   %~n0 rp2040 rgb222 nes
        echo   %~n0 rp2040 rgb332 wii
        echo   %~n0 rp2350 rgb222 nes
        echo   %~n0 rp2350 rgb332 wii
        exit /b 1
    )
)

IF NOT "%~3"=="" (
    IF /I NOT "%~3"=="nes" IF /I NOT "%~3"=="wii" (
        echo Usage: %~n0 [rp2040^|rp2350] [rgb222^|rgb332] [nes^|wii]
        echo.
        echo Examples:
        echo   %~n0
        echo   %~n0 rp2040
        echo   %~n0 rp2040 rgb222
        echo   %~n0 rp2040 rgb332
        echo   %~n0 rp2040 rgb222 nes
        echo   %~n0 rp2040 rgb332 wii
        echo   %~n0 rp2350 rgb222 nes
        echo   %~n0 rp2350 rgb332 wii
        exit /b 1
    )
)


SET VIDEO_SUFFIX=rgb222
SET CONTROLLER_SUFFIX=nes

IF "%USE_RGB332_PCB%"=="1" SET VIDEO_SUFFIX=rgb332
IF "%USE_NES_CLASSIC_OR_WII%"=="1" SET CONTROLLER_SUFFIX=wii
SET OUTPUT_NAME=gb_vga_%PLATFORM%_%VIDEO_SUFFIX%_%CONTROLLER_SUFFIX%

echo Configuration:
echo   PLATFORM=%PLATFORM%
echo   BOARD=%BOARD%
echo   USE_RGB332_PCB=%USE_RGB332_PCB%
echo   USE_NES_CLASSIC_OR_WII=%USE_NES_CLASSIC_OR_WII%
echo.

call "%~dp0..\find_picotool.cmd" gb_vga

cd %~dp0
rmdir /s /q build-mingw 2>nul
mkdir build-mingw
cd build-mingw
echo.
echo ===== Running CMake Configuration =====
cmake -G "MinGW Makefiles" -DPICO_PLATFORM=%PLATFORM% -DPICO_BOARD=%BOARD% -DUSE_RGB332_PCB=%USE_RGB332_PCB% -DUSE_NES_CLASSIC_OR_WII=%USE_NES_CLASSIC_OR_WII% %PICOTOOL_CMAKE_ARG% ..
if %errorlevel% neq 0 (
    echo.
    echo *** CMAKE CONFIGURATION FAILED ***
    exit /b %errorlevel%
)

echo.
echo ===== Building Project =====
cmake --build . --parallel 4
REM cmake --build . --target dmg
if %errorlevel% neq 0 (
    echo.
    echo *** BUILD FAILED ***
    exit /b %errorlevel%
)

echo.
echo ===== Build Successful =====
echo Binary size:
dir gb_vga.elf | find "gb_vga.elf"

REM Auto-detect RPI-RP2 drive for easy copying of UF2 file after build
set "DRIVE_LETTER="
call "%~dp0..\find_rpi_rp2_drive.cmd"
if defined DRIVE_LETTER (
    echo.
    echo ===== Copying UF2 to %DRIVE_LETTER%\ =====
    copy /y gb_vga.uf2 %DRIVE_LETTER%\ >nul
    if %errorlevel% neq 0 (
        echo *** COPY FAILED ***
        exit /b %errorlevel%
    )
    echo Copy successful!
) else (
    echo RPI-RP2 drive not found, skipping copy
)

copy /y gb_vga.uf2 ..\%OUTPUT_NAME%.uf2 >nul

echo.
echo Output: %OUTPUT_NAME%.uf2
echo ===== ALL DONE =====
cd %~dp0
exit /b 0