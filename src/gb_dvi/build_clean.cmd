@ECHO OFF
SETLOCAL EnableDelayedExpansion

SET PLATFORM=rp2040
SET BOARD=pico
REM SET PLATFORM=rp2350
REM SET BOARD=pico2
SET RESOLUTION_MODE=2
SET USE_NES_CLASSIC_OR_WII=1
SET PICOTOOL_CMAKE_ARG=

REM 0 = 640x480, horizontally scaled x4, vertically x3 --> 640x480 Full screen
REM 1 = 800x600, horizontally scaled x4, vertically x4 --> 640x576 window
REM 2 = 640x480, horizontally scaled x2, vertically x2 --> 320x288 window

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
        echo   %~n0 rp2040
        echo   %~n0 rp2040 0 nes
        echo   %~n0 rp2350 2 wii
        exit /b 1
    )
)

IF NOT "%~2"=="" (
    IF NOT "%~2"=="0" IF NOT "%~2"=="1" IF NOT "%~2"=="2" (
        echo Usage: %~n0 [rp2040^|rp2350] [0^|1^|2] [nes^|wii]
        echo.
        echo Examples:
        echo   %~n0
        echo   %~n0 rp2040
        echo   %~n0 rp2040 0 nes
        echo   %~n0 rp2350 2 wii
        exit /b 1
    )
)

IF NOT "%~3"=="" (
    IF /I NOT "%~3"=="nes" IF /I NOT "%~3"=="wii" (
        echo Usage: %~n0 [rp2040^|rp2350] [0^|1^|2] [nes^|wii]
        echo.
        echo Examples:
        echo   %~n0
        echo   %~n0 rp2040
        echo   %~n0 rp2040 0 nes
        echo   %~n0 rp2350 2 wii
        exit /b 1
    )
)

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
rmdir /s /q build 2>nul
mkdir build
cd build
echo.
echo ===== Running CMake Configuration =====
cmake -G "MinGW Makefiles" -DPICO_COPY_TO_RAM=1 -DPICO_PLATFORM=%PLATFORM% -DPICO_BOARD=%BOARD% -DRESOLUTION_MODE=%RESOLUTION_MODE% -DUSE_NES_CLASSIC_OR_WII=%USE_NES_CLASSIC_OR_WII% %PICOTOOL_CMAKE_ARG% ..
if %errorlevel% neq 0 (
    echo.
    echo *** CMAKE CONFIGURATION FAILED ***
    exit /b %errorlevel%
)

echo.
echo ===== Building Project =====
cmd /c make -j4
REM cmake --build . --target dmg_dvi
if %errorlevel% neq 0 (
    echo.
    echo *** BUILD FAILED ***
    exit /b %errorlevel%
)

echo.
echo ===== Build Successful =====
echo Binary size:
dir apps\dmg_dvi\dmg_dvi.elf | find "dmg_dvi.elf"

REM Auto-detect RPI-RP2 drive for easy copying of UF2 file after build
set "DRIVE_LETTER="
call "%~dp0..\find_rpi_rp2_drive.cmd"
if defined DRIVE_LETTER (
    echo.
    echo ===== Copying UF2 to %DRIVE_LETTER%\ =====
    copy /y apps\dmg_dvi\dmg_dvi.uf2 %DRIVE_LETTER%\ >nul
    if %errorlevel% neq 0 (
        echo *** COPY FAILED ***
        exit /b %errorlevel%
    )
    echo Copy successful!
) else (
    echo RPI-RP2 drive not found, skipping copy
)

copy /y apps\dmg_dvi\dmg_dvi.uf2 ..\%OUTPUT_NAME%.uf2 >nul

echo.
echo Output: %OUTPUT_NAME%.uf2
echo ===== ALL DONE =====
cd %~dp0
exit /b 0