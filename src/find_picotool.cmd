@echo off
setlocal EnableDelayedExpansion

set "PROJECT_HINT=%~1"
set "PICOTOOL_DIR_RESULT=%PICOTOOL_DIR%"
set "PICOTOOL_CMAKE_ARG_RESULT="

if defined PICOTOOL_DIR_RESULT (
    if not exist "!PICOTOOL_DIR_RESULT!\picotoolConfig.cmake" (
        echo PICOTOOL_DIR is set but picotoolConfig.cmake was not found there.
        set "PICOTOOL_DIR_RESULT="
    )
)

if not defined PICOTOOL_DIR_RESULT if defined PICO_SDK_PATH (
    for %%I in ("%PICO_SDK_PATH%\..") do set "PICO_SDK_ROOT=%%~fI"
    if /I "!PROJECT_HINT!"=="gb_dvi" (
        call :try_candidate "!PICO_SDK_ROOT!\gb_dvi\build\_deps\picotool"
        call :try_candidate "!PICO_SDK_ROOT!\gb_dvi\software\build\_deps\picotool"
        call :try_candidate "!PICO_SDK_ROOT!\gb_vga\build\_deps\picotool"
    ) else (
        call :try_candidate "!PICO_SDK_ROOT!\gb_vga\build\_deps\picotool"
        call :try_candidate "!PICO_SDK_ROOT!\gb_dvi\build\_deps\picotool"
        call :try_candidate "!PICO_SDK_ROOT!\gb_dvi\software\build\_deps\picotool"
    )
)

if defined PICOTOOL_DIR_RESULT (
    echo Reusing picotool from !PICOTOOL_DIR_RESULT!
    set "PICOTOOL_CMAKE_ARG_RESULT=-Dpicotool_DIR=!PICOTOOL_DIR_RESULT!"
)

endlocal & set "PICOTOOL_DIR=%PICOTOOL_DIR_RESULT%" & set "PICOTOOL_CMAKE_ARG=%PICOTOOL_CMAKE_ARG_RESULT%"
exit /b 0

:try_candidate
if defined PICOTOOL_DIR_RESULT exit /b 0
if exist "%~1\picotoolConfig.cmake" set "PICOTOOL_DIR_RESULT=%~1"
exit /b 0