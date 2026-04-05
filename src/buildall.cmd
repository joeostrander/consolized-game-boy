@ECHO OFF
SETLOCAL

ECHO Building all DVI variants...
call :run_build "%~dp0gb_dvi" build_clean.cmd rp2040 0 nes || goto :fail
call :run_build "%~dp0gb_dvi" build_clean.cmd rp2040 0 wii || goto :fail
call :run_build "%~dp0gb_dvi" build_clean.cmd rp2040 1 nes || goto :fail
call :run_build "%~dp0gb_dvi" build_clean.cmd rp2040 1 wii || goto :fail
call :run_build "%~dp0gb_dvi" build_clean.cmd rp2040 2 nes || goto :fail
call :run_build "%~dp0gb_dvi" build_clean.cmd rp2040 2 wii || goto :fail
call :run_build "%~dp0gb_dvi" build_clean.cmd rp2350 0 nes || goto :fail
call :run_build "%~dp0gb_dvi" build_clean.cmd rp2350 0 wii || goto :fail
call :run_build "%~dp0gb_dvi" build_clean.cmd rp2350 1 nes || goto :fail
call :run_build "%~dp0gb_dvi" build_clean.cmd rp2350 1 wii || goto :fail
call :run_build "%~dp0gb_dvi" build_clean.cmd rp2350 2 nes || goto :fail
call :run_build "%~dp0gb_dvi" build_clean.cmd rp2350 2 wii || goto :fail

ECHO.
ECHO Building all VGA variants...
call :run_build "%~dp0gb_vga" build_clean.cmd rp2040 rgb222 nes || goto :fail
call :run_build "%~dp0gb_vga" build_clean.cmd rp2040 rgb222 wii || goto :fail
call :run_build "%~dp0gb_vga" build_clean.cmd rp2040 rgb332 nes || goto :fail
call :run_build "%~dp0gb_vga" build_clean.cmd rp2040 rgb332 wii || goto :fail
call :run_build "%~dp0gb_vga" build_clean.cmd rp2350 rgb222 nes || goto :fail
call :run_build "%~dp0gb_vga" build_clean.cmd rp2350 rgb222 wii || goto :fail
call :run_build "%~dp0gb_vga" build_clean.cmd rp2350 rgb332 nes || goto :fail
call :run_build "%~dp0gb_vga" build_clean.cmd rp2350 rgb332 wii || goto :fail

ECHO.
ECHO All builds completed successfully.
exit /b 0

:run_build
set "BUILD_DIR=%~1"
set "BUILD_SCRIPT=%~2"
shift
shift
set "BUILD_ARGS=%1 %2 %3 %4 %5 %6 %7"

pushd "%BUILD_DIR%" || exit /b 1
echo.
echo ===== Running %BUILD_DIR%\%BUILD_SCRIPT% %BUILD_ARGS% =====
call "%BUILD_SCRIPT%" %BUILD_ARGS%
set "BUILD_RESULT=%ERRORLEVEL%"
popd

exit /b %BUILD_RESULT%

:fail
ECHO.
ECHO Build matrix stopped because a variant failed.
exit /b 1