@echo off
setlocal

set "DRIVE_LETTER_RESULT="
for /f %%D in ('powershell -NoProfile -Command ^
  "$v = Get-Volume | Where-Object { $_.FileSystemLabel -eq 'RPI-RP2' } | Select-Object -First 1 -ExpandProperty DriveLetter; if ($v) { $v + ':' }"') do (
    set "DRIVE_LETTER_RESULT=%%D"
)

endlocal & set "DRIVE_LETTER=%DRIVE_LETTER_RESULT%"
exit /b 0