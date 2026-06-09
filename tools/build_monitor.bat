@echo off
REM Build script for SBUS Monitor
REM Requires MinGW (gcc) to be installed and in PATH

echo Compiling SBUS Monitor...
gcc -o sbus_monitor.exe sbus_monitor.c -lwinmm

if %ERRORLEVEL% EQU 0 (
    echo.
    echo Build successful! Run with:
    echo   sbus_monitor.exe
    echo or
    echo   sbus_monitor.exe COM6
    echo.
) else (
    echo Build failed!
    echo Make sure MinGW gcc is installed and in PATH
)
