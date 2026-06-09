# Build script for SBUS Monitor using PowerShell
# Requires Visual Studio or MinGW to be installed

Write-Host "Compiling SBUS Monitor..." -ForegroundColor Cyan

# Try with cl.exe first (Visual Studio)
if (Get-Command cl.exe -ErrorAction SilentlyContinue) {
    Write-Host "Found Visual Studio cl.exe" -ForegroundColor Green
    cl.exe sbus_monitor.c
    
    if ($LASTEXITCODE -eq 0) {
        Write-Host "`nBuild successful!`n" -ForegroundColor Green
        Write-Host "Run with: .\sbus_monitor.exe [COM_PORT]" -ForegroundColor Yellow
        Write-Host "Example: .\sbus_monitor.exe COM6`n" -ForegroundColor Yellow
    } else {
        Write-Host "Build failed with cl.exe" -ForegroundColor Red
    }
}
# Try with gcc (MinGW)
elseif (Get-Command gcc.exe -ErrorAction SilentlyContinue) {
    Write-Host "Found MinGW gcc" -ForegroundColor Green
    gcc.exe -o sbus_monitor.exe sbus_monitor.c
    
    if ($LASTEXITCODE -eq 0) {
        Write-Host "`nBuild successful!`n" -ForegroundColor Green
        Write-Host "Run with: .\sbus_monitor.exe [COM_PORT]" -ForegroundColor Yellow
        Write-Host "Example: .\sbus_monitor.exe COM6`n" -ForegroundColor Yellow
    } else {
        Write-Host "Build failed with gcc" -ForegroundColor Red
    }
}
else {
    Write-Host "Error: Neither Visual Studio (cl.exe) nor MinGW (gcc.exe) found!" -ForegroundColor Red
    Write-Host "`nPlease install one of:" -ForegroundColor Yellow
    Write-Host "  - Visual Studio Community Edition" -ForegroundColor Yellow
    Write-Host "  - MinGW (https://www.mingw-w64.org/)" -ForegroundColor Yellow
}
