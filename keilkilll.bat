@echo off
setlocal
cd /d "%~dp0"

rem Remove only generated Keil output directories.
if exist "OBJ" rmdir /s /q "OBJ"
if exist "USER\Listings" rmdir /s /q "USER\Listings"

echo Keil build output cleaned.
endlocal
