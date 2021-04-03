@echo off & SETLOCAL ENABLEEXTENSIONS ENABLEDELAYEDEXPANSION

set FETCH_URL=https://github.com/Seneral/libusb/archive/master.zip
set FETCH_VERSION=custom

if exist "source\srcversion" (
	set /p SRC_VERSION=<"source\srcversion"
	if "!SRC_VERSION!"=="%FETCH_VERSION%" (
		echo Already fetched libusb %FETCH_VERSION% source!
		exit /B
	)
)

echo Downloading libusb %FETCH_VERSION% source
curl.exe -L -o source.zip %FETCH_URL%

if exist "source" rd /s /q "source"

echo Unpacking libusb %FETCH_VERSION% source
tar -xf source.zip
move "libusb-*" "source"

del source.zip
(echo %FETCH_VERSION%) > source/srcversion