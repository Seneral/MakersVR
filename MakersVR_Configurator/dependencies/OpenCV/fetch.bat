@echo off & SETLOCAL ENABLEDELAYEDEXPANSION

set FETCH_URL=https://github.com/opencv/opencv/archive/4.5.1.zip
set FETCH_VERSION=4.5.1

if exist "source\srcversion" (
	set /p SRC_VERSION=<"source\srcversion"
	if "!SRC_VERSION!"=="%FETCH_VERSION%" (
		echo Already fetched OpenCV %FETCH_VERSION% source!
		exit /B
	)
)

echo Downloading OpenCV %FETCH_VERSION% source
curl.exe -L -o source.zip %FETCH_URL%

if exist "source" rd /s /q "source"

echo Unpacking OpenCV %FETCH_VERSION% source
tar -xf source.zip
move "opencv-*" "source"

del source.zip
(echo %FETCH_VERSION%) > source/srcversion