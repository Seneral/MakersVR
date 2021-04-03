@echo off & SETLOCAL ENABLEEXTENSIONS ENABLEDELAYEDEXPANSION

:: Not standard packaged source, because of submodules
set FETCH_URL=https://github.com/wxWidgets/wxWidgets/releases/download/v3.1.4/wxWidgets-3.1.4.zip
set FETCH_VERSION=3.1.4

if exist "source\srcversion" (
	set /p SRC_VERSION=<"source\srcversion"
	if "!SRC_VERSION!"=="%FETCH_VERSION%" (
		echo Already fetched wxWidgets %FETCH_VERSION% source!
		exit /B
	)
)

echo Downloading wxWidgets %FETCH_VERSION% source
curl.exe -L -o source.zip %FETCH_URL%

if exist "source" rd /s /q "source"

echo Unpacking wxWidgets %FETCH_VERSION% source
md source
pushd source
tar -xf ../source.zip
popd

del source.zip
(echo %FETCH_VERSION%) > source/srcversion