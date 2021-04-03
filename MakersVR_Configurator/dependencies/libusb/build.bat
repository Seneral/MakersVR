@echo off & SETLOCAL ENABLEEXTENSIONS ENABLEDELAYEDEXPANSION

:: Make sure we have the environment variables necessary to build
if "%VSCMD_ARG_TGT_ARCH%" == "" (
	echo Please use the VS Developer Command Prompt! vcvars64
	exit /B
)

:: Make sure libusb source directory is valid
set LIBUSB_SRC_PATH=%1
if "%LIBUSB_SRC_PATH%"=="" (
	echo Please specify the libusb source download folder as an argument!
	exit /B
)
if not exist "%LIBUSB_SRC_PATH%" (
	echo %LIBUSB_SRC_PATH% does not exist!
	exit /B
) else (
	echo libusb Directory is %LIBUSB_SRC_PATH%
)

:: Select release or debug mode
set MODE=%2
if "%MODE%"=="" (
	set MODE=release
) else (
	set MODE
	call :tolower MODE
	if NOT "%MODE%"=="release" (
		if NOT "%MODE%" == "debug" (
			echo Invalid mode %MODE%
			exit /B
		)
	)
)

echo -----------------------------------------
echo Building %MODE%
echo -----------------------------------------

MSBuild.exe %LIBUSB_SRC_PATH%\msvc\libusb_dll_2019.vcxproj -property:Configuration=%MODE%

echo -----------------------------------------
echo Build completed
echo -----------------------------------------

:: Try to verify output
if not exist %LIBUSB_SRC_PATH%\%VSCMD_ARG_TGT_ARCH%\%MODE%\dll (
	echo Build failed - build\dll does not exist!
	exit /B
)

:: Copy resulting files to project directory
if "%CD:~-20%" neq "\dependencies\libusb" (
	echo Not in dependencies\libusb subfolder, can't automatically install files into project folder!
	exit /B
)

echo Copying contents of build\include to project folder include
if not exist ..\..\include\libusb md ..\..\include\libusb
copy %LIBUSB_SRC_PATH%\libusb\libusb.h ..\..\include\libusb\

echo Copying contents of build\lib to project folder lib
if not exist ..\..\lib\win\%MODE%\libusb md ..\..\lib\win\%MODE%\libusb
copy %LIBUSB_SRC_PATH%\%VSCMD_ARG_TGT_ARCH%\%MODE%\dll\* ..\..\lib\win\%MODE%\libusb\

echo Now you can call clean.bat if everything succeeded.

goto :EOF

:tolower
for %%L IN (a b c d e f g h i j k l m n o p q r s t u v w x y z) DO SET %1=!%1:%%L=%%L!
goto :EOF

:getabsolute
set %1=%~f2
goto :eof