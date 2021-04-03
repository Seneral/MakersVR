@echo off

:: Make sure wxWidgets source directory is valid
set WX_SRC_PATH=%1
if "%WX_SRC_PATH%"=="" (
	echo Please specify the wxWidgets source download folder as an argument!
	exit /B
)
if not exist "%WX_SRC_PATH%" (
	echo %WX_SRC_PATH% does not exist!
	exit /B
) else (
	echo wxWidgets Directory is %WX_SRC_PATH%
)

:: Select release or debug mode
set MODE=%2
call :tolower MODE

:: Clean build files

echo Cleaning build files
if exist %WX_SRC_PATH%\build\msw (
	pushd %WX_SRC_PATH%\build\msw
	for /f %%i in ('dir /a:d /b vc_*') do (
		echo - build\msw\%%i
		rd /s /q %%i
	)
	popd
) else (
	echo Build directory %WX_SRC_PATH%\build\msw doesn't exist!
)

:: Clean built libs

echo Cleaning built libs
if exist %WX_SRC_PATH%\lib (
	pushd %WX_SRC_PATH%\lib
	for /f %%i in ('dir /a:d /b vc_*') do (
		echo - lib\%%i
		rd /s /q %%i
	)
	popd
) else (
	echo Lib directory %WX_SRC_PATH%\lib doesn't exist!
)

if "%MODE%"=="" (
	echo Cleaning sources
	rd /s /q source
)

echo Cleanup done.

goto :EOF

:tolower
for %%L IN (a b c d e f g h i j k l m n o p q r s t u v w x y z) DO SET %1=!%1:%%L=%%L!
goto :EOF