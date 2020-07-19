@echo off

:: Make sure wxWidgets source directory is valid
set WX_SRC=%1
if "%WX_SRC%"=="" (
	echo Please specify the wxWidgets source download folder as an argument!
	pause exit
)
if not exist "%WX_SRC%" (
	echo %WX_SRC% does not exist!
	pause exit
) else (
	echo wxWidgets Directory is %WX_SRC%
)

:: Clean build files

echo Cleaning build files
if exist %WX_SRC%\build\msw (
	pushd %WX_SRC%\build\msw
	for /f %%i in ('dir /a:d /b vc_*') do (
		echo - build\msw\%%i
		rd /s /q %%i
	)
	popd
) else (
	echo Build directory %WX_SRC%\build\msw doesn't exist!
)

:: Clean built libs

echo Cleaning built libs
if exist %WX_SRC%\lib (
	pushd %WX_SRC%\lib
	for /f %%i in ('dir /a:d /b vc_*') do (
		echo - lib\%%i
		rd /s /q %%i
	)
	popd
) else (
	echo Lib directory %WX_SRC%\lib doesn't exist!
)

echo Cleanup done.
pause