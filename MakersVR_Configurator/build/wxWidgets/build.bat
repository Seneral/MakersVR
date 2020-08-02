@echo off

:: Make sure we have the environment variables necessary to build
if "%VSCMD_ARG_TGT_ARCH%" == "" (
	echo Please use the VS Developer Command Prompt! vcvars64
	pause exit
)

:: Make sure wxWidgets source directory is valid
set WX_SRC=%1
if "%WX_SRC%"=="" (
	echo Please specify the wxWidgets source download folder as an argument!
	exit /B
)
if not exist "%WX_SRC%" (
	echo %WX_SRC% does not exist!
	exit /B
) else (
	echo wxWidgets Directory is %WX_SRC%
)

:: Copy build configuration (setup.h) to wxWidgets build directory
if not exist %WX_SRC%\include\wx\msw\ (
	echo %WX_SRC% does not have \include\wx\msw\
	exit /B
) else (
	echo Copying setup.h to %WX_SRC%\include\wx\msw\
	copy setup.h %WX_SRC%\include\wx\msw\setup.h >NUL
)

:: Build wxWidgets in both debug and release configurations
:: Builds won't interfer with each other so install can be done afterwards

pushd %WX_SRC%\build\msw

echo -----------------------------------------
echo Building Debug
echo -----------------------------------------

nmake /f makefile.vc ^
USE_AUI=0 USE_HTML=0 USE_MEDIA=0 USE_PROPGRID=0 USE_QA=0 ^
USE_RIBBON=0 USE_RICHTEXT=0 USE_STC=0 USE_WEBVIEW=0 USE_XRC=0 ^
TARGET_CPU=%VSCMD_ARG_TGT_ARCH% ^
RUNTIME_LIBS=static BUILD=debug

echo -----------------------------------------
echo Building Release
echo -----------------------------------------

nmake /f makefile.vc ^
USE_AUI=0 USE_HTML=0 USE_MEDIA=0 USE_PROPGRID=0 USE_QA=0 ^
USE_RIBBON=0 USE_RICHTEXT=0 USE_STC=0 USE_WEBVIEW=0 USE_XRC=0 ^
TARGET_CPU=%VSCMD_ARG_TGT_ARCH% ^
RUNTIME_LIBS=static BUILD=release

echo -----------------------------------------
echo Build completed
echo -----------------------------------------

popd

:: Copy resulting files to project directory

if "%CD:~-16%" neq "\build\wxWidgets" (
	echo Not in build\wxWidgets subfolder, can't automatically install files into project folder!
	exit /B
)

set LIBNAME=vc_lib
if "%VSCMD_ARG_TGT_ARCH%"=="x64" set LIBNAME=vc_x64_lib

if not exist %WX_SRC%\lib\%LIBNAME% (
	echo Build failed - lib\%LIBNAME% does not exist!
	exit /B
)

echo Copying contents of wxWidgets\include to project folder include
robocopy %WX_SRC%\include ..\..\include /E /NFL /NDL /NJH /NJS
echo Copying contents of wxWidgets\lib\%LIBNAME% to project folder lib
robocopy %WX_SRC%\lib\%LIBNAME% ..\..\lib\%LIBNAME% /E /NFL /NDL /NJH /NJS
echo Now you can call clean.bat if everything succeeded.