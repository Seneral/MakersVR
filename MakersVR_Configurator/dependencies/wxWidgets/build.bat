@echo off & SETLOCAL ENABLEEXTENSIONS ENABLEDELAYEDEXPANSION

:: Make sure we have the environment variables necessary to build
if "%VSCMD_ARG_TGT_ARCH%" == "" (
	echo Please use the VS Developer Command Prompt! vcvars64
	exit /B
)

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
set D=
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
if "%MODE%" == "debug" (
	set D=d
)

:: Copy build configuration (setup.h) to wxWidgets build directory
if not exist %WX_SRC_PATH%\include\wx\msw\ (
	echo %WX_SRC_PATH% does not have \include\wx\msw\
	exit /B
) else (
	echo Copying setup.h to %WX_SRC_PATH%\include\wx\msw\
	copy setup.h %WX_SRC_PATH%\include\wx\msw\setup.h >NUL
)

:: Create temporary directories
set LIBNAME=vc_lib
if "%VSCMD_ARG_TGT_ARCH%"=="x64" set LIBNAME=vc_x64_lib

echo -----------------------------------------
echo Building %MODE%
echo -----------------------------------------

pushd %WX_SRC_PATH%\build\msw
nmake /f makefile.vc ^
USE_AUI=0 USE_HTML=0 USE_MEDIA=0 USE_PROPGRID=0 USE_QA=0 USE_RTTI=0 ^
USE_RIBBON=0 USE_RICHTEXT=0 USE_STC=0 USE_WEBVIEW=0 USE_XRC=0 ^
USE_OPENGL=1 ^
MONOLITHIC=1 ^
TARGET_CPU=%VSCMD_ARG_TGT_ARCH% ^
RUNTIME_LIBS=static BUILD=%MODE%
popd

echo -----------------------------------------
echo Build completed
echo -----------------------------------------

:: Try to verify output
if not exist %WX_SRC_PATH%\lib\%LIBNAME% (
	echo Build failed - lib\%LIBNAME% does not exist!
	exit /B
)

:: Copy resulting files to project directory
if "%CD:~-23%" neq "\dependencies\wxWidgets" (
	echo Not in build\wxWidgets subfolder, can't automatically install files into project folder!
	exit /B
)

echo Copying contents of wxWidgets\include\wx to project folder include
robocopy %WX_SRC_PATH%\include\wx ..\..\include\wx /E /NFL /NDL /NJH /NJS

echo Copying contents of wxWidgets\lib\%LIBNAME% to project folder lib
if not exist ..\..\lib\win\%MODE%\wx\wx\ mkdir ..\..\lib\win\%MODE%\wx\wx\
copy %WX_SRC_PATH%\lib\%LIBNAME%\mswu%D%\wx\setup.h ..\..\lib\win\%MODE%\wx\wx\
copy %WX_SRC_PATH%\lib\%LIBNAME%\wxmsw31u%D%.lib ..\..\lib\win\%MODE%\wx\
copy %WX_SRC_PATH%\lib\%LIBNAME%\wxmsw31u%D%_gl.lib ..\..\lib\win\%MODE%\wx\

echo Now you can call clean.bat if everything succeeded.

goto :EOF

:tolower
for %%L IN (a b c d e f g h i j k l m n o p q r s t u v w x y z) DO SET %1=!%1:%%L=%%L!
goto :EOF