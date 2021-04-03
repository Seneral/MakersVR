@echo off & SETLOCAL ENABLEEXTENSIONS ENABLEDELAYEDEXPANSION

:: Make sure we have the environment variables necessary to build
if "%VSCMD_ARG_TGT_ARCH%" == "" (
	echo Please use the VS Developer Command Prompt! vcvars64
	exit /B
)

:: Make sure OpenCV source directory is valid
set OPENCV_SRC_PATH=%1
if "%OPENCV_SRC_PATH%"=="" (
	echo Please specify the OpenCV source download folder as an argument!
	exit /B
)
if not exist "%OPENCV_SRC_PATH%" (
	echo %OPENCV_SRC_PATH% does not exist!
	exit /B
) else (
	echo OpenCV Directory is %OPENCV_SRC_PATH%
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

:: Create temporary directories
if not exist win\%MODE%\build md win\%MODE%\build
if not exist win\%MODE%\install md win\%MODE%\install

echo -----------------------------------------
echo Building %MODE%
echo -----------------------------------------

pushd win\%MODE%\build
cmake -C ..\..\..\CMakeConfig.txt -G "NMake Makefiles" -DCMAKE_INSTALL_PREFIX="..\install" -DCMAKE_BUILD_TYPE=%MODE% %OPENCV_SRC_PATH%
nmake install
popd

echo -----------------------------------------
echo Build completed
echo -----------------------------------------

:: Try to verify output
if not exist win\%MODE%\install\include (
	echo Build failed - install\include does not exist!
	exit /B
)

:: Copy resulting files to project directory
if "%CD:~-20%" neq "\dependencies\OpenCV" (
	echo Not in dependencies\OpenCV subfolder, can't automatically install files into project folder!
	exit /B
)

echo Copying contents of install\include to project folder include
robocopy win\%MODE%\install\include ..\..\include /E /NFL /NDL /NJH /NJS

echo Copying contents of build\lib to project folder lib
if not exist ..\..\lib\win\%MODE%\opencv2 md ..\..\lib\win\%MODE%\opencv2
robocopy win\%MODE%\build\lib ..\..\lib\win\%MODE%\opencv2 /E /NFL /NDL /NJH /NJS

echo Copying contents of build\3rdparty\lib to project folder lib
robocopy win\%MODE%\build\3rdparty\lib ..\..\lib\win\%MODE%\opencv2 /E /NFL /NDL /NJH /NJS

echo Now you can call clean.bat if everything succeeded.

goto :EOF

:tolower
for %%L IN (a b c d e f g h i j k l m n o p q r s t u v w x y z) DO SET %1=!%1:%%L=%%L!
goto :EOF
