@echo off

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

:: Build both debug and release configurations and install
:: Builds will overwrite each other so install is necessary immediately

if not exist build md build
pushd build

echo -----------------------------------------
echo Building Debug
echo -----------------------------------------

::cmake -C ../CMakeConfig.txt -G "NMake Makefiles" -DCMAKE_INSTALL_PREFIX="..\install" -DCMAKE_BUILD_TYPE=Debug %OPENCV_SRC_PATH%
::nmake install

echo -----------------------------------------
echo Building Release
echo -----------------------------------------

::cmake -C ../CMakeConfig.txt -G "NMake Makefiles" -DCMAKE_INSTALL_PREFIX="..\install" -DCMAKE_BUILD_TYPE=Release %OPENCV_SRC_PATH%
::nmake install

echo -----------------------------------------
echo Build completed
echo -----------------------------------------

popd

:: Copy resulting files to project directory

if "%CD:~-13%" neq "\build\opencv" (
	echo Not in build\opencv subfolder, can't automatically install files into project folder!
	exit /B
)
if not exist install\include (
	echo Build failed - install\include does not exist!
	exit /B
)

echo Copying contents of install\include to project folder include
robocopy install\include ..\..\include /E /NFL /NDL /NJH /NJS
echo Copying contents of build\lib to project folder lib
if not exist ..\..\lib\opencv2 md ..\..\lib\opencv2
robocopy build\lib ..\..\lib\opencv2 /E /NFL /NDL /NJH /NJS
echo Copying contents of build\3rdparty\lib to project folder lib
robocopy build\3rdparty\lib ..\..\lib\opencv2 /E /NFL /NDL /NJH /NJS
echo Now you can call clean.bat if everything succeeded.