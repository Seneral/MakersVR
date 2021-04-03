@echo off

:: Select release or debug mode
set MODE=%2
call :tolower MODE

:: Cleanup
if "%MODE%"=="build" (
	echo Cleaning windows debug and release builds
	if exist win rd /s /q win
) else if "%MODE%"=="release" (
	echo Cleaning windows release build
	if exist win\release rd /s /q win\release
) else if "%MODE%"=="debug" (
	echo Cleaning windows debug build
	if exist win\debug rd /s /q win\debug
) else (
	echo Cleaning all builds and sources
	if exist win rd /s /q win
	if exist linux rd /s /q linux
	if exist source rd /s /q source
)

echo Cleanup done.

goto :EOF

:tolower
for %%L IN (a b c d e f g h i j k l m n o p q r s t u v w x y z) DO SET %1=!%1:%%L=%%L!
goto :EOF