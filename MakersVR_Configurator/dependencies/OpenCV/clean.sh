#!/usr/bin/env bash

# Select release or debug mode
MODE="$(echo $2 | tr '[:upper:]' '[:lower:]')"
if [[ $MODE == build ]]; then
	echo Cleaning linux debug and release builds
	rm -r linux
elif [[ $MODE == release ]]; then
	echo Cleaning linux release build
	rm -r linux/release
elif [[ $MODE == debug ]]; then
	echo Cleaning linux debug build
	rm -r linux/debug
else
	echo Cleaning all builds and sources
	rm -r linux
	rm -r win
	rm -r source
fi

echo Cleanup done.