#!/usr/bin/env bash

# Not standard packaged source, because of submodules
FETCH_URL=https://github.com/wxWidgets/wxWidgets/releases/download/v3.1.4/wxWidgets-3.1.4.zip
FETCH_VERSION=3.1.4

if [[ -f "source/srcversion" ]]; then
	if [[ $(cat source/srcversion) == $FETCH_VERSION ]]; then
		echo Already fetched wxWidgets $FETCH_VERSION source!
		exit 0
	fi
fi

echo Downloading wxWidgets $FETCH_VERSION source
wget -O source.zip $FETCH_URL -q --show-progress

if [[ -d "source" ]]; then
	rm -r source
fi

echo Unpacking wxWidgets $FETCH_VERSION source
unzip -q source.zip -d wxWidgets
mv wxWidgets source

rm source.zip
echo $FETCH_VERSION > source/srcversion