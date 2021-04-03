#!/usr/bin/env bash

FETCH_URL=https://github.com/opencv/opencv/archive/4.5.1.zip
FETCH_VERSION=4.5.1

if [[ -f "source/srcversion" ]]; then
	if [[ $(cat source/srcversion) == $FETCH_VERSION ]]; then
		echo Already fetched OpenCV $FETCH_VERSION source!
		exit 0
	fi
fi

echo Downloading OpenCV $FETCH_VERSION source
wget -O source.zip $FETCH_URL -q --show-progress

if [[ -d "source" ]]; then
	rm -r source
fi

echo Unpacking OpenCV $FETCH_VERSION source
unzip -q source.zip
mv opencv-* source

rm source.zip
echo $FETCH_VERSION > source/srcversion