#!/usr/bin/env bash

FETCH_URL=https://github.com/Seneral/libusb/archive/master.zip
FETCH_VERSION=custom

if [[ -f "source/srcversion" ]]; then
	if [[ $(cat source/srcversion) == $FETCH_VERSION ]]; then
		echo Already fetched libusb $FETCH_VERSION source!
		exit 0
	fi
fi

echo Downloading libusb $FETCH_VERSION source
wget -O source.zip $FETCH_URL -q --show-progress

if [[ -d "source" ]]; then
	rm -r source
fi

echo Unpacking libusb $FETCH_VERSION source
unzip -q source.zip
mv libusb-* source

rm source.zip
echo $FETCH_VERSION > source/srcversion