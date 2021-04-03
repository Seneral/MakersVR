#!/usr/bin/env bash

# Make sure libusb source directory is valid
LIBUSB_SRC_PATH=$1
if [[ -z "$LIBUSB_SRC_PATH" ]]; then
	echo "Please specify the libusb source download folder as an argument!"
	exit 1
fi
if [[ ! -d "$LIBUSB_SRC_PATH" ]]; then
	echo "$LIBUSB_SRC_PATH does not exist!"
	exit 1
else
	LIBUSB_SRC_PATH=$(cd $LIBUSB_SRC_PATH; pwd) # Make absolute
	echo "libusb Directory is $LIBUSB_SRC_PATH"
fi

# Select release or debug mode
MODE="$(echo $2 | tr '[:upper:]' '[:lower:]')"
if [[ -z "$MODE" ]]; then
	MODE="release"
elif [[ $MODE != "debug" && $MODE != "release" ]]; then
	echo "Invalid mode $MODE!"
	exit 2
fi

# Create temporary directories
if [[ ! -d "linux/$MODE/install" ]]; then
	mkdir -p linux/$MODE/install
fi
if [[ ! -d "linux/$MODE/build" ]]; then
	mkdir -p linux/$MODE/build
fi
ABS_INSTALL_PATH=$(pwd)/linux/$MODE/install
MODE_PARAM=
if [[ $MODE == "debug" ]]; then
	MODE_PARAM=--enable-debug-log
fi

echo -----------------------------------------
echo Building $MODE
echo -----------------------------------------

pushd $LIBUSB_SRC_PATH
./bootstrap.sh
popd
pushd linux/$MODE/build
$LIBUSB_SRC_PATH/configure --prefix=$ABS_INSTALL_PATH --enable-shared=yes --enable-static=no $MODE_PARAM
make install -j4
popd

echo -----------------------------------------
echo Build completed
echo -----------------------------------------

# Try to verify output
if [[ ! -d "linux/$MODE/install/lib" ]]; then
	echo "Build failed - install/libs does not exist!"
	exit 3
fi

# Copy resulting files to project directory
if [[ "${PWD:(-20)}" != "/dependencies/libusb" ]]; then
	echo "Not in dependencies/libusb subfolder, can't automatically install files into project folder!"
	exit 4
fi

echo "Copying contents of install/include to project folder include"
if [[ ! -d "../../include/libusb" ]]; then
	mkdir -p ../../include/libusb
fi
cp -r linux/$MODE/install/include/libusb-*/* ../../include/libusb

echo "Copying contents of install/lib to project folder lib"
rm -r ../../lib/linux/$MODE/libusb
mkdir -p ../../lib/linux/$MODE/libusb
cp -r linux/$MODE/install/lib/* ../../lib/linux/$MODE/libusb

echo "Now you can call clean.bat if everything succeeded."