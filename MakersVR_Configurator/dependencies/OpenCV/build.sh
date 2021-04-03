#!/usr/bin/env bash

# Make sure OpenCV source directory is valid
OPENCV_SRC_PATH=$1
if [[ -z $OPENCV_SRC_PATH ]]; then
	echo "Please specify the OpenCV source download folder as an argument!"
	exit 1
fi
if [[ ! -d $OPENCV_SRC_PATH ]]; then
	echo "$OPENCV_SRC_PATH does not exist!"
	exit 1
else
	OPENCV_SRC_PATH=$(cd $OPENCV_SRC_PATH; pwd) # Make absolute
	echo "OpenCV Directory is $OPENCV_SRC_PATH"
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
if [[ ! -d "linux/$MODE/build" ]]; then
	mkdir -p linux/$MODE/build
fi
if [[ ! -d "linux/$MODE/install" ]]; then
	mkdir -p linux/$MODE/install
fi

echo -----------------------------------------
echo Building $MODE
echo -----------------------------------------

pushd linux/$MODE/build
cmake -C ../../../CMakeConfig.txt -DCMAKE_INSTALL_PREFIX="../install" -DCMAKE_BUILD_TYPE=$MODE $OPENCV_SRC_PATH -DWITH_QT=OFF -DWITH_GTK=OFF
make install -j4
popd

echo -----------------------------------------
echo Build completed
echo -----------------------------------------

# Try to verify output
if [[ ! -d "linux/$MODE/install/include" ]]; then
	echo "Build failed - install/include does not exist!"
	exit 3
fi

# Copy resulting files to project directory
if [[ "${PWD:(-20)}" != "/dependencies/OpenCV" ]]; then
	echo "Not in dependencies/OpenCV subfolder, can't automatically install files into project folder!"
	exit 4
fi

echo "Copying contents of install/include to project folder include"
if [[ ! -d "../../include/opencv2" ]]; then
	mkdir -p ../../include/opencv2
fi
cp -r linux/$MODE/install/include/opencv4/opencv2/* ../../include/opencv2

echo "Copying contents of build/lib to project folder lib"
rm -r ../../lib/linux/$MODE/opencv2
mkdir -p ../../lib/linux/$MODE/opencv2
cp -r linux/$MODE/build/lib/* ../../lib/linux/$MODE/opencv2

#echo "Copying contents of build/3rdparty/lib to project folder lib"
#cp -r build/3rdparty/lib ../../lib/opencv2-linux

echo "Now you can call clean.bat if everything succeeded."