#!/usr/bin/env bash

# Make sure wxWidgets source directory is valid
WX_SRC_PATH=$1
if [[ -z "$WX_SRC_PATH" ]]; then
	echo "Please specify the wxWidgets source download folder as an argument!"
	exit 1
fi
if [[ ! -d "$WX_SRC_PATH" ]]; then
	echo "$WX_SRC_PATH does not exist!"
	exit 1
else
	WX_SRC_PATH=$(cd $WX_SRC_PATH; pwd) # Make absolute
	echo "wxWidgets Directory is $WX_SRC_PATH"
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
ABS_INSTALL_PATH=$(pwd)/linux/$MODE/install
MODE_PARAM=
if [[ $MODE == "debug" ]]; then
	MODE_PARAM=--enable-debug
fi

echo -----------------------------------------
echo Building $MODE
echo -----------------------------------------

pushd linux/$MODE/build
$WX_SRC_PATH/configure $MODE_PARAM \
	--with-gtk=2 --with-opengl --disable-shared --prefix=$ABS_INSTALL_PATH --enable-unicode \
	--without-libpng --without-libjpeg --without-libtiff --without-zlib --without-expat --without-liblzma \
	--disable-mediactrl --disable-webview \
	--disable-pnm --disable-gif --disable-pcx --disable-iff --disable-svg \
	--disable-sockets \
	--disable-animatectrl --disable-snglinst \
	--disable-stdpaths --disable-mimetype --disable-intl --disable-validators \
	--disable-aui --disable-html --disable-propgrid --disable-printarch \
	--disable-ribbon --disable-richtext --disable-stc --disable-webview --disable-xrc \
	--disable-joystick --disable-graphics_ctx
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
if [[ "${PWD:(-23)}" != "/dependencies/wxWidgets" ]]; then
	echo "Not in dependencies/wxWidgets subfolder, can't automatically install files into project folder!"
	exit 4
fi

echo "Copying contents of install/include to project folder include"
if [[ ! -d "../../include/wx" ]]; then
	mkdir -p ../../include/wx
fi
cp -r linux/$MODE/install/include/wx-*/wx/* ../../include/wx

echo "Copying contents of build/lib to project folder lib"
rm -r ../../lib/linux/$MODE/wx
mkdir -p ../../lib/linux/$MODE/wx
cp -r linux/$MODE/build/lib/* ../../lib/linux/$MODE/wx

echo "Now you can call clean.bat if everything succeeded."