#!/bin/bash

if [[ ! -d build ]]; then
	mkdir build
fi

pushd build
cmake ..
make
popd
