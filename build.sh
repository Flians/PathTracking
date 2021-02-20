#!/bin/bash

build_dir="./build"

if [ ! -d "$build_dir" ]; then
    mkdir $build_dir
fi

cd build

if [ $# -ge 1 ]
then
    cmake -DCMAKE_BUILD_TYPE=$1 ..
else
    cmake -DCMAKE_BUILD_TYPE=Release ..
fi

make -j8