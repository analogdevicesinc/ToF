#!/bin/bash

git config --global --add safe.directory /ToF/libaditof
git config --global --add safe.directory /ToF/libaditof/glog
git config --global --add safe.directory /ToF/libaditof/protobuf
git config --global --add safe.directory /ToF/libaditof/libzmq
git config --global --add safe.directory /ToF/libaditof/cppzmq

project_dir=$1
pushd ${project_dir}

GLOG_INSTALL_DIR="/aditof-deps/installed/glog"
PROTOBUF_INSTALL_DIR="/aditof-deps/installed/protobuf"
OPENCV_INSTALL_DIR="/aditof-deps/installed/opencv"
LIBZMQ_INSTALL_DIR="/aditof-deps/installed/libzmq"
NUM_JOBS=4
ARGS="$2"

mkdir -p build
mkdir ../libs

pushd build
cmake .. ${ARGS} -DCMAKE_PREFIX_PATH="${GLOG_INSTALL_DIR};${PROTOBUF_INSTALL_DIR};${LIBZMQ_INSTALL_DIR};${OPENCV_INSTALL_DIR}" -DWITH_OPENCV=0
make -j${NUM_JOBS}
popd #build

popd # ${project_dir}
