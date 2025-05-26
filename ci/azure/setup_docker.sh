#!/bin/bash

# This script will install glog, libprotobuf, libzmq and cppzmq in an aditof-deps
# folder inside a docker

. lib.sh

GLOG_INSTALL_DIR="$PWD/installed/glog"
PROTOBUF_INSTALL_DIR="$PWD/installed/protobuf"
LIBZMQ_INSTALL_DIR="$PWD/installed/libzmq"
OPENCV_INSTALL_DIR="$PWD/installed/opencv"

if [[ "${OPENCV}" == "" ]]; then
        export OPENCV="3.4.1"
fi

build_and_install_opencv "opencv-${OPENCV}" ${OPENCV_INSTALL_DIR}
build_and_install_glog "glog" ${GLOG_INSTALL_DIR}
build_and_install_protobuf "protobuf" ${PROTOBUF_INSTALL_DIR}
build_and_install_libzmq "libzmq" ${LIBZMQ_INSTALL_DIR}
build_and_install_cppzmq "cppzmq" "libzmq"
