#!/bin/bash

NUM_JOBS=${nproc}

echo_red() { printf "\033[1;31m$*\033[m\n"; }
echo_green() { printf "\033[1;32m$*\033[m\n"; }

############################################################################
# Get source code for dependencies: glog, protobuf, libwebsockets
############################################################################
get_deps_source_code() {
    CLONE_DIRECTORY=$1
    pushd "${CLONE_DIRECTORY}"

    [ -d "glog" ] || {
       git clone --branch v0.3.5 --depth 1 https://github.com/google/glog
    }
    [ -d "protobuf" ] || {
       git clone --branch v3.9.0 --depth 1 https://github.com/protocolbuffers/protobuf
    }
    [ -d "libwebsockets" ] || {
       git clone --branch v3.1-stable --depth 1 https://github.com/warmcat/libwebsockets
    }

    popd
}

############################################################################
# Build and install v0.3.5 of glog from the specified repository
############################################################################
build_and_install_glog() {
    REPO_DIR=$1
    INSTALL_DIR=$2
    EXTRA_CMAKE_OPTIONS=$3
    BUILD_DIR=${REPO_DIR}/build_0_3_5

    mkdir -p ${BUILD_DIR}
    pushd ${BUILD_DIR}
    cmake .. -DWITH_GFLAGS=off -DCMAKE_INSTALL_PREFIX=${INSTALL_DIR} ${EXTRA_CMAKE_OPTIONS}
    make -j${NUM_JOBS}
    sudo make install
    popd
}

############################################################################
# Build and install v3.9.0 of protobuf from the specified repository
############################################################################
build_and_install_protobuf() {
    REPO_DIR=$1
    INSTALL_DIR=$2
    EXTRA_CMAKE_OPTIONS=$3
    BUILD_DIR=${REPO_DIR}/build_3_9_0

    mkdir -p ${BUILD_DIR}
    pushd ${BUILD_DIR}
    cmake ../cmake/ -Dprotobuf_BUILD_TESTS=OFF -DCMAKE_POSITION_INDEPENDENT_CODE=ON -DCMAKE_INSTALL_PREFIX=${INSTALL_DIR} ${EXTRA_CMAKE_OPTIONS}
    make -j${NUM_JOBS}
    sudo make install
    popd
}

############################################################################
# Build and install v3.1.0 of libwebsockets from the specified repository
############################################################################
build_and_install_websockets() {
    REPO_DIR=$1
    INSTALL_DIR=$2
    EXTRA_CMAKE_OPTIONS=$3
    BUILD_DIR=${REPO_DIR}/build_3_1_0

    mkdir -p ${BUILD_DIR}
    pushd ${BUILD_DIR}
    cmake .. -DLWS_WITH_SSL=OFF -DLWS_STATIC_PIC=ON -DCMAKE_INSTALL_PREFIX=${INSTALL_DIR} ${EXTRA_CMAKE_OPTIONS}
    make -j${NUM_JOBS}
    sudo make install
    popd
}
