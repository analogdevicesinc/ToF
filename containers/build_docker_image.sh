#!/bin/bash

# Build the given dockerfile with the given name

get_deps_source_code() {
    CLONE_DIRECTORY=$1
    pushd "${CLONE_DIRECTORY}"

    # use opencv 3.4.1 by default if no other version is specified
    if [[ "${OPENCV}" == "" ]]; then
        export OPENCV="3.4.1"
    fi

    # [ -d "glog" ] || {
    #     git clone --branch v0.6.0 --depth 1 https://github.com/google/glog
    # }

    # [ -d "protobuf" ] || {
    #     git clone --branch v3.9.0 --depth 1 https://github.com/protocolbuffers/protobuf
    # }

    # [ -d "libwebsockets" ] || {
    #     git clone --branch v3.1-stable --depth 1 https://github.com/warmcat/libwebsockets
    # }

    [ -d "opencv-${OPENCV}" ] || {
        curl -sL https://github.com/Itseez/opencv/archive/${OPENCV}.zip > opencv.zip
        unzip -q opencv.zip
    }

    if [[ ${CMAKE_OPTIONS} == *"WITH_OPEN3D=on"* ]]; then
        [ -d "Open3D" ] || {
            git clone --recursive --branch v0.9.0 --depth 1 https://github.com/intel-isl/Open3D.git
        }
    fi

    popd
}

mkdir -p temp_deps

get_deps_source_code temp_deps

dockername=$1
dockerfilepath=$2

docker build -t ${dockername} -f ${dockerfilepath} .

rm -rf temp_deps
