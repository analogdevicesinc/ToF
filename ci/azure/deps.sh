#!/bin/bash

set -ex

. ci/azure/lib.sh


deps_default() {
    sudo apt-get update
    mkdir -p ${DEPS_DIR}
   
    pushd ${BUILD_DIR}

    get_deps_source_code "${DEPS_DIR}"
    # build_and_install_glog "${DEPS_DIR}/glog" "${DEPS_DIR}/installed/glog"
    # build_and_install_protobuf "${DEPS_DIR}/protobuf" "${DEPS_DIR}/installed/protobuf"
    # build_and_install_libzmq "${DEPS_DIR}/libzmq" "${DEPS_DIR}/installed/libzmq"
	# build_and_install_cppzmq "${DEPS_DIR}/cppzmq" "${DEPS_DIR}/installed/libzmq"
	# build_and_install_opencv "${DEPS_DIR}/opencv-${OPENCV}" "${DEPS_DIR}/installed/opencv"

    if [[ ${CMAKE_OPTIONS} == *"WITH_OPEN3D=on"* ]]; then
        build_and_install_open3d "${DEPS_DIR}/Open3D" "${DEPS_DIR}/installed/Open3D"
    fi
    
    popd
}

deps_cppcheck() {    
    sudo apt-get install cppcheck
    echo_green "Cppcheck version: " `cppcheck --version`
    cppcheck --version
}

deps_clang_format() {
    sudo apt-get install clang-format
    echo_green "Clang-format version: " `/usr/bin/clang-format --version`
}

deps_deploy_doxygen() {
    install_doxygen
    echo_green "Doxygen version: " `doxygen --version`
}

deps_${BUILD_TYPE:-default}
