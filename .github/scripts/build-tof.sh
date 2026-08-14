#!/bin/bash

# Copyright 2026 Analog Devices, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

set -e

DEFAULT_CMAKE_FLAGS='-DWITH_NETWORK=1 -DWITH_PYTHON=1 -DWITH_OPENCV=1 -DWITH_EXAMPLES=1 -DWITH_DOC=0 -DWITH_OPEN3D=0 -DCI_BUILD=1'

if [[ "${BUILD_TYPE}" == "linux" ]]; then
    # Default ubuntu build
    mkdir -p ${BUILD_DIR}
    mkdir ../libs

    pushd ${BUILD_DIR}
    pwd
    cmake ${DEFAULT_CMAKE_FLAGS} ${EXTRA_CMAKE_FLAGS} \
        -DCMAKE_PREFIX_PATH="${DEPS_DIR}/installed/glog;${DEPS_DIR}/installed/protobuf;${DEPS_DIR}/installed/libzmq;${DEPS_DIR}/installed/Open3D;${DEPS_DIR}/installed/opencv" \
        .. 
    make -j${NUM_JOBS}
    popd
elif [[ "${BUILD_TYPE}" == "docker" ]]; then
    # Docker build
    sudo docker run --rm=true -v ${WORK_DIR}:/libaditof:rw \
        -e DEFAULT_CMAKE_FLAGS="${DEFAULT_CMAKE_FLAGS}" -e EXTRA_CMAKE_FLAGS="${EXTRA_CMAKE_FLAGS}" \
        -e NUM_JOBS=${NUM_JOBS} ${DOCKER_IMAGE} /bin/bash -xe /libaditof/.github/scripts/inside-docker.sh
fi
