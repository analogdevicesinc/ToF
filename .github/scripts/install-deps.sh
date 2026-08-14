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

# Install dependencies
if [[ "$BUILD_TYPE" == "linux" ]]; then
    sudo apt-get update
    sudo apt-get install -y \
        build-essential libgtk2.0-dev pkg-config \
        libavcodec-dev libavformat-dev libswscale-dev \
        libgl1-mesa-dev libglfw3-dev libopencv-dev
        
    sudo sh -c 'echo "${DEPS_DIR}/installed/opencv/lib" > /etc/ld.so.conf.d/opencv.conf'
    sudo ldconfig

    # Setup compiler
    if [[ "${COMPILER_CXX}" != "" ]]; then 
        echo "CXX=${COMPILER_CXX}" >> $GITHUB_ENV
    fi
    if [[ "${COMPILER_CC}" != "" ]]; then 
        echo "CC=${COMPILER_CC}" >> $GITHUB_ENV
    fi
elif [[ "$BUILD_TYPE" == "docker" ]]; then
    sudo apt-get -qq update
    sudo DEBIAN_FRONTEND=noninteractive apt-get install -y \
        qemu binfmt-support qemu-user-static
    docker run --rm --privileged multiarch/qemu-user-static --reset -p yes
    sudo service docker restart
fi