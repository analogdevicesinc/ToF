#!/usr/bin/env pwsh

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

#install OpenCV
choco install opencv --version 3.4.1

#build sdk
mkdir build_Release
mkdir build_Debug
mkdir ../libs

cd build_Release
cmake -DWITH_OPENCV=on -DWITH_PYTHON=on -DCMAKE_PREFIX_PATH="../deps_installed/Release/glog;../deps_installed/Release/protobuf;../deps_installed/Release/libzmq;../deps_installed/Release/cppzmq" -DOpenCV_DIR="C:/tools/opencv/build/x64/vc15/lib" ..
cmake --build . --target install --config Release -j 4

cd ../build_Debug
cmake -DWITH_OPENCV=on -DWITH_PYTHON=on -DCMAKE_PREFIX_PATH="../deps_installed/Debug/glog;../deps_installed/Debug/protobuf;../deps_installed/Debug/libzmq;../deps_installed/Debug/cppzmq" -DOpenCV_DIR="C:/tools/opencv/build/x64/vc15/lib" ..
cmake --build . --target install --config Debug -j 4
