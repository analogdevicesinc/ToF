$ARCH=$Env:ARCH
$GENERATOR=$Env:COMPILER

$local_path=$pwd
mkdir deps_installed

mkdir -p deps_installed/Release/glog
mkdir -p deps_installed/Debug/glog

mkdir -p deps_installed/Release/protobuf
mkdir -p deps_installed/Debug/protobuf 

mkdir -p deps_installed/Release/libzmq
mkdir -p deps_installed/Debug/libzmq

mkdir -p deps_installed/Release/cppzmq
mkdir -p deps_installed/Debug/cppzmq

#Install glog
git clone --branch v0.6.0 --depth 1 https://github.com/google/glog
cd glog
mkdir build_0_6_0_Release
mkdir build_0_6_0_Debug

cd build_0_6_0_Release
cmake -DWITH_GFLAGS=off -DCMAKE_INSTALL_PREFIX="$local_path/deps_installed/Release/glog" -G $GENERATOR ..
cmake --build . --target install --config Release -j 4

cd ../build_0_6_0_Debug
cmake -DWITH_GFLAGS=off -DCMAKE_INSTALL_PREFIX="$local_path/deps_installed/Debug/glog" -G $GENERATOR ..
cmake --build . --target install --config Debug -j 4

#Install libzmq
cd $local_path
git clone --branch v4.3.4 --depth 1 https://github.com/zeromq/libzmq.git
cd libzmq
mkdir build_4_3_4_Release
mkdir build_4_3_4_Debug

cd build_4_3_4_Release
cmake -DBUILD_STATIC=off -DBUILD_TESTS=off  -DCMAKE_INSTALL_PREFIX="$local_path/deps_installed/Release/libzmq" -G $GENERATOR ..
cmake --build . --target install --config Release -j 4

cd ../build_4_3_4_Debug
cmake -DBUILD_STATIC=off -DBUILD_TESTS=off  -DCMAKE_INSTALL_PREFIX="$local_path/deps_installed/Debug/libzmq" -G $GENERATOR ..
cmake --build . --target install --config Debug -j 4


#Install cppzmq
cd $local_path
git clone --branch v4.9.0 --depth 1 https://github.com/zeromq/cppzmq.git
cd cppzmq
mkdir build_4_9_0_Release
mkdir build_4_9_0_Debug

cd build_4_9_0_Release
cmake -DCPPZMQ_BUILD_TESTS=off  -DCMAKE_INSTALL_PREFIX="$local_path/deps_installed/Release/libzmq/build_4_3_4" -G $GENERATOR ..
cmake --build . --target install --config Release -j 4

cd ../build_4_9_0_Debug
cmake -DCPPZMQ_BUILD_TESTS=off  -DCMAKE_INSTALL_PREFIX="$local_path/deps_installed/Debug/libzmq/build_4_3_4" -G $GENERATOR ..
cmake --build . --target install --config Debug -j 4


cd $local_path
git clone --branch v3.9.0 --depth 1 https://github.com/protocolbuffers/protobuf
cd protobuf
mkdir build_3_9_0_Release
mkdir build_3_9_0_Debug

cd build_3_9_0_Release
cmake -Dprotobuf_BUILD_TESTS=OFF -Dprotobuf_MSVC_STATIC_RUNTIME=OFF -DCMAKE_INSTALL_PREFIX="$local_path/deps_installed/Release/protobuf" -G $GENERATOR ../cmake
cmake --build . --target install --config Release -j 4

cd ../build_3_9_0_Debug
cmake -Dprotobuf_BUILD_TESTS=OFF -Dprotobuf_MSVC_STATIC_RUNTIME=OFF -DCMAKE_INSTALL_PREFIX="$local_path/deps_installed/Debug/protobuf" -G $GENERATOR ../cmake
cmake --build . --target install --config Debug -j 4
