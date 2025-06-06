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

