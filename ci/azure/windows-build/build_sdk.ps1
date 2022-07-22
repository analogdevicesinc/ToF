#install OpenCV
cinst opencv --version 3.4.1

#build sdk
mkdir build_Release
mkdir build_Debug
mkdir ../libs

cd build_Release
cmake -DUSE_DEPTH_COMPUTE_STUBS=1 -DWITH_OPENCV=on -DWITH_PYTHON=on -DCMAKE_PREFIX_PATH="../deps_installed/Release/glog;../deps_installed/Release/protobuf;../deps_installed/Release/websockets;..\deps_installed\OpenSSL-Win64" -DOpenCV_DIR="C:/tools/opencv/build/x64/vc15/lib" -DOPENSSL_INCLUDE_DIRS="..\deps_installed\OpenSSL-Win64\include" ..
cmake --build . --target install --config Release -j 4

cd ../build_Debug
cmake -DUSE_DEPTH_COMPUTE_STUBS=1 -DWITH_OPENCV=on -DWITH_PYTHON=on -DCMAKE_PREFIX_PATH="../deps_installed/Debug/glog;../deps_installed/Debug/protobuf;../deps_installed/Debug/websockets;..\deps_installed\OpenSSL-Win64" -DOpenCV_DIR="C:/tools/opencv/build/x64/vc15/lib" -DOPENSSL_INCLUDE_DIRS="..\deps_installed\OpenSSL-Win64\include" ..
cmake --build . --target install --config Debug -j 4

