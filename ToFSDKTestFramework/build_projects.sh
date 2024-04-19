#!/bin/bash
cd ../build/

cmake -DCMAKE_PREFIX_PATH="C:\Users\AAbacan2\OneDrive - Analog Devices, Inc\Documents\GitHub\ToF\deps\glog\build_0_6_0\local_path\glog;C:\Users\AAbacan2\OneDrive - Analog Devices, Inc\Documents\GitHub\ToF\deps\protobuf\build_3_9_0\local_path\protobuf;C:\Users\AAbacan2\OneDrive - Analog Devices, Inc\Documents\GitHub\ToF\deps\libwebsockets\build_3_1\local_path\websockets" -G "Visual Studio 16 2019" -DWITH_EXAMPLES=on  -DWITH_PYTHON=on ..
cmake --build . --config Release

read -n1 -r -p "Press any key to continue..."