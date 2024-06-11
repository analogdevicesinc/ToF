@ECHO OFF

set "SDKVersion=5.0.0"

set "folder=tests/config"
cd /d "%folder%"
for /F "delims=" %%i in ('dir /b') do (
   rmdir "%%i" /s /q || del "%%i" /s /q
)


if "%SDKVersion%" GEQ "5.1.0" (
    xcopy /s "../../config_files/config_5_1_0" "."
) else (
    xcopy /s "../../config_files/config_5_0_0" "."
)

cd  ../..

set "folder=../examples"
cd /d "%folder%"
for /F "delims=" %%i in ('dir /b') do (
   rmdir "%%i" /s /q || del "%%i" /s /q
)

xcopy /s "../ToFSDKTestFramework/test_c++" "."

cd ..
mkdir build
cd build

cmake -DCMAKE_PREFIX_PATH="C:\Users\AAbacan2\OneDrive - Analog Devices, Inc\Documents\GitHub\ToF\deps\glog\build_0_6_0\local_path\glog;C:\Users\AAbacan2\OneDrive - Analog Devices, Inc\Documents\GitHub\ToF\deps\protobuf\build_3_9_0\local_path\protobuf;C:\Users\AAbacan2\OneDrive - Analog Devices, Inc\Documents\GitHub\ToF\deps\libwebsockets\build_3_1\local_path\websockets" -G "Visual Studio 16 2019" -DWITH_EXAMPLES=on  -DWITH_PYTHON=on ..
cmake --build . --config Release

@echo off
set /p "dummy=Press any key to continue..."

PAUSE