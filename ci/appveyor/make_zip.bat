mkdir C:\ToF\
pushd C:\ToF

if "%APPVEYOR_BUILD_WORKER_IMAGE%"=="Visual Studio 2015" (
    set folder=vs14
) else if "%APPVEYOR_BUILD_WORKER_IMAGE%"=="Visual Studio 2017" (
    set folder=vs15
) else if "%APPVEYOR_BUILD_WORKER_IMAGE%"=="Visual Studio 2019" (
    set folder=vs16
)
mkdir x64
mkdir x32
mkdir include
mkdir x64\%folder%
mkdir x32\%folder%
mkdir x64\%folder%\bin
mkdir x64\%folder%\bin\python
mkdir x64\%folder%\lib
mkdir x32\%folder%\bin
mkdir x32\%folder%\bin\python
mkdir x32\%folder%\lib

copy C:\projects\ToF\build\sdk\Release\aditof.dll C:\ToF\x64\%folder%\bin
copy C:\projects\ToF\build\sdk\Release\aditof.lib C:\ToF\x64\%folder%\lib

copy "C:\projects\ToF\build\bindings\python\Release\*.pyd" C:\ToF\x64\%folder%\bin\python


ren "C:\projects\ToF\build_debug\sdk\Debug\aditof.dll" "aditofd.dll"
ren "C:\projects\ToF\build_debug\sdk\Debug\aditof.lib" "aditofd.lib"
ren "C:\projects\ToF\build_debug\sdk\Debug\aditof.pdb" "aditofd.pdb"


copy C:\projects\ToF\build_debug\sdk\Debug\aditofd.dll C:\ToF\x64\%folder%\bin
copy C:\projects\ToF\build_debug\sdk\Debug\aditofd.pdb C:\ToF\x64\%folder%\bin
copy C:\projects\ToF\build_debug\sdk\Debug\aditofd.lib C:\ToF\x64\%folder%\lib
copy "C:\projects\ToF\build_debug\bindings\python\Debug\*.pyd" C:\ToF\x64\%folder%\bin\python

copy C:\projects\ToF\build_x32\sdk\Release\aditof.dll C:\ToF\x32\%folder%\bin
copy C:\projects\ToF\build_x32\sdk\Release\aditof.lib C:\ToF\x32\%folder%\lib
copy "C:\projects\ToF\build_x32\bindings\python\Release\*.pyd" C:\ToF\x32\%folder%\bin\python

ren "C:\projects\ToF\build_debug_x32\sdk\Debug\aditof.dll" "aditofd.dll"
ren "C:\projects\ToF\build_debug_x32\sdk\Debug\aditof.lib" "aditofd.lib"
ren "C:\projects\ToF\build_debug_x32\sdk\Debug\aditof.pdb" "aditofd.pdb"

copy C:\projects\ToF\build_debug_x32\sdk\Debug\aditofd.dll C:\ToF\x32\%folder%\bin
copy C:\projects\ToF\build_debug_x32\sdk\Debug\aditofd.pdb C:\ToF\x32\%folder%\bin
copy C:\projects\ToF\build_debug_x32\sdk\Debug\aditofd.lib C:\ToF\x32\%folder%\lib
copy "C:\projects\ToF\build_debug_x32\bindings\python\Debug\*.pyd" C:\ToF\x32\%folder%\bin\python

xcopy /E C:\projects\ToF\sdk\include C:\ToF\include

7z a "C:\ToF-%folder%.zip" C:\ToF
appveyor PushArtifact C:\ToF-%folder%.zip

popd
