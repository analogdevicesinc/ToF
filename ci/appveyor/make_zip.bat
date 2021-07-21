mkdir C:\aditof-sdk-rework\
pushd C:\aditof-sdk-rework

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

copy C:\projects\aditof-sdk-rework\build\sdk\Release\aditof.dll C:\aditof-sdk-rework\x64\%folder%\bin
copy C:\projects\aditof-sdk-rework\build\sdk\Release\aditof.lib C:\aditof-sdk-rework\x64\%folder%\lib

copy "C:\projects\aditof-sdk-rework\build\bindings\python\Release\*.pyd" C:\aditof-sdk-rework\x64\%folder%\bin\python


ren "C:\projects\aditof-sdk-rework\build_debug\sdk\Debug\aditof.dll" "aditofd.dll"
ren "C:\projects\aditof-sdk-rework\build_debug\sdk\Debug\aditof.lib" "aditofd.lib"
ren "C:\projects\aditof-sdk-rework\build_debug\sdk\Debug\aditof.pdb" "aditofd.pdb"


copy C:\projects\aditof-sdk-rework\build_debug\sdk\Debug\aditofd.dll C:\aditof-sdk-rework\x64\%folder%\bin
copy C:\projects\aditof-sdk-rework\build_debug\sdk\Debug\aditofd.pdb C:\aditof-sdk-rework\x64\%folder%\bin
copy C:\projects\aditof-sdk-rework\build_debug\sdk\Debug\aditofd.lib C:\aditof-sdk-rework\x64\%folder%\lib
copy "C:\projects\aditof-sdk-rework\build_debug\bindings\python\Debug\*.pyd" C:\aditof-sdk-rework\x64\%folder%\bin\python

copy C:\projects\aditof-sdk-rework\build_x32\sdk\Release\aditof.dll C:\aditof-sdk-rework\x32\%folder%\bin
copy C:\projects\aditof-sdk-rework\build_x32\sdk\Release\aditof.lib C:\aditof-sdk-rework\x32\%folder%\lib
copy "C:\projects\aditof-sdk-rework\build_x32\bindings\python\Release\*.pyd" C:\aditof-sdk-rework\x32\%folder%\bin\python

ren "C:\projects\aditof-sdk-rework\build_debug_x32\sdk\Debug\aditof.dll" "aditofd.dll"
ren "C:\projects\aditof-sdk-rework\build_debug_x32\sdk\Debug\aditof.lib" "aditofd.lib"
ren "C:\projects\aditof-sdk-rework\build_debug_x32\sdk\Debug\aditof.pdb" "aditofd.pdb"

copy C:\projects\aditof-sdk-rework\build_debug_x32\sdk\Debug\aditofd.dll C:\aditof-sdk-rework\x32\%folder%\bin
copy C:\projects\aditof-sdk-rework\build_debug_x32\sdk\Debug\aditofd.pdb C:\aditof-sdk-rework\x32\%folder%\bin
copy C:\projects\aditof-sdk-rework\build_debug_x32\sdk\Debug\aditofd.lib C:\aditof-sdk-rework\x32\%folder%\lib
copy "C:\projects\aditof-sdk-rework\build_debug_x32\bindings\python\Debug\*.pyd" C:\aditof-sdk-rework\x32\%folder%\bin\python

xcopy /E C:\projects\aditof-sdk-rework\sdk\include C:\aditof-sdk-rework\include

7z a "C:\aditof-sdk-rework-%folder%.zip" C:\aditof-sdk-rework
appveyor PushArtifact C:\aditof-sdk-rework-%folder%.zip

popd
