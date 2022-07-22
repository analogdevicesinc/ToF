set PATH=%PATH%;"C:\Program Files (x86)\Inno Setup 5"
ISCC C:\projects\ToF\build\aditof-setup.iss
appveyor PushArtifact C:\ADI-ITOF-Setup.exe
