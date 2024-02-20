@ECHO OFF

Python -m venv tof_sdk_env

call .\tof_sdk_env\Scripts\activate.bat
REM timeout /T 5
python -m pip install -r .\requirements.txt > nul

PAUSE