@ECHO OFF

setlocal

:: set directory of virtual environment
set VENV_PATH=./tof_sdk_env

:: Activate the virtual environment
call "%VENV_PATH%\Scripts\activate"

cd ./tests

:: Run your Python script
pytest test_capture_frames.py --html=test_reports\test_capture_frames_report.html -v
pytest test_access_ini_file.py --html=test_reports\test_access_ini_file_report.html -v
pytest test_fps.py --html=test_reports\test_fps_report.html -v
pytest test_getMetadata.py --html=test_reports\test_getMetadata.html -v
pytest test_saveCCBCFG.py --html=test_reports\test_saveCCBCFG.html -v


:: Deactivate the virtual environment
cd ..
call "%VENV_PATH%\Scripts\deactivate"



PAUSE