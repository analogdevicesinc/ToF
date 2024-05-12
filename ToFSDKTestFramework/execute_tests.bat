@ECHO OFF

setlocal

:: set directory of virtual environment
set VENV_PATH=./tof_sdk_env

:: Activate the virtual environment
call "%VENV_PATH%\Scripts\activate"

cd ./tests

:: Run your Python script
pytest test_capture_frames\test_capture_frames.py --html=test_reports\test_capture_frames_report.html -v
pytest test_access_ini\test_access_ini_file.py --html=test_reports\test_access_ini_file_report.html -v
pytest test_fps\test_fps.py --html=test_reports\test_fps_report.html -v
pytest test_getMetadata\test_getMetadata.py --html=test_reports\test_getMetadata_report.html -v
pytest test_saveCCBCFG\test_saveCCBCFG.py --html=test_reports\test_saveCCBCFG_report.html -v
pytest test_frameAPIs\test_frameAPIs.py --html=test_reports\test_frameAPIs_report.html -v
pytest test_get_camera_list\test_get_camera_list.py --html=test_reports\test_get_camera_list_report.html -v
pytest test_initialize_load_config\test_initialize_load_config.py --html=test_reports\test_initialize_load_config_report.html -v
pytest test_framehandler\test_framehandler.py -v --html=test_reports\test_framehandler_report.html -v
pytest test_temp_compensation\test_temp_compensation.py -v --html=test_reports\test_temp_compensation_report.html -v
pytest test_generictemplate\test_generictemplate.py --html=test_reports\test_generictemplate_report.html -v
pytest test_misc_APIs\test_misc_APIs.py --html=test_reports\test_misc_APIs_report.html -v
pytest test_sensor_enum\test_sensor_enum.py --html=test_reports\test_sensor_enum_report.html -v
pytest test_depth_sensor\test_depth_sensor.py --html=test_reports\test_depth_sensor_report.html -v
pytest test_depth_sensor_control\test_depth_sensor_control.py --html=test_reports\test_depth_sensor_control_report.html -v
pytest test_mipispeed\test_mipispeed.py -v --html=test_reports\test_mipispeed_report.html -v

:: Deactivate the virtual environment
cd ..
call "%VENV_PATH%\Scripts\deactivate"

PAUSE