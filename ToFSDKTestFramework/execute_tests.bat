@ECHO OFF

pytest test_access_ini_file.py --html=test_reports/test_access_ini_file_report.html
pytest test_capture_frames.py --html=test_reports/test_capture_frames_report.html

PAUSE