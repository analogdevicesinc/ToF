echo Running Cloning the repo and building...
powershell -Command "python SDK_Clone_Build_Automation_Script.py > \"SDK_Build_log_$(Get-Date -Format 'yyyyMMdd_HHmmss').txt\" 2>&1"

echo Running First Frame Using first_frame.exe...
powershell -Command "python First_Frame_Script.py > \"first_frame_log_$(Get-Date -Format 'yyyyMMdd_HHmmss').txt\" 2>&1"

echo Running Data collect using Data_collect.exe
powershell -Command "python Data_Collect_Script.py > \"Data_collect_log_$(Get-Date -Format 'yyyyMMdd_HHmmss').txt\" 2>&1"

echo Running Data collect py...
powershell -Command "python Python_data_collect.py > \"Python_Data_collect_log_$(Get-Date -Format 'yyyyMMdd_HHmmss').txt\" 2>&1"

echo All scripts executed.
