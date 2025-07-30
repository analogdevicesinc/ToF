import subprocess
import os
import inspect
import logging
from io import StringIO

# Buffer for deferred logging
log_buffer = StringIO()

# Configure logging to write to buffer
logging.basicConfig(stream=log_buffer, level=logging.INFO, format='LOG(INFO): %(message)s')

# Directory and executable paths
repo_folder = "ToF"

exe_paths = {
    "ADIToFGUI.exe": ("ToF\\build\\examples\\tof-viewer\\Release", 7),
    "first-frame.exe": ("ToF\\build\\examples\\first-frame\\Release", 2),
    "data_collect.exe": ("ToF\\build\\examples\\data_collect\\Release", 2),
    "aditof.dll": ("ToF\\build\\libaditof\\sdk\\Release", 3),
    "data_collect.py": ("ToF\\build\\bindings\\python\\examples\\data_collect\\Release", 5),
    "dnn.py": ("ToF\\build\\bindings\\python\\examples\\dnn\\Release", 5),
    "dual_cameras.py": ("ToF\\build\\bindings\\python\\examples\\dual_cameras\\Release", 5),
    "first_frame.py": ("ToF\\build\\bindings\\python\\examples\\first_frame\\Release", 5),
    "process.py": ("ToF\\build\\bindings\\python\\examples\\gesture_rec\\Release", 6),
    "saveCCBToFile.py": ("ToF\\build\\bindings\\python\\examples\\saveCCBToFile\\Release", 5),
    "showPointCloud.py": ("ToF\\build\\bindings\\python\\examples\\showPointCloud\\Release", 5),
    "skeletal_tracking.py": ("ToF\\build\\bindings\\python\\examples\\skeletal_tracking\\Release", 5),
    "skeletal_tracking_in_pointcloud.py": ("ToF\\build\\bindings\\python\\examples\\skeletal_tracking_in_pointcloud\\Release", 5),
    "depth-image-animation-pygame.py": ("ToF\\build\\bindings\\python\\examples\\streaming\\Release", 5),
}

# Store test results
test_results = []

def log_error(binary, message):
    line = inspect.currentframe().f_back.f_lineno
    logging.info(f"@@,{binary},FAIL,LN:{line},DN:{message}")
    test_results.append({"binary": binary, "status": "Fail"})

def log_success(binary, message):
    line = inspect.currentframe().f_back.f_lineno
    logging.info(f"@@,{binary},PASS,LN:{line},DN:{message}")
    test_results.append({"binary": binary, "status": "Pass"})

def check_and_delete_folder(folder):
    if os.path.exists(folder):
        log_success("git", f"Folder '{folder}' already exists. Deleting...")
        subprocess.run(["rmdir", "/s", "/q", folder], shell=True)
        log_success("git", "Folder deleted.")
    else:
        log_success("git", f"Folder '{folder}' does not exist. Proceeding...")

def clone_repository(branch_name="main"):
    log_success("git", "Cloning repository with submodules...")
    result = subprocess.run(["git", "clone", "--recurse-submodules", "--branch", branch_name, "https://github.com/analogdevicesinc/ToF.git"], shell=True)
    if result.returncode != 0:
        log_error("git", "Failed to clone repository. Exiting...")
        exit(1)
    log_success("git", "Clone complete.")

def run_setup_script():
    os.chdir(os.path.join(repo_folder, "scripts", "windows"))
    result = subprocess.run("setup_project.bat", input="y\n", text=True, shell=True)
    if result.returncode != 0:
        log_error("setup_project.bat", "setup_project.bat failed. Exiting...")
        exit(1)
    os.chdir("../../../")

def check_executable(folder, exe, expected_count):
    if not os.path.exists(folder):
        log_error(exe, f"Folder '{folder}' does not exist.")
        return
    files_in_folder = os.listdir(folder)
    exe_found = exe in files_in_folder
    count = len(files_in_folder)

    if exe_found and count == expected_count:
        log_success(exe, f"{exe} exists in {folder}. Exactly {expected_count} items found.")
    else:
        if not exe_found:
            log_error(exe, f"{exe} does NOT exist in {folder}. Executable missing.")
        if count != expected_count:
            log_error(exe, f"Item count in {folder} is {count}, expected {expected_count}.")

# Run the steps
check_and_delete_folder(repo_folder)
clone_repository()
run_setup_script()

log_success("check", "Starting the testing of presence of the files in release folder")

# Check all executables
for exe, (path, count) in exe_paths.items():
    check_executable(path, exe, count)

# Print all logs at the end
print("\n--- TEST LOG REPORT ---")
print(log_buffer.getvalue())
