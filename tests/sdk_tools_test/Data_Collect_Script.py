import subprocess
import os
import time
import inspect
import logging
from io import StringIO

# Buffer for deferred logging
log_buffer = StringIO()

# Configure logging to write to buffer
logging.basicConfig(stream=log_buffer, level=logging.INFO, format='LOG(INFO): %(message)s')

# Base directory and executable path
exe_path = r"ToF\\build\\examples\\data_collect\\Release"

# Store test results
test_results = []

def log_error(binary, message):
    line = inspect.currentframe().f_back.f_lineno
    logging.info(f"@@,{binary},FAIL,LN:{line},DN:{message}")

def log_success(binary, message):
    line = inspect.currentframe().f_back.f_lineno
    logging.info(f"@@,{binary},PASS,LN:{line},DN:{message}")

def run_data_collect(n_value, m_value, folder_name, ip_address="192.168.56.1"):
    binary_name = "data_collect.exe"
    folder_path = os.path.join(exe_path, folder_name)
    os.makedirs(folder_path, exist_ok=True)

    command = [
        os.path.join(exe_path, binary_name),
        "--f", folder_name,
        "--n", str(n_value),
        "--m", str(m_value),
        "--ip", ip_address
    ]

    result = {
        "n_value": n_value,
        "m_value": m_value,
        "folder": folder_name,
        "status": "Fail",
        "message": "",
        "file": "",
        "size": 0
    }

    try:
        subprocess.run(command, check=True, cwd=exe_path)
        bin_files = [f for f in os.listdir(folder_path) if f.endswith('.bin')]
        if not bin_files:
            result["message"] = "No .bin file found."
            log_error(binary_name, result["message"])
        else:
            for file in bin_files:
                dst = os.path.join(folder_path, file)
                size = os.path.getsize(dst)
                result["file"] = file
                result["size"] = size
                if size > 0:
                    result["status"] = "Pass"
                    result["message"] = f"File '{file}' created with size {size} bytes."
                    log_success(binary_name, result["message"])
                else:
                    result["message"] = f"File '{file}' is empty."
                    log_error(binary_name, result["message"])
    except subprocess.CalledProcessError as e:
        result["message"] = f"Command failed: {e}"
        log_error(binary_name, result["message"])
    except FileNotFoundError:
        result["message"] = "Executable not found."
        log_error(binary_name, result["message"])

    test_results.append(result)

# Run tests
for m in [0, 1, 2, 3, 5, 6]:
    time.sleep(10)
    folder = f"mode_{m}"
    for n in [1, 10, 50]:
        time.sleep(10)
        run_data_collect(n_value=n, m_value=m, folder_name=folder)
        time.sleep(10)

# Print all logs at the end
print("\n--- TEST LOG REPORT ---")
print(log_buffer.getvalue())
