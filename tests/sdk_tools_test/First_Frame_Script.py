import subprocess
import os
import time
import inspect
import logging
from io import StringIO

# Buffer for logs
log_buffer = StringIO()

# Configure logging to write to buffer
logging.basicConfig(stream=log_buffer, level=logging.INFO, format='LOG(INFO): %(message)s')

# Directory and executable path
exe_path = r"ToF\\build\\examples\\first-frame\\Release"

# Store test results
test_results = []

def log_error(binary, message):
    line = inspect.currentframe().f_back.f_lineno
    logging.info(f"@@,{binary},FAIL,LN:{line},DN:{message}")

def log_success(binary, message):
    line = inspect.currentframe().f_back.f_lineno
    logging.info(f"@@,{binary},PASS,LN:{line},DN:{message}")

def capture_first_frame(m_value):
    binary_name = "first-frame.exe"
    command = [os.path.join(exe_path, binary_name), "--ip", "192.168.56.1", "--m", str(m_value)]
    result = {"m_value": m_value, "status": "Fail", "message": "", "file": "", "size": 0}

    try:
        subprocess.run(command, check=True, cwd=exe_path)
        bin_files = [f for f in os.listdir(exe_path) if f.endswith('.bin')]
        if not bin_files:
            result["message"] = "No .bin file found."
            log_error(binary_name, result["message"])
        else:
            for file in bin_files:
                file_path = os.path.join(exe_path, file)
                size = os.path.getsize(file_path)
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

def remove_bin_files():
    for file in os.listdir(exe_path):
        if file.endswith('.bin'):
            os.remove(os.path.join(exe_path, file))

# Run tests
for m in [0, 1, 2, 3, 5, 6]:
    capture_first_frame(m)
    time.sleep(10)
    remove_bin_files()
    time.sleep(10)

# Print all logs at the end
print("\n--- TEST LOG REPORT ---")
print(log_buffer.getvalue())
