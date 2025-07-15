import subprocess
import os
import time
import inspect
import logging
from io import StringIO
import sys

# Buffer for deferred logging
log_buffer = StringIO()
logging.basicConfig(stream=log_buffer, level=logging.INFO, format='LOG(INFO): %(message)s')

# Relative path to the Python script
relative_script_path = os.path.join("ToF", "build", "bindings", "python", "examples", "data_collect", "Release", "data_collect.py")
script_path = os.path.abspath(relative_script_path)
exe_path = os.path.dirname(script_path)

# Relative path to the Python script

# Store test results
test_results = []

def log_error(binary, message):
    line = inspect.currentframe().f_back.f_lineno
    logging.info(f"@@,{binary},FAIL,LN:{line},DN:{message}")
    test_results.append({"binary": binary, "status": "Fail", "message": message})

def log_success(binary, message):
    line = inspect.currentframe().f_back.f_lineno
    logging.info(f"@@,{binary},PASS,LN:{line},DN:{message}")
    test_results.append({"binary": binary, "status": "Pass", "message": message})

def run_data_collect(n_value, m_value, folder_name, ip_address="192.168.56.1"):
    binary_name = "data_collect.py"
    folder_path = os.path.join(exe_path, folder_name)
    os.makedirs(folder_path, exist_ok=True)

    command = [
        sys.executable,
        script_path,
        "-f", folder_name,
        "-n", str(n_value),
        "-m", str(m_value),
        "-ip", ip_address
    ]

    try:
        subprocess.run(command, check=True, cwd=exe_path)
        bin_files = [f for f in os.listdir(folder_path) if f.endswith('.bin')]
        if not bin_files:
            log_error(binary_name, "No .bin file found.")
        else:
            for file in bin_files:
                dst = os.path.join(folder_path, file)
                size = os.path.getsize(dst)
                if size > 0:
                    log_success(binary_name, f"File '{file}' created with size {size} bytes.")
                else:
                    log_error(binary_name, f"File '{file}' is empty.")
    except subprocess.CalledProcessError as e:
        log_error(binary_name, f"Command failed: {e}")
    except FileNotFoundError:
        log_error(binary_name, "Python script not found.")

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
