from flask import Flask, render_template, request, jsonify, Response,stream_with_context,send_from_directory
import os
import json
import subprocess
import threading
import hashlib
import shutil
import logging
import time
from threading import Thread
from datetime import datetime, timedelta
from pytz import timezone
import pytz
from dateutil import parser

app = Flask(__name__)

# Directories and file paths
CHUNK_DIR = '/tmp/upload_chunks'
FINAL_FILE_PATH = '/tmp/final_file.zip'
UNZIP_DIR = '/tmp/unzipped_files'
TASK_STATUS_FILE = '/tmp/unzip_status.json'
DESTINATION_DIR_RO = '/oldroot/home/analog'
DESTINATION_DIR_RW = '/home/analog'

# Set up basic logging
logging.basicConfig(level=logging.INFO)
file_handler = logging.FileHandler('app.log')
file_handler.setLevel(logging.INFO)
formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
file_handler.setFormatter(formatter)
app.logger.addHandler(file_handler)

@app.route('/get-workspace', methods=['GET'])
def get_workspace():
    try:
        # Run the shell script and get the output
        result = subprocess.check_output(['./get-workspace.sh', '/home/analog']).decode('utf-8').strip()
        return jsonify({"workspace": result})
    except subprocess.CalledProcessError as e:
        # Handle errors if the script fails to execute
        return jsonify({"error": str(e)}), 500

def calculate_combined_md5(version_file_path, specified_file_path):
    """Calculate the combined MD5 checksum of two files."""
    hash_md5 = hashlib.md5()
    
    with open(version_file_path, "rb") as vf:
        for chunk in iter(lambda: vf.read(4096), b""):
            hash_md5.update(chunk)
    
    with open(specified_file_path, "rb") as sf:
        for chunk in iter(lambda: sf.read(4096), b""):
            hash_md5.update(chunk)
    
    return hash_md5.hexdigest()

def validate_and_process_unzipped_files(unzip_dir):
    """Validate the contents of the unzipped directory, move it if necessary, and run the post-process script."""
    access = app.config.get('RO_ACCESS', 'unknown')
    try:
        json_file_path = os.path.join(unzip_dir, "info.json")
        if not os.path.exists(json_file_path):
            return False, "Validation failed: info.json file is missing."

        with open(json_file_path, "r") as json_file:
            json_data = json.load(json_file)

        # this is for only one directory I need to change it for as many directory as I want
        # for firmware update
        firmware_file = json_data.get("firmware", "")
        firmware_file_path = os.path.join(unzip_dir, firmware_file)
        if not os.path.exists(firmware_file_path):
            return False, f"Validation failed: Specified container file {firmware_file} is missing."

        firmware_extract_dir = os.path.join(unzip_dir, os.path.splitext(firmware_file)[0])
        subprocess.run(['unzip', '-o', firmware_file_path, '-d', firmware_extract_dir], check=True)

        if (access['message'] == False):
            target_dir = os.path.join(DESTINATION_DIR_RW, os.path.basename(firmware_extract_dir))
            if os.path.exists(target_dir):
                return False, f"Error: Directory {os.path.basename(firmware_extract_dir)} already exists in {DESTINATION_DIR_RW}."
        else:
            target_dir = os.path.join(DESTINATION_DIR_RO, os.path.basename(firmware_extract_dir))
            if os.path.exists(target_dir):
                return False, f"Error: Directory {os.path.basename(firmware_extract_dir)} already exists in {DESTINATION_DIR_RO}."

        shutil.move(firmware_extract_dir, target_dir)
        logging.info(f"Directory moved to {target_dir}")


        # for SDK update
        SDK_file = json_data.get("SDK", "")
        SDK_file_path = os.path.join(unzip_dir, SDK_file)
        if not os.path.exists(SDK_file_path):
            return False, f"Validation failed: Specified container file {SDK_file} is missing."

        SDK_extract_dir = os.path.join(unzip_dir, os.path.splitext(SDK_file)[0])
        subprocess.run(['unzip', '-o', SDK_file_path, '-d', SDK_extract_dir], check=True)

        if (access['message'] == False):
            target_dir = os.path.join(DESTINATION_DIR_RW, os.path.basename(SDK_extract_dir))
            if os.path.exists(target_dir):
                return False, f"Error: Directory {os.path.basename(SDK_extract_dir)} already exists in {DESTINATION_DIR_RW}."
        else:
            target_dir = os.path.join(DESTINATION_DIR_RO, os.path.basename(SDK_extract_dir))
            if os.path.exists(target_dir):
                return False, f"Error: Directory {os.path.basename(SDK_extract_dir)} already exists in {DESTINATION_DIR_RO}."

        shutil.move(SDK_extract_dir, target_dir)
        logging.info(f"Directory moved to {target_dir}")

        # script_output = execute_post_script()
        return True, f"Validation and move successful. Directory moved to {target_dir}"

    except subprocess.CalledProcessError as e:
        logging.error(f"Subprocess error during unzipping: {str(e)}")
        return False, f"Unzipping error: {str(e)}"
    except Exception as e:
        logging.error(f"Validation error: {str(e)}")
        return False, f"Validation or move error: {str(e)}"

@app.route('/execute-post-script', methods=['GET'])
def execute_post_script_route():
    """Route to execute the post-process script and return its output."""
    output = execute_post_script()
    return jsonify(output.splitlines())  # Send output as a list of lines

def execute_post_script():
    """Run the specified script and return its output or error, excluding the first line."""
    try:
        result = subprocess.run(['./switch-workspace.sh ~'], capture_output=True, text=True, shell=True)
        if result.returncode != 0:
            error_message = f"Script error: {result.stderr.strip()}"
            logging.error(error_message)
            return error_message
        output_lines = result.stdout.splitlines()[1:]  # Exclude the first line
        return "\n".join(output_lines)
    except Exception as e:
        error_message = f"Script execution failed: {str(e)}"
        logging.error(error_message)
        return error_message

def clean_up():
    """Clean up temporary files and directories, and mark the process as done."""
    try:
        if os.path.exists(FINAL_FILE_PATH):
            os.remove(FINAL_FILE_PATH)
        if os.path.exists(UNZIP_DIR):
            shutil.rmtree(UNZIP_DIR)

        # Instead of removing the status file, set a "done" status
        if os.path.exists(TASK_STATUS_FILE):
            with open(TASK_STATUS_FILE, 'w') as f:
                json.dump({"status": "done"}, f)

        logging.info("Temporary files and directories cleaned up, status marked as done.")
    except Exception as e:
        logging.error(f"Error during cleanup: {str(e)}")

def unzip_in_background(zip_path, extract_to):
    """Unzips the file in a background thread, validates it, moves it if necessary, and runs a script."""
    logging.info("Unzip process started.")

    # Set initial status to "unpacking" and confirm the write
    try:
        with open(TASK_STATUS_FILE, 'w') as f:
            json.dump({"status": "unpacking"}, f)
        logging.info("Status set to 'unpacking' and written to status file.")
    except Exception as e:
        logging.error(f"Failed to write initial status: {e}")
        return

    # Run unzip and wait for completion
    process = subprocess.Popen(['unzip', '-o', zip_path, '-d', extract_to],
                               stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    stdout, stderr = process.communicate()

    if process.returncode == 0:
        logging.info("Unzip process completed successfully.")
        is_valid, validation_message = validate_and_process_unzipped_files(extract_to)
        status = {"status": "completed" if is_valid else "failed", "validation": validation_message}
        logging.info(f"Validation and move status: {status}")
    else:
        status = {"status": "failed", "error": stderr.decode().strip()}
        logging.error(f"Unzip failed with error: {status['error']}")

    # Execute post-processing and clean up
    # clean_up()

    # Final status update after cleanup
    try:
        with open(TASK_STATUS_FILE, 'w') as f:
            json.dump(status, f)
        logging.info(f"Final status written after cleanup: {status}")
    except Exception as e:
        logging.error(f"Failed to write final status: {e}")

def start_background_unzip(zip_path, extract_to):
    """Start the unzip process in a new thread."""
    thread = threading.Thread(target=unzip_in_background, args=(zip_path, extract_to))
    thread.start()

@app.route('/')
def index():
    return render_template('index.html',active_page='home')

@app.route('/system')
def system():
    return render_template('system.html',active_page='system')

@app.route('/document')
def document():
    return render_template('document.html',active_page='document')

@app.route('/ADSD3500')
def ADSD3500():
    return render_template('ADSD3500.html',active_page='ADSD3500')

@app.route('/upload-chunk', methods=['POST'])
def upload_chunk():

    r_access = None    
    # First check the file Read/Write access 
    command = ['sudo', '-S', './change-permission.sh','view']

    # Create the subprocess and pass the password to stdin
    process = subprocess.Popen(command, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)

    # Send the password to the subprocess and capture the output
    stdout, stderr = process.communicate(input='analog\n')

    if 'RO' in stdout:

        r_access = True
        command = ['sudo', 'mount', '-o', 'remount,rw', '/dev/root', '/oldroot']

        # Create the subprocess and pass the password to stdin
        process = subprocess.Popen(command, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)

        # Communicate with the process to capture output and errors
        stdout, stderr = process.communicate()

        # Check the return code
        if process.returncode == 0:
            print("Command executed successfully")
            print("Output:", stdout)
        else:
            print("Command failed with return code", process.returncode)
            print("Error:", stderr)
    else:
        r_access = False



    file = request.files.get('file')
    chunk_index = int(request.form['chunkIndex'])
    total_chunks = int(request.form['totalChunks'])

    # Create directory for chunks if it doesn't exist
    os.makedirs(CHUNK_DIR, exist_ok=True)
    chunk_path = os.path.join(CHUNK_DIR, f"chunk_{chunk_index}")

    # Save the received chunk
    with open(chunk_path, 'wb') as f:
        f.write(file.read())

    app.logger.info(f"Chunk {chunk_index + 1}/{total_chunks} received successfully")
    return jsonify({'message': r_access})

@app.route('/complete-upload', methods=['POST'])
def complete_upload():
    total_chunks = int(request.form['totalChunks'])

    # Combine chunks into the final file
    with open(FINAL_FILE_PATH, 'wb') as final_file:
        for i in range(total_chunks):
            chunk_file_path = os.path.join(CHUNK_DIR, f"chunk_{i}")
            with open(chunk_file_path, 'rb') as chunk_file:
                final_file.write(chunk_file.read())
            os.remove(chunk_file_path)  # Delete each chunk after combining

    # Clean up the chunk directory
    os.rmdir(CHUNK_DIR)

    # Start the unzip, validation, move, and script execution process in a background thread
    try:
        start_background_unzip(FINAL_FILE_PATH, UNZIP_DIR)
        return jsonify({'message': 'File uploaded successfully. Unzipping, validation, move, and script execution in progress.'}), 202
    except Exception as e:
        app.logger.error(f"Error during unzip initiation: {str(e)}")
        return jsonify({'message': 'Error starting unzip process.', 'error': str(e)}), 500

@app.route('/unzip-status', methods=['GET'])
def unzip_status():
    """Endpoint to check the status of the background unzip, validation, move, and script execution task."""
    if not os.path.exists(TASK_STATUS_FILE):
        return jsonify({"status": "no_task"})

    with open(TASK_STATUS_FILE, 'r') as f:
        try:
            status = json.load(f)
        except json.JSONDecodeError:
            logging.error("Status file contains invalid JSON.")
            return jsonify({"status": "error", "error": "Invalid status file format."}), 500

    return jsonify(status)

@app.route('/ro-access',methods=['POST'])
def ro_access():
    data = request.get_json()
    value = data.get('value')
    app.config['RO_ACCESS'] = value  
    return '', 204

@app.route('/switch-workspace', methods=['POST'])
def switch_workspace():
    data = request.get_json()
    entry = data.get("entry")

    if not entry:
        return jsonify({"error": "No entry provided"}), 400

    try:
        # Run the switch-workspace script with the selected entry
        result = subprocess.run(['./switch-workspace.sh', '~', entry],
                                capture_output=True, text=True, check=True)
        
        # Check if there’s an error
        if result.returncode != 0:
            return jsonify({"error": result.stderr.strip()}), 500

        return jsonify({"message": f"Workspace switched to {entry} successfully."})
    
    except subprocess.CalledProcessError as e:
        return jsonify({"error": str(e)}), 500
    except Exception as e:
        return jsonify({"error": f"Unexpected error: {str(e)}"}), 500

def reboot_async():
    time.sleep(1)  # Optional: Delay for better handling
    subprocess.run(['sudo', '/sbin/reboot'], check=True)

# Route for rebooting the system
@app.route('/reboot-system', methods=['POST'])
def reboot_system():
    try:
        subprocess.run(["sudo", "reboot"], check=True)
        return jsonify({"message": "Reboot initiated successfully."}), 200
    except subprocess.CalledProcessError:
        return jsonify({"error": "Failed to reboot the system."}), 500

# Route for powering down the system
@app.route('/power-down-system', methods=['POST'])
def power_down_system():
    try:
        subprocess.run(["sudo", "poweroff"], check=True)
        return jsonify({"message": "Power down initiated successfully."}), 200
    except subprocess.CalledProcessError:
        return jsonify({"error": "Failed to power down the system."}), 500

@app.route('/status', methods=['GET'])
def status():
    return jsonify({"status": "online"}), 200

@app.route('/run-service-backup-nvm', methods=['POST'])
def run_service():
    try:
        # Restart the service
        subprocess.run(
            ['sudo', '-S', 'systemctl', 'restart', 'adi-backup.service'], 
            input='your_password\n',  # Replace 'your_password' with the actual sudo password
            capture_output=True, 
            text=True, 
            check=True
        )

        # Get the current timestamp
        current_time = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

        # Fetch logs from journalctl
        journal_output = subprocess.run(
            ['sudo', '-S', 'journalctl', '-u', 'adi-backup.service', '--since', current_time, '--no-pager'],
            input='analog\n',  # Replace 'your_password' with the actual sudo password
            capture_output=True,
            text=True,
            check=True
        )

        return jsonify({"output": journal_output.stdout}), 200
    except subprocess.CalledProcessError as e:
        return jsonify({"output": f"Error: {e.stderr}"}), 500
    except Exception as e:
        return jsonify({"output": f"Unexpected error: {str(e)}"}), 500

@app.route('/check-file-sizes', methods=['GET'])
def check_file_sizes():
    ccb_path = '/boot/ADTF3175D.ccb'  # Replace with actual file path
    nvm_path = '/boot/ADTF3175D.nvm'  # Replace with actual file path

    # Define the size threshold in bytes
    size_threshold_ccb = 243426
    size_threshold_ncm = 5745472

    try:
        # Get file sizes
        ccb_size = os.path.getsize(ccb_path) if os.path.exists(ccb_path) else 0
        nvm_size = os.path.getsize(nvm_path) if os.path.exists(nvm_path) else 0

        # Check if both files meet the size threshold
        is_complete = ccb_size >= size_threshold_ccb and nvm_size >= size_threshold_ncm

        return jsonify({
            "ccb_size": ccb_size,
            "nvm_size": nvm_size,
            "is_complete": is_complete
        })

    except Exception as e:
        return jsonify({"error": str(e)}), 500


@app.route('/read-ini')
def read_ini():
    try:
        with open('config.ini', 'r') as file:
            ini_content = file.read()
        return ini_content
    except Exception as e:
        return str(e), 500

@app.route('/server-time', methods=['GET'])
def server_time():
    server_time_utc = datetime.now(pytz.utc).strftime("%Y-%m-%d %H:%M:%S")
    app.logger.info(f"Server time: {server_time_utc}")
    return jsonify({"server_time_utc": server_time_utc})

@app.route('/set-server-time', methods=['POST'])
def set_server_time():
    data = request.json
    browser_time_str = data.get('browser_time')  # Full ISO format including local timezone
    time_zone_name = data.get('time_zone_name')
    app.logger.info(f"Received browser time: {browser_time_str} (Timezone: {time_zone_name})")

    try:
        # Parse the browser time including its timezone
        browser_time = parser.parse(browser_time_str)
        app.logger.info(f"Parsed browser time: {browser_time}")

        # Convert to the received timezone
        local_time = browser_time
        app.logger.info(f"Local time: {local_time}")


        # Get the timezone object
        local_timezone = pytz.timezone(time_zone_name)

        # Get the timezone abbreviation
        timezone_abbr = local_time.astimezone(local_timezone).tzname()

        # Format the server time for the `date` command
        server_time_str = local_time.strftime("%Y-%m-%d %I:%M:%S %p")

        # Append the timezone abbreviation
        if timezone_abbr:
            server_time_str += f" {timezone_abbr}"

        app.logger.info(f"Formatted server time for `date` command: {server_time_str}")

        ## Set the server time
        subprocess.run(['sudo', 'date', '-s', server_time_str], check=True)

        return jsonify({"message": "Server time updated successfully"}), 200
    except Exception as e:
        app.logger.error(f"Error setting server time: {e}")
        return jsonify({"error": str(e)}), 500

# setup the wifi

@app.route('/get-username')
def getUserName():
    try:
        command = ['sudo','-S','./get-ssid.sh']

        usr_password = 'analog\n'

        result = subprocess.run(command, capture_output=True, text=True,input=usr_password)

        # Check if there was an error
        if result.returncode != 0:
            print("Error:", result.stderr)

        return jsonify({"username": result.stdout}),200
    except Exception as e:
        return jsonify({'error': str(e)}),500

@app.route('/setup-wifi', methods=['POST'])
def setup_wifi():
    data = request.get_json()
    username = data.get('username')
    password = data.get('password')

    try:
        current_workspace = get_workspace().get_json().get('workspace')
        
        script_path = f'/home/analog/Workspace-{current_workspace}/Tools/adi-enable-wifi.sh'

        # Define the command and password
        command = ['sudo', '-S', script_path, username, password]

        user_pass = 'analog\n'
        
        result = subprocess.run(command, capture_output=True, text=True,input=user_pass)
        if result.returncode == 0:
            return jsonify(message='WiFi setup successful')
        else:
            return jsonify(message='WiFi setup failed: ' + result.stderr), 500
    except Exception as e:
        return jsonify(message='Error setting up WiFi: ' + str(e)), 500

@app.route('/network-status')
def ping_host():

    try:
        command = ['ping', '-c', '1', '8.8.8.8']

        # Run the ping command
        result = subprocess.run(command, capture_output=True, text=True)

        # Check the return code to determine if the ping was successful
        if result.returncode == 0:
            return jsonify({'status':True}), 200
        else:
            return jsonify({'status':False}), 500
    except Exception as e:
        return jsonify({'error': str(e)}),500
    
# flash firmware

@app.route('/firmware_versions', methods=['GET'])
def firmware_versions():
    try:
        result = subprocess.check_output(['./get-firmware.sh', '/home/analog']).decode('utf-8').strip()
        versions = result.split('\n')
        return jsonify({'versions': versions})
    except subprocess.CalledProcessError as e:
        return jsonify({'error': str(e)}), 500

@app.route('/check_firmware')
def check_firmware():

    try:
        current_workspace = get_workspace().get_json().get('workspace')
        command = ['./check-firmware-version.sh',current_workspace]

        result = subprocess.check_output(command).decode('utf-8').strip()
        return jsonify({'version':result})
    
    except subprocess.CalledProcessError as e:
        return jsonify({'error': str(e)}), 500

@app.route('/list_firmware_versions')
def execute_list_firware():

    output = list_firmware_versions()
    return jsonify(output.splitlines())

def list_firmware_versions():
    try:
        result = subprocess.run(['./list-firmware-version.sh ~'], capture_output=True, text=True, shell=True)
        if result.returncode != 0:
            error_message = f"Script error: {result.stderr.strip()}"
            logging.error(error_message)
            return error_message
        return result.stdout
    except subprocess.CalledProcessError as e:
        return jsonify({'error': str(e)}), 500

@app.route('/current_firmware', methods=['GET'])
def current_firmware():
    try:
        curr_version = get_workspace().get_json().get('workspace')
        result = subprocess.check_output(['./get-firmware-version.sh', curr_version]).decode('utf-8').strip()
        return jsonify({'current_firmware': result})
    except subprocess.CalledProcessError as e:
        return jsonify({'error': str(e)}), 500
    

@app.route('/run_shell_script', methods=['POST'])
def run_shell_script():
    data = request.get_json()
    version = data.get('version')
    script = data.get('script')
    app.config['SCRIPT'] = script
    app.config['VERSION'] = version  
    return '', 204  

@app.route('/events')
def events():
    version = app.config.get('VERSION', 'unknown')

    def generate():
        try:
            curr_version = get_workspace().get_json().get('workspace')
            command = ['sudo', '-S', './flash_firmware.sh', curr_version, version]

            # Create the subprocess and pass the password to stdin
            process = subprocess.Popen(command, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)

            # Send the password to the subprocess
            process.stdin.write('analog\n')
            process.stdin.flush()
            for line in iter(process.stdout.readline, ''):
                # print(f"Output: {line.strip()}")
                yield f"data: {line}\n\n"
            process.stdout.close()
            process.wait()
        except subprocess.CalledProcessError as e:
            print(f"Error: {e.output}")
            yield f"data: Error: {e.output}\n\n"

    return Response(stream_with_context(generate()), mimetype='text/event-stream')

@app.route('/get-markdown', methods=['GET'])
def get_markdown():
    app.logger.info(f'Requested')
    """Serve a Markdown file dynamically based on directory and file parameters."""
    base_directory = '/home/analog'  # Define a safe base directory
    directory = request.args.get('directory', '')  # Retrieve the directory
    file_name = request.args.get('file')  # Retrieve the file name

    if not file_name:
        app.logger.error('File name is required but not provided.')
        return jsonify({'error': 'File name is required'}), 400

    # Construct the file path securely
    file_path = os.path.join(base_directory, directory, file_name)
    file_path = os.path.abspath(file_path)

    # Log the file path
    app.logger.info(f'Requested Markdown file path: {file_path}')

    # Ensure the file path is within the allowed base directory
    if not file_path.startswith(base_directory):
        app.logger.warning(f'Unauthorized file access attempt: {file_path}')
        return jsonify({'error': 'Access to this file is not allowed'}), 403

    try:
        with open(file_path, 'r') as f:
            content = f.read()
        return jsonify({'markdown': content})
    except FileNotFoundError:
        app.logger.error(f'File not found: {file_path}')
        return jsonify({'error': f'File {file_name} not found in {directory}'}), 404
    except Exception as e:
        app.logger.exception(f'Error reading file {file_path}')
        return jsonify({'error': str(e)}), 500

@app.route('/Change-Permission', methods=['POST'])
def changePermission():
    data = request.get_json()
    value = data.get('value')
    app.config['VALUE'] = value  
    return '', 204 

@app.route('/Change-Permission-Events')
def changePermissionEvents():
    value = app.config.get('VALUE', 'unknown')

    try:
        command = ['sudo', '-S', './change-permission.sh',value]

        # Create the subprocess and pass the password to stdin
        process = subprocess.Popen(command, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)

        # Send the password to the subprocess and capture the output
        stdout, stderr = process.communicate(input='analog\n')

        # Return the output as a JSON response
        return jsonify({'output': stdout})
    except subprocess.CalledProcessError as e:
        return jsonify({'error': str(e)}), 500

# NVM Read
    
@app.route('/run-script')
def run_script():
    version = app.config.get('VERSION', 'unknown')
    script  = app.config.get('SCRIPT','unknown')

    def generate():
        try:
            curr_version = get_workspace().get_json().get('workspace')
            command = ['sudo', '-S', script, curr_version, version]

            # Create the subprocess and pass the password to stdin
            process = subprocess.Popen(command, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)

            # Send the password to the subprocess
            process.stdin.write('analog\n')
            process.stdin.flush()

            # Read the output line by line
            for line in iter(process.stdout.readline, ''):
                # print(f"Output: {line.strip()}")
                yield f"data: {line}\n\n"
            process.stdout.close()
            process.wait()

            # Check for errors
            if process.returncode != 0:
                error_output = process.stderr.read()
                print(f"Error: {error_output}")
                yield f"data: Error: {error_output}\n\n"
            process.stderr.close()

        except Exception as e:
            print(f"Exception: {str(e)}")
            yield f"data: Exception: {str(e)}\n\n"

    return Response(stream_with_context(generate()), mimetype='text/event-stream')



@app.route('/download', methods=['POST'])
def download_file():
    data = request.get_json()
    file_name = data.get('filename')
    path = data.get('path')

    curr_version = get_workspace().get_json().get('workspace')

    download_path = f'/home/analog/Workspace-{curr_version}/{path}'

    try:
        # Check if the file exists before attempting to send it
        file_path = os.path.join(download_path, file_name)
        if not os.path.exists(file_path):
            return jsonify({'error': 'File not found'}), 404

        return send_from_directory(download_path, file_name, as_attachment=True)
    except Exception as e:
        print(f"Exception: {str(e)}")
        return jsonify({'error': str(e)}), 500
    finally:
        # Ensure the file is deleted after sending
        if os.path.exists(file_path):
            os.remove(file_path)
    
if __name__ == '__main__':
    app.run(debug=True, port = 5001)  # Enable debug mode for detailed logs
