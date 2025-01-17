# Web Interface Documentation

## Overview
This web interface provides a user-friendly platform to interact with various functionalities. It is divided into three main sections: Home, System, and ADSD3500. Each section offers specific features to enhance user experience and system management.

## Sections

### Home
The Home section serves as the landing page of the web interface. It provides an overview and quick access to essential features.

#### Functionalities:
- **Dashboard**: Displays Workspaces and current active workspace in the system.
- **Switch Workspace**: If you want to switch the workspace selct the workspace and click the button.
- **Notifications**: Shows recent relese which you can download in your system (host).

### System
The System section allows users to manage and configure system settings. It provides tools for system monitoring and maintenance.

#### Functionalities:
- **Power Down**: Shuts down the system safely.
- **Reboot**: Restarts the system.
- **Setup WiFi**: Configures WiFi settings.
- **Sync Time**: Synchronizes the system time with a time server.
- **Get Time**: Retrieves the current system time.

### ADSD3500
The ADSD3500 section is dedicated to the ADSD3500 device, providing specific controls and information related to its operation.

#### Functionalities:
- **Firmware Update**: Provides tools to update the firmware of the ADSD3500 device via a drop-down box for selecting the firmware version.
- **Change Mode of Operation**: Allows users to switch between different operational modes.
- **Change Version**: Enables users to update the device version.
- **Upload Zip File**: Allows users to upload a zip file to update the entire system. The update zip file should contain:
```
update.zip/
├─ ADSD3500-firmware-5.2.5.zip
├─ ToF-6.0.0.zip
└─ info.json

```
  - `ADSD3500-firmware-<version>.zip`: The firmware file for the ADSD3500 device.
  - `ToF-<version>.zip`: The firmware file for the Time-of-Flight (ToF) sensor.
  - `info.json`: A JSON file containing metadata about the update, with the following structure:
    ```json
    {
        "firmware": "ADSD3500-firmware-<version>.zip",
        "SDK": "ToF-<version>.zip"
    }
    ```


## Getting Started
To get started with the web interface, simply open http://10.43.0.1:8000/ in your web browser.


## Support
For any issues or questions, please contact our support team at support@example.com.