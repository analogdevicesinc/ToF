See **ADTF3175D-EvalKit-610.html** for documentation.

# ADI Time-of-Flight Evaluation

## Version 6.1.0

### Additions, Updates and Fixes

* Set metadata to 0 to eliminate its impact on scaling the AB image for libaditof.
* Fix the Python bindings issue in getting the confidence frame.
* first-frame.py :- Converted the pixel values from float32 to uint8 for confidence frames ( for qmp and QVGA use case)

### Known Issues

* PCM is disabled by default. To stream PCM build SDK on NXP with ENABLE_PCM flag and restart aditof_server.

* Currently ADI ToF GUI supports streaming only with 16-bit (AB, depth) and 8-bit confidence. Any other combinations might fail.
* FPS might reduce in point cloud, due to rendering issue (due to OpenGL being done via software).

## Version 6.0.0

### Additions, Updates and Fixes

This is a significant update. As a result, please carefully note the changes below.

* The need for config_platform.json files is removed in this release.
* ADSD3500 firmware version 5.2.5 is part of this installer.
* All the required information will be fetched from the camera calibration file.
* Added support for saving the default configuration file. To save the current configuration, make sure you start streaming in any mode.
* Added an option to load custom configuration files. To load the custom configuration, make sure you stop streaming.
* Auto playback is disabled in the recording. Users need to use the scroll bar provided in the UI to navigate between frames.
* Added support in ADI ToF SDK to allow multiple camera device connections with different IP addresses. For more details, refer dual_cameras.py example.

### Known Issues

* PCM is disabled by default. To stream PCM build SDK on NXP with ENABLE_PCM flag and restart aditof_server.

* Currently ADI ToF GUI supports streaming only with 16-bit (AB, depth) and 8-bit confidence. Any other combinations might fail.

* Streaming point cloud might reduce the FPS.

### Troubleshooting


* Once the user load the configuration, it will be applied for all the modes that are defined in the input config file. If user needs to switch back to the default configuration, GUI need to be closed and reopened.

* To save the default configuration please use save config option after start stream in any mode as soon as you open GUI. If you load custom configuration before using save configuration option, it will not save the default configuration.


## Version 5.0.0

### Additions, Updates and Fixes

This is a significant update. As a result, please carefully note the changes below.

* ADSD3500 firmware update version 5.2.0 is part of the package.
* The IP address of the device is now 10.43.0.1
* Depth compute is only partially done on the evaluation kit(point cloud generation). As a result, several items have been removed from the host installation: tofi_compute.dll, tofi_config.dll, and tofi_processor.obj, tofi_compute_depth.exe and the tools folder.
* The need to install OpenCL on the host has been removed for all platforms.
* With depth computed on the NXP, depth, AB and confidence frames are passed from the eval kit to the host. The point cloud is generated on the host.
* Metadata is passed to the host in the first 128 bytes of the AB frame.
* ADIToFGUI has undergone significant rework, including a change of the layout, reporting of elements of the metadata (temperature and frame count), showing of frame rate and frame loss, and ability to change the ini/config parameters at run time.
* Data collect, on the host, now only generates depth, confidence, AB, point cloud and metadata frames.
* The depth compute library now allows ini/config parameters to be changed post-initial configuration.
* Saving of frames is now in the SDK. As a result, data collection and ADIToFGUI generate the same file type when saving.
* A new Python tool has been added - rawparser.py. This Python script extracts the sub-frame of the saved raw file.
* Fixed a bug from release 4.3.0 that prevente the Ethernet and Wifi ports from working with the host software.
* Added support for read back of the CCB serial number.
*Bug Fixings to python bindings.
* Recordings done via data collect can now be played back via the viewer or parsed using rawparser.py.

### Known Issues
* While streaming PCM(mode 4), stop streaming need to be clicked second time if the streaming didn t get stop for the first try.
* Randomly 1/8 times, saving CCB via data collect may fail. Re-run the command will fix this.

### Troubleshooting
* If frame capture with data collect or first frame failed on NXP with error VIDIOC_REQBUFS error errno: 16 error: Device or resource busy, run command  sudo systemctl stop network-gadget  and retry.
* If you see Low FPS, try connecting module to the USB hub which is powered via external source and supports USB 3.0.
 

## Version 4.3.0

### Additions, Updates and Fixes

* ADI ToF Viewer UI changed. Now real time FPS will be displayed on the viewer.
* To change FPS edit corresponding FPS parameter in the ini files in .\config folder.
* 4.3.0 eval kit doesn t support old modes.
* First_frame example is updated to accept mode from user via command line option -m.

## Version 4.2.0

### Additions, Updates and Fixes

* Added sw version check tool to check if the current PC is upto date with respect to latest release.
* Added modes 5,6 support for ADTF3175D and ADSD3030.
* Added interrupts to validate if NVM and Firmware updated sucessful.
*Viewer supports recording only in .RAW format.
*A batch file FirmwareUpdate.bat to update firmware is added at .\bin path. Readme.md has instructions to update firmware using FirmwareUpdate.bat
Please note, the Viewer was built from the Master as opposed to the v4.2.0 tag.

### Known Issues

*Save binary records option under record options will not have any effect if it is selected after starting recording. It is recommended to select this option before starting recording to save frame by frame in .raw format.

## Version 4.1.1

### Additions, Updates and Fixes

* Added short range and long range support for new modes in ADIToFViewer.
* Added auto-scale and log-image view options to AB image in ADIToFViewer.
* Default to host boot mode of the ADSD3500
* Address a NVM write via the ADSD3500
* Fix of floating SPI pins on the NXP
* Reset of ADSD3500 on startup of the SDK
* ADSD3500 support merged into Master on GitHub
* Viewer issue at startup if NXP is not ready.
* ADSD3500 firmware update image in same zip file as the SD Card image for the installed eval software
* On the NXP - VDD control of 0.8V and 1.8V
* Unified ADSD3500 firmware 
* Toggle fysnc once for frame request from ADSD3500
* ADSD3500 host boot software included on the NXP file system

### Known issues 
* ADIToFViewer freezes for 20 seconds before starting recording. Wait 20 seconds after recording is started.
* ADIToFViewer viewer crash if we switch from sr-native to sr-qnative and vice versa without stopping streaming. Recommended steps for mode switching is to stop streaming first and switch mode.
