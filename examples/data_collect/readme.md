# Data collect Example

## Requirements
* Default Configuration and calibration files are in "sdk/config" folder

## Command Line Options

```
Usage:
      data_collect FILE
      data_collect [--f <folder>] [--n <ncapture>] [--m <mode>] [--ext_fsync <0|1>] [--fsf <0|1>] [--wt <warmup>] [--ip <ip>] [--ccb FILE] FILE
      data_collect (-h | --help)

    Arguments:
      FILE            Input config_default.json

    Options:
      -h --help          Show this screen.
      --f <folder>       Output folder (Max length: 512) [default: ./]
      --n <ncapture>     Number of frames to capture. [default: 1]
      --m <mode>         Mode to capture data in. [default: 10]
      --ext_fsync <0|1>  External FSYNC [0: Internal 1: External] [default: 0]
      --fsf <0|1>        FSF file type [0: Disable 1: Enable] [default: 0]
      --wt <warmup>      Warmup Time (in seconds) [default: 0]
      --ip <ip>          Camera IP
      --ccb <FILE>       The path to store CCB content

    Valid mode (--m) options are:
        3: Passive IR
        5: 1Mpixel with PCM
        7: QMpixel
       10: 1Mpixel
 
``` 

## Run
To run the example, run data_collect.exe with config_default.json file (which contains paths to camera calibration and camera configuration) and other options in command line inputs. Some example commands can be seen below. Microsoft power shell is used as command shell.

```
./data_collect.exe "config.json"
./data_collect.exe --f "../data_output" "config.json"
./data_collect.exe --f "../data_output" --m 10 "config.json"
./data_collect.exe --f "../data_output" --n 50 "config.json"
./data_collect.exe --f "../data_output" --m 10 --n 4 "config.json" --fsf 1 --wt 5
```

## Known Issues
None

## Troubleshooting
* If you see the error "Could not initialize camera!" it could because the USB device could not be opened. Check the USB device connection. If the USB connection is proper then, it is because of the config file path passed is incorrect. Check the config file passed in camera->initialize() function.
* If you see the error "Could not start camera!" it is because of depth compute initialization failed or downloading the firmware to camera is failed or may be due to setting calibration parameters, check the camera calibration file and sensor firmware file passed are correct.
* If you see the error "config Json file is not provided.." pass the --c option with valid json file.
