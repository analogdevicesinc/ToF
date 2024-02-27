# Data collect Example

## Requirements
* Default Configuration and calibration files are in "sdk/config" folder

## Command Line Options

```
Usage:
      data_collect CONFIG
      data_collect [--f <folder>] [--n <ncapture>] [--m <mode>] [--wt <warmup>] [--ccb FILE] [--ip <ip>] [--fw <firmware>] [-s | --split] [-t | --netlinktest] CONFIG
      data_collect (-h | --help)

    Arguments:
      CONFIG            Name of a configuration file (with .json extension)

    Options:
      -h --help          Show this screen.
      --f <folder>       Output folder (max name 512) [default: ./]
      --n <ncapture>     Capture frame num. [default: 1]
      --m <mode>         Mode to capture data in. [default: 0]
      --wt <warmup>      Warmup Time (sec) [default: 0]
      --ccb <FILE>       The path to store CCB content
      --ip <ip>          Camera IP
      --fw <firmware>    Adsd3500 fw file
      --split            Save each frame into a separate file (Debug)
      --netlinktest      Puts server on target in test mode (Debug)
      --singlethread     Store the frame to file using same tread

    Note: --m argument supports both index and string (0/sr-native) 

    Valid mode (--m) options are:
        0: short-range native
        1: long-range native
        2: short-range Qnative
        3: long-range Qnative
        4: pcm-native
        5: long-range mixed
        6: short-range mixed
``` 

## Run
To run the example, run data_collect.exe with config_adsd3500_adsd3100.json file (which contains paths to .ini files used for camera configuration) and other options in command line inputs. Some example commands can be seen below. Microsoft power shell is used as command shell.

```
./data_collect.exe "config_adsd3500_adsd3100.json"
./data_collect.exe --f "../data_output" "config_adsd3500_adsd3100.json"
./data_collect.exe --f "../data_output" --m 10 "config_adsd3500_adsd3100.json"
./data_collect.exe --f "../data_output" --n 50 "config_adsd3500_adsd3100.json"
./data_collect.exe --f "../data_output" --m 10 --n 4 "config_adsd3500_adsd3100.json"
```

## Known Issues
None

## Troubleshooting
* If you see the error "Could not initialize camera!" it could because the USB device could not be opened. Check the USB device connection. If the USB connection is proper then, it is because of the config file path passed is incorrect. Check the config file passed in camera->initialize() function.
* If you see the error "Could not start camera!" it is because of depth compute initialization failed or downloading the firmware to camera is failed or may be due to setting calibration parameters, check the camera calibration file and sensor firmware file passed are correct.
* If you see the error "config Json file is not provided.." pass the --c option with valid json file.
