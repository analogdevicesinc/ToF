# Data collect Example

## Requirements
* Default Configuration and calibration files are in "sdk/config" folder

## Command Line Options

```
usage: data_collect.py [-h] [-f <folder>] [-n <ncapture>] [-m <mode>]
                       [-wt <warmup>] [-ccb <FILE>] [-ip <ip>]
                       [-fw <firmware>] [-s] [-t] [-st]
					   [-ic <imager-configuration>]
                       [-scf <save-configuration-file>]
                       config

Script to run data collect python script

positional arguments:
  config               path to the configuration file (with .json extension)

optional arguments:
  -h, --help           show this help message and exit
  -f <folder>          output folder [default: ./]
  -n <ncapture>        number of frame captured[default: 1]
  -m <mode>            Valid mode (-m) options are:
                               0: short-range native;
                               1: long-range native;
                               2: short-range Qnative;
                               3: long-range Qnative
                               4: pcm-native;
                               5: long-range mixed;
                               6: short-range mixed

                               Note: --m argument supports both index and string (Default: 0/sr-native)
  -wt <warmup>         warmup time in seconds[default: 0]
  -ccb <FILE>          The path to store CCB content
  -ip <ip>             camera IP[default: 10.43.0.1]
  -fw <firmware>       Adsd3500 firmware file
  -s, --split          Save each frame into a separate file (Debug)
  -t, --netlinktest    Puts server on target in test mode (Debug)
  -st, --singlethread  Store the frame to file using same thread
  -ic <imager-configuration>
                       Select imager configuration. By default is standard.
  -scf <save-configuration-file>
                       Save current configuration to json file
 
``` 

## Run
To run this python example, run data_collect.py with provided configuration file (which contains paths to camera calibration and camera configuration) and other options in command line inputs. Some example commands can be seen below.

```
python data_collect.py config/config_adsd3500_adsd3100.json
python data_collect.py -f ./data_output config/config_adsd3500_adsd3100.json
python data_collect.py -f ./data_output -n 10 config/config_adsd3500_adsd3100.json
python data_collect.py -f ./data_output -n 50 config/config_adsd3500_adsd3100.json
python data_collect.py -f ./data_output -m sr-native -n 50 - config/config_adsd3500_adsd3100.json
python data_collect.py -f ./data_output -m 0 -n 4 -wt 5 config/config_adsd3500_adsd3100.json
```

## Known Issues
None