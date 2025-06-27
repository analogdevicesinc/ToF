# SDK Stream Test

## Description

This test is designed to test the SDK stream without exiting between sessions. Where the user can change on a per test basis:

* Mode
* Capture time (in ms)
* Frame rate per second
* Optional: JSON Configuration file

## Command Line Options

```
SDK Stream Test
    Usage:
      sdk_stream_test --i <test file> --f <output older path> [--ip <ip>]
      sdk_stream_test (-h | --help)

    Options:
      -h --help          Show this screen.
      --f <folder path>  Output folder path
      --ip <ip>          Camera IP
      --i <test file>    Test definition file
``` 

## Test Definition File

The test defintion file is a CSV with a header. For example:

```
mode,rtms,fps,cfg
1,5000,5,
2,200,30,
```
In this case two test specified:

| Test | Description |
|:-----|:------------|
| 1,5000,5, | Mode 1, 5000 ms collection, 5 fps, No JSON configuration file specified |
| 2,200,30, | Mode 2, 200 ms collection, 30 fps, No JSON configuration file specified |

## Example Session

```
$> sdk_stream_test.exe -f output -ip 10.43.0.1 -i test-example.vector 2> output.txt
$> dir output
06/26/2025  02:02 PM        62,915,328 frame2025_06_26_14_02_32_0.bin
06/26/2025  02:02 PM        25,691,008 frame2025_06_26_14_02_41_0.bin
$> grep.exe --text "@@" output.txt
I20250626 14:02:37.792209 49612 main.cpp:399] @@,sdk_stream_test.exe,PASS,LN399,DN:FPS:5.01981:FC:26:TST:1:5000:5:
I20250626 14:02:41.432104 49612 main.cpp:399] @@,sdk_stream_test.exe,PASS,LN399,DN:FPS:31.2389:FC:7:TST:2:200:30:
```

