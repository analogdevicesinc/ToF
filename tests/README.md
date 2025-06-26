# Tests

| Category | Name | Description |
|:---------|:-----|:------------|
|sdk | sdk_stream_test | Used for continuous stream testing without exiting the SDK. |

## Log Output Format

**@@,[executable],[PASS/FAIL],LN[Line Number]***,[Optional]*

Bold - Mandatory
Italic - Optional, where any number can be added, these are customer parameters

For example:

@@,sdk_stream_test.exe,PASS,LN399,DN:FPS:31.2389:FC:7:TST:2:200:30:

#### Optional Tags
| Code | Description |
|:-----|:------------|
|DN | Description Note |

Example usage:
sdk_stream_test.exe -f output -ip 10.43.0.1 -i test-example.vector 2> output.txt

$> grep.exe --text "@@" output.txt
I20250626 14:02:37.792209 49612 main.cpp:399] @@,sdk_stream_test.exe,PASS,LN399,DN:FPS:5.01981:FC:26:TST:1:5000:5:
I20250626 14:02:41.432104 49612 main.cpp:399] @@,sdk_stream_test.exe,PASS,LN399,DN:FPS:31.2389:FC:7:TST:2:200:30:
