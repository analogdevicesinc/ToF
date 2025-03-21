# ctrl_app

### Overview
ADSD3500 driver accept commands from user-space through V4L2 controls. This application interract directly with "chip_config" control and can send simple or burst commands to ADSD3500.

#### Building the application
g++ -o ctrl_app ctrl_app.cpp

#### Input arguments
ctrl_app take only one argument as input, the name of the input file containing the commands list "infile.txt"

#### Example command to run ctrl_app
./ctrl_app infile.txt
(Assuming both ctrl_app and infile.txt are in current working directory.)

#### Example input commands in infile.txt
First letter on each row of infile.txt define the transfer type.

| Transfer | Description |
| --------- | ----------- |
| R | Read transfer. Will retrieve the response from ADSD3500 for command number sent after R |
| W | Write transfer. Will send write data command to ADSD3500 |


| Example | Description |
| --------- | ----------- |
| R 01 12 | Read CHIP ID (simple command) |
| R 00 20 | Get system status (simple command) |
| W 00 19 00 00 | Switch to burst mode. All commands following this will have to be sent in burst mode format (simple command) |
| R AD 00 2C 05 00 00 00 00 31 00 00 00 01 00 00 00 | Get Firmware version and commit ID returns 44 bytes, first 4 bytes Fw version (burst command) |
| W AD 00 00 10 00 00 00 00 10 00 00 00 00 00 00 00 | Switch to simple mode. All commands following this will have to be sent in simple mode format (burst command) |
| R 01 12 | Read CHIP ID (simple command) |

