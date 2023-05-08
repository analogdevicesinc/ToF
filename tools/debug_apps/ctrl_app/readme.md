# ctrl_app

### Overview
ADSD3500 driver accept commands from user-space through V4L2 controls. This application interract directly with "chip_config" control and can send simple or burst commands to ADSD3500.

#### Building the application
g++ -o ctrl_app ctrl_app.cpp

#### Input arguments
ctrl_app take as input only one argument, the name of the input file containing the commands list "infile.txt"

#### Example input commands in infile.txt
First letter on each row of infile.txt define the transfer type.

| Transfer | Description |
| --------- | ----------- |
| R | Read transfer. Will retrieve the response from ADSD3500 for command number sent after R |
| W | Write transfer. Will send write data command to ADSD3500 |
| D | Delay between commands in ms units |

| Example | Description |
| --------- | ----------- |
| R 01 12 | Read CHIP ID (simple command) |
| R 00 15 | Get AB invalidation threshold (simple command) |
| D 10 | Wait 16 ms before sending the next command |
| W 00 10 00 10 | Set AB invalidation threshold to 16 (simple command) |
| W 00 19 00 00 | Switch to burst mode. All commands following this will have to be sent in burst mode format (simple command) |
| R AD 00 38 01 00 00 00 00 39 00 00 00 0A 00 00 00 | Get camera Intrinsic. Retun 56 bytes of intrinsic data for operating mode 10 (burst command) |
| W AD 00 00 10 00 00 00 00 10 00 00 00 00 00 00 00 | Switch to simple mode. All commands following this will have to be sent in simple mode format (burst command) |



