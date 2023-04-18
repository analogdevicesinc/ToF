import sys
from enum import Enum
from smbus2 import SMBus, i2c_msg
import binascii

# Select the I2C address of the ADSD3500.
address = 0x38

# Select the I2C buss number of the ADSD3500.
bus_no = 1

if len(sys.argv) > 1:
    if len(sys.argv) == 2:
        bus_no = int(sys.argv[1], 0)
    if len(sys.argv) == 3:
        address = int(sys.argv[2], 0)

# Open a handle to "/dev/i2c-2", representing the I2C bus.
bus = SMBus(bus_no)

# Write CHIP ID read command
msg = i2c_msg.write(address, [0x01, 0x12])
bus.i2c_rdwr(msg)

#Get back the response
msg = i2c_msg.read(address, 2)
bus.i2c_rdwr(msg)
data = list(msg)

print([hex(x) for x in data])

if (data[0] == 0x59) and (data[1] == 0x31):
    print("CHIP_ID is correct. Identified ADSD3500 on bus %d at address 0x%x" % (bus_no, address))
else:
    print("Wrong CHIP_ID")
    
bus.close()
