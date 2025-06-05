import os
import time
import sys
from binascii import hexlify
from enum import Enum
from smbus2 import SMBus, i2c_msg


# Open a handle to "/dev/i2c-1", representing the I2C bus.
bus = SMBus(1)
# Select the I2C address of the ADSD3500.
address = 0x38
#Binary file default name
bin_file_name = "./host_boot.stream"

ACK_ERR = 0
ACK_RECEIVED = 0
nAckCount = 0
nHdrCount = 0

""" Response codes """
class I2C_Slave_ACK(Enum):
    ADI_ROM_RESPONSE_INVALID_PACKET_CODE  = 0x01
    ADI_ROM_RESPONSE_INVALID_COMMAND_CODE = 0x02
    ADI_ROM_RESPONSE_INVALID_ADDRESS_CODE   = 0x03
    ADI_ROM_RESPONSE_INVALID_HDR_CHECKSUM_CODE = 0x04
    ADI_ROM_RESPONSE_INVALID_PLD_CHECKSUM_CODE = 0x05
    ADI_ROM_RESPONSE_INVALID_CRC_CODE       = 0x06
    ADI_ROM_RESPONSE_INVALID_SIZE_CODE      = 0x07
    ADI_ROM_RESPONSE_SPI_HW_ERROR_CODE      = 0x08
    ADI_ROM_RESPONSE_MISALIGNED_ADDR_ERROR_CODE  = 0x09
    ADI_ROM_RESPONSE_CRC_DMA_ERROR_CODE     = 0x0A
    ADI_ROM_RESPONSE_ACK_CODE               = 0x0B


def Read_ACK():
    global ACK_ERR
    global ACK_RECEIVED
    global nAckCount
    """
    Wait for Acknowledgement
    """
    data_In = bytearray([0x11,0x12])

    while True:
        msg = i2c_msg.read(address, 4)
        bus.i2c_rdwr(msg)
        data_In = list(msg)
        for Acknowledgement in I2C_Slave_ACK:
            if data_In[0] == int(Acknowledgement.value):
                if Acknowledgement.value == 0x0B:
                    ACK_ERR = 0
                    ACK_RECEIVED = 1
                    print("-------ACK Received--------")
                    break
                else:
                    ACK_ERR = 1
                    ACK_RECEIVED = 1
                    print("********ACK ERROR Received********", end = " ")
                    print(hexlify(bytes(data_In)))
                    break
            """
            TODO: Rarely, the ACK gets shifted by one nibble and
                  this is also considered as a valid ACK(To be debugged)
            """
            if data_In[2] == 0xB0:
                ACK_ERR = 0
                ACK_RECEIVED = 1
                print("--^^^^---ACK Received-----^^^^--")
                break
        if ACK_RECEIVED == 1:
            ACK_RECEIVED = 0
            break
    nAckCount += 1
    return ACK_ERR, data_In[0]


def I2C_sendFirmware():
    header_size_inbytes = 16
    global bin_file_name
    global address

    if len(sys.argv) > 1:
        if len(sys.argv) == 2:
                bin_file_name = sys.argv[1]
        if len(sys.argv) == 3:
                bin_file_name = sys.argv[1]
                address = int(sys.argv[2], 0)

    """
    Get the file size in bytes and read size number of bytes into
     an empty list, one byte at a time, as a hexadecimal byte
    """
    size = os.stat(bin_file_name).st_size
    data = []
    with open(bin_file_name, "rb") as binary_file:
        # Read the whole file at once
        for i in range(0, size):
            data.append(binary_file.read(1).hex())
    for i in range(0, len(data)):
        data[i] = int(data[i],16)
    print("size =",size)

    """
    Store the sizes, start and end locations of each
    data segment from the Header
    """
    segment_size = []
    segment_start = []
    segment_end = []

    i = 0
    nSegments = 0
    prev_seg_end = 0
    ACK_ERR = 0
    ACK_RECEIVED = 0
    nReset_CMD = 0
    
    while True:
        # Data size is available in 2nd and 3rd bytes of Header.
        segment_size.append((data[prev_seg_end+2]<<8)|data[prev_seg_end+1])
        # Data segment starts from the end of header packet,
        # which starts at the end of previous segment.
        segment_start.append(prev_seg_end + header_size_inbytes)
        # Data segment ends after the segment size from the start of the segment.
        segment_end.append(segment_start[i] + segment_size[i])
        prev_seg_end = segment_end[i]

        if nReset_CMD == 1:break
        # Parse the data till the segment size reaches 0, which is the last packet.
        if segment_size[i] == 0:
            nReset_CMD = 1
        i += 1

    prev_seg_end = 0
    global First_Time
    global nFirst_Header
    global modify_payload
    nCount =0
    print("No of headers =", len(segment_size))

    """
    segment_size is a list, with sizes of all the data packets.
    By looping through the no of such data packets, each data
    packet is sent over I2C with corresponding header, which is sent before it.
    For each header and data packet, the slave acknowledges and this ACK is
    verified .
    """
    while(nSegments < len(segment_size)):

        """
        Slice the header from the stream data that is captured in data list
        """
        global nHdrCount
        i = nSegments
        header_data = data[prev_seg_end:segment_start[i]]

        data_out_list_append = []
        for j in range(0, len(header_data)):
            data_out_list_append.extend(int(header_data[j]).to_bytes(1,byteorder="little"))

        data_out = bytes(data_out_list_append)
        print('Header written')
        print(data_out)
        data_in = []
        nHdrCount += 1
        
        msg = i2c_msg.write(address, data_out)
        bus.i2c_rdwr(msg)

        if data_out_list_append[3] == 0x55:
            global FW_TX_SUCCESS
            FW_TX_SUCCESS = 1
            print("Data Tx completed")
            return 1

        ACK_ERRROR, Response = Read_ACK()

        """
        If the NACK received is INVALID CRC,
        re-transmit the firmware
        """
        if Response == 0x06:
            return 0
        if data_out_list_append[3] == 0x45:
            prev_seg_end = segment_end[i]
            nSegments += 1;
            time.sleep(0.05)
            continue

        if ACK_ERRROR == 1:
            ACK_ERRROR = 0
            continue


        time.sleep(0.005)
        """
        Slice the data segment from the stream data that is captured in data list
        """
        segment_data = data[segment_start[i]:segment_end[i]]

        data_out_list_append = []
        length = len(segment_data)

        for j in range(0, length):
            data_out_list_append.extend(int(segment_data[j]).to_bytes(1,byteorder="little"))

        data_out = bytes(data_out_list_append)
        print('Payload written')
        data_in = []
        
        msg = i2c_msg.write(address, data_out)
        bus.i2c_rdwr(msg)

        ACK_ERRROR, Response = Read_ACK()

        if ACK_ERRROR == 1:
            ACK_ERRROR = 0
        else:
            prev_seg_end = segment_end[i]
            nSegments += 1;

I2C_sendFirmware()
