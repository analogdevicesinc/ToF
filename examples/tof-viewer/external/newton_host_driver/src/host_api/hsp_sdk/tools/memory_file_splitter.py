import sys
import os
import struct
import argparse
from collections import OrderedDict

# ...\dataFiles\dataFiles\useq_wave_ram.txt USEQ_WAVE_RAM 2048
# C:\HSP_fw\hspfwapi\tools\src\1sp_6_28.txt IMAGE_1SP 2048

gDataTypeDict = {
                'USEQ_SEQ_RAM' : '1002',
                'USEQ_WAVE_RAM': '1003',
                'USEQ_MAP_RAM' : '1004',
                'DATAPATH_RAM' : '1005',
                'DUMP_ENGINE_RAM' : '1006',
                'LPS1_RAM' : '1007',
                'LPS2_RAM' : '1008',
                'REG_CFG'   : '1009',
                'IMAGE_1SP' : '100A',
                'AUTH_CERT' : '100B'
                }

def get_cmd_1sp_header(len):
    hdr = '\n'.join([
        '0000', # Address
        '100A', # Command: 1SP
        '0001', # Attribute: Write
        f'{len:04X}', # Payload Size
    ])
    return hdr

def getMetaData(header):
    cmd = header[0]
    opCode = header[1]
    attr = header[2]
    size = header[3]
    return cmd, opCode, attr, size

def updateHeaderSize(memoryHeader, splitVal):
    if len(memoryHeader) == 4:
        splitVal = hex(splitVal)
        splitVal = splitVal[2:]
        if len(splitVal) < 4:
            splitVal = '0'*(4 - len(splitVal)) + splitVal
        elif len(splitVal) > 4:
            print("New header size does not make sense")
            exit(1)
        memoryHeader[3] = splitVal
    else:
        print(f"Memory header is not of correct size")
    return

def getHeaderSize(memoryHeader):
    if len(memoryHeader) == 4:
        retVal = int('0x' + memoryHeader[3], 16)
        return retVal
    else:
        print(f"Memory header is not of correct size")
    return

def splitter(filePath, code, splitVal):
    if code not in gDataTypeDict:
        print(f"code {code}, is not supported")
        exit(1)

    code = gDataTypeDict[code]

    with open(filePath, 'r') as input_file:
        input_data = input_file.read()
    input_file.close()
    # input_data = bytes(input_data)
    # print(f"input data is {input_data}")
    input_data = input_data.split('\n')
    header = input_data[:4]
    print(f"header is {header}")
    preface = []
    memoryHeader = []
    payload = []

    i = 0
    while (i < len(input_data)):
        header = input_data[i:i+4]
        addr, cmd, attr, size = getMetaData(header)

        # TODO: how to check that the cmd captured is the correct header/memory code and not data!
        if cmd == code:
            preface = input_data[:i]
            memoryHeader = header
            payload = input_data[i+4:]
            break
        else:
            size = int(size) // 2 # since its 2 bytes per item (per line)
            i += size + 4



    print(f"preface len is {len(preface)}")
    print(f"memoryHeader is {memoryHeader}")
    print(f"payload len is {len(payload)}")

    payloadSize = getHeaderSize(memoryHeader)
    print(f"payload size is {payloadSize}")
    updateHeaderSize(memoryHeader, splitVal)

    output_data = preface

    #TODO: check if memory size and splitVal make sense
    # len(payload) should be multiple of splitVal requested
    incrementVal = splitVal // 2
    remainingPayload = payloadSize
    # for i in range(0, payloadSize, incrementVal):
    i = 0
    while (remainingPayload > 0):
        if remainingPayload < splitVal:
            print(f"found a lesser than incrementVal payload!, remainingPayload is {remainingPayload}, splitVal is {splitVal}")
            updateHeaderSize(memoryHeader, remainingPayload)
            incrementVal = remainingPayload // 2
        new_data = memoryHeader + payload[i:i+incrementVal]
        i += incrementVal
        output_data = output_data + new_data
        remainingPayload -= splitVal

    len_output = len(output_data)
    print(f"len of output data is {len(output_data)}")

    str_output_data = '\n'.join(x for x in output_data)
    print(f"len of str_output_data is {len(str_output_data)}")

    splitVal = hex(splitVal)
    splitVal = splitVal[2:]
    print(f"filepath is {filePath}")
    if '.txt' in filePath:
        index = filePath.find('.txt')
        filePath = filePath[:index]
    print(f"filepath is {filePath}")
    filePath = filePath + '_' + splitVal + '.txt'
    with open(filePath, 'w') as output_file:
        output_file.write(str_output_data)
    output_file.close()



def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("src_bin", type=str, help="file path")
    parser.add_argument("cmd_code", type=str, help="mailbox command")
    parser.add_argument("split", type=int, help="how to split")

    args = parser.parse_args()
    splitter(args.src_bin, args.cmd_code, args.split)
    # input_data = open(args.src_bin, 'rb').read()
    # with open(args.cmd_out_txt, 'w') as out_file:
        # out_file.write(get_cmd_1sp_header(input_size))
        # write_bin_in_newton_format(out_file, input_data)


if __name__ == "__main__":
    main()
