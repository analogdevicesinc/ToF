#!/usr/bin/env python

import sys
import os
import struct

import xml.etree.ElementTree as ET

import crypto_functions as crypto

# ========================================
# Attributes
WRITE_ATTR = 0x0001
SIGNED_ATTR = 0x0002
ENCRYPTED_ATTR = 0x0004
GROUPED_ATTR = 0x0008
MSFT_MODE = 0x20
ADI_MODE = 0x10
UNSIGNED_MODE = 0x0

# Data Types
USEQ_SEQ_RAM = 0x1002
USEQ_WAVE_RAM = 0x1003
USEQ_MAP_RAM = 0x1004
DATAPATH_RAM = 0x1005
DUMP_ENGINE_RAM = 0x1006
LPS1_RAM = 0x1007
LPS2_RAM = 0x1008
REG_CFG = 0x1009
IMAGE_1SP = 0x100A
AUTH_CERT = 0x100B
GROUP_PAYLOAD = 0x1001

SIG_APPEND = 0x100D
SIG_SIZE = 64

gDataTypeDict = {
                'USEQ_SEQ_RAM' : 0x1002,
                'USEQ_WAVE_RAM': 0x1003, 
                'USEQ_MAP_RAM' : 0x1004, 
                'DATAPATH_RAM' : 0x1005, 
                'DUMP_ENGINE_RAM' : 0x1006, 
                'LPS1_RAM' : 0x1007, 
                'LPS2_RAM' : 0x1008,
                'REG_CFG'   : 0x1009,
                'IMAGE_1SP' : 0x100A,
                # 'AUTH_CERT' : 0x100B
                }

signMode = [0]
signedPack = [0]
encrytPack = [0]

# ========================================

def ValidateAndGetAbsPath(path, currentWorkDir):
    if '$(CONFIG)' in path:
        path = path.replace('$(CONFIG)', os.getenv('CONFIG'))
    if os.path.exists(currentWorkDir + '/' + path):
        return currentWorkDir + '/' + path
    p = os.path.abspath(path)
    if os.path.exists(p):
        return p
    sys.exit("File {} does not exist".format(path))

#get data from text file
def IterateAndSeparateFileData(pathToInputFiles, headerDataList, attributeDict, groupedParam, keyPathDict):
    workDir = os.path.dirname(pathToInputFiles)

    sizeOfData = 0
    encryptedDataValueList = []
    signedDataValueList = []
    xmldoc = ET.parse(pathToInputFiles).getroot()

    groupedParam = xmldoc.find('GroupedParam').text
    fileList = xmldoc.findall('Files/file')
    xmlDataAttributesList = xmldoc.findall('DataAttributes/data')
    xmlPrivateKey = xmldoc.findall('Keys/privatekey')
    xmlEncryptionKey = xmldoc.findall('Keys/encryptionkey')
    xmlInitialVector = xmldoc.findall('Keys/initialvector')

    xmlMode = xmldoc.findall('Modes/signMode')

    # Get the key path from xml file; the last path is the one that will be used
    for modeEntry in xmlMode:
        signMode[0] = modeEntry.get('name')
        print(f"signmode is {signMode[0]}")


    for privateKey in xmlPrivateKey:
        keyPathDict["privateKey path"] = ValidateAndGetAbsPath(privateKey.get('name'), workDir)

    for encryptionKey in xmlEncryptionKey:
        keyPathDict["encryption path"] = ValidateAndGetAbsPath(encryptionKey.get('name'), workDir)

    for initialvector in xmlInitialVector:
        keyPathDict["iv path"] = ValidateAndGetAbsPath(initialvector.get('name'), workDir)
    
    if groupedParam not in ['g', "ng", "ag"]:
        sys.exit("{}:\n  line: {}:\n    contains invalid parameter \'{}\'\nExpected [g, ng, ag] as valid parameter.".format(pathToInputFiles, 1, groupedParam))
    else:
        if groupedParam == 'ag':
            dataAttributesList = [x.get("name") for x in xmlDataAttributesList]
            for x in xmlDataAttributesList:
                if x.get("name") not in gDataTypeDict.keys():
                    sys.exit("XML contains invalid value {}".format(x))
                if x.text not in ['e', 's', '-']:
                    sys.exit("{}:\n  tag: {} contains invalid attribute \'{}\'\n  Expected [e,s,-] as valid attributes.".format(pathToInputFiles, x.attrib, x.text))
                if x.text == 'e':
                    encryptedDataValueList.append(gDataTypeDict[x.get("name")])
                    encrytPack[0] = True
                elif x.text == 's':
                    signedDataValueList.append(gDataTypeDict[x.get("name")])
                    signedPack[0] = True

        for x in fileList:
            if x.text not in ['e', 's', '-']:
                sys.exit("{}:\n  tag: {} contains invalid attribute \'{}\'\n  Expected [e,s,-] as valid attributes.".format(pathToInputFiles, x.attrib, x.text))

        for fileNode in fileList:
            inFileName = fileNode.get('name')
            inFileName = ValidateAndGetAbsPath(inFileName, workDir)

            inFileSize = os.path.getsize(inFileName)
            with open(inFileName) as inFile:
                while inFile.tell() < inFileSize:
                    l = SeparateHeaderAndData(inFile, groupedParam, inFileSize)
                    headerDataList.extend(l)

                    if groupedParam in ['g', 'ng']:
                        for idx, headerDataPair in enumerate(l):
                            if headerDataPair[0]['cmd'] != REG_CFG:
                                if fileNode.text == 'e':
                                    attributeDict.setdefault('e', []).append(idx)
                                    attributeDict.setdefault('s', []).append(idx)
                                elif fileNode.text == 's':
                                    attributeDict.setdefault('s', []).append(idx)
                            else:
                                if fileNode.text == 's':
                                    attributeDict.setdefault('s', []).append(idx)
                        
        
        for idx, headerDataPair in enumerate(headerDataList):
            if groupedParam == "ng" and headerDataPair[0]['cmd'] != REG_CFG:
                sizeOfData += headerDataPair[0]['size']
            elif groupedParam == "ag":
                sizeOfData += (len(headerDataPair[0])*2 + headerDataPair[0]['size'])
                if headerDataPair[0]['cmd'] in encryptedDataValueList:
                    attributeDict.setdefault('e', []).append(idx)
                    attributeDict.setdefault('s', []).append(idx)
                elif headerDataPair[0]['cmd'] in signedDataValueList:
                    attributeDict.setdefault('s', []).append(idx)
            else:
                sizeOfData += (len(headerDataPair[0])*2 + headerDataPair[0]['size'])
    return sizeOfData, groupedParam

#separate header from data
def SeparateHeaderAndData(inFile, groupedParam, fileSize):
    headerDataTmpList = []

    if groupedParam == 'ag':
        for i in range(4):
            inFile.readline()

    while inFile.tell() < fileSize:
        data = bytearray()
        # read cmd header
        addr = int(inFile.readline(), 16)
        cmd = int(inFile.readline(), 16)
        attr= int(inFile.readline(), 16)
        size = int(inFile.readline(), 16)

        print(f"size of payload is {size}, command is {hex(cmd)}, attr: {attr}")

        header = dict({'addr': addr, 'cmd': cmd, 'attr': attr, 'size': size})

        # get data
        for i in range(size//2):
            dataStr = inFile.readline().strip()
            dataHalfWord = int(dataStr, 16)

            data.extend(struct.pack('<H', dataHalfWord))

        headerDataTmpList.append([header, data])

    return headerDataTmpList

def WriteModifiedHeaderToFile(header, outFile, groupedParam, attributeDict):
    if header is None:
        return

    modeAttr = 0

    if signMode[0] == "MSFT":
        modeAttr = MSFT_MODE
    elif signMode[0] == "ADI":
        modeAttr = ADI_MODE

    signAttr = 0
    if len(attributeDict['s']) > 0:
        signAttr = SIGNED_ATTR

    encrAttr = 0
    if len(attributeDict['e']) > 0 and (header['cmd'] == IMAGE_1SP or header['cmd'] == GROUP_PAYLOAD):
        encrAttr = ENCRYPTED_ATTR
    # if len(attributeDict['e']) > 0:
    #     encrAttr = ENCRYPTED_ATTR

    data16 = header['addr']
    outFile.write('{0:0{1}X}'.format(data16, 4) + "\n")
    data16 = header['cmd']
    outFile.write('{0:0{1}X}'.format(data16, 4) + "\n")
    # modify header attribute for grouped header
    data16 = header['attr']
    if groupedParam != "ng":
        data16 = header['attr'] | GROUPED_ATTR | WRITE_ATTR | modeAttr | signAttr | encrAttr

    outFile.write('{0:0{1}X}'.format(data16, 4) + "\n")
    data16 = header['size']
    outFile.write('{0:0{1}X}'.format(data16, 4) + "\n")

def WriteDataToFile(data, outFile):
    for i in range(0, len(data), 2):
        outStr = '{0:0{1}X}'.format(data[i+1], 2)
        outStr += '{0:0{1}X}'.format(data[i], 2)
        outStr += '\n'
        outFile.write(outStr)

def ConvertHeaderDataListToGroupedText(groupedHeader, headerDataList, outputFileName, groupedParam, attributeDict):
    with open(outputFileName, 'w') as outFile:
        WriteModifiedHeaderToFile(groupedHeader, outFile, groupedParam, attributeDict)
        for header, data in headerDataList:
            WriteModifiedHeaderToFile(header, outFile, groupedParam, attributeDict)
            WriteDataToFile(data, outFile)
    outFile.close()

def ConvertHeaderToByteArray(header):
    byteArray = bytearray()
    byteArray.extend(struct.pack('<H', header['addr']))
    byteArray.extend(struct.pack('<H', header['cmd']))
    byteArray.extend(struct.pack('<H', header['attr']))
    byteArray.extend(struct.pack('<H', header['size']))
    return byteArray

def ExtendByteArray(attributeDict, headerDataList, attr):
    byteArray = bytearray()
    # for header, data in headerDataList:
    hasSignedPackets = False
    hasEncryptedPackets = False
    if len(attributeDict['s']) > 0:
        hasSignedPackets = True
    if len(attributeDict['e']) > 0:
        hasEncryptedPackets = True

    for i in range(len(headerDataList)):
        attr_updated  = attr
        header = headerDataList[i][0]
        data =  headerDataList[i][1]
        if hasSignedPackets:
            attr_updated |= SIGNED_ATTR
            if hasEncryptedPackets and (header['cmd'] == IMAGE_1SP or header['cmd'] == GROUP_PAYLOAD):
                attr_updated |= ENCRYPTED_ATTR

        ModifyHeader(header, attr_updated)
        byteArray.extend(ConvertHeaderToByteArray(header))
        byteArray.extend(data)
    return byteArray

def ModifyHeader(header, attr):
    # Access the command header and modify attribute
    header['attr'] |= attr

def ModifyDataWithEncryption(headerDataList, encryptedIdx, keyDict):
    #TODO: returning back for now since we are not encrypting 
    return
    header, data = headerDataList[encryptedIdx]

    encryptionFilepath = ''
    ivFilePath = ''

    if "encryption path" in keyDict:
        encryptionFilepath = keyDict["encryption path"]

    if "iv path" in keyDict:
        ivFilePath = keyDict["iv path"]

    # for encryption, need to pad to 16 byte
    # if len(data) % 16:
    #     padding = b'0' * (len(data) % 16)
    #     data = data + bytearray(padding)
    #     header['size'] = len(data)

    print(f"Header command is {header['cmd']}")
    print(f"size of the data is {len(data)}, type of data is {type(data)}")

    ciphertext = None
    if header['cmd'] == USEQ_SEQ_RAM:
        crypto.encryptData(data, 0, encryptionFilepath, ivFilePath)
        ModifyHeader(header, ENCRYPTED_ATTR | SIGNED_ATTR)

    elif header['cmd'] == USEQ_WAVE_RAM:
        crypto.encryptData(data, 0, encryptionFilepath, ivFilePath)
        ModifyHeader(header, ENCRYPTED_ATTR | SIGNED_ATTR)

    elif header['cmd'] == USEQ_MAP_RAM:
        crypto.encryptData(data, 11, encryptionFilepath, ivFilePath)
        ModifyHeader(header, ENCRYPTED_ATTR | SIGNED_ATTR)

    elif header['cmd'] == DATAPATH_RAM:
        crypto.encryptData(data, 0, encryptionFilepath, ivFilePath)
        ModifyHeader(header, ENCRYPTED_ATTR | SIGNED_ATTR)

    elif header['cmd'] == DUMP_ENGINE_RAM:
        crypto.encryptData(data, 0, encryptionFilepath, ivFilePath)
        ModifyHeader(header, ENCRYPTED_ATTR | SIGNED_ATTR)

    elif header['cmd'] == LPS1_RAM:
        crypto.encryptData(data, 24, encryptionFilepath, ivFilePath)
        ModifyHeader(header, ENCRYPTED_ATTR | SIGNED_ATTR)

    elif header['cmd'] == LPS2_RAM:
        crypto.encryptData(data, 24, encryptionFilepath, ivFilePath)
        ModifyHeader(header, ENCRYPTED_ATTR | SIGNED_ATTR)

    elif header['cmd'] == IMAGE_1SP:
        print("Not going to encrypt the 1sp")
        # 1sp is already encrypted
        return
        # ciphertext = crypto.encryptData(data, 0, encryptionFilepath, ivFilePath)
        # ModifyHeader(header, ENCRYPTED_ATTR | SIGNED_ATTR)

    elif header['cmd'] == AUTH_CERT:
        ciphertext = crypto.encryptData(data, 0, encryptionFilepath, ivFilePath)
        if signMode[0] == "MSFT":
            print("MSFT mode selected")
            ModifyHeader(header, ENCRYPTED_ATTR | SIGNED_ATTR | MSFT_MODE)
        elif signMode[0] == "ADI":
            print("ADI mode selected")
            ModifyHeader(header, ENCRYPTED_ATTR | SIGNED_ATTR | ADI_MODE)
        else:
            print("Unsigned Mode for Authorization Certificate")

    else:
        pass
    # headerDataList[encryptedIdx][1] = ciphertext
    return

def SignDataIndividualPacket(headerDataList, signedIdx, keyDict, outputFileName):
    header, data = headerDataList[signedIdx]
    print(f"data size is {len(data)}")
    print(f"data size in header is {header['size']}")

    # filepath = ''
    if "privateKey path" in keyDict:
        filepath = keyDict["privateKey path"]

    modeAttr = 0

    if signMode[0] == "MSFT":
        modeAttr = MSFT_MODE
    elif signMode[0] == "ADI":
        modeAttr = ADI_MODE

    if header['cmd'] == USEQ_SEQ_RAM:
        signature = crypto.signData(data, 0, filepath, outputFileName)
        ModifyHeader(header, SIGNED_ATTR | modeAttr)

    elif header['cmd'] == USEQ_WAVE_RAM:
        signature = crypto.signData(data, 0, filepath, outputFileName)
        ModifyHeader(header, SIGNED_ATTR | modeAttr)

    elif header['cmd'] == USEQ_MAP_RAM:
        signature = crypto.signData(data, 11, filepath, outputFileName)
        ModifyHeader(header, SIGNED_ATTR | modeAttr)

    elif header['cmd'] == DATAPATH_RAM:
        signature = crypto.signData(data, 0, filepath, outputFileName)
        ModifyHeader(header, SIGNED_ATTR | modeAttr)

    elif header['cmd'] == DUMP_ENGINE_RAM:
        signature = crypto.signData(data, 0, filepath, outputFileName)
        ModifyHeader(header, SIGNED_ATTR | modeAttr)

    elif header['cmd'] == LPS1_RAM:
        signature = crypto.signData(data, 24, filepath, outputFileName)
        ModifyHeader(header, SIGNED_ATTR | modeAttr)

    elif header['cmd'] == LPS2_RAM:
        signature = crypto.signData(data, 24, filepath, outputFileName)
        ModifyHeader(header, SIGNED_ATTR | modeAttr)

    elif header['cmd'] == REG_CFG:
        signature = crypto.signData(data, 0, filepath, outputFileName)
        print(f"reg_cfg, signature is {signature}")
        ModifyHeader(header, SIGNED_ATTR | modeAttr)

    elif header['cmd'] == IMAGE_1SP:
        signature = crypto.signData(data, 0, filepath, outputFileName)
        ModifyHeader(header, SIGNED_ATTR | modeAttr)

    elif header['cmd'] == AUTH_CERT:
        print(f"Authorization cert, signMode is {signMode[0]}")
        signature = crypto.signData(data, 0, filepath, outputFileName)
        ModifyHeader(header, SIGNED_ATTR | modeAttr)
        # if signMode[0] == "MSFT":
        #     print("MSFT mode selected")
        #     ModifyHeader(header, SIGNED_ATTR | MSFT_MODE)
        # elif signMode[0] == "ADI":
        #     print("ADI mode selected")
        #     ModifyHeader(header, SIGNED_ATTR | ADI_MODE)
        # else:
        #     print("Unsigned Mode for Authorization Certificate")

    else:
        signature = None
        pass
    return signature

def RunCryptoOperations(attributeDict, headerDataList, groupedParam, keyDict, outputFileName):
    signature = bytearray()
    # Run encryptions

    print(f"encrypt indices are {attributeDict['e']}")
    for encryptedIdx in attributeDict['e']:
        # print("Calling encryption")
        ModifyDataWithEncryption(headerDataList, encryptedIdx, keyDict)

    modeAttr = 0

    if signMode[0] == "MSFT":
        modeAttr = MSFT_MODE
    elif signMode[0] == "ADI":
        modeAttr = ADI_MODE

    # Run signature
    if len(attributeDict['s']) > 0:
        if groupedParam == 'ng':
            print(f"signing non group packet")
            signature = SignDataIndividualPacket(headerDataList, attributeDict['s'][0], keyDict, outputFileName)
        else:
            filepath = ''
            if "privateKey path" in keyDict:
                filepath = keyDict["privateKey path"]
            # combine everything into binary array
            data = ExtendByteArray(attributeDict, headerDataList, modeAttr | GROUPED_ATTR | SIGNED_ATTR)
            #TODO:file path here
            signature = crypto.signData(data, 0, filepath, outputFileName)
    return signature

def AppendSignatureToFile(signature, outputFileName):

    print(f"AppendSignatureToFile: signature is {signature.hex()}")
    if len(signature) != 0:
        with open(outputFileName, 'a') as outFile:
            # addr
            data16 = 0
            outFile.write('{0:0{1}X}'.format(data16, 4) + "\n")
            # command
            data16 = SIG_APPEND
            outFile.write('{0:0{1}X}'.format(data16, 4) + "\n")
            # modify header attribute for grouped header
            data16 = 0
            outFile.write('{0:0{1}X}'.format(data16, 4) + "\n")
            # size
            data16 = SIG_SIZE
            outFile.write('{0:0{1}X}'.format(data16, 4) + "\n")
            #writing signature
            WriteDataToFile(signature, outFile)
        outFile.close()

# ============================

def main():
    if len(sys.argv) != 3:
        sys.exit("Invalid parameters.\nExpected: generate_crypto_packets.py <path/to/input/xml> <output/txt/file/name>")

    pathToInputFiles = sys.argv[1].strip()
    outputFileName = sys.argv[2].strip()

    if not os.path.exists(os.path.dirname(outputFileName)):
        print("Invalid input path")
        os.exit(1)

    # list of headers and data
    headerDataList = []
    # dictionary of attributes for each headerData pair
    attributeDict = {'e': [], 's': []}
    # dictionary of keys
    keyPathDict = {"private path": [], "encryption path": []}
    # grouped parameter
    groupedParam = None

    # =============================
    # Iterate through files and separate header and data
    keyDict = {}
    groupedDataSize, groupedParam = IterateAndSeparateFileData(pathToInputFiles, headerDataList, attributeDict, groupedParam, keyDict)
    # =============================


    groupedHeader = {'cmd': 0x1001, 'size': groupedDataSize, 'addr': 0x0000, 'attr': GROUPED_ATTR}
   
    # =============================
    # Crypto Operations
    fileSplit = outputFileName.split('.')
    fileSplit = fileSplit[0].split('\\')
    fileName = fileSplit[-1]
    signature = RunCryptoOperations(attributeDict, headerDataList, groupedParam, keyDict,fileName)

    #==============================

    # =============================
    # convert back to text file
    if groupedParam == "ng":
        ConvertHeaderDataListToGroupedText(None, headerDataList, outputFileName, groupedParam, attributeDict)
        print(f"Before AppendSignatureToFile: signature is {signature.hex()}")
        AppendSignatureToFile(signature, outputFileName)
    else:
        ConvertHeaderDataListToGroupedText(groupedHeader, headerDataList, outputFileName, groupedParam, attributeDict)
        print(f"Before AppendSignatureToFile: signature is {signature.hex()}")
        AppendSignatureToFile(signature, outputFileName)
    # =============================


if __name__ == "__main__":
    main()