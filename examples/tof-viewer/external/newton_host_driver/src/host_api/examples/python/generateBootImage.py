#!/usr/bin/env python

""" Newton Generate Boot Images

Usage:
    generateBootImage.py <target> <file_name> [--sim][--frontdoor][--seed=<seed_value>][--count=<word_count>][--hsp_fw_0p97]

Options:
    -h --help Shows this help message.
Target is one of the following:
    useq_seq_ram  : Microsequencer Sequence RAM
    useq_map_ram  : Microsequencer MAP RAM
    useq_wave_ram : Microsequencer Wave RAM
    datapath_ram  : Gain Correction RAM
    de_ram        : Dump Engine RAM
    lps1_ram      : LPS1 
    lps2_ram      : LPS2 
    grouped       : Grouped data packet 
"""

from __future__ import print_function
from __future__ import absolute_import
from __future__ import unicode_literals

from docopt import docopt
import sys
import io
import os
import time
import struct
import subprocess
import re
import random
import ctypes
import newton_control as newton

def writeFile( fileName, totalByteCount ):
    ofile = open( fileName, "w" )

    index = 0
    while index < len( commandData ):
        destAddress = commandData[index]
        index += 1
        command = commandData[index]
        index += 1
        attributes = commandData[index]
        index += 1
        byteCount = commandData[index]
        index += 1
        wordCount = int( byteCount / 2 )

        if command == newton.CMD_GROUPED_DATA:
            # Modify the byteCount with totalByteCount
            byteCount = totalByteCount
            wordCount = int( byteCount / 2 )

        ofile.write( '{0:0{1}X}'.format( destAddress, 4 ) + "\n" )
        ofile.write( '{0:0{1}X}'.format( command, 4 ) + "\n" )
        ofile.write( '{0:0{1}X}'.format( attributes, 4 ) + "\n" )
        ofile.write( '{0:0{1}X}'.format( byteCount, 4 ) + "\n" )

        for i in range(0, wordCount):
            cmdWord = commandData[index]
            index += 1
            ofile.write( '{0:0{1}X}'.format( cmdWord, 4 ) + "\n" )

    ofile.close( )

def generateCommandHeader( cmd, attr, destAddr, byteCount ):

    data16 = destAddr # Destination Address
    commandData.append( data16 )
    data16 = cmd  # Mail Box Command
    commandData.append( data16 )
    data16 = attr # Attribute
    commandData.append( data16 )
    data16 = byteCount # Byte Count
    commandData.append( data16 )

def generateRegisterWriteCommand( writeAddr, writeData, attributes ):

    attr = attributes | newton.WRITE_ATTR
    cmd  = newton.CMD_REGISTER_CFG
    byteCount = 4 
    totalByteCount = byteCount + 8
    
    generateCommandHeader( cmd, attr, 0, byteCount )
    
    # Generate register list.
    data16 = writeData
    commandData.append( data16 )
    data16 = writeAddr
    commandData.append( data16 )

    return totalByteCount

def generateRegisterWriteListCommand( writeList, attributes ):

    attr = attributes | newton.WRITE_ATTR
    cmd  = newton.CMD_REGISTER_CFG
    wordCount = len( writeList )
    byteCount = int( wordCount * 2 )
    totalByteCount = byteCount + 8
    
    generateCommandHeader( cmd, attr, 0, byteCount )

    for writeData in writeList:
        # Generate register list.
        commandData.append( writeData )

    return totalByteCount

def generateRamWriteCommand( target, wordCount, attributes ):

    attr = attributes | newton.WRITE_ATTR
    totalByteCount = 0

    if target == "useq_seq_ram":
        cmd       = newton.CMD_SEQ_RAM
        depth     = newton.USEQ_SEQ_RAM_DEPTH
        bitWidth  = newton.USEQ_SEQ_RAM_WIDTH
        byteWidth = newton.USEQ_SEQ_RAM_WIDTH_BYTES
        addr      = newton.USEQ_REGS_USEQRAMLOADDATA

        if wordCount == 0:
            wordCount = random.randrange(32,depth)

        print( "INFO: Adding CMD_SEQ_RAM sub-command with wordCount = " + str( wordCount ) )
        
        r = newton.ADI_USEQ_REGS_MAP1_USEQRAMLOADADDR_s()
        r.LD_RAM_SEL = 0
        r.LD_ADDR = 0
        byteCount = generateRegisterWriteCommand( newton.USEQ_REGS_USEQRAMLOADADDR, r.VALUE16, attr )
        totalByteCount += byteCount
    elif target == "useq_wave_ram":
        cmd       = newton.CMD_WAVE_RAM
        depth     = newton.USEQ_WAVE_RAM_DEPTH
        bitWidth  = newton.USEQ_WAVE_RAM_WIDTH
        byteWidth = newton.USEQ_WAVE_RAM_WIDTH_BYTES
        addr      = newton.USEQ_REGS_USEQRAMLOADDATA

        if wordCount == 0:
            wordCount = random.randrange(32,depth)

        print( "INFO: Adding CMD_WAVE_RAM sub-command with wordCount = " + str( wordCount ) )
        
        r = newton.ADI_USEQ_REGS_MAP1_USEQRAMLOADADDR_s()
        r.LD_RAM_SEL = 1
        r.LD_ADDR = 0
        byteCount = generateRegisterWriteCommand( newton.USEQ_REGS_USEQRAMLOADADDR, r.VALUE16, attr )
        totalByteCount += byteCount
    elif target == "useq_map_ram":
        cmd       = newton.CMD_MAP_RAM
        depth     = newton.USEQ_MAP_RAM_DEPTH
        bitWidth  = newton.USEQ_MAP_RAM_WIDTH
        byteWidth = newton.USEQ_MAP_RAM_WIDTH_BYTES
        addr      = newton.USEQ_REGS_USEQRAMLOADDATA

        if wordCount == 0:
            wordCount = random.randrange(32,depth)

        print( "INFO: Adding CMD_MAP_RAM sub-command with wordCount = " + str( wordCount ) )

        r = newton.ADI_USEQ_REGS_MAP1_USEQRAMLOADADDR_s()
        r.LD_RAM_SEL = 2
        r.LD_ADDR = 0
        byteCount = generateRegisterWriteCommand( newton.USEQ_REGS_USEQRAMLOADADDR, r.VALUE16, attr )
        totalByteCount += byteCount
    elif target == "datapath_ram":
        cmd       = newton.CMD_DATAPATH_RAM
        depth     = newton.DATAPATH_RAM_DEPTH
        bitWidth  = newton.DATAPATH_RAM_WIDTH
        byteWidth = newton.DATAPATH_RAM_WIDTH_BYTES
        addr      = newton.DATAPATH_REGS_IA_WRDATA_REG

        if wordCount == 0:
            wordCount = random.randrange(32,depth)

        print( "INFO: Adding CMD_DATAPATH_RAM sub-command with wordCount = " + str( wordCount ) )

        writeList = []
        r1 = newton.ADI_DATAPATH_IA_SELECT_s()
        r1.IA_ENA = 1
        writeList.append( r1.VALUE16 )
        writeList.append( newton.DATAPATH_REGS_IA_SELECT )
        
        r2 = newton.ADI_DATAPATH_IA_ADDR_REG_s()
        r2.IA_START_ADDR = 0
        writeList.append( r2.VALUE16 )
        writeList.append( newton.DATAPATH_REGS_IA_ADDR_REG )
        
        byteCount = generateRegisterWriteListCommand( writeList, attr )
        totalByteCount += byteCount
    elif target == "de_ram":
        cmd       = newton.CMD_DUMP_ENGINE_RAM
        depth     = newton.DE_RAM_DEPTH
        bitWidth  = newton.DE_RAM_WIDTH
        byteWidth = newton.DE_RAM_WIDTH_BYTES
        addr      = newton.DE_REGS_DE_IA_WRDATA_REG

        if wordCount == 0:
            wordCount = random.randrange(32,depth)

        print( "INFO: Adding CMD_DUMP_ENGINE_RAM sub-command with wordCount = " + str( wordCount ) )

        writeList = []
        r1 = newton.ADI_DE_REGS_YODA_DE_IA_SELECT_s()
        r1.RAM = 1
        writeList.append( r1.VALUE16 )
        writeList.append( newton.DE_REGS_DE_IA_SELECT )
        
        r2 = newton.ADI_DE_REGS_YODA_DE_IA_ADDR_REG_s()
        r2.RAM_ADDR = 0
        writeList.append( r2.VALUE16 )
        writeList.append( newton.DE_REGS_DE_IA_ADDR_REG )
        
        byteCount = generateRegisterWriteListCommand( writeList, attr )
        totalByteCount += byteCount
    elif target == "lps1_ram":
        cmd       = newton.CMD_LPS1_RAM
        depth     = newton.LPS1_RAM_DEPTH
        bitWidth  = newton.LPS1_RAM_WIDTH
        byteWidth = newton.LPS1_RAM_WIDTH_BYTES
        addr      = newton.LPS1_REGS_LPSRAMDATA

        if wordCount == 0:
            wordCount = random.randrange(32,depth)

        print( "INFO: Adding CMD_LPS1_RAM sub-command with wordCount = " + str( wordCount ) )

        writeList = []
        r1 = newton.ADI_LPS_REGS_YODA_LPSRAMRDCMD_s()
        r1.LPS_RAM_READ_EN = 0
        r1.LPS_RAM_READ_RDY = 0
        writeList.append( r1.VALUE16 )
        writeList.append( newton.LPS1_REGS_LPSRAMRDCMD )
        
        r2 = newton.ADI_LPS_REGS_YODA_LPSRAMADDR_s()
        r2.LPS_RAM_ADDR = 0
        writeList.append( r2.VALUE16 )
        writeList.append( newton.LPS1_REGS_LPSRAMADDR )
        
        byteCount = generateRegisterWriteListCommand( writeList, attr )
        totalByteCount += byteCount
    elif target == "lps2_ram":
        cmd       = newton.CMD_LPS2_RAM
        depth     = newton.LPS2_RAM_DEPTH
        bitWidth  = newton.LPS2_RAM_WIDTH
        byteWidth = newton.LPS2_RAM_WIDTH_BYTES
        addr      = newton.LPS2_REGS_LPSRAMDATA

        if wordCount == 0:
            wordCount = random.randrange(32,depth)

        print( "INFO: Adding CMD_LPS2_RAM sub-command with wordCount = " + str( wordCount ) )

        writeList = []
        r1 = newton.ADI_LPS_REGS_YODA_LPSRAMRDCMD_s()
        r1.LPS_RAM_READ_EN = 0
        r1.LPS_RAM_READ_RDY = 0
        writeList.append( r1.VALUE16 )
        writeList.append( newton.LPS2_REGS_LPSRAMRDCMD )
        
        r2 = newton.ADI_LPS_REGS_YODA_LPSRAMADDR_s()
        r2.LPS_RAM_ADDR = 0
        writeList.append( r2.VALUE16 )
        writeList.append( newton.LPS2_REGS_LPSRAMADDR )
        
        byteCount = generateRegisterWriteListCommand( writeList, attr )
        totalByteCount += byteCount

    wordCount = wordCount & 0xfffe
    byteCount = wordCount * byteWidth

    totalByteCount += byteCount + 8

    generateCommandHeader( cmd, attr, addr, byteCount )

    for i in range(0, wordCount):
        ramWord = random.getrandbits( bitWidth )
        if bitWidth <= 16:
            data16 = ramWord
            commandData.append( data16 )
        elif bitWidth <= 32:
            data16 = ramWord & 0xffff
            commandData.append( data16 )
            data16 = (ramWord >> 16) & 0xffff
            commandData.append( data16 )
        elif bitWidth <= 64:
            data16 = ramWord & 0xffff
            commandData.append( data16 )
            data16 = (ramWord >> 16) & 0xffff
            commandData.append( data16 )
            data16 = (ramWord >> 32) & 0xffff
            commandData.append( data16 )
            data16 = (ramWord >> 48) & 0xffff
            commandData.append( data16 )

    return totalByteCount
 
def generateGroupedCommand( target, count ):

    attr = newton.GROUPED_ATTR | newton.WRITE_ATTR
    cmd  = newton.CMD_GROUPED_DATA
    totalByteCount = 0

    generateCommandHeader( cmd, attr, 0, totalByteCount ) # Actual type count filled in later by the writeFile routine.

    print( "INFO: Generating grouped command ..." )

    if count == 0 or count > newton.USEQ_SEQ_RAM_DEPTH:
        wordCount = newton.USEQ_SEQ_RAM_DEPTH
    else:
        wordCount = count
    byteCount = generateRamWriteCommand( "useq_seq_ram", wordCount, attr )
    totalByteCount += byteCount

    if count == 0 or count > newton.USEQ_WAVE_RAM_DEPTH:
        wordCount = newton.USEQ_WAVE_RAM_DEPTH
    else:
        wordCount = count
    byteCount = generateRamWriteCommand( "useq_wave_ram", wordCount, attr )
    totalByteCount += byteCount

    if count == 0 or count > newton.USEQ_MAP_RAM_DEPTH:
        wordCount = newton.USEQ_MAP_RAM_DEPTH
    else:
        wordCount = count
    byteCount = generateRamWriteCommand( "useq_map_ram", wordCount, attr )
    totalByteCount += byteCount

    if count == 0 or count > newton.DATAPATH_RAM_DEPTH:
        wordCount = newton.DATAPATH_RAM_DEPTH
    else:
        wordCount = count
    byteCount = generateRamWriteCommand( "datapath_ram", wordCount, attr )
    totalByteCount += byteCount

    if count == 0 or count > newton.DE_RAM_DEPTH:
        wordCount = newton.DE_RAM_DEPTH
    else:
        wordCount = count
    byteCount = generateRamWriteCommand( "de_ram", wordCount, attr )
    totalByteCount += byteCount

    if count == 0 or count > newton.LPS1_RAM_DEPTH:
        wordCount = newton.LPS1_RAM_DEPTH
    else:
        wordCount = count
    byteCount = generateRamWriteCommand( "lps1_ram", wordCount, attr )
    totalByteCount += byteCount

    if count == 0 or count > newton.LPS2_RAM_DEPTH:
        wordCount = newton.LPS2_RAM_DEPTH
    else:
        wordCount = count
    byteCount = generateRamWriteCommand( "lps2_ram", wordCount, attr )
    totalByteCount += byteCount

    return totalByteCount

def processRegisterFileList( file_name, attributes ):

    cmd  = newton.CMD_REGISTER_CFG
    attr = attributes | newton.WRITE_ATTR
    totalByteCount = 0

    print( "INFO:: Reading file " + file_name + " ..." )
    with open( file_name ) as ifile:  
        line = ifile.readline()
        line = re.sub( r"\n", r"", line )
        while line:
            if re.search( r'^\w+,\w+', line ):
                items = line.split( "," )
            elif re.search( r'^\w+\s+\w+', line ):
                items = line.split( " " )
            address  = items[0].upper()
            data     = items[1].upper()
            address  = re.sub( r"0X", r"", address )
            data     = re.sub( r"0X", r"", data )
            addr_int = int( address, 16 )
            data_int = int( data, 16 )

            if addr_int == newton.DE_REGS_DE_IA_ADDR_REG:
                deRamAddress = data_int
            elif addr_int == newton.DE_REGS_DE_IA_WRDATA_REG:
                deRamAddress += 1
            elif addr_int == newton.USEQ_REGS_USEQRAMLOADADDR:
                seqRamAddress = data_int
            elif addr_int == newton.USEQ_REGS_USEQRAMLOADDATA:
                seqRamAddress += 1
            else:
                registerWrite = {}
                registerWrite["address"] = int( address, 16 )
                registerWrite["data"] = int( data, 16 )

                if hsp_fw_0p97 == True:
                    if registerWrite["address"] == 0x000c:
                        print( "INFO: Skipping useqControlRegister write, data = " + hex(registerWrite["data"]) );
                    elif registerWrite["address"] == 0x0014:
                        print( "INFO: Modifying write to the digPwrDown to make sure the LPS1 and DE blocks are enabled, data = " + hex(registerWrite["data"]) );
                        registerWrite["data"] = registerWrite["data"] & 0xbffe
                        registerWriteList.append( registerWrite )
                    else:
                        registerWriteList.append( registerWrite )
                else:
                    registerWriteList.append( registerWrite )

            line = ifile.readline()
            line = re.sub( r"\n", r"", line )

        ifile.close( )
    
def addRegisterWriteList( attributes ):

    cmd  = newton.CMD_REGISTER_CFG
    attr = attributes | newton.WRITE_ATTR
    totalByteCount = 0

    listSize = len( registerWriteList )
    commandCount = newton.MAX_REG_LIST

    if listSize <= newton.MAX_REG_LIST:
        commandCount = 1
    elif (listSize % newton.MAX_REG_LIST) == 0:
        commandCount = listSize // newton.MAX_REG_LIST
    else:
        commandCount = listSize // newton.MAX_REG_LIST + 1

    if listSize > 0:
        totalByteCount += commandCount * 8
        totalByteCount += listSize * 4

    print( "INFO:: Register list size = " + str( listSize ) )

    index = 0
    for i in range(0, int( commandCount )):
        if i < (commandCount - 1):
            regCount = newton.MAX_REG_LIST
        else:
            regCount = listSize - index

        if listSize > 0:
            generateCommandHeader( cmd, attr, 0, regCount * 4 )

        for j in range(0, regCount):
            registerWrite = registerWriteList[index]
            index += 1
            # Generate register list.
            data16 = registerWrite["data"]
            commandData.append( data16 )
            data16 = registerWrite["address"]
            commandData.append( data16 )
    
    return totalByteCount

def process_wave_reg_txt( file_name, attributes ):

    cmd  = newton.CMD_WAVE_RAM
    attr = attributes | newton.WRITE_ATTR
    totalByteCount = 0
    wave_ram = {}

    for i in range(0, newton.USEQ_WAVE_RAM_DEPTH):
        wave_ram[i] = 0

    r = newton.ADI_USEQ_REGS_MAP1_USEQRAMLOADADDR_s()
    memoryAddress = 0

    print( "INFO:: Reading file " + file_name + " ..." )
    with open( file_name ) as ifile:  
        line = ifile.readline()
        line = re.sub( r"\n", r"", line )
        while line:
            m = re.search( r'(\w+)\s+(\w+)', line )
            address = m.group(1).upper()
            data    = m.group(2).upper()
            address = re.sub( r"0X", r"", address )
            data    = re.sub( r"0X", r"", data )
            address = int( address, 16 )
            data    = int( data, 16 )
            
            if address == newton.USEQ_REGS_USEQRAMLOADADDR:
                r.VALUE16 = data
                memoryAddress = r.LD_ADDR
            elif address == newton.USEQ_REGS_USEQRAMLOADDATA:
                wave_ram[memoryAddress] = data
                memoryAddress += 1

            line = ifile.readline()
            line = re.sub( r"\n", r"", line )

        ifile.close( )

    cmd       = newton.CMD_WAVE_RAM
    depth     = newton.USEQ_WAVE_RAM_DEPTH
    bitWidth  = newton.USEQ_WAVE_RAM_WIDTH
    byteWidth = newton.USEQ_WAVE_RAM_WIDTH_BYTES
    addr      = newton.USEQ_REGS_USEQRAMLOADDATA

    r.LD_RAM_SEL = 1
    r.LD_ADDR = 0
    byteCount = generateRegisterWriteCommand( newton.USEQ_REGS_USEQRAMLOADADDR, r.VALUE16, attr )
    totalByteCount += byteCount

    byteCount = depth * byteWidth

    totalByteCount += byteCount + 8

    generateCommandHeader( cmd, attr, addr, byteCount )

    for i in range(0, newton.USEQ_WAVE_RAM_DEPTH):
        commandData.append( wave_ram[i] )

    return totalByteCount

def extractRamAccesses( file_name, attributes ):
    global de_ram_temp
    global seq_ram_temp
    global wave_ram_temp
    global map_ram_temp
    totalByteCount = 0
    de_ram_temp = {}
    seq_ram_temp = {}
    wave_ram_temp = {}
    map_ram_temp = {}

    r = newton.ADI_USEQ_REGS_MAP1_USEQRAMLOADADDR_s()
    seqRamAddress = 0

    r1 = newton.ADI_DE_REGS_YODA_DE_IA_SELECT_s()
    r1.RAM = 1
    r2 = newton.ADI_DE_REGS_YODA_DE_IA_ADDR_REG_s()
    r2.RAM_ADDR = 0
    deRamAddress = 0
    hwordCount = 0
    temp = 0

    print( "INFO: Reading file " + file_name + " ..." )
    with open( file_name ) as ifile:  
        line = ifile.readline()
        line = re.sub( r"\n", r"", line )
        while line:
            m = re.search( r'(\w+)\s+(\w+)', line )
            address = m.group(1).upper()
            data    = m.group(2).upper()
            address = re.sub( r"0X", r"", address )
            data    = re.sub( r"0X", r"", data )
            address = int( address, 16 )
            data    = int( data, 16 )

            if address == newton.DE_REGS_DE_IA_ADDR_REG:
                r2.VALUE16 = data
                deRamAddress = r2.RAM_ADDR
            elif address == newton.DE_REGS_DE_IA_WRDATA_REG:
                temp += (data << (16*hwordCount))
                de_ram_temp[deRamAddress] = temp
                if hwordCount == 3:
                    hwordCount = 0
                    deRamAddress += 1
                    temp = 0
                else:
                    hwordCount += 1
            elif address == newton.USEQ_REGS_USEQRAMLOADADDR:
                r.VALUE16 = data
                seqRamAddress = r.LD_ADDR
                seqRamSel = r.LD_RAM_SEL
            elif address == newton.USEQ_REGS_USEQRAMLOADDATA:
                if seqRamSel == 0:
                    seq_ram_temp[seqRamAddress] = data
                elif seqRamSel == 1:
                    wave_ram_temp[seqRamAddress] = data
                elif seqRamSel == 2:
                    map_ram_temp[seqRamAddress] = data
                seqRamAddress += 1

            line = ifile.readline()
            line = re.sub( r"\n", r"", line )

        ifile.close( )

    return totalByteCount

def processSeqRamFile( attributes ):

    cmd  = newton.CMD_SEQ_RAM
    attr = attributes | newton.WRITE_ATTR
    totalByteCount = 0
    seq_ram = {}

    for i in range(0, newton.USEQ_SEQ_RAM_DEPTH):
        seq_ram[i] = 0

    memoryAddress = 0

    file_name = "seq_ram.txt"
    print( "INFO: Reading file " + file_name + " ..." )
    with open( file_name ) as ifile:  
        line = ifile.readline()
        line = re.sub( r"\n", r"", line )
        while line:
            data = line.upper()
            data = int( data, 16 ) & newton.USEQ_SEQ_RAM_MASK # Parity is the MSB
            
            seq_ram[memoryAddress] = data
            memoryAddress += 1
            line = ifile.readline()
            line = re.sub( r"\n", r"", line )

    ifile.close( )

    for memoryAddress in seq_ram_temp.keys():
        seq_ram[memoryAddress] = seq_ram_temp[memoryAddress]

    cmd       = newton.CMD_SEQ_RAM
    depth     = newton.USEQ_SEQ_RAM_DEPTH
    bitWidth  = newton.USEQ_SEQ_RAM_WIDTH
    byteWidth = newton.USEQ_SEQ_RAM_WIDTH_BYTES
    addr      = newton.USEQ_REGS_USEQRAMLOADDATA

    r = newton.ADI_USEQ_REGS_MAP1_USEQRAMLOADADDR_s()
    r.LD_RAM_SEL = 0
    r.LD_ADDR = 0
    byteCount = generateRegisterWriteCommand( newton.USEQ_REGS_USEQRAMLOADADDR, r.VALUE16, attr )
    totalByteCount += byteCount

    byteCount = depth * byteWidth

    totalByteCount += byteCount + 8

    generateCommandHeader( cmd, attr, addr, byteCount )

    for i in range(0, newton.USEQ_SEQ_RAM_DEPTH):
        commandData.append( seq_ram[i] )

    return totalByteCount

# Read Wave RM contents from wave_ram.txt and wave_reg.txt files
def processWaveRamFile( attributes ):

    cmd  = newton.CMD_WAVE_RAM
    attr = attributes | newton.WRITE_ATTR
    totalByteCount = 0
    wave_ram = {}

    for i in range(0, newton.USEQ_WAVE_RAM_DEPTH):
        wave_ram[i] = 0

    memoryAddress = 0

    file_name = "wave_ram.txt"
    print( "INFO: Reading file " + file_name + " ..." )
    with open( file_name ) as ifile:  
        line = ifile.readline()
        line = re.sub( r"\n", r"", line )
        while line:
            data = line.upper()
            data = int( data, 16 ) & newton.USEQ_WAVE_RAM_MASK # Parity is the MSB
            wave_ram[memoryAddress] = data
            memoryAddress += 1
            line = ifile.readline()
            line = re.sub( r"\n", r"", line )

    ifile.close( )

    for memoryAddress in wave_ram_temp.keys():
        wave_ram[memoryAddress] = wave_ram_temp[memoryAddress]

    r = newton.ADI_USEQ_REGS_MAP1_USEQRAMLOADADDR_s()

    file_name = "wave_reg.txt"
    print( "INFO: Reading file " + file_name + " ..." )
    with open( file_name ) as ifile:  
        line = ifile.readline()
        line = re.sub( r"\n", r"", line )
        while line:
            m = re.search( r'(\w+)\s+(\w+)', line )
            address = m.group(1).upper()
            data    = m.group(2).upper()
            address = re.sub( r"0X", r"", address )
            data    = re.sub( r"0X", r"", data )
            address = int( address, 16 )
            data    = int( data, 16 )
            
            if address == newton.USEQ_REGS_USEQRAMLOADADDR:
                r.VALUE16 = data
                memoryAddress = r.LD_ADDR
            elif address == newton.USEQ_REGS_USEQRAMLOADDATA:
                wave_ram[memoryAddress] = data
                memoryAddress += 1
            line = ifile.readline()
            line = re.sub( r"\n", r"", line )

    ifile.close( )

    cmd       = newton.CMD_WAVE_RAM
    depth     = newton.USEQ_WAVE_RAM_DEPTH
    bitWidth  = newton.USEQ_WAVE_RAM_WIDTH
    byteWidth = newton.USEQ_WAVE_RAM_WIDTH_BYTES
    addr      = newton.USEQ_REGS_USEQRAMLOADDATA

    r = newton.ADI_USEQ_REGS_MAP1_USEQRAMLOADADDR_s()
    r.LD_RAM_SEL = 1
    r.LD_ADDR = 0
    byteCount = generateRegisterWriteCommand( newton.USEQ_REGS_USEQRAMLOADADDR, r.VALUE16, attr )
    totalByteCount += byteCount

    byteCount = depth * byteWidth

    totalByteCount += byteCount + 8

    generateCommandHeader( cmd, attr, addr, byteCount )

    for i in range(0, newton.USEQ_WAVE_RAM_DEPTH):
        commandData.append( wave_ram[i] )

    return totalByteCount

def processMapRamFile( attributes ):

    cmd  = newton.CMD_MAP_RAM
    attr = attributes | newton.WRITE_ATTR
    totalByteCount = 0
    map_ram = {}

    for i in range(0, newton.USEQ_MAP_RAM_DEPTH):
        map_ram[i] = 0

    memoryAddress = 0

    file_name = "map_ram.txt"
    print( "INFO: Reading file " + file_name + " ..." )
    with open( file_name ) as ifile:  
        line = ifile.readline()
        line = re.sub( r"\n", r"", line )
        while line:
            data = line.upper()
            data = int( data, 16 ) & newton.USEQ_SEQ_RAM_MASK # Parity is the MSB
            
            map_ram[memoryAddress] = data
            memoryAddress += 1
            line = ifile.readline()
            line = re.sub( r"\n", r"", line )

    ifile.close( )

    for memoryAddress in map_ram_temp.keys():
        map_ram[memoryAddress] = map_ram_temp[memoryAddress]

    cmd       = newton.CMD_MAP_RAM
    depth     = newton.USEQ_MAP_RAM_DEPTH
    bitWidth  = newton.USEQ_MAP_RAM_WIDTH
    byteWidth = newton.USEQ_MAP_RAM_WIDTH_BYTES
    addr      = newton.USEQ_REGS_USEQRAMLOADDATA

    r = newton.ADI_USEQ_REGS_MAP1_USEQRAMLOADADDR_s()
    r.LD_RAM_SEL = 2
    r.LD_ADDR = 0
    byteCount = generateRegisterWriteCommand( newton.USEQ_REGS_USEQRAMLOADADDR, r.VALUE16, attr )
    totalByteCount += byteCount

    byteCount = depth * byteWidth

    totalByteCount += byteCount + 8

    generateCommandHeader( cmd, attr, addr, byteCount )

    for i in range(0, newton.USEQ_MAP_RAM_DEPTH):
        commandData.append( map_ram[i] )

    return totalByteCount

def processDatapathMemoryFiles( attributes ):

    cmd  = newton.CMD_DATAPATH_RAM
    attr = attributes | newton.WRITE_ATTR
    totalByteCount = 0
    datapath_ram = {}

    for i in range(0, newton.DATAPATH_RAM_DEPTH):
        datapath_ram[i] = 0

    r1 = newton.ADI_DATAPATH_IA_SELECT_s()
    r2 = newton.ADI_DATAPATH_IA_ADDR_REG_s()
    r2.IA_START_ADDR = 0
    memoryAddress = 0

    for i in range(0, 16):
        file_name = "PCM_Correction_val_" + str( i ) + ".txt"
        print( "INFO: Reading file " + file_name + " ..." )
        with open( file_name ) as ifile:  
            line = ifile.readline()
            line = re.sub( r"\n", r"", line )
            while line:
                data = line.upper()
                data = int( data, 16 ) & newton.DATAPATH_RAM_MASK # Parity is the MSB
            
                datapath_ram[memoryAddress] = data
                memoryAddress += 1
                line = ifile.readline()
                line = re.sub( r"\n", r"", line )

        ifile.close( )

    cmd       = newton.CMD_DATAPATH_RAM
    depth     = newton.DATAPATH_RAM_DEPTH
    bitWidth  = newton.DATAPATH_RAM_WIDTH
    byteWidth = newton.DATAPATH_RAM_WIDTH_BYTES
    addr      = newton.DATAPATH_REGS_IA_WRDATA_REG

    writeList = []
    
    r1.IA_ENA = 1
    writeList.append( r1.VALUE16 )
    writeList.append( newton.DATAPATH_REGS_IA_SELECT )

    r2.IA_START_ADDR = 0
    writeList.append( r2.VALUE16 )
    writeList.append( newton.DATAPATH_REGS_IA_ADDR_REG )
    
    byteCount = generateRegisterWriteListCommand( writeList, attr )
    totalByteCount += byteCount

    byteCount = depth * byteWidth

    totalByteCount += byteCount + 8

    generateCommandHeader( cmd, attr, addr, byteCount )

    for i in range(0, newton.DATAPATH_RAM_DEPTH):
        commandData.append( datapath_ram[i] )

    r1.IA_ENA = 0
    byteCount = generateRegisterWriteCommand( newton.DATAPATH_REGS_IA_SELECT, r1.VALUE16, attr )
    totalByteCount += byteCount
    
    return totalByteCount

def processDumpEngineMemoryFile( attributes ):

    cmd  = newton.CMD_DUMP_ENGINE_RAM
    attr = attributes | newton.WRITE_ATTR
    totalByteCount = 0
    de_ram = {}

    for i in range(0, newton.DE_RAM_DEPTH):
        de_ram[i] = 0

    r1 = newton.ADI_DE_REGS_YODA_DE_IA_SELECT_s()
    r1.RAM = 1
    r2 = newton.ADI_DE_REGS_YODA_DE_IA_ADDR_REG_s()
    r2.RAM_ADDR = 0
    memoryAddress = 0

    file_name = "De_config_all_bkdoor.hex"
    print( "INFO: Reading file " + file_name + " ..." )
    with open( file_name ) as ifile:  
        line = ifile.readline()
        line = re.sub( r"\n", r"", line )
        while line:
            data = line.upper()
            data = int( data, 16 ) & newton.DE_RAM_MASK # Parity is the MSB
            de_ram[memoryAddress] = data
            memoryAddress += 1
            line = ifile.readline()
            line = re.sub( r"\n", r"", line )

    ifile.close( )

    for memoryAddress in de_ram_temp.keys():
        de_ram[memoryAddress] = de_ram_temp[memoryAddress]
        
    cmd       = newton.CMD_DUMP_ENGINE_RAM
    depth     = newton.DE_RAM_DEPTH
    bitWidth  = newton.DE_RAM_WIDTH
    byteWidth = newton.DE_RAM_WIDTH_BYTES
    addr      = newton.DE_REGS_DE_IA_WRDATA_REG

    writeList = []

    r1.RAM = 1
    writeList.append( r1.VALUE16 )
    writeList.append( newton.DE_REGS_DE_IA_SELECT )

    r2.RAM_ADDR = 0
    writeList.append( r2.VALUE16 )
    writeList.append( newton.DE_REGS_DE_IA_ADDR_REG )

    byteCount = generateRegisterWriteListCommand( writeList, attr )
    totalByteCount += byteCount

    byteCount = depth * byteWidth

    totalByteCount += byteCount + 8

    generateCommandHeader( cmd, attr, addr, byteCount )

    for i in range(0, newton.DE_RAM_DEPTH):
        commandData.append(  de_ram[i]        & 0xFFFF )
        commandData.append( (de_ram[i] >> 16) & 0xFFFF )
        commandData.append( (de_ram[i] >> 32) & 0xFFFF )
        commandData.append( (de_ram[i] >> 48) & 0xFFFF )

    return totalByteCount

def processLps1RamFile( attributes ):

    cmd  = newton.CMD_LPS1_RAM
    attr = attributes | newton.WRITE_ATTR
    totalByteCount = 0
    lps1_ram = {}

    for i in range(0, newton.LPS1_RAM_DEPTH):
        lps1_ram[i] = 0

    memoryAddress = 0

    file_name = "lps1_ram.hex"
    print( "INFO: Reading file " + file_name + " ..." )
    with open( file_name ) as ifile:  
        line = ifile.readline()
        line = re.sub( r"\n", r"", line )
        while line:
            data = line.upper()
            data = int( data, 16 ) & newton.LPS1_RAM_MASK # Parity is the MSB
            lps1_ram[memoryAddress] = data
            memoryAddress += 1
            line = ifile.readline()
            line = re.sub( r"\n", r"", line )

    ifile.close( )

    cmd       = newton.CMD_LPS1_RAM
    depth     = newton.LPS1_RAM_DEPTH
    bitWidth  = newton.LPS1_RAM_WIDTH
    byteWidth = newton.LPS1_RAM_WIDTH_BYTES
    addr      = newton.LPS1_REGS_LPSRAMDATA

    writeList = []

    r1 = newton.ADI_LPS_REGS_YODA_LPSRAMRDCMD_s()
    r1.LPS_RAM_READ_EN = 0
    r1.LPS_RAM_READ_RDY = 0
    writeList.append( r1.VALUE16 )
    writeList.append( newton.LPS1_REGS_LPSRAMRDCMD )

    r2 = newton.ADI_LPS_REGS_YODA_LPSRAMADDR_s()
    r2.LPS_RAM_ADDR = 0
    writeList.append( r2.VALUE16 )
    writeList.append( newton.LPS1_REGS_LPSRAMADDR )

    byteCount = generateRegisterWriteListCommand( writeList, attr )
    totalByteCount += byteCount

    byteCount = depth * byteWidth

    totalByteCount += byteCount + 8

    generateCommandHeader( cmd, attr, addr, byteCount )

    for i in range(0, newton.LPS1_RAM_DEPTH):
        commandData.append(  lps1_ram[i]        & 0xFFFF )
        commandData.append( (lps1_ram[i] >> 16) & 0x00FF )

    return totalByteCount

def processLps2RamFile( attributes ):

    cmd  = newton.CMD_LPS2_RAM
    attr = attributes | newton.WRITE_ATTR
    totalByteCount = 0
    lps2_ram = {}

    for i in range(0, newton.LPS2_RAM_DEPTH):
        lps2_ram[i] = 0

    memoryAddress = 0

    file_name = "lps2_ram.hex"
    print( "INFO: Reading file " + file_name + " ..." )
    with open( file_name ) as ifile:  
        line = ifile.readline()
        line = re.sub( r"\n", r"", line )
        while line:
            data = line.upper()
            data = int( data, 16 ) & newton.LPS2_RAM_MASK # Parity is the MSB
            lps2_ram[memoryAddress] = data
            memoryAddress += 1
            line = ifile.readline()
            line = re.sub( r"\n", r"", line )

    ifile.close( )

    cmd       = newton.CMD_LPS2_RAM
    depth     = newton.LPS2_RAM_DEPTH
    bitWidth  = newton.LPS2_RAM_WIDTH
    byteWidth = newton.LPS2_RAM_WIDTH_BYTES
    addr      = newton.LPS2_REGS_LPSRAMDATA

    writeList = []

    r1 = newton.ADI_LPS_REGS_YODA_LPSRAMRDCMD_s()
    r1.LPS_RAM_READ_EN = 0
    r1.LPS_RAM_READ_RDY = 0
    writeList.append( r1.VALUE16 )
    writeList.append( newton.LPS2_REGS_LPSRAMRDCMD )

    r2 = newton.ADI_LPS_REGS_YODA_LPSRAMADDR_s()
    r2.LPS_RAM_ADDR = 0
    writeList.append( r2.VALUE16 )
    writeList.append( newton.LPS2_REGS_LPSRAMADDR )

    byteCount = generateRegisterWriteListCommand( writeList, attr )
    totalByteCount += byteCount

    byteCount = depth * byteWidth

    totalByteCount += byteCount + 8

    generateCommandHeader( cmd, attr, addr, byteCount )

    for i in range(0, newton.LPS2_RAM_DEPTH):
        commandData.append(  lps2_ram[i]        & 0xFFFF )
        commandData.append( (lps2_ram[i] >> 16) & 0x00FF )

    return totalByteCount

def generateGroupedCommandSimulation( frontdoor ):
    global registerWriteList
    
    attr = newton.GROUPED_ATTR | newton.WRITE_ATTR
    totalByteCount = 0
    registerWriteList = []

    if frontdoor == True:
        byteCount = processDatapathMemoryFiles( newton.WRITE_ATTR )
        byteCount = processLps2RamFile( newton.WRITE_ATTR )

        generateCommandHeader( newton.CMD_OPERATING_MODE, newton.MBX_UNSIGNED_SEQ_WFI, 0, 0 )
        generateCommandHeader( newton.CMD_GROUPED_DATA, attr, 0, totalByteCount ) # Actual type count filled in later by the writeFile routine.

        byteCount = extractRamAccesses( "test_csv.txt", attr )
        totalByteCount += byteCount

        processRegisterFileList( "De_config_all_bkdoor.csv", attr )
        processRegisterFileList( "test_csv.txt", attr )
        processRegisterFileList( "config_reg.txt", attr )

        byteCount = addRegisterWriteList( attr )
        totalByteCount += byteCount

        byteCount = processSeqRamFile( attr )
        totalByteCount += byteCount
        byteCount = processMapRamFile( attr )
        totalByteCount += byteCount
        byteCount = processWaveRamFile( attr )
        totalByteCount += byteCount

        byteCount = processDumpEngineMemoryFile( attr )
        totalByteCount += byteCount
        byteCount = processLps1RamFile( attr )
        totalByteCount += byteCount
    else:
        generateCommandHeader( newton.CMD_OPERATING_MODE, newton.WRITE_ATTR, 0, 0 )
        generateCommandHeader( newton.CMD_GROUPED_DATA, attr, 0, totalByteCount ) # Actual type count filled in later by the writeFile routine.

        processRegisterFileList( "De_config_all_bkdoor.csv", attr )
        processRegisterFileList( "test_csv.txt", attr )
        byteCount = process_wave_reg_txt( "wave_reg.txt", attr )
        totalByteCount += byteCount

        byteCount = addRegisterWriteList( attr )
        totalByteCount += byteCount

    return totalByteCount
 
if __name__ == "__main__":
    global commandData
    global simFilesFrontDoor
    global isGroupedCommand
    global hsp_fw_0p97
    maxSpiBytes = 256
    wordCount = 0
    seed = 1
    frontdoor = False
    isGroupedCommand = False
    commandData = []
    hsp_fw_0p97 = False
    simFilesFrontDoor = {}
    simFiles = {}

    args = docopt(__doc__, version='0.1')
    
    if args['--count']:
       wordCount = int( args['--count'] )

    if args['--seed']:
       seed = int( args['--seed'] )

    if args['--frontdoor']:
       frontdoor = True

    if args['--hsp_fw_0p97']:
       hsp_fw_0p97 = True

    random.seed( seed )

    simFilesFrontDoor["De_config_all_bkdoor.csv"] = "De_config_all_bkdoor.csv"
    simFilesFrontDoor["wave_reg.txt"] = "wave_reg.txt"
    simFilesFrontDoor["test_csv.txt"] = "test_csv.txt"

    if args['<target>'] == "grouped":
        isGroupedCommand = True
        
        if args['--sim']:
            totalByteCount = generateGroupedCommandSimulation( frontdoor )
        else:
            totalByteCount = generateGroupedCommand( args['<target>'], wordCount )
    else:
        totalByteCount = generateRamWriteCommand( args['<target>'], wordCount, 0 )

    writeFile( args['<file_name>'], totalByteCount )

    sys.exit( 0 )

