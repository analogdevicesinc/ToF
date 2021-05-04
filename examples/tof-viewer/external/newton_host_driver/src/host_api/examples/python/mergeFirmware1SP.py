#!/usr/bin/env python

""" Merge Newton Firmware image with 1SP image to create a new command file

Usage:
    mergeFirmware1SP.py <newton_file_name> <1sp_file_name> <new_file_name>

Options:
    -h --help Shows this help message.
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

def writeFile( file_name ):
    ofile = open( file_name, "w" )

    index = 0
    for word in commandData:
        ofile.write( word + "\n" )

    ofile.close( )

def readNewtonFirmwareCommandFile( file_name ):
    global newtonCommandData
    newtonCommandData = []

    print( "INFO: Reading file " + file_name + " ..." )
    with open( file_name ) as ifile:  
        line = ifile.readline()
        line = re.sub( r"\n", r"", line )
        while line:
            newtonCommandData.append( line )
            line = ifile.readline()
            line = re.sub( r"\n", r"", line )

    ifile.close( )
    print( "INFO: Completed reading file " + file_name + " ..." )

def generateNewCommandFile( ):
    global commandData
    commandData = []
    state = newton.CMD_LOAD_HEADER

    oneSpByteCount = len( oneSpCommandData ) * 2

    index = 0
    while index < len( newtonCommandData ):
        if state == newton.CMD_LOAD_HEADER:
            destAddress  = int( newtonCommandData[index], 16 )
            index += 1
            command      = int( newtonCommandData[index], 16 )
            index += 1
            attributes   = int( newtonCommandData[index], 16 )
            index += 1
            byteCount    = int( newtonCommandData[index], 16 )
            index += 1
            numDataWords = int( byteCount / 2 )

            if command == newton.CMD_GROUPED_DATA:
                print( "INFO: Start of Grouped Command ("+ str( byteCount ) +" bytes) ..." )
                state = newton.CMD_GROUPED_DATA_WRITE
            elif command == newton.CMD_SEQ_RAM:
                print( "INFO: Loading useq_seq_ram ("+ str( byteCount ) +" bytes) ..." )
                state = newton.CMD_LOAD_RAM_WRITE
            elif command == newton.CMD_WAVE_RAM:
                print( "INFO: Loading useq_wave_ram ("+ str( byteCount ) +" bytes) ..." )
                state = newton.CMD_LOAD_RAM_WRITE
            elif command == newton.CMD_MAP_RAM:
                print( "INFO: Loading useq_map_ram ("+ str( byteCount ) +" bytes) ..." )
                state = newton.CMD_LOAD_RAM_WRITE
            elif command == newton.CMD_DATAPATH_RAM:
                print( "INFO: Loading datapath_ram ("+ str( byteCount ) +" bytes) ..." )
                state = newton.CMD_LOAD_RAM_WRITE
            elif command == newton.CMD_DUMP_ENGINE_RAM:
                print( "INFO: Loading de_ram ("+ str( byteCount ) +" bytes) ..." )
                state = newton.CMD_LOAD_RAM_WRITE
            elif command == newton.CMD_LPS1_RAM:
                print( "INFO: Loading lps1_ram ("+ str( byteCount ) +" bytes) ..." )
                state = newton.CMD_LOAD_RAM_WRITE
            elif command == newton.CMD_LPS2_RAM:
                print( "INFO: Loading lps2_ram ("+ str( byteCount ) +" bytes) ..." )
                state = newton.CMD_LOAD_RAM_WRITE
            elif command == newton.CMD_REGISTER_CFG:
                if (attributes & 1) == 0:
                    print( "INFO: Register read ("+ str( byteCount ) +" bytes) ..." )
                    state = newton.CMD_LOAD_REG_READ
                else:
                    print( "INFO: Register write ("+ str( byteCount ) +" bytes) ..." )
                    state = newton.CMD_LOAD_REG_WRITE
            elif command == newton.CMD_1SP_IMAGE:
                print( "INFO: Loading 1SP image ("+ str( byteCount ) +" bytes) ..." )
                state = newton.CMD_LOAD_1SP_IMAGE
            elif command == newton.CMD_ERROR_LOG:
                print( "INFO: Loading public Key ("+ str( byteCount ) +" bytes) ..." )
                state = newton.CMD_LOAD_PUBLIC_KEY
            elif command == newton.CMD_PUBLIC_KEY:
                print( "INFO: Loading public Key ("+ str( byteCount ) +" bytes) ..." )
                state = newton.CMD_LOAD_PUBLIC_KEY
            elif command == newton.CMD_SIGNATURE:
                print( "INFO: Loading signature ("+ str( byteCount ) +" bytes) ..." )
                state = newton.CMD_LOAD_SIGNATURE
            elif command == newton.CMD_OPERATING_MODE:
                print( "INFO: Setting operating mode ("+ str( byteCount ) +" bytes) ..." )

                attributes = newton.MBX_UNSIGNED_SEQ_1SP
                commandData.append( '{0:0{1}X}'.format( destAddress, 4 ) )
                commandData.append( '{0:0{1}X}'.format( command, 4 ) )
                commandData.append( '{0:0{1}X}'.format( attributes, 4 ) )
                commandData.append( '{0:0{1}X}'.format( byteCount, 4 ) )
                state = newton.CMD_LOAD_HEADER
        elif state == newton.CMD_GROUPED_DATA_WRITE:
            byteCount += oneSpByteCount
            commandData.append( '{0:0{1}X}'.format( destAddress, 4 ) )
            commandData.append( '{0:0{1}X}'.format( command, 4 ) )
            commandData.append( '{0:0{1}X}'.format( attributes, 4 ) )
            commandData.append( '{0:0{1}X}'.format( byteCount, 4 ) )

            state = newton.CMD_LOAD_HEADER
        elif state == newton.CMD_LOAD_REG_WRITE:
            commandData.append( '{0:0{1}X}'.format( destAddress, 4 ) )
            commandData.append( '{0:0{1}X}'.format( command, 4 ) )
            commandData.append( '{0:0{1}X}'.format( attributes, 4 ) )
            commandData.append( '{0:0{1}X}'.format( byteCount, 4 ) )
            for i in range(0, numDataWords):
                commandData.append( newtonCommandData[index] )
                index += 1

            state = newton.CMD_LOAD_HEADER
        elif state == newton.CMD_LOAD_REG_READ:
            commandData.append( '{0:0{1}X}'.format( destAddress, 4 ) )
            commandData.append( '{0:0{1}X}'.format( command, 4 ) )
            commandData.append( '{0:0{1}X}'.format( attributes, 4 ) )
            commandData.append( '{0:0{1}X}'.format( byteCount, 4 ) )
            for i in range(0, numDataWords):
                commandData.append( newtonCommandData[index] )
                index += 1

            state = newton.CMD_LOAD_HEADER
        elif state == newton.CMD_LOAD_RAM_WRITE:
            commandData.append( '{0:0{1}X}'.format( destAddress, 4 ) )
            commandData.append( '{0:0{1}X}'.format( command, 4 ) )
            commandData.append( '{0:0{1}X}'.format( attributes, 4 ) )
            commandData.append( '{0:0{1}X}'.format( byteCount, 4 ) )
            for i in range(0, numDataWords):
                commandData.append( newtonCommandData[index] )
                index += 1

            state = newton.CMD_LOAD_HEADER
        elif state == newton.CMD_LOAD_ERROR_LOG:
            print( "ERROR: CMD_LOAD_ERROR_LOG was not expected." )
            state = newton.CMD_LOAD_HEADER
        elif state == newton.CMD_LOAD_1SP_IMAGE:
            commandData.append( '{0:0{1}X}'.format( destAddress, 4 ) )
            commandData.append( '{0:0{1}X}'.format( command, 4 ) )
            commandData.append( '{0:0{1}X}'.format( attributes, 4 ) )
            commandData.append( '{0:0{1}X}'.format( byteCount, 4 ) )
            for i in range(0, numDataWords):
                commandData.append( newtonCommandData[index] )
                index += 1

            state = newton.CMD_LOAD_HEADER
        elif state == newton.CMD_LOAD_SIGNATURE:
            commandData.append( '{0:0{1}X}'.format( destAddress, 4 ) )
            commandData.append( '{0:0{1}X}'.format( command, 4 ) )
            commandData.append( '{0:0{1}X}'.format( attributes, 4 ) )
            commandData.append( '{0:0{1}X}'.format( byteCount, 4 ) )
            for i in range(0, numDataWords):
                commandData.append( newtonCommandData[index] )
                index += 1

            state = newton.CMD_LOAD_HEADER
        elif state == newton.CMD_LOAD_PUBLIC_KEY:
            commandData.append( '{0:0{1}X}'.format( destAddress, 4 ) )
            commandData.append( '{0:0{1}X}'.format( command, 4 ) )
            commandData.append( '{0:0{1}X}'.format( attributes, 4 ) )
            commandData.append( '{0:0{1}X}'.format( byteCount, 4 ) )
            for i in range(0, numDataWords):
                commandData.append( newtonCommandData[index] )
                index += 1

            state = newton.CMD_LOAD_HEADER
        elif state == newton.CMD_LOAD_OPERATING_MODE:
            if isGroupedCommand == 1:
                print( "ERROR: CMD_LOAD_OPERATING_MODE is not expected in grouped command." )
            state = newton.CMD_LOAD_HEADER

    index = 0
    state = newton.CMD_LOAD_HEADER
    while index < len( oneSpCommandData ):
        if state == newton.CMD_LOAD_HEADER:
            destAddress  = int( oneSpCommandData[index], 16 )
            index += 1
            command      = int( oneSpCommandData[index], 16 )
            index += 1
            attributes   = int( oneSpCommandData[index], 16 )
            index += 1
            byteCount    = int( oneSpCommandData[index], 16 )
            index += 1
            numDataWords = int( byteCount / 2 )

            if command == newton.CMD_1SP_IMAGE:
                print( "INFO: Loading 1SP image ("+ str( byteCount ) +" bytes) ..." )
                state = newton.CMD_LOAD_1SP_IMAGE
        elif state == newton.CMD_LOAD_1SP_IMAGE:
            attributes |= newton.GROUPED_ATTR

            commandData.append( '{0:0{1}X}'.format( destAddress, 4 ) )
            commandData.append( '{0:0{1}X}'.format( command, 4 ) )
            commandData.append( '{0:0{1}X}'.format( attributes, 4 ) )
            commandData.append( '{0:0{1}X}'.format( byteCount, 4 ) )
            for i in range(0, numDataWords):
                commandData.append( oneSpCommandData[index] )
                index += 1

            state = newton.CMD_LOAD_HEADER

def read1SpFirmwareCommandFile( file_name ):
    global oneSpCommandData
    oneSpCommandData = []

    print( "INFO: Reading file " + file_name + " ..." )
    with open( file_name ) as ifile:  
        line = ifile.readline()
        line = re.sub( r"\n", r"", line )
        while line:
            oneSpCommandData.append( line )
            line = ifile.readline()
            line = re.sub( r"\n", r"", line )

    ifile.close( )
    print( "INFO: Completed reading file " + file_name + " ..." )

if __name__ == "__main__":

    args = docopt(__doc__, version='0.1')
    
    readNewtonFirmwareCommandFile( args['<newton_file_name>'] )
    read1SpFirmwareCommandFile( args['<1sp_file_name>'] )
    generateNewCommandFile( )
    writeFile( args['<new_file_name>'] )

    sys.exit( 0 )

