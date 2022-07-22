#!/usr/bin/env python

""" Newton Check Boot Image

Usage:
    checkBootImage.py <file_name>

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

def checkGroupedCommand( file_name ):
    commandData = []
    groupByteCount = 0
    isGroupedCommand = 0
    state = newton.CMD_LOAD_HEADER

    print( "INFO: Reading file " + file_name + " ..." )
    with open( file_name ) as ifile:  
        line = ifile.readline()
        line = re.sub( r"\n", r"", line )
        while line:
            commandData.append( line )
            line = ifile.readline()
            line = re.sub( r"\n", r"", line )

    ifile.close( )
    print( "INFO: Completed reading file " + file_name + " ..." )

    index = 0
    while index < len( commandData ):
        if state == newton.CMD_LOAD_HEADER:
            destAddress  = int( commandData[index], 16 )
            index += 1
            command      = int( commandData[index], 16 )
            index += 1
            attributes   = int( commandData[index], 16 )
            index += 1
            byteCount    = int( commandData[index], 16 )
            index += 1
            numDataWords = int( byteCount / 2 )

            print( "destAddress = " + '0x{0:04x}'.format( destAddress ) )
            print( "command = " + '0x{0:04x}'.format( command  ) )
            print( "byteCount = " + str( byteCount ) )
            print( "numDataWords = " + str( numDataWords ) )
            print( "attributes = " + '0x{0:04x}'.format( attributes ) )
            print( "index = " + str( index ) )

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
                state = newton.CMD_LOAD_HEADER
        elif state == newton.CMD_GROUPED_DATA_WRITE:
            groupCmdByteCount = byteCount
            isGroupedCommand = 1
            state = newton.CMD_LOAD_HEADER
        elif state == newton.CMD_LOAD_REG_WRITE:
            if isGroupedCommand == 1:
                groupByteCount += 8
                groupByteCount += byteCount
            i = 0
            while i < numDataWords:
                data = int( commandData[index], 16 )
                index += 1
                i += 1
                regAddress = int( commandData[index], 16 )
                index += 1
                i += 1
                print( "regAddress = " + '0x{0:04x}'.format( regAddress ) + ", data = " + '0x{0:04x}'.format( data ) )
            # index += numDataWords
            state = newton.CMD_LOAD_HEADER
        elif state == newton.CMD_LOAD_REG_READ:
            if isGroupedCommand == 1:
                groupByteCount += 8
                groupByteCount += byteCount
            index += numDataWords
            state = newton.CMD_LOAD_HEADER
        elif state == newton.CMD_LOAD_RAM_WRITE:
            if isGroupedCommand == 1:
                groupByteCount += 8
                groupByteCount += byteCount
            index += numDataWords
            state = newton.CMD_LOAD_HEADER
        elif state == newton.CMD_LOAD_ERROR_LOG:
            print( "ERROR: CMD_LOAD_ERROR_LOG was not expected." )
            state = newton.CMD_LOAD_HEADER
        elif state == newton.CMD_LOAD_1SP_IMAGE:
            if isGroupedCommand == 1:
                groupByteCount += 8
                groupByteCount += byteCount
            index += numDataWords
            state = newton.CMD_LOAD_HEADER
        elif state == newton.CMD_LOAD_SIGNATURE:
            if isGroupedCommand == 1:
                groupByteCount += 8
                groupByteCount += byteCount
                print( "ERROR: CMD_LOAD_OPERATING_MODE is not expected in grouped command." )
            index += numDataWords
            state = newton.CMD_LOAD_HEADER
        elif state == newton.CMD_LOAD_PUBLIC_KEY:
            if isGroupedCommand == 1:
                groupByteCount += 8
                groupByteCount += byteCount
            index += numDataWords
            state = newton.CMD_LOAD_HEADER
        elif state == newton.CMD_LOAD_OPERATING_MODE:
            if isGroupedCommand == 1:
                print( "ERROR: CMD_LOAD_OPERATING_MODE is not expected in grouped command." )
            state = newton.CMD_LOAD_HEADER

    if groupByteCount != groupCmdByteCount:
        print( "ERROR: Grouped command byte count mismatch, expect = " + str( groupCmdByteCount ) + ", actual = " + str( groupByteCount ) + ", difference = " + str( groupCmdByteCount-groupByteCount) + "." )

if __name__ == "__main__":

    args = docopt(__doc__, version='0.1')
    
    checkGroupedCommand( args['<file_name>'] )

    sys.exit( 0 )

