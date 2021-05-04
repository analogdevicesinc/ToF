#!/usr/bin/env python

""" Analyze Boot Image

Usage:
    analyzeBootImage.py <input_file_name> <output_file_name>

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
from enum import Enum

class States( Enum ) :
    CMD_LOAD_HEADER = 0
    CMD_GROUPED_DATA_WRITE = 1
    CMD_LOAD_REG_WRITE = 2
    CMD_LOAD_RAM_WRITE = 3
    CMD_1SP_IMAGE_WRITE = 4
    CMD_AUTHORIZATION_CERT_WRITE = 5
    CMD_PUBLIC_KEY_WRITE = 6

def readInputFile( file_name ):
    global commandData
    commandData = []

    print( "INFO:: Reading file " + file_name + " ..." )
    with open( file_name ) as ifile:  
        line = ifile.readline()
        line = re.sub( r"\n", r"", line )
        while line:
            m = re.search( r'0x(\w+)\s+0x(\w+)', line )
            word1 = m.group(1)
            word0 = m.group(2)
            commandData.append( word0 )
            commandData.append( word1 )
            line = ifile.readline()
            line = re.sub( r"\n", r"", line )

    ifile.close( )

def writeOutputFile( file_name ):
    ofile = open( file_name, "w" )

    for cmdWord in commandData:
        ofile.write( cmdWord + "\n" )

    ofile.close( )

def analyzeCommands( file_name ):
    commandData = []
    state = States.CMD_LOAD_HEADER
    index = 0

    print( "INFO:: Reading file " + file_name + " ..." )
    with open( file_name ) as ifile:  
        line = ifile.readline()
        line = re.sub( r"\n", r"", line )
        while line:
            m = re.search( r'(\w+)', line )
            word = m.group(1)
            commandData.append( word )
            line = ifile.readline()
            line = re.sub( r"\n", r"", line )

    ifile.close( )

    while index < len( commandData ):
        if state == States.CMD_LOAD_HEADER:
            destAddress = int( commandData[index], 16 )
            index += 1
            command     = int( commandData[index], 16 )
            index += 1
            attributes  = int( commandData[index], 16 )
            index += 1
            byteCount   = int( commandData[index], 16 )
            index += 1
            numDataWords = int( byteCount / 2 )

            if command == newton.CMD_GROUPED_DATA:
                isGroupedCommand = 1
                print( "Sending CMD_GROUPED_DATA (" + hex( command ) + ") ..." )
                print( "byteCount = " + str( byteCount ) )
                state = States.CMD_GROUPED_DATA_WRITE
            elif command == newton.CMD_SEQ_RAM:
                print( "Sending CMD_SEQ_RAM (" + hex( command ) + ") ..." )
                print( "byteCount = " + str( byteCount ) )
                state = States.CMD_LOAD_RAM_WRITE
            elif command == newton.CMD_WAVE_RAM:
                print( "Sending CMD_WAVE_RAM (" + hex( command ) + ") ..." )
                print( "byteCount = " + str( byteCount ) )
                state = States.CMD_LOAD_RAM_WRITE
            elif command == newton.CMD_MAP_RAM:
                print( "Sending CMD_MAP_RAM (" + hex( command ) + ") ..." )
                print( "byteCount = " + str( byteCount ) )
                state = States.CMD_LOAD_RAM_WRITE
            elif command == newton.CMD_DATAPATH_RAM:
                print( "Sending CMD_DATAPATH_RAM (" + hex( command ) + ") ..." )
                print( "byteCount = " + str( byteCount ) )
                state = States.CMD_LOAD_RAM_WRITE
            elif command == newton.CMD_DUMP_ENGINE_RAM:
                print( "Sending CMD_DUMP_ENGINE_RAM (" + hex( command ) + ") ..." )
                print( "byteCount = " + str( byteCount ) )
                state = States.CMD_LOAD_RAM_WRITE
            elif command == newton.CMD_LPS1_RAM:
                print( "Sending CMD_LPS1_RAM (" + hex( command ) + ") ..." )
                print( "byteCount = " + str( byteCount ) )
                state = States.CMD_LOAD_RAM_WRITE
            elif command == newton.CMD_LPS2_RAM:
                print( "Sending CMD_LPS2_RAM (" + hex( command ) + ") ..." )
                print( "byteCount = " + str( byteCount ) )
                state = States.CMD_LOAD_RAM_WRITE
            elif command == newton.CMD_REGISTER_CFG:
                print( "Sending CMD_LOAD_REG (" + hex( command ) + ") ..." )
                print( "byteCount = " + str( byteCount ) )
                state = States.CMD_LOAD_REG_WRITE
            elif command == newton.CMD_1SP_IMAGE:
                print( "Sending CMD_1SP_IMAGE (" + hex( command ) + ") ..." )
                print( "byteCount = " + str( byteCount ) )
                state = States.CMD_1SP_IMAGE_WRITE
            elif command == newton.CMD_AUTHORIZATION_CERT:
                print( "Sending CMD_AUTHORIZATION_CERT (" + hex( command ) + ") ..." )
                print( "byteCount = " + str( byteCount ) )
                state = States.CMD_AUTHORIZATION_CERT_WRITE
            elif command == newton.CMD_PUBLIC_KEY:
                print( "Sending CMD_PUBLIC_KEY (" + hex( command ) + ") ..." )
                print( "byteCount = " + str( byteCount ) )
                state = States.CMD_PUBLIC_KEY_WRITE
            elif command == newton.CMD_SIGNATURE:
                print( "Sending CMD_SIGNATURE (" + hex( command ) + ") ..." )
                print( "byteCount = " + str( byteCount ) )
                state = States.CMD_LOAD_SIGNATURE
            elif command == newton.CMD_OPERATING_MODE:
                print( "Sending CMD_OPERATING_MODE (" + hex( command ) + ") ..." )
                state = States.CMD_LOAD_HEADER
        elif state == States.CMD_GROUPED_DATA_WRITE:
            state = States.CMD_LOAD_HEADER
        elif state == States.CMD_LOAD_REG_WRITE:
            for j in range(0, int( numDataWords / 2 )):
                data    = commandData[index]
                index += 1
                address = commandData[index]
                index += 1
                print( address + " = " + data )
            state = States.CMD_LOAD_HEADER
        elif state == States.CMD_LOAD_RAM_WRITE:
            index += numDataWords
            #send_command( command, destAddress, numDataWords, attributes );
            #send_data( writeData, numDataWords )
            state = States.CMD_LOAD_HEADER
        elif state == States.CMD_1SP_IMAGE_WRITE:
            print( "Sending CMD_1SP_IMAGE ..." )
            index += numDataWords
            #send_command( command, destAddress, numDataWords, attributes );
            #send_data( writeData, numDataWords )
            state = States.CMD_LOAD_HEADER
        elif state == States.CMD_AUTHORIZATION_CERT_WRITE:
            print( "Sending CMD_AUTHORIZATION_CERT ..." )
            index += numDataWords
            #send_command( command, destAddress, numDataWords, attributes );
            #send_data( writeData, numDataWords )
            state = States.CMD_LOAD_HEADER
        elif state == States.CMD_PUBLIC_KEY_WRITE:
            print( "Sending CMD_PUBLIC_KEY ..." )
            index += numDataWords
            #send_command( command, destAddress, numDataWords, attributes )
            #send_data( writeData, numDataWords );
            state = States.CMD_LOAD_HEADER

if __name__ == "__main__":
    args = docopt(__doc__, version='0.1')
    
    input_file_name  = args['<input_file_name>']
    output_file_name = args['<output_file_name>']

    readInputFile( input_file_name )
    writeOutputFile( output_file_name )
    analyzeCommands( output_file_name )

    sys.exit( 0 )

