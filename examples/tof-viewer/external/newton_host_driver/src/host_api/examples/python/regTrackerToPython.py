#!/usr/bin/env python

""" Translate reg_tracker.txt to a python script using the host_api

Usage:
    regTrackerToPython.py <input_file_name> <output_file_name> [--no_firmware_load]

Options:
    --help Shows this help message.
"""

from __future__ import print_function
from __future__ import absolute_import
from __future__ import unicode_literals

from docopt import docopt
import sys
import os
import subprocess
import re
import stat
import errno
import encodings
import shutil
import datetime

def readRegTrackerFile( inputFileName ):
   global regAccesses
   global useqStarted
   global useqStartedData
   global useqControlRegisterWritten
   global startProccessingRegAccesses

   inputLines = []
   de_ram_temp = {}
   seq_ram_temp = {}
   wave_ram_temp = {}
   map_ram_temp = {}
   datapath_ram_temp = {}
   lps1_ram_temp = {}
   lps2_ram_temp = {}
   regWrites = {}
   regAccesses = []
   ramAccess = "NONE"
   ramSelect = 0
   ramAddress = 0
   useqStarted = 0
   useqStartedData = "0x0000"
   firmwareLoaded = 0
   useqControlRegisterWritten = False
   startProccessingRegAccesses = False

   print( "INFO: Reading file " + inputFileName + " ..."  )

   with open( inputFileName ) as ifile:  
      line = ifile.readline()
      while line:
         line = line.rstrip('\r\n')
         inputLines.append( line )
         line = ifile.readline()

   ifile.close( )

   for linein in inputLines:
      if re.search( r"\|\s+16\'h[\w]+\s+", linein ):
         #                          address        regName                  data           read         source       blockName
         m = re.search( r"\|\s+16\'h([\w]+)\s+\|\s+([\w_\[\]]+)\s+\|\s+16\'h([\w]+)\s+\|\s+(\w+)\s+\|\s+(\w+)\s+\|\s+([\w_]+)\s+", linein )
         address   = int( m.group(1), 16 )
         regName   = m.group(2)
         data      = int( m.group(3), 16 )
         read      = m.group(4)
         source    = m.group(5)
         blockName = m.group(6)

         if source == "HSP":
            if read == "write":
               if regName == "IA_SELECT" and blockName == "datapath_regs":
                  # NOTE: RAMs are loaded using using Grouped Command. 
                  ramAccess = 0
               elif regName == "IA_ADDR_REG" and blockName == "datapath_regs":
                  # NOTE: RAMs are loaded using using Grouped Command. 
                  ramAddress = data
               elif regName == "IA_WRDATA_REG" and blockName == "datapath_regs":
                  # NOTE: RAMs are loaded using using Grouped Command. 
                  datapath_ram_temp[ramAddress] = data
                  ramAddress += 1
               elif regName == "de_ia_select" and blockName == "de_regs":
                  # NOTE: RAMs are loaded using using Grouped Command. 
                  ramAccess = 0
               elif regName == "de_ia_addr_reg" and blockName == "de_regs":
                  # NOTE: RAMs are loaded using using Grouped Command. 
                  ramAddress = data
               elif regName == "de_ia_wrdata_reg" and blockName == "de_regs":
                  # NOTE: RAMs are loaded using using Grouped Command. 
                  datapath_ram_temp[ramAddress] = data
                  ramAddress += 1
               elif regName == "lpsRamRdCmd" and blockName == "lps1_regs":
                  # NOTE: RAMs are loaded using using Grouped Command. 
                  ramAddress = data
               elif regName == "lpsRamData" and blockName == "lps1_regs":
                  # NOTE: RAMs are loaded using using Grouped Command. 
                  lps1_ram_temp[ramAddress] = data
                  ramAddress += 1
               elif regName == "lpsRamRdCmd" and blockName == "lps2_regs":
                  # NOTE: RAMs are loaded using using Grouped Command. 
                  ramAddress = data
               elif regName == "lpsRamData" and blockName == "lps2_regs":
                  # NOTE: RAMs are loaded using using Grouped Command. 
                  lps2_ram_temp[ramAddress] = data
                  ramAddress += 1
               elif regName == "useqRamLoadAddr" and blockName == "useq_regs":
                  # NOTE: RAMs are loaded using using Grouped Command. 
                  ramAddress = data & 0xfffc
                  ramAccess  = data & 0x3
               elif regName == "useqRamLoadData" and blockName == "useq_regs" and ramAccess == 0:
                  # NOTE: RAMs are loaded using using Grouped Command. 
                  seq_ram_temp[ramAddress] = data
                  ramAddress += 1
                  firmwareLoaded |= 1
               elif regName == "useqRamLoadData" and blockName == "useq_regs" and ramAccess == 1:
                  # NOTE: RAMs are loaded using using Grouped Command. 
                  wave_ram_temp[ramAddress] = data
                  ramAddress += 1
                  firmwareLoaded |= 2
               elif regName == "useqRamLoadData" and blockName == "useq_regs" and ramAccess == 2:
                  # NOTE: RAMs are loaded using using Grouped Command. 
                  map_ram_temp[ramAddress] = data
                  ramAddress += 1
                  firmwareLoaded |= 4
               elif regName == "useqControlRegister":
                  # Started the micro-sequencer
                  regAccess = {}
                  regAccess["rw"] = "write"
                  regAccess["address"] = address
                  regAccess["data"] = data
                  regAccess["name"] = regName
                  if ((data & 1) == 1) and (firmwareLoaded != 0):
                     # Microsequencer Started
                     # FIXME: regAccesses.append( regAccess )
                     useqStarted = 1
                     useqStartedData = '0x{0:04x}'.format( data )
                     startProccessingRegAccesses = True
                     # print( "write_register( " + '0x{0:04x}'.format( address ) + ", " + '0x{0:04x}'.format( data ) + " ) # " + regName )
                  if ((data & 1) == 0) and (firmwareLoaded != 0):
                     useqControlRegisterWritten = True
               # FIXME: elif (regName == "SCRATCHPAD[34]") and (useqControlRegisterWritten == True):
               # FIXME:    startProccessingRegAccesses = True
               # FIXME:    scratchPad34WriteData = '0x{0:04x}'.format( data )
               elif (startProccessingRegAccesses == True) or (noFirmwareLoad == True):
                  regAccess = {}
                  regAccess["rw"] = "write"
                  regAccess["address"] = address
                  regAccess["data"] = data
                  regAccess["name"] = regName
                  regAccesses.append( regAccess )
            elif read == "read":
               # FIXME: if (startProccessingRegAccesses == False) and (noFirmwareLoad == False):
               # FIXME:    print( "ERROR: read_register: startProccessingRegAccesses should have been set!" )
               regAccess = {}
               regAccess["rw"] = "read"
               regAccess["address"] = address
               regAccess["data"] = data
               regAccess["name"] = regName
               regAccesses.append( regAccess )
            else:
               print( "ERROR: Unexpected HSP read field (" + read + ") found. address = " + '0x{0:04x}'.format( address ) + ", data = " + '0x{0:04x}'.format( data ) )
         elif source == "USEQ":
            if read == "write":
               skip = True
               # print( "   # USEQ write: write_register( " + '0x{0:04x}'.format( address ) + ", " + '0x{0:04x}'.format( data ) + " )" )
            elif read == "read":
               skip = True
               # print( "   # USEQ read: read_register( " + '0x{0:04x}'.format( address ) + ", " + '0x{0:04x}'.format( data ) + " )" )
            else:
               print( "ERROR: unexpected read field (" + read + ") found." )
         else:
            print( "ERROR: unexpected source field (" + source + ") found." )

def registerAccessess( output_file_name ):

   dirName, fileName = os.path.split( output_file_name )
   testName = fileName.replace( ".py", "" )

   if noFirmwareLoad == True:
      loadFirmware = False
   elif useqStarted == 1:
      loadFirmware = True
   else:
      loadFirmware = False

   s = f"""\
#!/usr/bin/env python

\"\"\" Script generated from simulation of the {testName} test case.

"""
   outputLines.append( s )

   if noFirmwareLoad == True:
      s = f"""\
Usage:
    {fileName} [--no_reset]
"""
      outputLines.append( s )
   else:
      s = f"""\
Usage:
    {fileName} [--no_fw_load][--no_reset]
"""
      outputLines.append( s )

   s = f"""\
Options:
    --help Shows this help message.
\"\"\"

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
import ctypes
from collections import OrderedDict
import threading
from newton_control_main import newton as newton

if __name__ == "__main__":
   performReset = True
"""
   outputLines.append( s )

   if noFirmwareLoad == False:
      s = f"""\
   loadFirmware = {loadFirmware}
"""
      outputLines.append( s )

   s = f"""\
   args = docopt(__doc__, version='0.1')

   rc = newton.adi_newton_config( 0 )
   if rc != 0:
      print( \"ERROR: newton.adi_newton_config return an error (\" + str( rc ) + \").\" )
      sys.exit( rc )
"""
   outputLines.append( s )

   if noFirmwareLoad == False:
      s = f"""\
   if args['--no_fw_load']:
      loadFirmware = False
"""
      outputLines.append( s )

   s = f"""\
   if args['--no_reset']:
      performReset = False

   if performReset == True:
      newton.adi_reset_newton( newton.PIN_MODE_HSP_DEBUG )
"""
   outputLines.append( s )

   if noFirmwareLoad == False:
      s = f"""\
   if loadFirmware == True:
      cmd_file = os.path.expanduser( \"./tests/dms_eval_tests/{testName}/{testName}.txt\" )
      cmd_file_bytes = cmd_file.encode(encoding='utf-8')
      newton.adi_load_command_file( cmd_file_bytes )
      newton.adi_write_register( 0x000C, {useqStartedData} ) # useqControlRegister
"""
      outputLines.append( s )

   for regAccess in regAccesses:
      rw      = regAccess["rw"]
      address = regAccess["address"]
      data    = regAccess["data"]
      regName = regAccess["name"]
      
      if rw == "read":
         outputLines.append( "   newton.adi_check_register_py( " + '0x{0:04x}'.format( address ) + ", " + '0x{0:04x}'.format( data ) + " ) # " + regName )
      else:
         outputLines.append( "   newton.adi_write_register( " + '0x{0:04x}'.format( address ) + ", " + '0x{0:04x}'.format( data ) + " ) # " + regName ) 

def writePythonScript( outputFileName ):

   ofile = open( outputFileName, "w" )

   for outLine in outputLines:
      ofile.write( outLine + "\n" )

   ofile.close( )


if __name__ == "__main__":
   global outputLines
   global noFirmwareLoad
   outputLines = []
   noFirmwareLoad = False
   
   args = docopt(__doc__, version='0.1')

   if args['--no_firmware_load']:
       noFirmwareLoad = True

   input_file_name  = args['<input_file_name>']
   output_file_name = args['<output_file_name>']

   readRegTrackerFile( input_file_name )
   registerAccessess( output_file_name )
   writePythonScript( output_file_name )
