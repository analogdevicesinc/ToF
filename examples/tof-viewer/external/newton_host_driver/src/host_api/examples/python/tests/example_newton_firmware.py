#!/usr/bin/env python

""" Test Newton firmware load example

Usage:
    example_newton_firmware.py

Options:
    --help Shows this help message.
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
import ctypes
from collections import OrderedDict
import threading
from newton_control_main import newton as newton

if __name__ == "__main__":
   args = docopt(__doc__, version='0.1')

   newtonTarget = os.environ["NEWTON_TARGET"]
   
   cmd_file = os.path.expanduser( "~/host_api/dataFiles/newton_load_firmware_example.txt" )
   cmd_file_bytes = cmd_file.encode(encoding='utf-8')

   rc = newton.adi_newton_config( 0 )
   if rc != 0:
      print( "ERROR: newton.adi_newton_config return an error (" + str( rc ) + ")." )
      sys.exit( rc )

   print( "INFO: Loading command file \"" + cmd_file + "\" ..." )
   rc = newton.adi_load_command_file( cmd_file_bytes )
   if rc != 0:
      print( "ERROR: newton.adi_load_command_file returned error (" + str( rc ) + ")." )
      print( "INFO: test FAILED!" )
      sys.exit( rc )

   print( "INFO: Verifying RAMs loaded by command file \"" + cmd_file + "\" ..." )
   if newtonTarget == "FPGA":
      rc = newton.adi_verify_command_file( cmd_file_bytes )
   else:
      rc = newton.adi_verify_command_file_hsp( cmd_file_bytes )
   if rc != 0:
      print( "ERROR: newton.adi_verify_command_file returned error (" + str( rc ) + ")." )
      sys.exit( rc )
   else:
      print( "INFO: test PASSED!" )

