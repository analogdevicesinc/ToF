#!/usr/bin/env python

""" Test Newton RAM load commands

Usage:
    test_ram.py [--target=<ram_target>][--count=<word_count>]

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

def reset_hsp( ):

   print( "INFO: Resetting newton ..." )
   rc = newton.adi_reset_hsp()
   if rc != 0:
      print( "ERROR: Error returned from adi_reset_hsp (" + str( rc ) + ")." )
      print( "INFO: test FAILED!" )
      sys.exit( rc )

   return rc

if __name__ == "__main__":
   args = docopt(__doc__, version='0.1')

   cmd_file = os.path.expanduser( "~/host_api/dataFiles/hsp_bad_command.txt" )
   cmd_file_bytes = cmd_file.encode(encoding='utf-8')

   rc = newton.adi_newton_config( 0 )
   if rc != 0:
      print( "ERROR: newton.adi_newton_config return an error (" + str( rc ) + ")." )
      sys.exit( rc )

   print( "INFO: Loading command file \"" + cmd_file + "\"" )
   rc = newton.adi_load_command_file( cmd_file_bytes )
   if rc != 0:
      print( "ERROR: Error newton_control.exe return an error (" + str( rc ) + ")." )
      print( "INFO: test FAILED!" )
      sys.exit( rc )

   print( "INFO: test PASSED!" )

