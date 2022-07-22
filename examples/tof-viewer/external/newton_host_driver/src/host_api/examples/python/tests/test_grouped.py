#!/usr/bin/env python

""" Script to test grouped command

Usage:
    test_grouped.py [--count=<word_count>]

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
   
   if args['--count']:
      count = int( args['--count'] )
   else:
      count = 0

   cmd_file = os.path.expanduser( "~/host_api/dataFiles/grouped.txt" )
   cmd_file_bytes = cmd_file.encode(encoding='utf-8')

   rc = newton.adi_newton_config( 0 )
   if rc != 0:
      print( "ERROR: newton.adi_newton_config return an error (" + str( rc ) + ")." )
      sys.exit( rc )

   print( "INFO: Generating file ~/host_api/dataFiles/grouped.txt with count = " + str( count ) )
   cmd = "~/host_api/examples/python/generateBootImage.py grouped " + cmd_file + " --seed=1 --count=" + str( count )
   rc = os.system( cmd )
   if rc != 0:
      print( "ERROR: Error generateBootImage.py return an error (" + str( rc ) + ")." )
      sys.exit( rc )
 
   print( "INFO: Loading command file \"" + cmd_file + "\"" )
   rc = newton.adi_load_command_file( cmd_file_bytes )
   if rc != 0:
      print( "ERROR: Error newton.adi_load_command_file returned error (" + str( rc ) + ")." )
      sys.exit( rc )

   print( "INFO: Verifying RAMs loaded by command file \"" + cmd_file + "\"" )
   if newtonTarget == "FPGA":
      rc = newton.adi_verify_command_file( cmd_file_bytes )
   else:
      rc = newton.adi_verify_command_file_hsp( cmd_file_bytes )
   if rc != 0:
      print( "ERROR: Error newton.adi_verify_command_file returned error (" + str( rc ) + ")." )
      sys.exit( rc )
   else:
      print( "INFO: test PASSED!" )

