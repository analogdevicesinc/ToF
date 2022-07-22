#!/usr/bin/env python

""" Python script example

Usage:
    test_maxram.py [--target=<ram_target>][--count=<word_count>]

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
   
   if args['--target']:
      ram_target = args['--target']
   else:
      ram_target = "all"

   if args['--count']:
      count = int( args['--count'] )
   else:
      count = 0

   rc = newton.adi_newton_config( 0 )
   if rc != 0:
      print( "ERROR: newton.adi_newton_config return an error (" + str( rc ) + ")." )
      sys.exit( rc )

   if ram_target == "all" or ram_target == "useq_wave_ram":
      if count == 0 or count > 2048:
         count = 2048

      cmd_file = os.path.expanduser( "~/host_api/dataFiles/useq_wave_ram.txt" )
      cmd_file_bytes = cmd_file.encode(encoding='utf-8')

      print( "INFO: Generating file \"" + cmd_file + "\" with count = " + str( count ) )
      rc = os.system( "~/host_api/examples/python/generateBootImage.py useq_wave_ram " + cmd_file + " --seed=1 --count=" + str( count ) )
      if rc != 0:
         print( "ERROR: Error generateBootImage.py return an error (" + str( rc ) + ")." )
         sys.exit( rc )
 
      print( "INFO: Loading command file \"" + cmd_file + "\" ..." )
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

   if ram_target == "all" or ram_target == "useq_map_ram":
      if count == 0 or count > 128:
         count = 128

      cmd_file = os.path.expanduser( "~/host_api/dataFiles/useq_map_ram.txt" )
      cmd_file_bytes = cmd_file.encode(encoding='utf-8')

      print( "INFO: Generating file \"" + cmd_file + "\" with count = " + str( count ) )
      rc = os.system( "~/host_api/examples/python/generateBootImage.py useq_map_ram " + cmd_file + " --seed=1 --count=" + str( count ) )
      if rc != 0:
         print( "ERROR: Error generateBootImage.py return an error (" + str( rc ) + ")." )
         sys.exit( rc )
 
      print( "INFO: Loading command file \"" + cmd_file + "\" ..." )
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

   elif ram_target == "all" or ram_target == "useq_seq_ram":
      if count == 0 or count > 4096:
         count = 4096

      cmd_file = os.path.expanduser( "~/host_api/dataFiles/useq_seq_ram.txt" )
      cmd_file_bytes = cmd_file.encode(encoding='utf-8')

      print( "INFO: Generating file \"" + cmd_file + "\" with count = " + str( count ) )
      rc = os.system( "~/host_api/examples/python/generateBootImage.py useq_seq_ram " + cmd_file + " --seed=1 --count=" + str( count ) )
      if rc != 0:
         print( "ERROR: Error generateBootImage.py return an error (" + str( rc ) + ")." )
         sys.exit( rc )
 
      print( "INFO: Loading command file \"" + cmd_file + "\" ..." )
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

   if ram_target == "all" or ram_target == "datapath_ram":
      if count == 0 or count > 4096:
         count = 4096

      cmd_file = os.path.expanduser( "~/host_api/dataFiles/datapath_ram.txt" )
      cmd_file_bytes = cmd_file.encode(encoding='utf-8')

      print( "INFO: Generating file \"" + cmd_file + "\" with count = " + str( count ) )
      rc = os.system( "~/host_api/examples/python/generateBootImage.py datapath_ram " + cmd_file + " --seed=1 --count=" + str( count ) )
      if rc != 0:
         print( "ERROR: Error generateBootImage.py return an error (" + str( rc ) + ")." )
         sys.exit( rc )
 
      print( "INFO: Loading command file \"" + cmd_file + "\" ..." )
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

   if ram_target == "all" or ram_target == "de_ram":
      if count == 0 or count > 512:
         count = 512

      cmd_file = os.path.expanduser( "~/host_api/dataFiles/de_ram.txt" )
      cmd_file_bytes = cmd_file.encode(encoding='utf-8')

      print( "INFO: Generating file \"" + cmd_file + "\" with count = " + str( count ) )
      rc = os.system( "~/host_api/examples/python/generateBootImage.py de_ram " + cmd_file + " --seed=1 --count=" + str( count ) )
      if rc != 0:
         print( "ERROR: Error generateBootImage.py return an error (" + str( rc ) + ")." )
         sys.exit( rc )
 
      print( "INFO: Loading command file \"" + cmd_file + "\" ..." )
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

   if ram_target == "all" or ram_target == "lps1_ram":
      if count == 0 or count > 256:
         count = 256

      cmd_file = os.path.expanduser( "~/host_api/dataFiles/lps1_ram.txt" )
      cmd_file_bytes = cmd_file.encode(encoding='utf-8')

      print( "INFO: Generating file \"" + cmd_file + "\" with count = " + str( count ) )
      rc = os.system( "~/host_api/examples/python/generateBootImage.py lps1_ram " + cmd_file + " --seed=1 --count=" + str( count ) )
      if rc != 0:
         print( "ERROR: Error generateBootImage.py return an error (" + str( rc ) + ")." )
         sys.exit( rc )
 
      print( "INFO: Loading command file \"" + cmd_file + "\" ..." )
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

   if ram_target == "all" or ram_target == "lps2_ram":
      if count == 0 or count > 256:
         count = 256

      cmd_file = os.path.expanduser( "~/host_api/dataFiles/lps2_ram.txt" )
      cmd_file_bytes = cmd_file.encode(encoding='utf-8')

      print( "INFO: Generating file \"" + cmd_file + "\" with count = " + str( count ) )
      rc = os.system( "~/host_api/examples/python/generateBootImage.py lps2_ram \"" + cmd_file + "\" --seed=1 --count=" + str( count ) )
      if rc != 0:
         print( "ERROR: Error generateBootImage.py return an error (" + str( rc ) + ")." )
         sys.exit( rc )
 
      print( "INFO: Loading command file \"" + cmd_file + "\" ..." )
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

   print( "INFO: test PASSED!" )

