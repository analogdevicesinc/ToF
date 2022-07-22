#!/usr/bin/env python

""" Script to load run the ADI production 1SP sequence

Usage:
    adi_production_1sp_sequence.py

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

   firmware_version = "1.0-rc4"
   oneSP_version = "1.0-rc5"

   cmd_file1 = os.path.expanduser( "~/host_api/firmware/" + oneSP_version + "/test_files/operating_sequences/msft_prod_1sp_operating_seq_for_adi_prod.txt" )
   cmd_file2 = os.path.expanduser( "~/host_api/firmware/" + oneSP_version + "/Real_Pri_OBS_Build/newton_payloads/signedZero_adi-drop-1.0-rc5-msft-prod1sp-debug-fpga.txt" )
   cmd_file3 = os.path.expanduser( "~/host_api/firmware/" + oneSP_version + "/test_files/operating_sequences/adi_prod_1sp_operating_seq.txt" )
   cmd_file4 = os.path.expanduser( "~/host_api/firmware/" + oneSP_version + "/test_files/adi_keys/adi_public_key_hkms624.txt" )
   cmd_file5 = os.path.expanduser( "~/host_api/firmware/" + oneSP_version + "/test_files/adi_prod1sp/debug-fpga/adi_signed_group_prod_1sp_not_encrypted.txt" )
   cmd_file1_bytes = cmd_file1.encode(encoding='utf-8')
   cmd_file2_bytes = cmd_file2.encode(encoding='utf-8')
   cmd_file3_bytes = cmd_file3.encode(encoding='utf-8')
   cmd_file4_bytes = cmd_file4.encode(encoding='utf-8')
   cmd_file5_bytes = cmd_file5.encode(encoding='utf-8')

   output_dir = "~/host_api/output_files"
   efuse_file1 = os.path.expanduser( output_dir + "/efuse_file1.vxh" )
   efuse_file1_bytes = efuse_file1.encode(encoding='utf-8')

   rc = newton.adi_newton_config( 0 )
   if rc != 0:
      print( "ERROR: newton.adi_newton_config return an error (" + str( rc ) + ")." )
      sys.exit( rc )

   rc = os.system( "~/host_api/examples/python/tests/startup.py --security_state=prod" )
   if rc != 0:
      print( "ERROR: startup.py return an error (" + str( rc ) + ")." )
      sys.exit( rc )

   #===========================================================
   print( "INFO: Running the MSFT Production Sequence ..." )

   print( "INFO: Loading the command file \"" + cmd_file1 + "\"..." )
   rc = newton.adi_load_command_file( cmd_file1_bytes )
   if rc != 0:
      print( "ERROR: newton.adi_load_command_file returned error (" + str( rc ) + ")." )
      sys.exit( rc )

   print( "INFO: Loading the command file \"" + cmd_file2 + "\"..." )
   rc = newton.adi_load_command_file( cmd_file2_bytes )
   if rc != 0:
      print( "ERROR: newton.adi_load_command_file returned error (" + str( rc ) + ")." )
      sys.exit( rc )

   print( "INFO: Checking the 1SP completion code ..." )
   rc = newton.adi_check_done_code( )
   if rc != 0:
      print( "ERROR: newton.adi_check_done_code returned error (" + str( rc ) + ")." )
      sys.exit( rc )

   print( "INFO: Unloading the eFuse to file \"" + efuse_file1 + "\"..." )
   rc = newton.adi_unload_hsp( newton.EFUSE, efuse_file1_bytes )
   if rc != 0:
      print( "ERROR: newton.adi_unload_hsp returned error (" + str( rc ) + ")." )
      sys.exit( rc )

   print( "INFO: Resetting the HSP ..." )
   rc = newton.adi_reset_newton( newton.PIN_MODE_HSP_DEBUG )
   if rc != 0:
      print( "ERROR: newton.adi_reset_newton returned error (" + str( rc ) + ")." )
      sys.exit( rc )
   #===========================================================
   #===========================================================
   print( "INFO: Loading the command file \"" + cmd_file3 + "\"..." )
   rc = newton.adi_load_command_file( cmd_file3_bytes )
   if rc != 0:
      print( "ERROR: newton.adi_load_command_file returned error (" + str( rc ) + ")." )
      sys.exit( rc )

   print( "INFO: Loading the command file \"" + cmd_file4 + "\"..." )
   rc = newton.adi_load_command_file( cmd_file4_bytes )
   if rc != 0:
      print( "ERROR:  newton.adi_load_command_file returned error (" + str( rc ) + ")." )
      sys.exit( rc )

   print( "INFO: Loading the command file \"" + cmd_file5 + "\"..." )
   rc = newton.adi_load_command_file( cmd_file5_bytes )
   if rc != 0:
      print( "ERROR: newton.adi_load_command_file returned error (" + str( rc ) + ")." )
      sys.exit( rc )
   else:
      print( "INFO: Load of command file completed." )

   print( "INFO: Checking the 1SP completion code ..." )
   rc = newton.adi_check_done_code( )
   if rc != 0:
      print( "ERROR: newton.adi_check_done_code returned error (" + str( rc ) + ")." )
      sys.exit( rc )

   print( "INFO: Resetting the HSP ..." )
   rc = newton.adi_reset_newton( newton.PIN_MODE_HSP_DEBUG )
   if rc != 0:
      print( "ERROR:  newton.adi_reset_newton returned error (" + str( rc ) + ")." )
      sys.exit( rc )

   print( "INFO: Loading the command file \"" + cmd_file3 + "\"..." )
   rc = newton.adi_load_command_file( cmd_file3_bytes )
   if rc != 0:
      print( "ERROR: newton.adi_load_command_file returned error (" + str( rc ) + ")." )
      sys.exit( rc )

   print( "INFO: Loading the command file \"" + cmd_file4 + "\"..." )
   rc = newton.adi_load_command_file( cmd_file4_bytes )
   if rc != 0:
      print( "ERROR: newton.adi_load_command_file returned error (" + str( rc ) + ")." )
      sys.exit( rc )

   print( "INFO: Loading the command file \"" + cmd_file5 + "\"..." )
   rc = newton.adi_load_command_file( cmd_file5_bytes )
   if rc != 0:
      print( "ERROR: newton.adi_load_command_file returned error (" + str( rc ) + ")." )
      sys.exit( rc )
   else:
      print( "INFO: Load of command file completed." )

   print( "INFO: Checking the 1SP completion code ..." )
   rc = newton.adi_check_done_code( )
   if rc != 0:
      print( "ERROR: newton.adi_check_done_code returned error (" + str( rc ) + ")." )
      sys.exit( rc )
   else:
      print( "INFO: Load of command file completed." )
      
   print( "INFO: Resetting the HSP ..." )
   rc = newton.adi_reset_newton( newton.PIN_MODE_HSP_DEBUG )
   if rc != 0:
      print( "ERROR: newton.adi_reset_newton returned error (" + str( rc ) + ")." )
      sys.exit( rc )
   #===========================================================

   print( "INFO: Startup PASSED!" )
