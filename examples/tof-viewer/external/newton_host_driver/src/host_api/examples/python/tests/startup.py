#!/usr/bin/env python

""" Start up script

Usage:
    startup.py [--security_state=blank|test|prod|secure|retest][--pin_mode=functional|hsp_debug|prod_scan|fa_scan|keep_current]

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
   firmware_version = "1.0-rc4"
   
   if args['--security_state']:
      state = args['--security_state']
   else:
      state = "test"
      
   if args['--pin_mode']:
      pin_mode = args['--pin_mode']
   else:
      pin_mode = "hsp_debug"

   if pin_mode == "functional":
      pinModeValue = newton.PIN_MODE_FUNCTIONAL
   elif pin_mode == "hsp_debug":
      pinModeValue = newton.PIN_MODE_HSP_DEBUG
   elif pin_mode == "prod_scan":
      pinModeValue = newton.PIN_MODE_DFT_JTAG
   elif pin_mode == "fa_scan":
      pinModeValue = newton.PIN_MODE_FIELD_RETURN_SCAN
   elif pin_mode == "keep_current":
      pinModeValue = newton.PIN_MODE_KEEP_CURRENT_MODE
   else:
      print( "ERROR: Undefined pin_mode (" + pin_mode + ") specified." )
      sys.exit( -1 )

   efuse_file = os.path.expanduser( "~/host_api/efuse/fuse_" + state + ".vhx" )
   efuse_file_bytes = efuse_file.encode(encoding='utf-8')
   rom_file =  os.path.expanduser( "~/host_api/firmware/" + firmware_version + "/Real_Pri_OBS_Build/patched_rom/adi-drop-" + firmware_version + "-release_fpga_24MHz_patched.vhx" )
   rom_file_bytes = rom_file.encode(encoding='utf-8')

   rc = newton.adi_newton_config( 0 )
   if rc != 0:
      print( "ERROR: newton.adi_newton_config return an error (" + str( rc ) + ")." )
      sys.exit( rc )

   if newtonTarget == "FPGA":
      print( "INFO: Loading the eFuse file \"" + efuse_file + "\"..." )
      rc = newton.adi_load_hsp( newton.EFUSE, efuse_file_bytes )
      if rc != 0:
         print( "ERROR: newton_control.exe return an error (" + str( rc ) + ")." )
         sys.exit( rc )

      print( "INFO: Verifying the eFuse ..." )
      rc = newton.adi_verify_hsp( newton.EFUSE, efuse_file_bytes )
      if rc != 0:
         print( "ERROR: newton_control.exe return an error (" + str( rc ) + ")." )
         sys.exit( rc )

      print( "INFO: Loading the HSP firmware file \"" + rom_file + "\"..." )
      rc = newton.adi_load_hsp( newton.HSP_ROM, rom_file_bytes )
      if rc != 0:
         print( "ERROR: newton_control.exe return an error (" + str( rc ) + ")." )
         sys.exit( rc )

      print( "INFO: Verifying the HSP firmware file \"" + rom_file + "\"..." )
      rc = newton.adi_verify_hsp( newton.HSP_ROM, rom_file_bytes )
      if rc != 0:
         print( "ERROR: newton_control.exe return an error (" + str( rc ) + ")." )
         sys.exit( rc )

   print( "INFO: Resetting newton ..." )

   # rc = newton.adi_reset_newton( newton.PIN_MODE_KEEP_CURRENT_MODE )
   rc = newton.adi_reset_newton( pinModeValue )
   if rc != 0:
      print( "ERROR: newton.adi_reset_newton return an error (" + str( rc ) + ")." )
      sys.exit( rc )

   print( "INFO: Startup DONE!" )

