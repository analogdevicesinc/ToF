#!/usr/bin/env python

""" Script generated from simulation of the fc_asm_mscm_dll_lock_range_5p5ns_10Mhz test case.


Usage:
    fc_asm_mscm_dll_lock_range_5p5ns_10Mhz.py [--no_fw_load][--no_reset]

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
   performReset = True

   loadFirmware = True

   args = docopt(__doc__, version='0.1')

   rc = newton.adi_newton_config( 0 )
   if rc != 0:
      print( "ERROR: newton.adi_newton_config return an error (" + str( rc ) + ")." )
      sys.exit( rc )

   if args['--no_fw_load']:
      loadFirmware = False

   if args['--no_reset']:
      performReset = False

   if performReset == True:
      newton.adi_reset_newton( newton.PIN_MODE_HSP_DEBUG )

   if loadFirmware == True:
      cmd_file = os.path.expanduser( "./tests/dms_eval_tests/fc_asm_mscm_dll_lock_range_5p5ns_10Mhz/fc_asm_mscm_dll_lock_range_5p5ns_10Mhz.txt" )
      cmd_file_bytes = cmd_file.encode(encoding='utf-8')
      newton.adi_load_command_file( cmd_file_bytes )
      newton.adi_write_register( 0x000C, 0x00c5 ) # useqControlRegister

   newton.adi_check_register_py( 0x0142, 0x0500 ) # pll_status
   newton.adi_write_register( 0x0244, 0x0020 ) # SCRATCHPAD[34]
   newton.adi_check_register_py( 0x0032, 0x0000 ) # errorStatus
   newton.adi_check_register_py( 0x0140, 0x0033 ) # pll_ctrl
   newton.adi_write_register( 0x0140, 0x0032 ) # pll_ctrl
   newton.adi_write_register( 0x0140, 0x0030 ) # pll_ctrl
   newton.adi_check_register_py( 0x0032, 0x0000 ) # errorStatus
   newton.adi_check_register_py( 0x0142, 0x0501 ) # pll_status
   newton.adi_write_register( 0x0158, 0x020b ) # sspll_ctrl2_s1
   newton.adi_write_register( 0x0138, 0x0001 ) # lsctrl0_s1
   newton.adi_write_register( 0x013a, 0x0005 ) # lsmod_en
   newton.adi_write_register( 0x0116, 0x3f00 ) # ckgen_s1
   newton.adi_write_register( 0x0150, 0x0105 ) # regif_ctrl
   newton.adi_check_register_py( 0x0032, 0x0000 ) # errorStatus
   newton.adi_check_register_py( 0x0150, 0x0101 ) # regif_ctrl
   newton.adi_write_register( 0x0144, 0x0000 ) # power_down_0
   newton.adi_write_register( 0x0188, 0x0008 ) # dll_control
   newton.adi_write_register( 0x0188, 0x0018 ) # dll_control
   newton.adi_write_register( 0x011e, 0x000f ) # clktree0
   newton.adi_check_register_py( 0x0032, 0x0000 ) # errorStatus
   newton.adi_check_register_py( 0x0034, 0x0004 ) # gpioCtrl
   newton.adi_write_register( 0x0034, 0x0000 ) # gpioCtrl
   newton.adi_write_register( 0x0138, 0x0035 ) # lsctrl0_s1
   newton.adi_write_register( 0x0150, 0x010d ) # regif_ctrl
   newton.adi_check_register_py( 0x0032, 0x0000 ) # errorStatus
   newton.adi_check_register_py( 0x0150, 0x0101 ) # regif_ctrl
   newton.adi_check_register_py( 0x0032, 0x0000 ) # errorStatus
   newton.adi_check_register_py( 0x0034, 0x0004 ) # gpioCtrl
   newton.adi_write_register( 0x0034, 0x0000 ) # gpioCtrl
   newton.adi_write_register( 0x0138, 0x0035 ) # lsctrl0_s1
   newton.adi_write_register( 0x0150, 0x010d ) # regif_ctrl
   newton.adi_check_register_py( 0x0032, 0x0000 ) # errorStatus
   newton.adi_check_register_py( 0x0150, 0x0101 ) # regif_ctrl
   newton.adi_write_register( 0x0138, 0x0024 ) # lsctrl0_s1
   newton.adi_write_register( 0x0150, 0x0105 ) # regif_ctrl
   newton.adi_check_register_py( 0x0032, 0x0000 ) # errorStatus
   newton.adi_check_register_py( 0x0150, 0x0101 ) # regif_ctrl
