#!/usr/bin/env python

""" Script generated from simulation of the fc_asm_mscm_rdriv_1M test case.


Usage:
    fc_asm_mscm_rdriv_1M.py [--no_fw_load][--no_reset]

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
      cmd_file = os.path.expanduser( "./tests/dms_eval_tests/fc_asm_mscm_rdriv_1M/fc_asm_mscm_rdriv_1M.txt" )
      cmd_file_bytes = cmd_file.encode(encoding='utf-8')
      newton.adi_load_command_file( cmd_file_bytes )
      newton.adi_write_register( 0x000C, 0x00c5 ) # useqControlRegister

   newton.adi_check_register_py( 0x0142, 0x0500 ) # pll_status
   newton.adi_write_register( 0x0104, 0x00f2 ) # adc_ctrl2_s1
   newton.adi_write_register( 0x0102, 0x00c2 ) # adc_ctrl1_s1
   newton.adi_write_register( 0x0100, 0x0014 ) # adc_ctrl0_s1
   newton.adi_write_register( 0x0244, 0x0020 ) # SCRATCHPAD[34]
   newton.adi_write_register( 0x0014, 0x3910 ) # digPwrDown
   newton.adi_write_register( 0x0146, 0x007b ) # power_down_adc_others
   newton.adi_write_register( 0x011a, 0x0001 ) # clk_de_ctrl_s1
   newton.adi_write_register( 0x0e00, 0x0082 ) # de_control
   newton.adi_write_register( 0x0e5a, 0x0003 ) # array_init_vec_dark
   newton.adi_write_register( 0x0d0c, 0x0702 ) # USE_CASE_MIPI_PACKET_CONTROL
   newton.adi_check_register_py( 0x0032, 0x0040 ) # errorStatus
   newton.adi_check_register_py( 0x0256, 0x0001 ) # SCRATCHPAD[43]
   newton.adi_check_register_py( 0x0032, 0x0040 ) # errorStatus
   newton.adi_check_register_py( 0x0256, 0x0001 ) # SCRATCHPAD[43]
   newton.adi_check_register_py( 0x0032, 0x0040 ) # errorStatus
   newton.adi_check_register_py( 0x0256, 0x0001 ) # SCRATCHPAD[43]
   newton.adi_check_register_py( 0x0032, 0x0040 ) # errorStatus
   newton.adi_check_register_py( 0x0256, 0x0001 ) # SCRATCHPAD[43]
   newton.adi_check_register_py( 0x0032, 0x0040 ) # errorStatus
   newton.adi_check_register_py( 0x0256, 0x0001 ) # SCRATCHPAD[43]
   newton.adi_check_register_py( 0x0032, 0x0040 ) # errorStatus
   newton.adi_check_register_py( 0x0256, 0x0001 ) # SCRATCHPAD[43]
   newton.adi_check_register_py( 0x0032, 0x0040 ) # errorStatus
   newton.adi_check_register_py( 0x0256, 0x0002 ) # SCRATCHPAD[43]
   newton.adi_write_register( 0x0200, 0x0007 ) # SCRATCHPAD[0]
   newton.adi_check_register_py( 0x0032, 0x0040 ) # errorStatus
   newton.adi_check_register_py( 0x0256, 0x0002 ) # SCRATCHPAD[43]
