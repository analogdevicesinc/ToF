#!/usr/bin/env python

""" Script generated from simulation of the fc_asm_mscm_amp_test test case.


Usage:
    fc_asm_mscm_amp_test.py [--no_fw_load][--no_reset]

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
      cmd_file = os.path.expanduser( "./tests/dms_eval_tests/fc_asm_mscm_amp_test/fc_asm_mscm_amp_test.txt" )
      cmd_file_bytes = cmd_file.encode(encoding='utf-8')
      newton.adi_load_command_file( cmd_file_bytes )
      newton.adi_write_register( 0x000C, 0x00c5 ) # useqControlRegister

   newton.adi_check_register_py( 0x0142, 0x0500 ) # pll_status
   newton.adi_write_register( 0x0244, 0x0020 ) # SCRATCHPAD[34]
   newton.adi_write_register( 0x0014, 0x3918 ) # digPwrDown
   newton.adi_write_register( 0x0100, 0x0030 ) # adc_ctrl0_s1
   newton.adi_write_register( 0x0192, 0x8380 ) # ana_serial_spare_2
   newton.adi_write_register( 0x0128, 0x8000 ) # dac_ctrl2
   newton.adi_write_register( 0x0132, 0x0080 ) # dac_data
   newton.adi_write_register( 0x0126, 0x0004 ) # dac_ctrl1
   newton.adi_write_register( 0x0132, 0x00a8 ) # dac_data
   newton.adi_write_register( 0x0126, 0x0080 ) # dac_ctrl1
   newton.adi_write_register( 0x0160, 0x0001 ) # ana_test_mux_s1
   newton.adi_write_register( 0x0150, 0x0905 ) # regif_ctrl
   newton.adi_check_register_py( 0x0032, 0x0000 ) # errorStatus
   newton.adi_check_register_py( 0x0150, 0x0101 ) # regif_ctrl
   newton.adi_write_register( 0x0e1c, 0x0100 ) # amp_mux_sel_EE_low
   newton.adi_write_register( 0x0e1e, 0x0004 ) # amp_mux_sel_EE_high
   newton.adi_check_register_py( 0x0032, 0x0000 ) # errorStatus
   newton.adi_check_register_py( 0x0256, 0x0001 ) # SCRATCHPAD[43]
   newton.adi_check_register_py( 0x0032, 0x0000 ) # errorStatus
   newton.adi_check_register_py( 0x0256, 0x0001 ) # SCRATCHPAD[43]
   newton.adi_check_register_py( 0x0032, 0x0000 ) # errorStatus
   newton.adi_check_register_py( 0x0256, 0x0001 ) # SCRATCHPAD[43]
   newton.adi_check_register_py( 0x0032, 0x0000 ) # errorStatus
   newton.adi_check_register_py( 0x0256, 0x0001 ) # SCRATCHPAD[43]
   newton.adi_check_register_py( 0x0032, 0x0000 ) # errorStatus
   newton.adi_check_register_py( 0x0256, 0x0001 ) # SCRATCHPAD[43]
   newton.adi_check_register_py( 0x0032, 0x0000 ) # errorStatus
   newton.adi_check_register_py( 0x0256, 0x0002 ) # SCRATCHPAD[43]
   newton.adi_write_register( 0x0200, 0x0005 ) # SCRATCHPAD[0]
   newton.adi_check_register_py( 0x0032, 0x0000 ) # errorStatus
   newton.adi_check_register_py( 0x0256, 0x0002 ) # SCRATCHPAD[43]
   newton.adi_write_register( 0x0132, 0x0080 ) # dac_data
   newton.adi_write_register( 0x0126, 0x0004 ) # dac_ctrl1
   newton.adi_write_register( 0x0132, 0x0081 ) # dac_data
   newton.adi_write_register( 0x0126, 0x0080 ) # dac_ctrl1
   newton.adi_write_register( 0x0132, 0x0080 ) # dac_data
   newton.adi_write_register( 0x0126, 0x0004 ) # dac_ctrl1
   newton.adi_write_register( 0x0132, 0x0088 ) # dac_data
   newton.adi_write_register( 0x0126, 0x0080 ) # dac_ctrl1
   newton.adi_write_register( 0x0132, 0x0080 ) # dac_data
   newton.adi_write_register( 0x0126, 0x0004 ) # dac_ctrl1
   newton.adi_write_register( 0x0132, 0x0090 ) # dac_data
   newton.adi_write_register( 0x0126, 0x0080 ) # dac_ctrl1
   newton.adi_write_register( 0x0132, 0x0080 ) # dac_data
   newton.adi_write_register( 0x0126, 0x0004 ) # dac_ctrl1
   newton.adi_write_register( 0x0132, 0x00a8 ) # dac_data
   newton.adi_write_register( 0x0126, 0x0080 ) # dac_ctrl1
   newton.adi_write_register( 0x0146, 0x0020 ) # power_down_adc_others
