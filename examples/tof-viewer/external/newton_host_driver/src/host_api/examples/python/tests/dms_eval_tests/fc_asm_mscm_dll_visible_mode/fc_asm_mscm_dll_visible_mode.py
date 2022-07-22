#!/usr/bin/env python

""" Script generated from simulation of the fc_asm_mscm_dll_visible_mode test case.


Usage:
    fc_asm_mscm_dll_visible_mode.py [--no_fw_load][--no_reset]

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
      cmd_file = os.path.expanduser( "./tests/dms_eval_tests/fc_asm_mscm_dll_visible_mode/fc_asm_mscm_dll_visible_mode.txt" )
      cmd_file_bytes = cmd_file.encode(encoding='utf-8')
      newton.adi_load_command_file( cmd_file_bytes )
      newton.adi_write_register( 0x000C, 0x00c5 ) # useqControlRegister

   newton.adi_check_register_py( 0x0142, 0x0500 ) # pll_status
   newton.adi_write_register( 0x0244, 0x0020 ) # SCRATCHPAD[34]
   newton.adi_write_register( 0x0028, 0x0000 ) # systemClockControl
   newton.adi_write_register( 0x0140, 0x0133 ) # pll_ctrl
   newton.adi_write_register( 0x015e, 0x8512 ) # syspll_ctrl2_s1
   newton.adi_write_register( 0x015a, 0xea10 ) # syspll_ctrl0_s1
   newton.adi_write_register( 0x0146, 0x007b ) # power_down_adc_others
   newton.adi_write_register( 0x017c, 0x0520 ) # xosc_ctrl
   newton.adi_write_register( 0x0150, 0x0105 ) # regif_ctrl
   newton.adi_check_register_py( 0x0032, 0x0000 ) # errorStatus
   newton.adi_check_register_py( 0x0150, 0x0101 ) # regif_ctrl
   newton.adi_write_register( 0x0140, 0x0033 ) # pll_ctrl
   newton.adi_write_register( 0x0028, 0x0001 ) # systemClockControl
   newton.adi_check_register_py( 0x0032, 0x0000 ) # errorStatus
   newton.adi_check_register_py( 0x0140, 0x0033 ) # pll_ctrl
   newton.adi_write_register( 0x0140, 0x0032 ) # pll_ctrl
   newton.adi_write_register( 0x0140, 0x0030 ) # pll_ctrl
   newton.adi_check_register_py( 0x0032, 0x0000 ) # errorStatus
   newton.adi_check_register_py( 0x0142, 0x0701 ) # pll_status
   newton.adi_write_register( 0x0158, 0x700e ) # sspll_ctrl2_s1
   newton.adi_write_register( 0x0146, 0x007f ) # power_down_adc_others
   newton.adi_write_register( 0x0144, 0x0001 ) # power_down_0
   newton.adi_write_register( 0x0140, 0x1033 ) # pll_ctrl
   newton.adi_write_register( 0x018e, 0x8aaa ) # ana_serial_spare_0
   newton.adi_write_register( 0x0138, 0x0042 ) # lsctrl0_s1
   newton.adi_write_register( 0x011c, 0x0002 ) # clk_lvdstx_s1
   newton.adi_write_register( 0x0136, 0x0002 ) # ls_lvdstx_s1
   newton.adi_write_register( 0x0900, 0x0040 ) # lpsCtrl
   newton.adi_write_register( 0x013a, 0x0005 ) # lsmod_en
   newton.adi_write_register( 0x0116, 0x0000 ) # ckgen_s1
   newton.adi_write_register( 0x0114, 0x0000 ) # ckgen_ctrl
   newton.adi_write_register( 0x0114, 0x0029 ) # ckgen_ctrl
   newton.adi_write_register( 0x0150, 0x0105 ) # regif_ctrl
   newton.adi_check_register_py( 0x0032, 0x0000 ) # errorStatus
   newton.adi_check_register_py( 0x0150, 0x0101 ) # regif_ctrl
   newton.adi_write_register( 0x0144, 0x0005 ) # power_down_0
   newton.adi_write_register( 0x0188, 0x0008 ) # dll_control
   newton.adi_write_register( 0x0188, 0x0018 ) # dll_control
   newton.adi_write_register( 0x0122, 0x0000 ) # clktree_s1
   newton.adi_write_register( 0x0150, 0x0105 ) # regif_ctrl
   newton.adi_check_register_py( 0x0032, 0x0000 ) # errorStatus
   newton.adi_check_register_py( 0x0150, 0x0101 ) # regif_ctrl
   newton.adi_check_register_py( 0x0032, 0x0000 ) # errorStatus
   newton.adi_check_register_py( 0x0034, 0x0004 ) # gpioCtrl
   newton.adi_write_register( 0x0034, 0x0000 ) # gpioCtrl
   newton.adi_write_register( 0x0138, 0x0067 ) # lsctrl0_s1
   newton.adi_write_register( 0x0150, 0x010d ) # regif_ctrl
   newton.adi_check_register_py( 0x0032, 0x0000 ) # errorStatus
   newton.adi_check_register_py( 0x0150, 0x0101 ) # regif_ctrl
