#!/usr/bin/env python

""" Script generated from simulation of the fc_asm_mscm_plls_test test case.


Usage:
    fc_asm_mscm_plls_test.py [--no_reset]

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

   args = docopt(__doc__, version='0.1')

   rc = newton.adi_newton_config( 0 )
   if rc != 0:
      print( "ERROR: newton.adi_newton_config return an error (" + str( rc ) + ")." )
      sys.exit( rc )

   if args['--no_reset']:
      performReset = False

   if performReset == True:
      newton.adi_reset_newton( newton.PIN_MODE_HSP_DEBUG )

   newton.adi_check_register_py( 0x0142, 0x0500 ) # pll_status
   newton.adi_write_register( 0x0028, 0x0000 ) # systemClockControl
   newton.adi_write_register( 0x0140, 0x0233 ) # pll_ctrl
   newton.adi_write_register( 0x0140, 0x0033 ) # pll_ctrl
   newton.adi_write_register( 0x015e, 0x8312 ) # syspll_ctrl2_s1
   newton.adi_check_register_py( 0x0032, 0x0000 ) # errorStatus
   newton.adi_check_register_py( 0x0142, 0x0100 ) # pll_status
   newton.adi_write_register( 0x0138, 0x00b0 ) # lsctrl0_s1
   newton.adi_write_register( 0x0150, 0x0105 ) # regif_ctrl
   newton.adi_check_register_py( 0x0032, 0x0000 ) # errorStatus
   newton.adi_check_register_py( 0x0150, 0x0101 ) # regif_ctrl
   newton.adi_write_register( 0x013a, 0x0039 ) # lsmod_en
   newton.adi_write_register( 0x018e, 0x8aea ) # ana_serial_spare_0
   newton.adi_write_register( 0x0150, 0x0105 ) # regif_ctrl
   newton.adi_check_register_py( 0x0032, 0x0000 ) # errorStatus
   newton.adi_check_register_py( 0x0150, 0x0101 ) # regif_ctrl
   newton.adi_write_register( 0x0140, 0x0133 ) # pll_ctrl
   newton.adi_write_register( 0x0140, 0x0033 ) # pll_ctrl
   newton.adi_write_register( 0x0138, 0x00c0 ) # lsctrl0_s1
   newton.adi_write_register( 0x0150, 0x0105 ) # regif_ctrl
   newton.adi_check_register_py( 0x0032, 0x0000 ) # errorStatus
   newton.adi_check_register_py( 0x0150, 0x0101 ) # regif_ctrl
   newton.adi_write_register( 0x013a, 0x0039 ) # lsmod_en
   newton.adi_write_register( 0x018e, 0x8a2a ) # ana_serial_spare_0
   newton.adi_write_register( 0x0150, 0x0105 ) # regif_ctrl
   newton.adi_check_register_py( 0x0032, 0x0000 ) # errorStatus
   newton.adi_check_register_py( 0x0150, 0x0101 ) # regif_ctrl
   newton.adi_write_register( 0x0138, 0x008a ) # lsctrl0_s1
   newton.adi_write_register( 0x0150, 0x0105 ) # regif_ctrl
   newton.adi_check_register_py( 0x0032, 0x0000 ) # errorStatus
   newton.adi_check_register_py( 0x0150, 0x0101 ) # regif_ctrl
   newton.adi_write_register( 0x013a, 0x0035 ) # lsmod_en
   newton.adi_write_register( 0x010a, 0x2114 ) # adcpll_ctrl2_s1
   newton.adi_write_register( 0x0150, 0x0105 ) # regif_ctrl
   newton.adi_check_register_py( 0x0032, 0x0000 ) # errorStatus
   newton.adi_check_register_py( 0x0150, 0x0101 ) # regif_ctrl
   newton.adi_write_register( 0x0140, 0x0323 ) # pll_ctrl
   newton.adi_write_register( 0x0140, 0x0303 ) # pll_ctrl
   newton.adi_write_register( 0x0118, 0x0000 ) # clk_ctrl
   newton.adi_check_register_py( 0x0032, 0x0000 ) # errorStatus
   newton.adi_check_register_py( 0x0142, 0x0210 ) # pll_status
   newton.adi_write_register( 0x0114, 0x000f ) # ckgen_ctrl
   newton.adi_write_register( 0x0116, 0x3f00 ) # ckgen_s1
   newton.adi_write_register( 0x0138, 0x00a0 ) # lsctrl0_s1
   newton.adi_write_register( 0x0150, 0x0105 ) # regif_ctrl
   newton.adi_check_register_py( 0x0032, 0x0000 ) # errorStatus
   newton.adi_check_register_py( 0x0150, 0x0101 ) # regif_ctrl
   newton.adi_write_register( 0x013a, 0x0035 ) # lsmod_en
   newton.adi_write_register( 0x0158, 0x200e ) # sspll_ctrl2_s1
   newton.adi_write_register( 0x0150, 0x0105 ) # regif_ctrl
   newton.adi_check_register_py( 0x0032, 0x0000 ) # errorStatus
   newton.adi_check_register_py( 0x0150, 0x0101 ) # regif_ctrl
   newton.adi_write_register( 0x0140, 0x0332 ) # pll_ctrl
   newton.adi_write_register( 0x0140, 0x0330 ) # pll_ctrl
   newton.adi_check_register_py( 0x0032, 0x0000 ) # errorStatus
   newton.adi_check_register_py( 0x0142, 0x0221 ) # pll_status
