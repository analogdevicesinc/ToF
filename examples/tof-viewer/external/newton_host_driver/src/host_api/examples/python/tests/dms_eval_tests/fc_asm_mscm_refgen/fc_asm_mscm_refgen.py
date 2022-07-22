#!/usr/bin/env python

""" Script generated from simulation of the fc_asm_mscm_refgen test case.


Usage:
    fc_asm_mscm_refgen.py [--no_reset]

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
   newton.adi_write_register( 0x0140, 0x0100 ) # pll_ctrl
   newton.adi_write_register( 0x0146, 0x00fb ) # power_down_adc_others
   newton.adi_write_register( 0x0146, 0x007b ) # power_down_adc_others
   newton.adi_write_register( 0x0160, 0x0003 ) # ana_test_mux_s1
   newton.adi_write_register( 0x0150, 0x0105 ) # regif_ctrl
   newton.adi_check_register_py( 0x0032, 0x0000 ) # errorStatus
   newton.adi_check_register_py( 0x0150, 0x0101 ) # regif_ctrl
   newton.adi_check_register_py( 0x0032, 0x0000 ) # errorStatus
   newton.adi_check_register_py( 0x0160, 0x0003 ) # ana_test_mux_s1
   newton.adi_write_register( 0x0160, 0x0004 ) # ana_test_mux_s1
   newton.adi_write_register( 0x0150, 0x0105 ) # regif_ctrl
   newton.adi_check_register_py( 0x0032, 0x0000 ) # errorStatus
   newton.adi_check_register_py( 0x0150, 0x0101 ) # regif_ctrl
   newton.adi_check_register_py( 0x0032, 0x0000 ) # errorStatus
   newton.adi_check_register_py( 0x0160, 0x0004 ) # ana_test_mux_s1
   newton.adi_write_register( 0x0160, 0x0005 ) # ana_test_mux_s1
   newton.adi_write_register( 0x0150, 0x0105 ) # regif_ctrl
   newton.adi_check_register_py( 0x0032, 0x0000 ) # errorStatus
   newton.adi_check_register_py( 0x0150, 0x0101 ) # regif_ctrl
   newton.adi_check_register_py( 0x0032, 0x0000 ) # errorStatus
   newton.adi_check_register_py( 0x0160, 0x0005 ) # ana_test_mux_s1
