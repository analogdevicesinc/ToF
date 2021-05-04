#!/usr/bin/env python

""" Script generated from simulation of the fc_asm_mscm_dac test case.


Usage:
    fc_asm_mscm_dac.py [--no_reset]

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
   newton.adi_write_register( 0x0146, 0x007b ) # power_down_adc_others
   newton.adi_write_register( 0x0146, 0x005b ) # power_down_adc_others
   newton.adi_write_register( 0x0146, 0x004b ) # power_down_adc_others
   newton.adi_write_register( 0x0128, 0x83ff ) # dac_ctrl2
   newton.adi_write_register( 0x0128, 0x83ff ) # dac_ctrl2
   newton.adi_write_register( 0x0128, 0x03ff ) # dac_ctrl2
   newton.adi_write_register( 0x0128, 0x0000 ) # dac_ctrl2
   newton.adi_write_register( 0x0128, 0x0000 ) # dac_ctrl2
   newton.adi_write_register( 0x012e, 0x0915 ) # dac_ctrl2_s1
   newton.adi_write_register( 0x0130, 0x8016 ) # dac_ctrl3_s1
   newton.adi_write_register( 0x012e, 0x0000 ) # dac_ctrl2_s1
   newton.adi_write_register( 0x0130, 0x0000 ) # dac_ctrl3_s1
   newton.adi_write_register( 0x0150, 0x0105 ) # regif_ctrl
   newton.adi_check_register_py( 0x0032, 0x0000 ) # errorStatus
   newton.adi_check_register_py( 0x0150, 0x0101 ) # regif_ctrl
   newton.adi_write_register( 0x0132, 0x0000 ) # dac_data
   newton.adi_write_register( 0x0126, 0x0001 ) # dac_ctrl1
   newton.adi_write_register( 0x0132, 0x0000 ) # dac_data
   newton.adi_write_register( 0x0126, 0x0002 ) # dac_ctrl1
   newton.adi_write_register( 0x0132, 0x0000 ) # dac_data
   newton.adi_write_register( 0x0126, 0x0004 ) # dac_ctrl1
   newton.adi_write_register( 0x0132, 0x0000 ) # dac_data
   newton.adi_write_register( 0x0126, 0x0008 ) # dac_ctrl1
   newton.adi_write_register( 0x0132, 0x0000 ) # dac_data
   newton.adi_write_register( 0x0126, 0x0010 ) # dac_ctrl1
   newton.adi_write_register( 0x0132, 0x0000 ) # dac_data
   newton.adi_write_register( 0x0126, 0x0020 ) # dac_ctrl1
   newton.adi_write_register( 0x0132, 0x0000 ) # dac_data
   newton.adi_write_register( 0x0126, 0x0040 ) # dac_ctrl1
   newton.adi_write_register( 0x0132, 0x0000 ) # dac_data
   newton.adi_write_register( 0x0126, 0x0080 ) # dac_ctrl1
   newton.adi_write_register( 0x0132, 0x0000 ) # dac_data
   newton.adi_write_register( 0x0126, 0x0100 ) # dac_ctrl1
   newton.adi_write_register( 0x012a, 0x0000 ) # dac_ctrl0_s1
   newton.adi_write_register( 0x0150, 0x0105 ) # regif_ctrl
   newton.adi_check_register_py( 0x0032, 0x0000 ) # errorStatus
   newton.adi_check_register_py( 0x0150, 0x0101 ) # regif_ctrl
   newton.adi_write_register( 0x012a, 0x0001 ) # dac_ctrl0_s1
   newton.adi_write_register( 0x0150, 0x0105 ) # regif_ctrl
   newton.adi_check_register_py( 0x0032, 0x0000 ) # errorStatus
   newton.adi_check_register_py( 0x0150, 0x0101 ) # regif_ctrl
   newton.adi_write_register( 0x0160, 0x0013 ) # ana_test_mux_s1
   newton.adi_write_register( 0x0150, 0x0105 ) # regif_ctrl
   newton.adi_check_register_py( 0x0032, 0x0000 ) # errorStatus
   newton.adi_check_register_py( 0x0150, 0x0101 ) # regif_ctrl
   newton.adi_check_register_py( 0x0032, 0x0000 ) # errorStatus
   newton.adi_check_register_py( 0x0160, 0x0013 ) # ana_test_mux_s1
   newton.adi_write_register( 0x012a, 0x0002 ) # dac_ctrl0_s1
   newton.adi_write_register( 0x0150, 0x0105 ) # regif_ctrl
   newton.adi_check_register_py( 0x0032, 0x0000 ) # errorStatus
   newton.adi_check_register_py( 0x0150, 0x0101 ) # regif_ctrl
   newton.adi_write_register( 0x012a, 0x0003 ) # dac_ctrl0_s1
   newton.adi_write_register( 0x0150, 0x0105 ) # regif_ctrl
   newton.adi_check_register_py( 0x0032, 0x0000 ) # errorStatus
   newton.adi_check_register_py( 0x0150, 0x0101 ) # regif_ctrl
   newton.adi_write_register( 0x012a, 0x0004 ) # dac_ctrl0_s1
   newton.adi_write_register( 0x0150, 0x0105 ) # regif_ctrl
   newton.adi_check_register_py( 0x0032, 0x0000 ) # errorStatus
   newton.adi_check_register_py( 0x0150, 0x0101 ) # regif_ctrl
   newton.adi_write_register( 0x012a, 0x0005 ) # dac_ctrl0_s1
   newton.adi_write_register( 0x0150, 0x0105 ) # regif_ctrl
   newton.adi_check_register_py( 0x0032, 0x0000 ) # errorStatus
   newton.adi_check_register_py( 0x0150, 0x0101 ) # regif_ctrl
   newton.adi_write_register( 0x012a, 0x0006 ) # dac_ctrl0_s1
   newton.adi_write_register( 0x0150, 0x0105 ) # regif_ctrl
   newton.adi_check_register_py( 0x0032, 0x0000 ) # errorStatus
   newton.adi_check_register_py( 0x0150, 0x0101 ) # regif_ctrl
   newton.adi_write_register( 0x012a, 0x0007 ) # dac_ctrl0_s1
   newton.adi_write_register( 0x0150, 0x0105 ) # regif_ctrl
   newton.adi_check_register_py( 0x0032, 0x0000 ) # errorStatus
   newton.adi_check_register_py( 0x0150, 0x0101 ) # regif_ctrl
   newton.adi_write_register( 0x012a, 0x0008 ) # dac_ctrl0_s1
   newton.adi_write_register( 0x0150, 0x0105 ) # regif_ctrl
   newton.adi_check_register_py( 0x0032, 0x0000 ) # errorStatus
   newton.adi_check_register_py( 0x0150, 0x0101 ) # regif_ctrl
   newton.adi_write_register( 0x012a, 0x0009 ) # dac_ctrl0_s1
   newton.adi_write_register( 0x0150, 0x0105 ) # regif_ctrl
   newton.adi_check_register_py( 0x0032, 0x0000 ) # errorStatus
   newton.adi_check_register_py( 0x0150, 0x0101 ) # regif_ctrl
   newton.adi_write_register( 0x0160, 0x0014 ) # ana_test_mux_s1
   newton.adi_write_register( 0x0150, 0x0105 ) # regif_ctrl
   newton.adi_check_register_py( 0x0032, 0x0000 ) # errorStatus
   newton.adi_check_register_py( 0x0150, 0x0101 ) # regif_ctrl
   newton.adi_check_register_py( 0x0032, 0x0000 ) # errorStatus
   newton.adi_check_register_py( 0x0160, 0x0014 ) # ana_test_mux_s1
   newton.adi_write_register( 0x0160, 0x0015 ) # ana_test_mux_s1
   newton.adi_write_register( 0x0150, 0x0105 ) # regif_ctrl
   newton.adi_check_register_py( 0x0032, 0x0000 ) # errorStatus
   newton.adi_check_register_py( 0x0150, 0x0101 ) # regif_ctrl
   newton.adi_check_register_py( 0x0032, 0x0000 ) # errorStatus
   newton.adi_check_register_py( 0x0160, 0x0015 ) # ana_test_mux_s1
   newton.adi_write_register( 0x0160, 0x0016 ) # ana_test_mux_s1
   newton.adi_write_register( 0x0150, 0x0105 ) # regif_ctrl
   newton.adi_check_register_py( 0x0032, 0x0000 ) # errorStatus
   newton.adi_check_register_py( 0x0150, 0x0101 ) # regif_ctrl
   newton.adi_check_register_py( 0x0032, 0x0000 ) # errorStatus
   newton.adi_check_register_py( 0x0160, 0x0016 ) # ana_test_mux_s1
   newton.adi_write_register( 0x0128, 0x0000 ) # dac_ctrl2
   newton.adi_write_register( 0x012e, 0x42c3 ) # dac_ctrl2_s1
   newton.adi_write_register( 0x0130, 0xf12e ) # dac_ctrl3_s1
   newton.adi_write_register( 0x012e, 0x2020 ) # dac_ctrl2_s1
   newton.adi_write_register( 0x0130, 0x2020 ) # dac_ctrl3_s1
   newton.adi_write_register( 0x0150, 0x0105 ) # regif_ctrl
   newton.adi_check_register_py( 0x0032, 0x0000 ) # errorStatus
   newton.adi_check_register_py( 0x0150, 0x0101 ) # regif_ctrl
   newton.adi_write_register( 0x0132, 0x0080 ) # dac_data
   newton.adi_write_register( 0x0126, 0x0001 ) # dac_ctrl1
   newton.adi_write_register( 0x0132, 0x0080 ) # dac_data
   newton.adi_write_register( 0x0126, 0x0002 ) # dac_ctrl1
   newton.adi_write_register( 0x0132, 0x0080 ) # dac_data
   newton.adi_write_register( 0x0126, 0x0004 ) # dac_ctrl1
   newton.adi_write_register( 0x0132, 0x0080 ) # dac_data
   newton.adi_write_register( 0x0126, 0x0008 ) # dac_ctrl1
   newton.adi_write_register( 0x0132, 0x0080 ) # dac_data
   newton.adi_write_register( 0x0126, 0x0010 ) # dac_ctrl1
   newton.adi_write_register( 0x0132, 0x0080 ) # dac_data
   newton.adi_write_register( 0x0126, 0x0020 ) # dac_ctrl1
   newton.adi_write_register( 0x0132, 0x0080 ) # dac_data
   newton.adi_write_register( 0x0126, 0x0040 ) # dac_ctrl1
   newton.adi_write_register( 0x0132, 0x0080 ) # dac_data
   newton.adi_write_register( 0x0126, 0x0080 ) # dac_ctrl1
   newton.adi_write_register( 0x0132, 0x0080 ) # dac_data
   newton.adi_write_register( 0x0126, 0x0100 ) # dac_ctrl1
   newton.adi_write_register( 0x012a, 0x0000 ) # dac_ctrl0_s1
   newton.adi_write_register( 0x0150, 0x0105 ) # regif_ctrl
   newton.adi_check_register_py( 0x0032, 0x0000 ) # errorStatus
   newton.adi_check_register_py( 0x0150, 0x0101 ) # regif_ctrl
   newton.adi_write_register( 0x012a, 0x0001 ) # dac_ctrl0_s1
   newton.adi_write_register( 0x0150, 0x0105 ) # regif_ctrl
   newton.adi_check_register_py( 0x0032, 0x0000 ) # errorStatus
   newton.adi_check_register_py( 0x0150, 0x0101 ) # regif_ctrl
   newton.adi_write_register( 0x0160, 0x0013 ) # ana_test_mux_s1
   newton.adi_write_register( 0x0150, 0x0105 ) # regif_ctrl
   newton.adi_check_register_py( 0x0032, 0x0000 ) # errorStatus
   newton.adi_check_register_py( 0x0150, 0x0101 ) # regif_ctrl
   newton.adi_check_register_py( 0x0032, 0x0000 ) # errorStatus
   newton.adi_check_register_py( 0x0160, 0x0013 ) # ana_test_mux_s1
   newton.adi_write_register( 0x012a, 0x0002 ) # dac_ctrl0_s1
   newton.adi_write_register( 0x0150, 0x0105 ) # regif_ctrl
   newton.adi_check_register_py( 0x0032, 0x0000 ) # errorStatus
   newton.adi_check_register_py( 0x0150, 0x0101 ) # regif_ctrl
   newton.adi_write_register( 0x012a, 0x0003 ) # dac_ctrl0_s1
   newton.adi_write_register( 0x0150, 0x0105 ) # regif_ctrl
   newton.adi_check_register_py( 0x0032, 0x0000 ) # errorStatus
   newton.adi_check_register_py( 0x0150, 0x0101 ) # regif_ctrl
   newton.adi_write_register( 0x012a, 0x0004 ) # dac_ctrl0_s1
   newton.adi_write_register( 0x0150, 0x0105 ) # regif_ctrl
   newton.adi_check_register_py( 0x0032, 0x0000 ) # errorStatus
   newton.adi_check_register_py( 0x0150, 0x0101 ) # regif_ctrl
   newton.adi_write_register( 0x012a, 0x0005 ) # dac_ctrl0_s1
   newton.adi_write_register( 0x0150, 0x0105 ) # regif_ctrl
   newton.adi_check_register_py( 0x0032, 0x0000 ) # errorStatus
   newton.adi_check_register_py( 0x0150, 0x0101 ) # regif_ctrl
   newton.adi_write_register( 0x012a, 0x0006 ) # dac_ctrl0_s1
   newton.adi_write_register( 0x0150, 0x0105 ) # regif_ctrl
   newton.adi_check_register_py( 0x0032, 0x0000 ) # errorStatus
   newton.adi_check_register_py( 0x0150, 0x0101 ) # regif_ctrl
   newton.adi_write_register( 0x012a, 0x0007 ) # dac_ctrl0_s1
   newton.adi_write_register( 0x0150, 0x0105 ) # regif_ctrl
   newton.adi_check_register_py( 0x0032, 0x0000 ) # errorStatus
   newton.adi_check_register_py( 0x0150, 0x0101 ) # regif_ctrl
   newton.adi_write_register( 0x012a, 0x0008 ) # dac_ctrl0_s1
   newton.adi_write_register( 0x0150, 0x0105 ) # regif_ctrl
   newton.adi_check_register_py( 0x0032, 0x0000 ) # errorStatus
   newton.adi_check_register_py( 0x0150, 0x0101 ) # regif_ctrl
   newton.adi_write_register( 0x012a, 0x0009 ) # dac_ctrl0_s1
   newton.adi_write_register( 0x0150, 0x0105 ) # regif_ctrl
   newton.adi_check_register_py( 0x0032, 0x0000 ) # errorStatus
   newton.adi_check_register_py( 0x0150, 0x0101 ) # regif_ctrl
   newton.adi_write_register( 0x0160, 0x0014 ) # ana_test_mux_s1
   newton.adi_write_register( 0x0150, 0x0105 ) # regif_ctrl
   newton.adi_check_register_py( 0x0032, 0x0000 ) # errorStatus
   newton.adi_check_register_py( 0x0150, 0x0101 ) # regif_ctrl
   newton.adi_check_register_py( 0x0032, 0x0000 ) # errorStatus
   newton.adi_check_register_py( 0x0160, 0x0014 ) # ana_test_mux_s1
   newton.adi_write_register( 0x0160, 0x0015 ) # ana_test_mux_s1
   newton.adi_write_register( 0x0150, 0x0105 ) # regif_ctrl
   newton.adi_check_register_py( 0x0032, 0x0000 ) # errorStatus
   newton.adi_check_register_py( 0x0150, 0x0101 ) # regif_ctrl
   newton.adi_check_register_py( 0x0032, 0x0000 ) # errorStatus
   newton.adi_check_register_py( 0x0160, 0x0015 ) # ana_test_mux_s1
   newton.adi_write_register( 0x0160, 0x0016 ) # ana_test_mux_s1
   newton.adi_write_register( 0x0150, 0x0105 ) # regif_ctrl
   newton.adi_check_register_py( 0x0032, 0x0000 ) # errorStatus
   newton.adi_check_register_py( 0x0150, 0x0101 ) # regif_ctrl
   newton.adi_check_register_py( 0x0032, 0x0000 ) # errorStatus
   newton.adi_check_register_py( 0x0160, 0x0016 ) # ana_test_mux_s1
   newton.adi_write_register( 0x0128, 0x0000 ) # dac_ctrl2
   newton.adi_write_register( 0x012e, 0x8a00 ) # dac_ctrl2_s1
   newton.adi_write_register( 0x0130, 0xe0c0 ) # dac_ctrl3_s1
   newton.adi_write_register( 0x012e, 0x3f3f ) # dac_ctrl2_s1
   newton.adi_write_register( 0x0130, 0x3f3f ) # dac_ctrl3_s1
   newton.adi_write_register( 0x0150, 0x0105 ) # regif_ctrl
   newton.adi_check_register_py( 0x0032, 0x0000 ) # errorStatus
   newton.adi_check_register_py( 0x0150, 0x0101 ) # regif_ctrl
   newton.adi_write_register( 0x0132, 0x00ff ) # dac_data
   newton.adi_write_register( 0x0126, 0x0001 ) # dac_ctrl1
   newton.adi_write_register( 0x0132, 0x00ff ) # dac_data
   newton.adi_write_register( 0x0126, 0x0002 ) # dac_ctrl1
   newton.adi_write_register( 0x0132, 0x00ff ) # dac_data
   newton.adi_write_register( 0x0126, 0x0004 ) # dac_ctrl1
   newton.adi_write_register( 0x0132, 0x00ff ) # dac_data
   newton.adi_write_register( 0x0126, 0x0008 ) # dac_ctrl1
   newton.adi_write_register( 0x0132, 0x00ff ) # dac_data
   newton.adi_write_register( 0x0126, 0x0010 ) # dac_ctrl1
   newton.adi_write_register( 0x0132, 0x00ff ) # dac_data
   newton.adi_write_register( 0x0126, 0x0020 ) # dac_ctrl1
   newton.adi_write_register( 0x0132, 0x00ff ) # dac_data
   newton.adi_write_register( 0x0126, 0x0040 ) # dac_ctrl1
   newton.adi_write_register( 0x0132, 0x00ff ) # dac_data
   newton.adi_write_register( 0x0126, 0x0080 ) # dac_ctrl1
   newton.adi_write_register( 0x0132, 0x00ff ) # dac_data
   newton.adi_write_register( 0x0126, 0x0100 ) # dac_ctrl1
   newton.adi_write_register( 0x012a, 0x0000 ) # dac_ctrl0_s1
   newton.adi_write_register( 0x0150, 0x0105 ) # regif_ctrl
   newton.adi_check_register_py( 0x0032, 0x0000 ) # errorStatus
   newton.adi_check_register_py( 0x0150, 0x0101 ) # regif_ctrl
   newton.adi_write_register( 0x012a, 0x0001 ) # dac_ctrl0_s1
   newton.adi_write_register( 0x0150, 0x0105 ) # regif_ctrl
   newton.adi_check_register_py( 0x0032, 0x0000 ) # errorStatus
   newton.adi_check_register_py( 0x0150, 0x0101 ) # regif_ctrl
   newton.adi_write_register( 0x0160, 0x0013 ) # ana_test_mux_s1
   newton.adi_write_register( 0x0150, 0x0105 ) # regif_ctrl
   newton.adi_check_register_py( 0x0032, 0x0000 ) # errorStatus
   newton.adi_check_register_py( 0x0150, 0x0101 ) # regif_ctrl
   newton.adi_check_register_py( 0x0032, 0x0000 ) # errorStatus
   newton.adi_check_register_py( 0x0160, 0x0013 ) # ana_test_mux_s1
   newton.adi_write_register( 0x012a, 0x0002 ) # dac_ctrl0_s1
   newton.adi_write_register( 0x0150, 0x0105 ) # regif_ctrl
   newton.adi_check_register_py( 0x0032, 0x0000 ) # errorStatus
   newton.adi_check_register_py( 0x0150, 0x0101 ) # regif_ctrl
   newton.adi_write_register( 0x012a, 0x0003 ) # dac_ctrl0_s1
   newton.adi_write_register( 0x0150, 0x0105 ) # regif_ctrl
   newton.adi_check_register_py( 0x0032, 0x0000 ) # errorStatus
   newton.adi_check_register_py( 0x0150, 0x0101 ) # regif_ctrl
   newton.adi_write_register( 0x012a, 0x0004 ) # dac_ctrl0_s1
   newton.adi_write_register( 0x0150, 0x0105 ) # regif_ctrl
   newton.adi_check_register_py( 0x0032, 0x0000 ) # errorStatus
   newton.adi_check_register_py( 0x0150, 0x0101 ) # regif_ctrl
   newton.adi_write_register( 0x012a, 0x0005 ) # dac_ctrl0_s1
   newton.adi_write_register( 0x0150, 0x0105 ) # regif_ctrl
   newton.adi_check_register_py( 0x0032, 0x0000 ) # errorStatus
   newton.adi_check_register_py( 0x0150, 0x0101 ) # regif_ctrl
   newton.adi_write_register( 0x012a, 0x0006 ) # dac_ctrl0_s1
   newton.adi_write_register( 0x0150, 0x0105 ) # regif_ctrl
   newton.adi_check_register_py( 0x0032, 0x0000 ) # errorStatus
   newton.adi_check_register_py( 0x0150, 0x0101 ) # regif_ctrl
   newton.adi_write_register( 0x012a, 0x0007 ) # dac_ctrl0_s1
   newton.adi_write_register( 0x0150, 0x0105 ) # regif_ctrl
   newton.adi_check_register_py( 0x0032, 0x0000 ) # errorStatus
   newton.adi_check_register_py( 0x0150, 0x0101 ) # regif_ctrl
   newton.adi_write_register( 0x012a, 0x0008 ) # dac_ctrl0_s1
   newton.adi_write_register( 0x0150, 0x0105 ) # regif_ctrl
   newton.adi_check_register_py( 0x0032, 0x0000 ) # errorStatus
   newton.adi_check_register_py( 0x0150, 0x0101 ) # regif_ctrl
   newton.adi_write_register( 0x012a, 0x0009 ) # dac_ctrl0_s1
   newton.adi_write_register( 0x0150, 0x0105 ) # regif_ctrl
   newton.adi_check_register_py( 0x0032, 0x0000 ) # errorStatus
   newton.adi_check_register_py( 0x0150, 0x0101 ) # regif_ctrl
   newton.adi_write_register( 0x0160, 0x0014 ) # ana_test_mux_s1
   newton.adi_write_register( 0x0150, 0x0105 ) # regif_ctrl
   newton.adi_check_register_py( 0x0032, 0x0000 ) # errorStatus
   newton.adi_check_register_py( 0x0150, 0x0101 ) # regif_ctrl
   newton.adi_check_register_py( 0x0032, 0x0000 ) # errorStatus
   newton.adi_check_register_py( 0x0160, 0x0014 ) # ana_test_mux_s1
   newton.adi_write_register( 0x0160, 0x0015 ) # ana_test_mux_s1
   newton.adi_write_register( 0x0150, 0x0105 ) # regif_ctrl
   newton.adi_check_register_py( 0x0032, 0x0000 ) # errorStatus
   newton.adi_check_register_py( 0x0150, 0x0101 ) # regif_ctrl
   newton.adi_check_register_py( 0x0032, 0x0000 ) # errorStatus
   newton.adi_check_register_py( 0x0160, 0x0015 ) # ana_test_mux_s1
   newton.adi_write_register( 0x0160, 0x0016 ) # ana_test_mux_s1
   newton.adi_write_register( 0x0150, 0x0105 ) # regif_ctrl
   newton.adi_check_register_py( 0x0032, 0x0000 ) # errorStatus
   newton.adi_check_register_py( 0x0150, 0x0101 ) # regif_ctrl
   newton.adi_check_register_py( 0x0032, 0x0000 ) # errorStatus
   newton.adi_check_register_py( 0x0160, 0x0016 ) # ana_test_mux_s1
