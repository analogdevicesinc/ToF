#!/usr/bin/env python

""" JTAG Mailbox script

Usage:
    jtag_mailbox.py blank2test
    jtag_mailbox.py test2prod
    jtag_mailbox.py test2retest
    jtag_mailbox.py read_state
    jtag_mailbox.py idcode

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
import newton_control as newton

def blank2test( command ):
   if newtonTarget == "FPGA":
      efuse_file = os.path.expanduser( "~/host_api/efuse/fuse_blank.vhx" )
      efuse_file_bytes = efuse_file.encode(encoding='utf-8')
      rc = newton.adi_load_hsp( newton.EFUSE, efuse_file_bytes )
      if rc != 0:
         print( "ERROR: newton_control.exe return an error (" + str( rc ) + ")." )
         sys.exit( rc )

      print( "INFO: Resetting the HSP ..." )
      rc = newton.adi_reset_newton( newton.PIN_MODE_HSP_DEBUG )
      if rc != 0:
         print( "ERROR: newton_control.exe return an error (" + str( rc ) + ")." )
         sys.exit( rc )

   rc = os.system( "~/host_api/examples/c/linux/jtag_mailbox.exe " + command )
   if rc != 0:
      print( "ERROR: newton_control.exe return an error (" + str( rc ) + ")." )

def test2prod( command ):
   if newtonTarget == "FPGA":
      efuse_file = os.path.expanduser( "~/host_api/efuse/fuse_test.vhx" )
      efuse_file_bytes = efuse_file.encode(encoding='utf-8')
      rc = newton.adi_load_hsp( newton.EFUSE, efuse_file_bytes )
      if rc != 0:
         print( "ERROR: newton_control.exe return an error (" + str( rc ) + ")." )
         sys.exit( rc )

      print( "INFO: Resetting the HSP ..." )
      rc = newton.adi_reset_newton( newton.PIN_MODE_HSP_DEBUG )
      if rc != 0:
         print( "ERROR: newton_control.exe return an error (" + str( rc ) + ")." )
         sys.exit( rc )

   rc = os.system( "~/host_api/examples/c/linux/jtag_mailbox.exe " + command )
   if rc != 0:
      print( "ERROR: newton_control.exe return an error (" + str( rc ) + ")." )

def test2retest( command ):
   if newtonTarget == "FPGA":
      efuse_file = os.path.expanduser( "~/host_api/efuse/fuse_test.vhx" )
      efuse_file_bytes = efuse_file.encode(encoding='utf-8')
      rc = newton.adi_load_hsp( newton.EFUSE, efuse_file_bytes )
      if rc != 0:
         print( "ERROR: newton_control.exe return an error (" + str( rc ) + ")." )
         sys.exit( rc )

      print( "INFO: Resetting the HSP ..." )
      rc = newton.adi_reset_newton( newton.PIN_MODE_HSP_DEBUG )
      if rc != 0:
         print( "ERROR: newton_control.exe return an error (" + str( rc ) + ")." )
         sys.exit( rc )

   rc = os.system( "~/host_api/examples/c/linux/jtag_mailbox.exe " + command )
   if rc != 0:
      print( "ERROR: newton_control.exe return an error (" + str( rc ) + ")." )

def read_state( command ):
   rc = os.system( "~/host_api/examples/c/linux/jtag_mailbox.exe " + command )
   if rc != 0:
      print( "ERROR: newton_control.exe return an error (" + str( rc ) + ")." )

def idcode( command ):
   rc = os.system( "~/host_api/examples/c/linux/jtag_mailbox.exe " + command )
   if rc != 0:
      print( "ERROR: newton_control.exe return an error (" + str( rc ) + ")." )

if __name__ == "__main__":
   global newtonTarget
   
   args = docopt(__doc__, version='0.1')

   newtonTarget = os.environ["NEWTON_TARGET"]
   
   rc = newton.adi_newton_config( 0 )
   if rc != 0:
      print( "ERROR: newton.adi_newton_config return an error (" + str( rc ) + ")." )
      sys.exit( rc )

   if args['blank2test']:
      blank2test( "blank2test" )
   elif args['test2prod']:
      test2prod( "test2prod" )
   elif args['test2retest']:
      test2retest( "test2retest" )
   elif args['read_state']:
      read_state( "read_state" )
   elif args['idcode']:
      idcode( "idcode" )
   else:
      print( "ERROR: missing command argument." )

