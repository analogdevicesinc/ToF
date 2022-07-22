#!/usr/bin/env python

""" Python script esample

Usage:
    test.py 

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

   writeData = 0x1111
   rc = newton.adi_write_register( 0x0060, writeData )
   if rc != newton.ADI_NO_ERROR:
      print( "ERROR: Error \"" + newton.adi_error_msg( rc ) + "\" returned from newton.adi_write_register." )
      sys.exit( rc )

   readData = newton.adi_read_register_py( 0x0060 )
   if rc != newton.ADI_NO_ERROR:
      print( "ERROR: Error \"" + newton.adi_error_msg( rc ) + "\" returned from newton.adi_read_register." )
      sys.exit( rc )
   if readData != writeData:
      print( "ERROR: miscompare: expect = " + hex( writeData ) + ", actual = " + hex( readData ) )


