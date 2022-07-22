#!/usr/bin/env python

""" Write and read register

Usage:
    reg_write_read.py <address> <write_data>

Options:
    --help Shows this help message.
"""

from __future__ import print_function
from __future__ import absolute_import
from __future__ import unicode_literals

from docopt import docopt
import struct
import ctypes
from newton_control_main import newton as newton

if __name__ == "__main__":
   args = docopt(__doc__, version='0.1')

   address    = int( args['<address>'], 16 )
   write_data = int( args['<write_data>'], 16 )

   rc = newton.adi_newton_config( 0 )
   if rc != 0:
      print( "ERROR: newton.adi_newton_config return an error (" + str( rc ) + ")." )
      sys.exit( rc )

   newton.adi_write_register( address, write_data )
   read_data = newton.adi_read_register_py( address )
   if read_data != write_data:
       print( "ERROR: miscompare: actual = " + hex( read_data ) + " , expected = " + hex ( write_data ) )


