#!/usr/bin/env python

""" Test load reg file

Usage:
    test_load_reg.py <file_name>

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

   fileName = args['<file_name>']

   rc = newton.adi_newton_config( 0 )
   if rc != 0:
      print( "ERROR: newton.adi_newton_config return an error (" + str( rc ) + ")." )
      sys.exit( rc )

   rc = newton.adi_load_register_file( fileName )
   if rc != 0:
      print( "ERROR: newton.adi_load_register_file return an error (" + str( rc ) + ")." )
      sys.exit( rc )

   write_data  = 0x7900
   expect_data = 0x7919
   read_data = newton.adi_read_register_py( 0x0014 )
   if read_data != expect_data:
       print( "ERROR: miscompare: actual = " + hex( read_data ) + " , expected = " + hex ( write_data ) )

   write_data  = 0x00F3
   expect_data = 0x00F3
   read_data = newton.adi_read_register_py( 0x0104 )
   if read_data != write_data:
       print( "ERROR: miscompare: actual = " + hex( read_data ) + " , expected = " + hex ( write_data ) )

   write_data  = 0x007B
   expect_data = 0x007B
   read_data = newton.adi_read_register_py( 0x0102 )
   if read_data != write_data:
       print( "ERROR: miscompare: actual = " + hex( read_data ) + " , expected = " + hex ( write_data ) )

   write_data  = 0x181F
   expect_data = 0x181F
   read_data = newton.adi_read_register_py( 0x012E )
   if read_data != write_data:
       print( "ERROR: miscompare: actual = " + hex( read_data ) + " , expected = " + hex ( write_data ) )

   write_data  = 0x2019
   expect_data = 0x2019
   read_data = newton.adi_read_register_py( 0x0130 )
   if read_data != write_data:
       print( "ERROR: miscompare: actual = " + hex( read_data ) + " , expected = " + hex ( write_data ) )
