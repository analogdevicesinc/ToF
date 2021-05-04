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

   # adi_write_register( u16 addr, u16 wr_data )

   # adi_send_command( CMD_REGISTER_CFG, 0, 2, WRITE_ATTR );
   newton.adi_spi_write_word( 0x8004, 0x0000 ) # ADI_S2H_MBX_FIFO_PUSH, Address (not used)
   newton.adi_spi_write_word( 0x8004, 0x1009 ) # ADI_S2H_MBX_FIFO_PUSH, CMD_REGISTER_CFG
   newton.adi_spi_write_word( 0x8004, 0x0001 ) # ADI_S2H_MBX_FIFO_PUSH, WRITE_ATTR
   newton.adi_spi_write_word( 0x8004, 0x0004 ) # ADI_S2H_MBX_FIFO_PUSH, Byte Count

   # Check number of entries in FIFO
   readData = newton.adi_spi_read_word_py( 0x0002 ) # ADI_S2H_MBX_STS
   print( "INFO: ADI_S2H_MBX_STS = " + hex ( readData ) )

   fifo_cnt = (readData & 0xFF00) >> 8
   if fifo_cnt != 2:
      print( "ERROR: incorrect fifo_cnt, expect = 2, actual = " + str( fifo_cnt ) )

   # Set s2h_valid
   newton.adi_spi_write_word( 0x8002, 0x0004 ) # ADI_S2H_MBX_STS
   # adi_wait_for_s2h_not_valid
   readData = newton.adi_spi_read_word_py( 0x0002 ) # ADI_S2H_MBX_STS

   print( "INFO: ADI_S2H_MBX_STS = " + hex ( readData ) )

   valid = (readData & 0x0004) >> 2
   if valid != 0:
      print( "ERROR: incorrect valid, expect = 0, actual = " + str( valid ) )

   # adi_send_write_register_list: word_count=2
   newton.adi_spi_write_word( 0x8004, 0x1111 ) # ADI_S2H_MBX_FIFO_PUSH, data
   newton.adi_spi_write_word( 0x8004, 0x0060 ) # ADI_S2H_MBX_FIFO_PUSH, address

   # Check entries in FIFO
   
   # Set s2h_valid
   newton.adi_spi_write_word( 0x8002, 0x0004 ) # ADI_S2H_MBX_STS
   # adi_wait_for_s2h_not_valid
   readData = newton.adi_spi_read_word_py( 0x0002 ) # ADI_S2H_MBX_STS

   print( "INFO: ADI_S2H_MBX_STS = " + hex ( readData ) )

   valid = (readData & 0x0004) >> 2
   if valid != 0:
      print( "ERROR: incorrect valid, expect = 0, actual = " + str( valid ) )

   # adi_read_register( u16 addr )

   # adi_send_command( CMD_REGISTER_CFG, 0, 2, WRITE_ATTR );
   newton.adi_spi_write_word( 0x8004, 0x0000 ) # ADI_S2H_MBX_FIFO_PUSH, Address (not used)
   newton.adi_spi_write_word( 0x8004, 0x1009 ) # ADI_S2H_MBX_FIFO_PUSH, CMD_REGISTER_CFG
   newton.adi_spi_write_word( 0x8004, 0x0000 ) # ADI_S2H_MBX_FIFO_PUSH, no attributes
   newton.adi_spi_write_word( 0x8004, 0x0004 ) # ADI_S2H_MBX_FIFO_PUSH, Byte Count

   # Check number of entries in FIFO
   readData = newton.adi_spi_read_word_py( 0x0002 ) # ADI_S2H_MBX_STS
   print( "INFO: ADI_S2H_MBX_STS = " + hex ( readData ) )

   fifo_cnt = (readData & 0xFF00) >> 8
   if fifo_cnt != 2:
      print( "ERROR: incorrect fifo_cnt, expect = 2, actual = " + str( fifo_cnt ) )

   # Set s2h_valid
   newton.adi_spi_write_word( 0x8002, 0x0004 ) # ADI_S2H_MBX_STS
   # adi_wait_for_s2h_not_valid
   readData = newton.adi_spi_read_word_py( 0x0002 ) # ADI_S2H_MBX_STS

   print( "INFO: ADI_S2H_MBX_STS = " + hex ( readData ) )

   valid = (readData & 0x0004) >> 2
   if valid != 0:
      print( "ERROR: incorrect valid, expect = 0, actual = " + str( valid ) )

   # adi_send_write_register_list: word_count=2
   newton.adi_spi_write_word( 0x8004, 0x0060 ) # ADI_S2H_MBX_FIFO_PUSH, data
   newton.adi_spi_write_word( 0x8004, 0xffff ) # ADI_S2H_MBX_FIFO_PUSH, address

   # Check entries in FIFO
   
   # Set s2h_valid
   newton.adi_spi_write_word( 0x8002, 0x0004 ) # ADI_S2H_MBX_STS
   # adi_wait_for_s2h_not_valid
   readData = newton.adi_spi_read_word_py( 0x0002 ) # ADI_S2H_MBX_STS

   # adi_get_data( 1, readData );

   readData = newton.adi_spi_read_word_py( 0x000e ) # ADI_H2S_MBX_FIFO_POP
   print( "INFO: readData = " + hex( readData ) )

   # adi_clear_h2s_valid( );
   newton.adi_spi_write_word( 0x800a, 0x0004 ) # 

