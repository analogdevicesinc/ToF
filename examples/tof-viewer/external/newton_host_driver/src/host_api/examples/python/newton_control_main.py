#!/usr/bin/env python

""" Newton Host API Implementation

Usage:
    newton_control_main.py load <target> <file_name>
    newton_control_main.py verify <target> <file_name>
    newton_control_main.py unload <target> <file_name>
    newton_control_main.py spi_write <address> <data>
    newton_control_main.py spi_read <address>
    newton_control_main.py reg_write <address> <data>
    newton_control_main.py reg_write_backdoor <address> <data>
    newton_control_main.py reg_read <address>
    newton_control_main.py reg_read_backdoor <address>
    newton_control_main.py reg_dump <address> <word_count>
    newton_control_main.py reg_fill <address> <data> <word_count>
    newton_control_main.py reset_hsp
    newton_control_main.py gen_grouped_cmd <file_name>

Options:
    -h --help Shows this help message.
Where:
    load               : Loads the contents of the specified file into the target memory.
    verify             : Compares the contents of the specified file to the contents of the target memory.
    unload             : Dumps the content of the target memory to the specified file.
    spi_write          : Write data to the address. For accessing HSP MBOX registers.
    spi_read           : Read data at address. For accessing HSP MBOX registers.
    reg_write          : Write data to the Newton register at address
    reg_write_backdoor : Write data to the Newton register at address
    reg_read           : Read the Newton register at address
    reg_read_backdoor  : Read the Newton register at address
    reg_dump           : Read and display word_count Newton registers starting at address.
    reg_fill           : Write word_count word_count Newton registers with data starting at address.
    reset_hsp          : Reset the HSP hardware
    gen_grouped_cmd    : Generate group commmand file
target is one of the following:
    useq_seq_ram  : Microsequencer Sequence RAM
    useq_map_ram  : Microsequencer MAP RAM
    useq_wave_ram : Microsequencer Wave RAM
    datapath_ram  : Gain Correction RAM
    de_ram        : Dump Engine RAM
    lps1_ram      : LPS1 
    lps2_ram      : LPS2 
    hsp_rom       : HSP ROM (FPGA only)
    hsp_ram       : HSP RAM (FPGA only)
    efuse         : eFuse   (FPGA only)
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
import json
from collections import OrderedDict
import threading
import newton_control as newton

def generateGroupedCommand( file_name ):
   linkPath = str( Path(htmlFile).resolve() )
   linkPath = re.sub( r"/adsim_rml_top.html", r"", linkPath )
   for (root,dirs,files) in os.walk( linkPath ): 
      for file in files: 
         fileName = root + "/" + file
         if re.search( r'.html$', fileName ):
            fixHtmlReport( fileName )

if __name__ == "__main__":
    maxSpiBytes = 256

    rc = newton.adi_newton_config( 0 )
    if rc != newton.ADI_NO_ERROR:
        print( "ERROR: Error \"" + newton.adi_error_msg( rc ) + "\" returned from newton.adi_newton_config." )
        sys.exit( rc )

    args = docopt(__doc__, version='0.1')

    if args['load'] == True:
        if args['<target>'] == "hsp_rom":
            rc = newton.adi_load_hsp( newton.HSP_ROM, args['<file_name>'] )
            if rc != newton.ADI_NO_ERROR:
                print( "ERROR: Error \"" + newton.adi_error_msg( rc ) + "\" returned from newton.adi_load_hsp." )
                sys.exit( rc )
        elif args['<target>'] == "hsp_ram":
            rc = newton.adi_load_hsp( newton.HSP_RAM, args['<file_name>'] )
            if rc != newton.ADI_NO_ERROR:
                print( "ERROR: Error \"" + newton.adi_error_msg( rc ) + "\" returned from newton.adi_load_hsp." )
                sys.exit( rc )
        elif args['<target>'] == "efuse":
            rc = newton.adi_load_hsp( newton.EFUSE, args['<file_name>'] )
            if rc != newton.ADI_NO_ERROR:
                print( "ERROR: Error \"" + newton.adi_error_msg( rc ) + "\" returned from newton.adi_load_hsp." )
                sys.exit( rc )

    if args['verify'] == True:
        if args['<target>'] == "hsp_rom":
            rc = newton.adi_verify_hsp( newton.HSP_ROM, args['<file_name>'] )
            if rc != newton.ADI_NO_ERROR:
                print( "ERROR: Error \"" + newton.adi_error_msg( rc ) + "\" returned from newton.adi_verify_hsp." )
                sys.exit( rc )
        elif args['<target>'] == "hsp_ram":
            rc = newton.adi_verify_hsp( newton.HSP_RAM, args['<file_name>'] )
            if rc != newton.ADI_NO_ERROR:
                print( "ERROR: Error \"" + newton.adi_error_msg( rc ) + "\" returned from newton.adi_verify_hsp." )
                sys.exit( rc )
        elif args['<target>'] == "efuse":
            rc = newton.adi_verify_hsp( newton.EFUSE, args['<file_name>'] )
            if rc != newton.ADI_NO_ERROR:
                print( "ERROR: Error \"" + newton.adi_error_msg( rc ) + "\" returned from newton.adi_verify_hsp." )
                sys.exit( rc )

    if args['unload'] == True:
        if args['<target>'] == "hsp_rom":
            rc = newton.adi_unload_hsp( newton.HSP_ROM, args['<file_name>'] )
            if rc != newton.ADI_NO_ERROR:
                print( "ERROR: Error \"" + newton.adi_error_msg( rc ) + "\" returned from newton.adi_unload_hsp." )
                sys.exit( rc )
        elif args['<target>'] == "hsp_ram":
            rc = newton.adi_unload_hsp( newton.HSP_RAM, args['<file_name>'] )
            if rc != newton.ADI_NO_ERROR:
                print( "ERROR: Error \"" + newton.adi_error_msg( rc ) + "\" returned from newton.adi_unload_hsp." )
                sys.exit( rc )
        elif args['<target>'] == "efuse":
            rc = newton.adi_unload_hsp( newton.EFUSE, args['<file_name>'] )
            if rc != newton.ADI_NO_ERROR:
                print( "ERROR: Error \"" + newton.adi_error_msg( rc ) + "\" returned from newton.adi_unload_hsp." )
                sys.exit( rc )

    if args['spi_write'] == True:
        rc = newton.adi_spi_write_word( int(args['<address>'], 0), int(args['<data>'], 0) )
        if rc != newton.ADI_NO_ERROR:
            print( "ERROR: Error \"" + newton.adi_error_msg( rc ) + "\" returned from newton.adi_spi_write_word." )
            sys.exit( rc )

    if args['reg_write'] == True:
        rc = newton.adi_write_register( int(args['<address>'], 0), int(args['<data>'], 0) )
        if rc != newton.ADI_NO_ERROR:
            print( "ERROR: Error \"" + newton.adi_error_msg( rc ) + "\" returned from newton.adi_spi_write_word." )
            sys.exit( rc )

    if args['spi_read'] == True:
        rc, readData = newton.adi_spi_read_word( int(args['<address>'], 0) )
        if rc != newton.ADI_NO_ERROR:
            print( "ERROR: Error \"" + newton.adi_error_msg( rc ) + "\" returned from newton.adi_spi_read_word." )
            sys.exit( rc )
        print( hex(readData) )

    if args['reg_read'] == True:
        rc, readData = newton.adi_read_register( int(args['<address>'], 0) )
        if rc != newton.ADI_NO_ERROR:
            print( "ERROR: Error \"" + newton.adi_error_msg( rc ) + "\" returned from newton.adi_spi_read_word." )
            sys.exit( rc )
        print( hex(readData) )

    if args['reg_dump'] == True:
        rc = newton.adi_reg_dump( int(args['<address>'], 0), int(args['<word_count>'], 0) )
        if rc != newton.ADI_NO_ERROR:
            print( "ERROR: Error \"" + newton.adi_error_msg( rc ) + "\" returned from newton.adi_spi_write_word." )
            sys.exit( rc )

    if args['reg_fill'] == True:
        rc = newton.adi_reg_fill( int(args['<address>'], 0), int(args['<data>'], 0), int(args['<word_count>'], 0) )
        if rc != newton.ADI_NO_ERROR:
            print( "ERROR: Error \"" + newton.adi_error_msg( rc ) + "\" returned from newton.adi_spi_write_word." )
            sys.exit( rc )

    if args['reset_hsp'] == True:
        rc = newton.adi_reset_hsp( )
        if rc != newton.ADI_NO_ERROR:
            print( "ERROR: Error \"" + newton.adi_error_msg( rc ) + "\" returned from newton.adi_reset_hsp." )
            sys.exit( rc )

    if args['gen_grouped_cmd'] == True:
        rc = generateGroupedCommand( int(args['<file_name>'], 0) )
        if rc != newton.ADI_NO_ERROR:
            print( "ERROR: The gen_grouped_cmd returned and error return code." )
            sys.exit( rc )

