#!/usr/bin/env python

import sys
import os
import subprocess
import re
import tempfile

def usage( ):
    print( "Usage:" )
    print( "    build.py project buildSet projectsDir workspaceDir" )

def main( ):

    if( len( sys.argv ) < 2):
        print( "Not enough arguments specified." )
        usage()
        sys.exit()
    else:
        inputFileName = sys.argv[1]
        outputFileName = sys.argv[2]

    state = "IDLE"
    
    ifile = open( inputFileName, "r" )
    lines = ifile.readlines()
    ifile.close()

    ofile = open( outputFileName, "w" )

    for i in range( len( lines ) ):
        line = lines[i]

        if state == "IDLE":
            if re.match( r'^\s*#ifdef\sSWIG', line ):
                state = "IFDEF_SWIG"
            elif re.match( r'^\s*#ifndef\sSWIG', line ):
                state = "IFNDEF_SWIG"
            else:
                ofile.write( line ) 
        elif state == "IFDEF_SWIG":
            if re.match( r'^\s*#endif', line ):
                state = "IDLE"
            else:
                ofile.write( line ) 
        elif state == "IFNDEF_SWIG":
            if re.match( r'^\s*#endif', line ):
                state = "IDLE"

    ofile.close()

if __name__ == "__main__":
    main( )

