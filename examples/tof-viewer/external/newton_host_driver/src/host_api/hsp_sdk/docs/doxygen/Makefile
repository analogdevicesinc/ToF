#------------------------------------------------------------------------------
# Project            : HSP API
# File Name          : Makefile
# Original Author    : wpeet
# Description        : Makefile to run doxygen
#------------------------------------------------------------------------------
# SVN Revision       : $Rev: 3434 $
# Last Commit Date   : $Date: 2015-07-14 15:18:23 -0400 (Tue, 14 Jul 2015) $
# Last Commit Author : $Author: wpeet $
#------------------------------------------------------------------------------
# Copyright (c) 2020 by ADI. This code is the confidential and
# proprietary property of ADI and the possession or use of this
# file requires a written license from ADI.
#------------------------------------------------------------------------------

DocsDir  = .
Config   = $(DocsDir)/hspapi.cfg
DocsLog  = $(DocsDir)/externalDocs.log

# Targets
.PHONY: default
default: all

all : external_docs

external_docs : 
	doxygen $(Config)

clean :
	rm -fr $(DocsDir)/html
	rm -fr $(DocsLog)

.PHONY : all mkdirs clean

help :
	@echo Available options:
	@echo     "make        -- generate documentation"
	@echo     "make clean  -- remove documentation build directory"
