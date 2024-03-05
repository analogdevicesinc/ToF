#
# BSD 3-Clause License
#
# Copyright (c) 2019, Analog Devices, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
import subprocess
import pytest
import os.path
import numpy as np
import logging
import os
import re

Logger = logging.getLogger(__name__)

test_filename = "trial"

def test_saveCCBCFG(ip_set, config_file):

    ccb_file_extension = ".ccb"
    cfg_file_extension = ".cfg"
    
    #delete all ccb files in current directiry
    files_to_delete = [fname for fname in os.listdir('.') if fname.endswith(ccb_file_extension)]
    # Delete each file
    for file_name in files_to_delete:
        try:
            os.remove(file_name)
        except OSError:
            print(f"Error deleting: {file_name}")
            
    #delete all cfg files in current directiry
    files_to_delete = [fname for fname in os.listdir('.') if fname.endswith(cfg_file_extension)]
    # Delete each file
    for file_name in files_to_delete:
        try:
            os.remove(file_name)
        except OSError:
            print(f"Error deleting: {file_name}")
        
    #Run the exe file
    exe_path = "../../build/examples/test_saveCCBCFG/release/test_saveCCBCFG.exe"
    process = subprocess.run([exe_path,  ip_set, config_file, test_filename], 
        text=True,stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        
    assert process.returncode == 0
    print(process.stdout)
    output = process.stdout
    
    assert os.path.exists(test_filename + ".ccb")
    os.remove(test_filename + ".ccb")
    assert os.path.exists(test_filename + ".cfg")
    os.remove(test_filename + ".cfg")