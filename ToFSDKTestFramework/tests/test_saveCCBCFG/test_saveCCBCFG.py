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

test_filename = "test_saveCCBCFG/trial"
API_list = ["saveCCB", "saveCFG"]
@pytest.fixture(params=[API_list.index("saveCCB"),
                        API_list.index("saveCFG")
                        ])
def API_to_test(request):
    return request.param

def case_CCB(stdout, returncode):
    ccb_file_extension = ".ccb"
    
    #delete all ccb files in current directiry
    files_to_delete = [fname for fname in os.listdir('.') if fname.endswith(ccb_file_extension)]
    # Delete each file
    for file_name in files_to_delete:
        try:
            os.remove(file_name)
        except OSError:
            print(f"Error deleting: {file_name}")
            
    assert os.path.exists(test_filename + ".ccb"),"ccb file not saved"
    os.remove(test_filename + ".ccb")
    
    assert returncode == 0," ccb API not successful"

def case_CFG(stdout, returncode):
    cfg_file_extension = ".cfg"
    
    #delete all cfg files in current directiry
    files_to_delete = [fname for fname in os.listdir('.') if fname.endswith(cfg_file_extension)]
    # Delete each file
    for file_name in files_to_delete:
        try:
            os.remove(file_name)
        except OSError:
            print(f"Error deleting: {file_name}")
        
    assert os.path.exists(test_filename + ".cfg"),"cfg file not saved"
    os.remove(test_filename + ".cfg")
    

    assert returncode == 0," cfg API not successful"
    
def switch_case(x, stdout, returncode):
    switcher = {
        0: case_CCB,
        1: case_CFG
    }
    
    # Call the function associated with x, or a default function if x is not found
    return switcher.get(x, lambda: "Invalid case")(stdout, returncode)
    
def test_saveCCBCFG(ip_set, config_file, API_to_test):
    if not API_to_test != 1:
        pytest.skip("CGF not yet available")
    #parse the ini file
    #Run the exe file
    exe_path = "../../build/examples/test_saveCCBCFG/release/test_saveCCBCFG.exe"

    process = subprocess.run([exe_path,  ip_set, config_file, test_filename, str(API_to_test)], 
        text=True,stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    
    Logger.info(process.stdout)    
    print(process.stdout)

    switch_case(API_to_test, process.stdout, process.returncode)

