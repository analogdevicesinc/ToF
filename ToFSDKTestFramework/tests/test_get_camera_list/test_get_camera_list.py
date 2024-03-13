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

@pytest.fixture()
def ip_invalid_string():
    return "None"
    
@pytest.fixture()
def ip_no_camera():
    return "10.43.0.1"
    
Logger = logging.getLogger(__name__)

@pytest.mark.parametrize("ip_list", ["10.42.0.1", "10.43.0.1", "None", 1234])

def test_get_cameralist_input(ip_set,ip_list,ip_no_camera):
    try:
        #Run the exe file
        exe_path = "../../build/examples/test_get_camera_list/release/test_get_camera_list.exe"
        process = subprocess.run([exe_path, ip_list],
            text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
            
        print(process.stdout)
        output = process.stdout
        
        if ip_list == ip_set:
            assert process.returncode == 0
            assert "Status::OK" in output,"camera not detected @ ip_set for camera"
        
        elif ip_list == ip_no_camera:
            assert "Status::UNREACHABLE" in output,"wrong status"
        
        else:
            assert "getaddrinfo failed: 11001" in output,"wrong status"
            
    except TypeError:  
        print("tried an integer as an input")
    
