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

@pytest.fixture(params=[1,2,3,4,5,6,7,8,9])
def mipi_speed_val(request):
    return request.param

    
def test_mipispeed(ip_set, config_file, mipi_speed_val):
    
    
    exe_path = "../../build/examples/test_mipi_speed/Release/test_mipi_speed.exe"
    process = subprocess.run([exe_path, ip_set, config_file, str(mipi_speed_val)], 
        text=True,stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    print(process.stdout)
    output = process.stdout
    
    if mipi_speed_val == 9:
        assert process.returncode == 1
    else:
        assert process.returncode == 0 
        
    Logger.info(process.stdout)
    
    match = re.search("mipi output speed: *?(\\d+)", output)
    assert match is not None    # Checks if the pattern is found
    value = match.groups()[0]
    mipiSpeedVal = int(value)
    assert mipiSpeedVal == mipi_speed_val,"does not match with desired mipi speed"
    print("mipi speed verified")

