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
import time
import re

Logger = logging.getLogger(__name__)

@pytest.fixture(params=[0,1,2,3,4,5,6])
def available_modes_ini(request):
    return request.param

@pytest.fixture(autouse=True)
def delay_between_tests():
    time.sleep(2) # sleep for 1 second
    

 
def test_access_ini_apis_program(available_modes_ini):

    mode_name = {0: "3500_sr-native", 1: "3500_lr-native", 2: "3500_sr-qnative", 3: "3500_lr-qnative", 4: "_pcm-native", 5: "3500_lr-mixed", 6: "3500_sr-mixed"}
   
    #parse the ini file
    with open('./config/RawToDepthAdsd' + mode_name[available_modes_ini] + '.ini', mode='r') as file:
        lines = file.readlines()
    ini_data = {}
    for line in lines:
        variable, value = line.strip().split('=')
        try:
            ini_data[variable] = float(value)
        except ValueError:
            ini_data[variable] = str(value)  
    
    
    #Run the exe file
    exe_path = "../../build/examples/test_access_ini/release/test_access_ini.exe"
    process = subprocess.run([exe_path, str(available_modes_ini)],text=True,stdout=subprocess.PIPE, stderr=subprocess.STDOUT)

    print(process.stdout)
    output = process.stdout
    
    match = re.search("abThreshMin: *?(\\d+)", output)
    assert match is not None    # Checks if the pattern is found
    value = match.groups()[0]
    abThreshMinVal = int(value)
    assert abThreshMinVal == ini_data['abThreshMin']  # Checks with the .ini file
    
    match = re.search("confThresh: *?(\\d+)", output)
    assert match is not None    # Checks if the pattern is found
    value = match.groups()[0]
    confThreshVal = int(value)
    assert confThreshVal == ini_data['confThresh']  # Checks with the .ini file
    
    match = re.search("RadialThresholdMin: *?(\\d+)", output)
    assert match is not None    # Checks if the pattern is found
    value = match.groups()[0]
    RadialThresholdMinVal = int(value)
    assert RadialThresholdMinVal == ini_data['radialThreshMin']  # Checks with the .ini file
    
    match = re.search("RadialThresholdMax: *?(\\d+)", output)
    assert match is not None    # Checks if the pattern is found
    value = match.groups()[0]
    RadialThresholdMaxVal = int(value)
    assert RadialThresholdMaxVal == ini_data['radialThreshMax']  # Checks with the .ini file
    
    match = re.search("jblfApplyFlag: *?(\\d+)", output)
    assert match is not None    # Checks if the pattern is found
    value = match.groups()[0]
    jblfApplyFlagVal = bool(value)
    assert jblfApplyFlagVal == ini_data['jblfApplyFlag']  # Checks with the .ini file
    
    match = re.search("jblfWindowSize: *?(\\d+)", output)
    assert match is not None    # Checks if the pattern is found
    value = match.groups()[0]
    jblfWindowSizeVal = int(value)
    assert jblfWindowSizeVal == ini_data['jblfWindowSize']  # Checks with the .ini file
    
    match = re.search("JBLFGaussianSigma: *?(\\d+)", output)
    assert match is not None    # Checks if the pattern is found
    value = match.groups()[0]
    JBLFGaussianSigmaVal = int(value)
    assert JBLFGaussianSigmaVal == ini_data['jblfGaussianSigma']  # Checks with the .ini file
    
    
    match = re.search("jblfExponentialTerm: *?(\\d+)", output)
    assert match is not None    # Checks if the pattern is found
    value = match.groups()[0]
    jblfExponentialTermVal = int(value)
    assert jblfExponentialTermVal == ini_data['jblfExponentialTerm']  # Checks with the .ini file
    
    match = re.search("fps: *?(\\d+)", output)
    assert match is not None    # Checks if the pattern is found
    value = match.groups()[0]
    fpsVal = int(value)
    assert fpsVal == ini_data['fps']  # Checks with the .ini file
    
    assert process.returncode == 0

    Logger.info(process.stdout)