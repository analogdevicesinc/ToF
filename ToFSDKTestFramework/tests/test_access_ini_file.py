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

def test_access_ini_apis_program(available_modes_ini, ip_set,config_file):

    mode_name = {0: "3500_sr-native", 1: "3500_lr-native", 2: "3500_sr-qnative", 3: "3500_lr-qnative",
        4: "_pcm-native", 5: "3500_lr-mixed", 6: "3500_sr-mixed"}
   
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
    process = subprocess.run([exe_path, str(available_modes_ini), ip_set, config_file],text=True,
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    
    assert process.returncode == 0
    print(process.stdout)
    output = process.stdout
    
    match = re.search("abThreshMin: *?(\\d+)", output)
    assert match is not None    # Checks if the pattern is found
    value = match.groups()[0]
    abThreshMinVal = int(value)
    assert abThreshMinVal == ini_data['abThreshMin'],"does not match with .ini file"
    print("abThreshMin verified with .ini file")
    
    match = re.search("confThresh: *?(\\d+)", output)
    assert match is not None    # Checks if the pattern is found
    value = match.groups()[0]
    confThreshVal = int(value)
    assert confThreshVal == ini_data['confThresh'],"does not match with .ini file"
    print("confThresh verified with .ini file")
    
    match = re.search("RadialThresholdMin: *?(\\d+)", output)
    assert match is not None    # Checks if the pattern is found
    value = match.groups()[0]
    RadialThresholdMinVal = int(value)
    assert RadialThresholdMinVal == ini_data['radialThreshMin'],"does not match with .ini file"
    print("confThreshVal verified with .ini file")
    
    match = re.search("RadialThresholdMax: *?(\\d+)", output)
    assert match is not None    # Checks if the pattern is found
    value = match.groups()[0]
    RadialThresholdMaxVal = int(value)
    assert RadialThresholdMaxVal == ini_data['radialThreshMax'],"does not match with .ini file"
    print("RadialThresholdMax verified with .ini file")
    
    match = re.search("jblfApplyFlag: *?(\\d+)", output)
    assert match is not None    # Checks if the pattern is found
    value = match.groups()[0]
    jblfApplyFlagVal = bool(value)
    assert jblfApplyFlagVal == ini_data['jblfApplyFlag'],"does not match with .ini file"
    print("jblfApplyFlag verified with .ini file")
    
    match = re.search("jblfWindowSize: *?(\\d+)", output)
    assert match is not None    # Checks if the pattern is found
    value = match.groups()[0]
    jblfWindowSizeVal = int(value)
    assert jblfWindowSizeVal == ini_data['jblfWindowSize'],"does not match with .ini file"
    print("jblfWindowSize verified with .ini file")
    
    match = re.search("JBLFGaussianSigma: *?(\\d+)", output)
    assert match is not None    # Checks if the pattern is found
    value = match.groups()[0]
    JBLFGaussianSigmaVal = int(value)
    assert JBLFGaussianSigmaVal == ini_data['jblfGaussianSigma'],"does not match with .ini file"
    print("JBLFGaussianSigma verified with .ini file")
    
    match = re.search("jblfExponentialTerm: *?(\\d+)", output)
    assert match is not None    # Checks if the pattern is found
    value = match.groups()[0]
    jblfExponentialTermVal = int(value)
    assert jblfExponentialTermVal == ini_data['jblfExponentialTerm'],"does not match with .ini file"
    print("jblfExponentialTerm verified with .ini file")
    
    if available_modes_ini != 4:
        match = re.search("EnableMetadatainABValue: *?(\\d+)", output)
        assert match is not None    # Checks if the pattern is found
        value = match.groups()[0]
        EnableMetadatainABinLogs = int(value)
        if ini_data['headerSize'] == 128:
            EnableMetadatainABinINI = 1
        else:
            EnableMetadatainABinINI = 0
        assert EnableMetadatainABinLogs == EnableMetadatainABinINI,"does not match with .ini file"
        print("headerSize verified with .ini file")
    
    match = re.search("partialDepthEnable: *?(\\d+)", output)
    assert match is not None    # Checks if the pattern is found
    value = match.groups()[0]
    partialDepthEnableVal = int(value)
    assert partialDepthEnableVal == ini_data['partialDepthEnable'], "does not match with .ini file"  
    print("partialDepthEnable verified with .ini file")
    
    match = re.search("bitsInConf: *?(\\d+)", output)
    assert match is not None    # Checks if the pattern is found
    value = match.groups()[0]
    bitsInConfVal = int(value)
    if bitsInConfVal == 2:
        bitsInConfVal = 8
    elif bitsInConfVal == 1:
        bitsInConfVal = 4      
    elif bitsInConfVal == 0:
        bitsInConfVal = ini_data['bitsInConf']   
    assert bitsInConfVal == ini_data['bitsInConf'], "does not match with .ini file"  
    print("bitsInConf verified with .ini file")
    
    match = re.search("bitsInPhaseOrDepth: *?(\\d+)", output)
    assert match is not None    # Checks if the pattern is found
    value = match.groups()[0]
    bitsInPhaseOrDepthVal = int(value)
    if bitsInPhaseOrDepthVal == 6:
        bitsInPhaseOrDepthVal = 16
    elif bitsInPhaseOrDepthVal == 5:
        bitsInPhaseOrDepthVal = 14   
    elif bitsInPhaseOrDepthVal == 4:
        bitsInPhaseOrDepthVal = 12 
    elif bitsInPhaseOrDepthVal == 3:
        bitsInPhaseOrDepthVal = 10    
    elif bitsInPhaseOrDepthVal == 2:
        bitsInPhaseOrDepthVal = ini_data['bitsInPhaseOrDepth']   
    assert bitsInPhaseOrDepthVal == ini_data['bitsInPhaseOrDepth'], "does not match with .ini file"  
    print("bitsInPhaseOrDepth verified with .ini file")
    
    match = re.search("bitsInAB: *?(\\d+)", output)
    assert match is not None    # Checks if the pattern is found
    value = match.groups()[0]
    bitsInABVal = int(value)
    if bitsInABVal == 6:
        bitsInABVal = 16
    elif bitsInABVal == 5:
        bitsInABVal = 14   
    elif bitsInABVal == 4:
        bitsInABVal = 12 
    elif bitsInABVal == 3:
        bitsInABVal = 10    
    elif bitsInABVal == 2:
        bitsInABVal = 8   
    elif bitsInABVal == 0:
        bitsInABVal == ini_data['bitsInAB']
    assert bitsInABVal == ini_data['bitsInAB'], "does not match with .ini file"  
    print("bitsInAB verified with .ini file")
    
    
    match = re.search("fps: *?(\\d+)", output)
    assert match is not None    # Checks if the pattern is found
    value = match.groups()[0]
    fpsVal = int(value)
    assert fpsVal == ini_data['fps'],"does not match with .ini file"  # Checks with the .ini file
    print("fps verified with .ini file")

    Logger.info(process.stdout)