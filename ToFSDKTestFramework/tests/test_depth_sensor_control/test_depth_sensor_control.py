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
import logging
import os
from enum import Enum
import re

Logger = logging.getLogger(__name__)

control_list = ["netlinktest", "inputFormat", "fps", "modeInfoVersion", "confidenceBits", 
"imagerType", "abBits", "phaseDepthBits", "depthEnable", "abAveraging"]

@pytest.fixture(params=[
                        [control_list.index("netlinktest"),'0'],                    #0
                        [control_list.index("netlinktest"),'1'],                    #1
                        [control_list.index("netlinktest"),'2'],                    #2
                        [control_list.index("inputFormat"),"mipiRaw12"],            #3
                        [control_list.index("inputFormat"),"mipiRaw12_11"],         #4
                        [control_list.index("inputFormat"),"raw8"],                 #5
                        [control_list.index("inputFormat"),"raw16"],                #6
                        [control_list.index("inputFormat"),"raw16_bits11_shift4"],  #7
                        [control_list.index("inputFormat"),"raw16_bits11_shift0"],  #8
                        [control_list.index("inputFormat"),"raw16_bits12_shift4"],  #9
                        [control_list.index("inputFormat"),"raw16_bits12_shift0"],  #10
                        [control_list.index("inputFormat"),"test"],                 #11
                        [control_list.index("fps"),'10'],                           #12,pass
                        [control_list.index("fps"),'20'],                           #13,pass
                        [control_list.index("fps"),'30'],                           #14,pass
                        [control_list.index("modeInfoVersion"),'1'],                #15,pass
                        [control_list.index("modeInfoVersion"),'2'],                #16,pass
                        [control_list.index("modeInfoVersion"),'3'],                #17,pass
                        [control_list.index("confidenceBits"),'0'],                 #18,pass
                        [control_list.index("confidenceBits"),'1'],                 #19,pass
                        [control_list.index("confidenceBits"),'2'],                 #20,pass
                        [control_list.index("confidenceBits"),'4'],                 #21,fail
                        [control_list.index("imagerType"),'0'],                     #22,pass
                        [control_list.index("abBits"),'0'],                         #23,fail
                        [control_list.index("abBits"),'2'],                         #24,pass
                        [control_list.index("abBits"),'3'],                         #25,pass
                        [control_list.index("abBits"),'4'],                         #26,pass
                        [control_list.index("abBits"),'5'],                         #27,pass
                        [control_list.index("abBits"),'6'],                         #28,pass
                        [control_list.index("abBits"),'8'],                         #29,fail
                        [control_list.index("abBits"),'10'],                        #30,fail
                        [control_list.index("abBits"),'12'],                        #31,fail
                        [control_list.index("abBits"),'14'],                        #32,fail
                        [control_list.index("abBits"),'16'],                        #33,fail
                        [control_list.index("phaseDepthBits"),'2'],                 #34,pass
                        [control_list.index("phaseDepthBits"),'3'],                 #35,pass
                        [control_list.index("phaseDepthBits"),'4'],                 #36pass
                        [control_list.index("phaseDepthBits"),'5'],                 #37,pass
                        [control_list.index("phaseDepthBits"),'6'],                 #38,pass
                        [control_list.index("phaseDepthBits"),'8'],                 #39,fail
                        [control_list.index("phaseDepthBits"),'10'],                #40,fail
                        [control_list.index("phaseDepthBits"),'12'],                #41,fail
                        [control_list.index("phaseDepthBits"),'14'],                #42,fail
                        [control_list.index("phaseDepthBits"),'16'],                #43,fail
                        [control_list.index("depthEnable"),'0'],                    #44,pass
                        [control_list.index("depthEnable"),'1'],                    #45,pass
                        [control_list.index("depthEnable"),'2'],                    #46,pass
                        [control_list.index("abAveraging"),'0'],                    #47,pass
                        [control_list.index("abAveraging"),'1'],                    #48,pass
                        [control_list.index("abAveraging"),'2']                     #49,pass
                        ])
def control_test_case(request):
    return request.param

def case_inputFormat(stdout, returncode, control_val):
    assert returncode == 0
    match = re.search("inputFormat test value set to *(.*)", stdout)
    assert match is not None    # Checks if the pattern is found
    inputFormat = match.groups()[0]

    if control_val == "test":
        assert inputFormat != control_val, "not a valid input format"
    print(inputFormat, " verified")
    
def case_netLinkTest(stdout, returncode, control_val):
    
    assert returncode == 0
    match = re.search("Netlinktest is write-only control", stdout)
    assert match is not None    # Checks if the pattern is found
    
def case_fps(stdout, returncode, control_val):
    assert returncode == 0
    match = re.search("FPS is write only control", stdout)
    assert match is not None    # Checks if the pattern is found
    
def case_modeInfoVersion(stdout, returncode, control_val):
    
    assert returncode == 0
    match = re.search("modeInfo version: *?(\\d+)", stdout)
    assert match is not None    # Checks if the pattern is found
    modeInfoVal = int(match.groups()[0])
    assert modeInfoVal == 2
  
def case_confidenceBits(stdout, returncode, control_val):

    if 0 <= int(control_val) <= 2:
    
        print(control_val, " is valid value.")
        assert returncode == 0, "error on executing API"
    
        match = re.search("ConfidenceBits value set to: *?(\\d+)", stdout)
        assert match is not None    # Checks if the pattern is found
        confBitsSet = int(match.groups()[0])
        
        match = re.search("confidenceBits: *?(\\d+)", stdout)
        assert match is not None    # Checks if the pattern is found
        confBitsGet = int(match.groups()[0])
        
        assert confBitsSet == confBitsGet
        
    else:
        print(control_val, " is invalid value.")
        assert returncode == 1
        
        match = re.search("cannot set confidenceBits value", stdout)
        assert match is not None, "error not found"    # Checks if the pattern is found
        
def case_ImagerType(stdout, returncode, control_val):

    assert returncode == 0
    
    match = re.search("imagerType is  read only", stdout)
    assert match is not None    # Checks if the pattern is found
    match = re.search("imagerType: 1", stdout)
    assert match is not None    # Checks if the pattern is found  
    
def case_ABBits(stdout, returncode, control_val):
    
    if 2 <= int(control_val) <= 6 or int(control_val) == 0:
        print(control_val, " is valid value.")
        assert returncode == 0
        
        match = re.search("abBits value set to: *?(\\d+)", stdout)
        assert match is not None    # Checks if the pattern is found
        abBitsSet = int(match.groups()[0])
        
        match = re.search("abBits: *?(\\d+)", stdout)
        assert match is not None    # Checks if the pattern is found
        abBitsGet = int(match.groups()[0])
        
        assert abBitsSet == abBitsGet
        
    else:
        print(control_val, " is invalid value.")
        assert returncode == 1
        
        match = re.search("cannot set abBits value", stdout)
        assert match is not None, "error not found"
    
def case_phaseDepthBits(stdout, returncode, control_val):
    print(control_val)
    if 2 <= int(control_val) <= 6 or int(control_val) == 0:
        print(control_val, " is valid value.")
        assert returncode == 0
    
        match = re.search("phaseDepthBits value set to: *?(\\d+)", stdout)
        assert match is not None    # Checks if the pattern is found
        phaseDepthBitsSet = int(match.groups()[0])
    
        match = re.search("phaseDepthBits: *?(\\d+)", stdout)
        assert match is not None    # Checks if the pattern is found
        phaseDepthBitsGet = int(match.groups()[0])
        
        assert phaseDepthBitsSet == phaseDepthBitsGet 
        
    else:
        print(control_val, " is invalid value.")
        assert returncode == 1
        
        match = re.search("cannot set phaseDepthBits value", stdout)
        assert match is not None, "error not found"
    
def case_depthEnable(stdout, returncode, control_val):

    assert returncode == 0
    
    match = re.search("depthEnable value set to: *?(\\d+)", stdout)
    assert match is not None    # Checks if the pattern is found
    depthEnableSet = int(match.groups()[0])
    
    match = re.search("depthEnable: *?(\\d+)", stdout)
    assert match is not None    # Checks if the pattern is found
    depthEnableGet = int(match.groups()[0])
    
    if 0 <= int(control_val) <= 1:
        assert depthEnableSet == depthEnableGet 
    else:
        assert depthEnableGet == 1

def case_abAveraging(stdout, returncode, control_val):

    assert returncode == 0
    
    match = re.search("abAveraging value set to: *?(\\d+)", stdout)
    assert match is not None    # Checks if the pattern is found
    abAveragingSet = int(match.groups()[0])
    
    match = re.search("abAveraging: *?(\\d+)", stdout)
    assert match is not None    # Checks if the pattern is found
    abAveragingGet = int(match.groups()[0])
    
    if 0 <= int(control_val) <= 1:
        assert abAveragingSet == abAveragingGet 
    else:
        assert abAveragingGet == 1
    


def switch_case(x, stdout, returncode, control_val):
    switcher = {
        0: case_netLinkTest,
        1: case_inputFormat,
        2: case_fps,
        3: case_modeInfoVersion,
        4: case_confidenceBits,
        5: case_ImagerType,
        6: case_ABBits,
        7: case_phaseDepthBits,
        8: case_depthEnable,
        9: case_abAveraging
    }
    
    # Call the function associated with x, or a default function if x is not found
    return switcher.get(x, lambda: "Invalid case")(stdout, returncode, control_val)

def test_depth_sensor_control(ip_set, control_test_case):
    
    control, control_val = control_test_case
    
    exe_path = "../../build/examples/test_depth_sensor_control/Release/test_depth_sensor_control.exe"
    process = subprocess.run([exe_path, ip_set, str(control), control_val], 
        text=True,stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    print(process.stdout)
    Logger.info(process.stdout)

    switch_case(control, process.stdout, process.returncode, control_val)