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
modemap = {
0: {"width":1024, "height":1024},
1: {"width":1024, "height":1024},
2: {"width":512, "height":512},
3: {"width":512, "height":512},
4: {"width":1024, "height":1024},
5: {"width":512, "height":512},
6: {"width":512, "height":512}
}
def test_getMetadataStruct(available_modes_ini, ip_set, config_file, sdk_version):
    if  available_modes_ini == 4:
        pytest.skip("Not available in pcm-native")
    #parse the ini file
    
    if sdk_version == "5.1.0":
        with open('./config/RawToDepthAdsd3500_' + str(available_modes_ini) + '.ini', mode='r') as file:
            lines = file.readlines()

    else:
        mode_name = {0: "3500_sr-native", 1: "3500_lr-native", 2: "3500_sr-qnative", 3: "3500_lr-qnative",
            4: "_pcm-native", 5: "3500_lr-mixed", 6: "3500_sr-mixed"}
   
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
    exe_path = "../../build/examples/test_getMetadata/release/test_getMetadata.exe"
    process = subprocess.run([exe_path, str(available_modes_ini), ip_set, config_file],text=True,
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    
    assert process.returncode == 0
    print(process.stdout)
    output = process.stdout
    
    match = re.search("width: *?(\\d+)", output)
    assert match is not None    # Checks if the pattern is found
    value = match.groups()[0]
    widthVal = int(value)
    assert widthVal == modemap[available_modes_ini]["width"],"does not match mode width"
    print("verified dimension")
    
    match = re.search("height: *?(\\d+)", output)
    assert match is not None    # Checks if the pattern is found
    value = match.groups()[0]
    heightVal = int(value)
    assert heightVal == modemap[available_modes_ini]["height"],"does not match mode height"
    print("verified dimension")
    
    match = re.search("bitsInConfidence: *?(\\d+)", output)
    assert match is not None    # Checks if the pattern is found
    value = match.groups()[0]
    bitsInConfVal = int(value)
    assert bitsInConfVal == ini_data['bitsInConf'], "does not match with .ini file"  
    print("bitsInConf verified with .ini file")
    
    match = re.search("bitsInDepth: *?(\\d+)", output)
    assert match is not None    # Checks if the pattern is found
    value = match.groups()[0]
    bitsInPhaseOrDepthVal = int(value)     
    assert bitsInPhaseOrDepthVal == ini_data['bitsInPhaseOrDepth'], "does not match with .ini file"  
    print("bitsInPhaseOrDepth verified with .ini file")
    
    match = re.search("bitsInAB: *?(\\d+)", output)
    assert match is not None    # Checks if the pattern is found
    value = match.groups()[0]
    bitsInABVal = int(value)  
    assert bitsInABVal == ini_data['bitsInAB'], "does not match with .ini file"  
    print("bitsInAB verified with .ini file")
    
    match = re.search("outputConfiguration: *?(\\d+)", output)
    assert match is not None    # Checks if the pattern is found
    value = match.groups()[0]
    outputConfigurationVal = int(value)
    if ini_data['bitsInAB'] == 0 and ini_data['bitsInConf'] == 0 and ini_data['bitsInPhaseOrDepth'] == 12:
        outputConfigOnIni = 0
    elif ini_data['bitsInAB'] == 0 and ini_data['bitsInConf'] == 0 and ini_data['bitsInPhaseOrDepth'] == 8:
        outputConfigOnIni = 1
    elif ini_data['bitsInAB'] != 0 and ini_data['bitsInConf'] == 0 and ini_data['bitsInPhaseOrDepth'] == 0:
        outputConfigOnIni = 2    
    elif ini_data['bitsInAB'] == 0 and ini_data['bitsInConf'] != 0 and ini_data['bitsInPhaseOrDepth'] == 0:
        outputConfigOnIni = 3
    elif ini_data['bitsInAB'] != 0 and ini_data['bitsInConf'] == 0 and ini_data['bitsInPhaseOrDepth'] == 8:
        outputConfigOnIni = 4
    elif ini_data['bitsInAB'] == 16 and ini_data['bitsInConf'] == 0 and ini_data['bitsInPhaseOrDepth'] == 12:
        outputConfigOnIni = 5
    elif ini_data['bitsInAB'] == 16 and ini_data['bitsInConf'] == 4 and ini_data['bitsInPhaseOrDepth'] != 0:
        outputConfigOnIni = 6
    elif ini_data['bitsInAB'] == 16 and ini_data['bitsInConf'] == 8 and ini_data['bitsInPhaseOrDepth'] == 16:
        outputConfigOnIni = 7
    
    print("outputConfigurationVal: ",outputConfigurationVal )
    print("outputConfigOnIni: ",outputConfigOnIni )
    assert outputConfigurationVal == outputConfigOnIni, "does not match with .ini file"  
    print("outputConfiguration verified")
    
    Logger.info(process.stdout)
