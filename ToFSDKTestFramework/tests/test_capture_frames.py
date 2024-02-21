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
import math

Logger = logging.getLogger(__name__)

metadataLength = 128

frame_dir = "./data_dir/"
frame_types = ["ab" , "depth" , "conf" , "metadata"]
mode_name = {0: "sr-native", 1: "lr-native", 2: "sr-qnative", 3: "lr-qnative",
    4: "pcm-native", 5: "lr-mixed", 6: "sr-mixed"}
DataTypeSize = {"ab":2 , "depth":2 , "conf":4 , "metadata":128}
FrameSize = {0:1024, 1: 1024, 2: 512, 3:512,
    4: 1024, 5: 512, 6: 512}
    
def test_capture_frames(available_modes_ini, ip_set, config_file):
    # Run the 'dir ..' command and capture the output
    exe_path = "../../build/examples/test_frame_capture/release/test_frame_capture.exe"
    process = subprocess.run([exe_path, str(available_modes_ini), ip_set, config_file], 
        text=True,stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    print(process.stdout)
    assert process.returncode == 0 
    Logger.info(process.stdout)
    assert os.path.isdir(frame_dir),"folder does not exist"
    
    #for modeName in modeName:
    for frameType in frame_types:
        assert os.path.exists(frame_dir + 'out_' + frameType + "_" + 
            mode_name[available_modes_ini] + ".bin" ),"binary file, not saved"
        
        if frameType == "metadata":
            data_size = metadataLength
        else:
            data_size = (FrameSize[available_modes_ini]**2)*DataTypeSize[frameType]
     
        file_size = os.path.getsize (frame_dir + 'out_' + frameType + "_" + mode_name[available_modes_ini] + ".bin")
        
        assert data_size == file_size,"not compatible with data size"
     
        if mode_name[available_modes_ini] == "pcm-native":
            break
        
        

