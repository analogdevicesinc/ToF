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

frame_dir = "./data_dir/"
mode_names = ["sr-native" , "lr-native" , "sr-qnative" , "lr-qnative" , "pcm-native" , "lr-mixed" , "sr-mixed"]

def test_fps(available_modes_ini, ip_set, config_file):


    #Run the exe file
    exe_path = "../../build/examples/test_measure_fps/release/test_measure_fps.exe"
    process = subprocess.run([exe_path, str(available_modes_ini), ip_set, config_file], 
        text=True,stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        
    assert process.returncode == 0
    print(process.stdout)
    output = process.stdout
    
    match = re.search("Camera FPS set from Ini file at: *?(\\d+)", output)
    assert match is not None    # Checks if the pattern is found
    value = match.groups()[0]
    fps_setting = float(value)
    print("FPS set from Ini file: ", fps_setting)
    
    match = re.search("Measured FPS: *?(\d+(\.\d*)?)", output)
    assert match is not None    # Checks if the pattern is found
    measured_fps = float(match.group(1))
    print("Measured fps: ", measured_fps)

    assert (fps_setting+5) >= measured_fps >= (fps_setting-5),"desired fps not obtained"
    Logger.info(process.stdout)
    