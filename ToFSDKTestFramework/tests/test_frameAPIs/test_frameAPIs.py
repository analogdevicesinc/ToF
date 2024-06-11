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

def test_frameAPIs(available_modes_ini, ip_set, config_file, sdk_version):

    mode_name = {0:"sr-native", 1:"lr-native", 2:"sr-qnative", 3:"lr-qnative",
        4:"pcm-native", 5:"lr-mixed", 6:"sr-mixed"}

    
    #parse the ini file
    with open('./test_frameAPIs/frame_details_' + str(available_modes_ini) + '.ini', mode='r') as file:
        lines = file.readlines()
    ini_data = {}
    for line in lines:
        variable, value = line.strip().split(':')
        try:
            ini_data[variable] = float(value)
        except ValueError:
            ini_data[variable] = str(value)  
    
    #Run the exe file
    exe_path = "../../build/examples/test_frameAPIs/release/test_frameAPIs.exe"
    process = subprocess.run([exe_path, str(available_modes_ini), ip_set, config_file],text=True,
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    
    assert process.returncode == 0
    print(process.stdout)
    output = process.stdout
    
    if sdk_version == "5.0.0":
        match = re.search("fDetailsFrameType: *?(\\S+)", output)
        assert match is not None    # Checks if the pattern is found
        value = match.groups()[0]
        frametype = value
        assert frametype == ini_data['fdetails_frametype'],"does not match with .ini file"
        print("fdetails_frametype verified with .ini file")
    
    match = re.search("fDetailsWidth: *?(\\d+)", output)
    assert match is not None    # Checks if the pattern is found
    value = match.groups()[0]
    width = int(value)
    assert width == ini_data['fdetails_width'],"does not match with .ini file"
    print("fdetails_width verified with .ini file")

    
    match = re.search("fDetailsHeight: *?(\\d+)", output)
    assert match is not None    # Checks if the pattern is found
    value = match.groups()[0]
    height = int(value)
    assert height == ini_data['fdetails_height'],"does not match with .ini file"
    print("fdetails_height verified with .ini file")
    
    match = re.search("fDetailsTotalCapture: *?(\\d+)", output)
    assert match is not None    # Checks if the pattern is found
    value = match.groups()[0]
    totalCapture = int(value)
    assert totalCapture == ini_data['fdetails_totalCapture'],"does not match with .ini file"
    print("fdetails_totalCapture verified with .ini file")
    
    match = re.search("fDetailsPassiveIRCaptured: *?(\\d+)", output)
    assert match is not None    # Checks if the pattern is found
    value = match.groups()[0]
    passiveIRCaptured = int(value)
    assert passiveIRCaptured == ini_data['fdetails_passiveIRCaptured'],"does not match with .ini file"
    print("fdetails_passiveIRCaptured verified with .ini file")
    
    frame_types = {"ab", "depth", "conf", "xyz", "metadata"}
    
    for frame_type in frame_types:
        if not (mode_name[available_modes_ini] == "pcm-native" and frame_type != "ab"):
            match = re.search((frame_type +"-type: *?(\\S+)"), output)
            assert match is not None    # Checks if the pattern is found
            value = match.groups()[0]
            frameType = value
            assert frameType == ini_data[frame_type+"-type"],"does not match with .ini file"
            print(frame_type+"-type verified with .ini file")
        
            match = re.search((frame_type +"-width: *?(\\d+)"), output)
            assert match is not None    # Checks if the pattern is found
            value = match.groups()[0]
            width = int(value)
            assert width == ini_data[frame_type+"-width"],"does not match with .ini file"
            print(frame_type+"-width verified with .ini file")
        
            match = re.search((frame_type +"-height: *?(\\d+)"), output)
            assert match is not None    # Checks if the pattern is found
            value = match.groups()[0]
            height = int(value)
            assert height == ini_data[frame_type+"-height"],"does not match with .ini file"
            print(frame_type+"-height verified with .ini file")
            
            match = re.search((frame_type +"-subelementSize: *?(\\d+)"), output)
            assert match is not None    # Checks if the pattern is found
            value = match.groups()[0]
            subelementSize = int(value)
            assert subelementSize == ini_data[frame_type+"-subelementSize"],"does not match with .ini file"
            print(frame_type+"-subelementSize verified with .ini file")
            
            match = re.search((frame_type +"-subelementsPerElement: *?(\\d+)"), output)
            assert match is not None    # Checks if the pattern is found
            value = match.groups()[0]
            subelementsPerElement = int(value)
            assert subelementsPerElement ==ini_data[frame_type+"-subelementsPerElement"],"does not match with .ini file"
            print(frame_type+"-subelementsPerElement verified with .ini file")
            
            match = re.search((frame_type +"-bytesCount: *?(\\d+)"), output)
            assert match is not None    # Checks if the pattern is found
            value = match.groups()[0]
            bytesCount = int(value)
            assert bytesCount == ini_data[frame_type+"-bytesCount"] == width*height*subelementsPerElement*subelementSize, "does not match with .ini file"
            print(frame_type+"-bytesCount verified with .ini file")
    
    if sdk_version != "5.1.0":    
        match = re.search("embed_hdr_length: *?(\\d+)", output)
        assert match is not None    # Checks if the pattern is found
        value = match.groups()[0]
        embedHdrLength = int(value)
        assert embedHdrLength == ini_data["attrib_embed_hdr_length"],"does not match with .ini file"
        print("attrib_embed_hdr_length verified with .ini file")
            
        match = re.search("embed_height: *?(\\d+)", output)
        assert match is not None    # Checks if the pattern is found
        value = match.groups()[0]
        embedHeight = int(value)
        assert embedHeight == ini_data["attrib_embed_height"],"does not match with .ini file"
        print("attrib_embed_height verified with .ini file")
        
        match = re.search("embed_width: *?(\\d+)", output)
        assert match is not None    # Checks if the pattern is found
        value = match.groups()[0]
        embedWidth = int(value)
        assert embedWidth == ini_data["attrib_embed_width"],"does not match with .ini file"
        print("attrib_embed_width verified with .ini file")
        
        match = re.search("attrib_height: *?(\\d+)", output)
        assert match is not None    # Checks if the pattern is found
        value = match.groups()[0]
        attribHeight = int(value)
        assert attribHeight == ini_data["attrib_height"],"does not match with .ini file"
        print("attrib_height verified with .ini file")
        Logger.info(process.stdout)
        
        match = re.search("attrib_mode: *?(\\d+)", output)
        assert match is not None    # Checks if the pattern is found
        value = match.groups()[0]
        attribMode = int(value)
        assert attribMode == ini_data["attrib_mode"],"does not match with .ini file"
        print("attrib_mode verified with .ini file")
        Logger.info(process.stdout)
        
        match = re.search("passive_ir: *?(\\d+)", output)
        assert match is not None    # Checks if the pattern is found
        value = match.groups()[0]
        passiveIr = int(value)
        assert passiveIr == ini_data["attrib_passive_ir"],"does not match with .ini file"
        print("attrib_passive_ir verified with .ini file")
        Logger.info(process.stdout)
        
        match = re.search("total_captures: *?(\\d+)", output)
        assert match is not None    # Checks if the pattern is found
        value = match.groups()[0]
        totalCaptures = int(value)
        assert totalCaptures == ini_data["attrib_total_captures"],"does not match with .ini file"
        print("attrib_total_captures verified with .ini file")
        Logger.info(process.stdout)
        
        match = re.search("attrib_width: *?(\\d+)", output)
        assert match is not None    # Checks if the pattern is found
        value = match.groups()[0]
        attribWidth = int(value)
        assert attribWidth == ini_data["attrib_width"],"does not match with .ini file"
        print("attrib_width verified with .ini file")
        Logger.info(process.stdout)
