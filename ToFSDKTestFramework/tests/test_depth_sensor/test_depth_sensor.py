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
import re

@pytest.fixture()
def uboot_ver():
    return "imx_v2020.04_5.4.70_2.3.0" 

@pytest.fixture()
def kernel_ver():
    return "lf-5.10.72-2.2.0" 
@pytest.fixture()
def sdcard_ver():
    return "microsd-4.3.0-08d887e8.img" 

@pytest.fixture()
def sensor_name():
    return "adsd3500"     

Logger = logging.getLogger(__name__)
    
def test_misc_APIs(ip_set, uboot_ver, kernel_ver, sdcard_ver, sensor_name):
    
    exe_path = "../../build/examples/test_sensor_enum/Release/test_sensor_enum.exe"
    process = subprocess.run([exe_path, ip_set], 
        text=True,stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    print(process.stdout)
    assert process.returncode == 0 
    Logger.info(process.stdout)
    output = process.stdout
    
    match = re.search("Uboot Version: *(.*)", output)
    assert match is not None    # Checks if the pattern is found
    value = match.groups()[0]
    assert value == uboot_ver,"doesnt match with expectd uboot version"
    
    match = re.search("KernelVersion: *(.*)", output)
    assert match is not None    # Checks if the pattern is found
    value = match.groups()[0]
    assert value == kernel_ver,"doesnt match with expectd kernel version"
    
    match = re.search("sd card version: *(.*)", output)
    assert match is not None    # Checks if the pattern is found
    value = match.groups()[0]
    assert value == sdcard_ver,"doesnt match with expectd sdcard version"
    
    match = re.search("sensorname: *(.*)", output)
    assert match is not None    # Checks if the pattern is found
    value = match.groups()[0]
    assert value == sensor_name,"doesnt match with expectd sensor_name"