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

@pytest.fixture(params=[0,1,2,3,4])
def gen_temp_test_case(request):
    return request.param


def case_ABinvalidation(stdout, returncode):

    assert returncode == 0

    print("comparing  Values before generic template set")
    match = re.search("abThreshMin via adsd3500GetABinvalidationThreshold: *?(\\d+)", stdout)
    assert match is not None    # Checks if the pattern is found
    value_api = match.groups()[0]
    print(f"via adsd3500GetABinvalidationThreshold: {value_api}")
    match = re.search("abThreshMin via generic template: *?(\\d+)", stdout)
    assert match is not None    # Checks if the pattern is found
    value_generic_api = match.groups()[0]
    print(f"via generic template: {value_generic_api}")
    assert value_api == value_generic_api,"value not much with generic api output"
    print("values match")
    
    print("comparing  Values after generic template set")
    match = re.search("abThreshMin via adsd3500GetABinvalidationThreshold after setting: *?(\\d+)", stdout)
    assert match is not None    # Checks if the pattern is found
    value_api = match.groups()[0]
    print(f"via adsd3500GetABinvalidationThreshold: {value_api}")
    match = re.search("abThreshMin via generic template after setting: *?(\\d+)", stdout)
    assert match is not None    # Checks if the pattern is found
    value_generic_api = match.groups()[0]
    print(f"via generic template: {value_generic_api}")
    assert value_api == value_generic_api,"value not much with generic api output"
    print("values match") 
    
def case_framerate(stdout, returncode):

    assert returncode == 0

    print("comparing  Values before generic template set")
    match = re.search("frameRate via adsd3500GetFrameRate: *?(\\d+)", stdout)
    assert match is not None    # Checks if the pattern is found
    value_api = match.groups()[0]
    print(f"via adsd3500GetFrameRate: {value_api}")
    match = re.search("frameRate via generic template: *?(\\d+)", stdout)
    assert match is not None    # Checks if the pattern is found
    value_generic_api = match.groups()[0]
    print(f"via generic template: {value_generic_api}")
    assert value_api == value_generic_api,"value not much with generic api output"
    print("values match")
    
    print("comparing  Values after generic template set")
    match = re.search("frameRate via adsd3500GetFrameRate after setting: *?(\\d+)", stdout)
    assert match is not None    # Checks if the pattern is found
    value_api = match.groups()[0]
    print(f"via adsd3500GetFrameRate: {value_api}")
    match = re.search("frameRate via generic template: after setting: *?(\\d+)", stdout)
    assert match is not None    # Checks if the pattern is found
    value_generic_api = match.groups()[0]
    print(f"via generic template: {value_generic_api}")
    assert value_api == value_generic_api,"value not much with generic api output"
    print("values match")
    
def case_EnableMetadatainAB(stdout, returncode):

    assert returncode == 0

    print("comparing  Values before generic template set")
    match = re.search("EnableMetadatainAB via adsd3500GetEnableMetadatainAB: *?(\\d+)", stdout)
    assert match is not None    # Checks if the pattern is found
    value_api = match.groups()[0]
    print(f"via adsd3500GetEnableMetadatainAB: : {value_api}")
    match = re.search("EnableMetadatainAB via generic template: *?(\\d+)", stdout)
    assert match is not None    # Checks if the pattern is found
    value_generic_api = match.groups()[0]
    print(f"via generic template: {value_generic_api}")
    assert value_api == value_generic_api,"value not much with generic api output"
    print("values match")
    
    print("comparing  Values after generic template set")
    match = re.search("EnableMetadatainAB value  via adsd3500GetEnableMetadatainAB after setting: *?(\\d+)", stdout)
    assert match is not None    # Checks if the pattern is found
    value_api = match.groups()[0]
    print(f"via adsd3500GetEnableMetadatainAB: {value_api}")
    match = re.search("EnableMetadatainAB via generic template after setting: *?(\\d+)", stdout)
    assert match is not None    # Checks if the pattern is found
    value_generic_api = match.groups()[0]
    print(f"via generic template: {value_generic_api}")
    assert value_api == value_generic_api,"value not much with generic api output"
    print("values match")
    
def case_use_get_on_write_register(stdout, returncode):
    assert returncode == 1, "Test should return an error. should not use get on write register"
    match = re.search("Cannnot retreive EnableMetadatainAB value via generic template!", stdout)
    assert match is not None,"Cannnot retreive EnableMetadatainAB value via generic template!"

def case_use_set_on_read_register(stdout, returncode):
    
    print("comparing  Values after generic template set")
    match = re.search("EnableMetadatainAB via generic template: *?(\\d+)", stdout)
    assert match is not None    # Checks if the pattern is found
    value_api = match.groups()[0]
    print(f"via adsd3500GetEnableMetadatainAB: {value_api}")
    match = re.search("EnableMetadatainAB via generic template after setting: *?(\\d+)", stdout)
    assert match is not None    # Checks if the pattern is found
    value_generic_api = match.groups()[0]
    print(f"via generic template: {value_generic_api}")
    assert value_api == value_generic_api,"value should not match"
    print("values  succesfully not set to 0")



def switch_case(x, stdout,returncode):
    switcher = {
        0: case_ABinvalidation,
        1: case_framerate,
        2: case_EnableMetadatainAB,
        3: case_use_get_on_write_register,
        4: case_use_set_on_read_register
    }
    
    # Call the function associated with x, or a default function if x is not found
    return switcher.get(x, lambda: "Invalid case")(stdout, returncode)

def test_generictemplate(available_modes_ini, ip_set, config_file, gen_temp_test_case):
    
    exe_path = "../../build/examples/test_generic_template/Release/test_generic_template.exe"
    process = subprocess.run([exe_path, str(available_modes_ini), ip_set, config_file, str(gen_temp_test_case)], 
        text=True,stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    print(process.stdout)
    Logger.info(process.stdout)

    switch_case(gen_temp_test_case, process.stdout,process.returncode)