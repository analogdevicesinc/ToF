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
import shutil

Logger = logging.getLogger(__name__)
home_dir = "./test_frameHandler/"
filename_1 = "trial1.bin"
filename_2 = "trial2.bin"

def reset_folders():
    directories = ["frame_multiple","frame_multithread","frame_rename","frame_single"]
    for test_dir in  directories:
        test_dir = home_dir + test_dir
        if os.path.isdir(test_dir):
            for filename in os.listdir(test_dir):
                file_path = os.path.join(test_dir, filename)
                if os.path.isfile(file_path):  # Check if it's a file (not a subdirectory)
                    os.remove(file_path)
        else:
            os.mkdir(test_dir)

def test_frameHandler(available_modes_ini, ip_set, config_file):

    reset_folders()
    
    bytes_on_ab = 2
    bytes_on_xyz = 6
    bytes_on_depth =2
    bytes_on_conf =4
    bytes_on_metadata = 128
    frame_size_qmp = 512*512*(bytes_on_ab + bytes_on_xyz + bytes_on_depth + bytes_on_conf)+128
    frame_size_mp = 1024*1024*(bytes_on_ab + bytes_on_xyz + bytes_on_depth)+128
    
    if available_modes_ini != 4:
        #Run the exe file

        #exe_path = "../../../../../../Downloads/ToF-main/ToF-main/build/examples/test_frameHandler/Release/test_frameHandler.exe"
        exe_path = "../../build/examples/test_frameHandler/release/test_frameHandler.exe"
        process = subprocess.run([exe_path, str(available_modes_ini), ip_set, config_file,
            filename_1, filename_2],text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        
        if (available_modes_ini == 0 or available_modes_ini == 1):
            frame_size = frame_size_mp
        else:
            frame_size = frame_size_qmp
            
        assert process.returncode == 0
        print(process.stdout)
        
        test_path = home_dir + "frame_multiple/"
        files_in_directory = os.listdir(test_path)
        assert len(files_in_directory) == 1
        assert os.path.getsize(test_path + files_in_directory[0]) == frame_size*10
        
        test_path = home_dir + "frame_multithread/"
        files_in_directory = os.listdir(test_path)
        assert len(files_in_directory) == 1
        assert os.path.getsize(test_path + files_in_directory[0]) == frame_size
        
        test_path = home_dir + "frame_single/"
        files_in_directory = os.listdir(test_path)
        assert len(files_in_directory) == 10
        for filename in os.listdir(test_path):
            assert os.path.getsize(test_path + filename) == frame_size

        test_path = home_dir + "frame_rename/"
        files_in_directory = os.listdir(test_path)
        assert len(files_in_directory) == 2
        for filename in os.listdir(test_path):
            assert os.path.getsize(test_path + filename) == frame_size
        #assert filename_1 in files_in_directory,"setInputfileName failed"
        assert filename_2 in files_in_directory,"setting fileName via saveFrameToFile failed"
        Logger.info(process.stdout)
    
    reset_folders()
    
