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
import aditofpython as tof
import sys
import time
import os

ccb_dir = 'ccb_directory'
ccb_prefix = 'ccb_'

if __name__ == "__main__":

    if len(sys.argv) > 1 :
        if sys.argv[1] in ["--help", "-h"]: 
            print("save_ccb.py usage:")
            print("USB / Local connection: save_ccb.py")
            print("Network connection: save_ccb.py <ip>")
            exit(1)

    cameras = []
    ip = ""
    if len(sys.argv) == 2 :
        ip = sys.argv[1]
        print (f"Looking for camera on network @ {ip}.")
        ip = "ip:" + ip
    elif len(sys.argv) == 1 :
        print (f"Looking for camera on UVC.")
    else :
        print("Too many arguments provided!")
        exit(1)

    dir_path = os.path.join(os.path.dirname( os.path.abspath(__file__)), ccb_dir)
    if os.path.exists(dir_path):
        print(f"The directory {dir_path} already exists.")
    else:
        # Create the directory
        os.makedirs(dir_path)
        print(f"The directory {dir_path} was created.")

    system = tof.System()

    print("SDK version: ", tof.getApiVersion(), " | branch: ", tof.getBranchVersion(), " | commit: ", tof.getCommitVersion())

    status = system.getCameraList(cameras, ip)
    print("system.getCameraList()", status)

    camera1 = cameras[0]

    status = camera1.initialize()
    print(f"camera1.initialize()", status)
    serial_no=''
    status = camera1.readSerialNumber(serial_no, False)
    serial_no = status[1].rstrip('\x00')
    print('Module serial number is: %s' %(serial_no))
    ccb_prefix = ccb_prefix+serial_no+'_'

    camDetails = tof.CameraDetails()
    status = camera1.getDetails(camDetails)
    print("camera1.getDetails()", status)
    print("camera1 details:", "id:", camDetails.cameraId )
    print("connection: ", camDetails.connection)
    print("mode: ", camDetails.mode)
    print("mindepth: ", camDetails.minDepth)
    print("maxdepth: ", camDetails.maxDepth)

    print("Intrinsic Parameters: ")
    # Get intrinsic parameters from camera
    intrinsicParameters = camDetails.intrinsics
    print("fx: ",intrinsicParameters.fx)
    print("fy: ",intrinsicParameters.fy)
    print("cx: ",intrinsicParameters.cx)
    print("cy: ",intrinsicParameters.cy)
    print("codx: ",intrinsicParameters.codx)
    print("cody: ",intrinsicParameters.cody)
    print("k1: ",intrinsicParameters.k1)
    print("k2: ",intrinsicParameters.k2)
    print("k3: ",intrinsicParameters.k3)
    print("k4: ",intrinsicParameters.k4)
    print("k5: ",intrinsicParameters.k5)
    print("k6: ",intrinsicParameters.k6)
    print("p2: ",intrinsicParameters.p2)
    print("p1: ",intrinsicParameters.p1)

    ccb_filename = ccb_prefix + time.strftime('%y%m%d%H%M') + '.ccb'
    
    status = camera1.saveModuleCCB(os.path.join(dir_path,ccb_filename))
    print("camera1.saveModuleCCB()", status)

    file_exists = os.path.isfile(os.path.join(dir_path, ccb_filename))
    if (file_exists):
        print(f"{ccb_filename} saved in {dir_path}")
    else:
        print("ccb not saved")
        
