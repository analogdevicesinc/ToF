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
import numpy as np
import matplotlib.pyplot as plt
import sys
import os

system = tof.System()

print("SDK version: ", tof.getApiVersion(), " | branch: ", tof.getBranchVersion(), " | commit: ", tof.getCommitVersion())

modeCamera1 = 0
modeCamera2 = 0

ipCamera1 = "ip:"
ipCamera2 = "ip:"

if len(sys.argv) != 3:
    print("3 arguments are required")
    print("Usage: python dual_camera.py 'ip of 1st camera' 'ip of second camera'")
    sys.exit(-1)

ipCamera1 = ipCamera1 + str(sys.argv[1])
ipCamera2 = ipCamera2 + str(sys.argv[2])
cameras1 = []
cameras2 = []

#Search and initialize first camera
status = system.getCameraList(cameras1, ipCamera1)
print("system.getCameraList() for first camera: ", status)

camera1 = cameras1[0]
status = camera1.initialize()
print("camera1.initialize()", status)

#Search and initialize second camera
status = system.getCameraList(cameras2, ipCamera2)
print("system.getCameraList() for second camera: ", status)

camera2 = cameras2[0]
status = camera2.initialize()
print("camera2.initialize()", status)

modesCamera1 = []
modesCamera2 = []

status = camera1.getAvailableModes(modesCamera1)
print("camera1.getAvailableModes()", status)
print(modesCamera1)

status = camera1.getAvailableModes(modesCamera2)
print("camera2.getAvailableModes()", status)
print(modesCamera2)

if int(modeCamera1) not in modesCamera1:
    print(f"Error: Unknown mode for camera 1:  {modeCamera1}")
    exit(-3)

if int(modeCamera2) not in modesCamera2:
    print(f"Error: Unknown mode for camera 1:  {modeCamera2}")
    exit(-3)

camDetails1 = tof.CameraDetails()
status = camera1.getDetails(camDetails1)
print("camera1.getDetails()", status)
print("camera1 details:", "id:", camDetails1.cameraId, "connection:", camDetails1.connection)

camDetails2 = tof.CameraDetails()
status = camera2.getDetails(camDetails2)
print("camera1.getDetails()", status)
print("camera1 details:", "id:", camDetails2.cameraId, "connection:", camDetails2.connection)

status = camera1.setMode(int(modeCamera1))
print("camera1.setMode(",modeCamera1,")", status)

status = camera2.setMode(int(modeCamera2))
print("camera1.setMode(",modeCamera2,")", status)

status = camera1.start()
print("camera1.start()", status)

status = camera2.start()
print("camera1.start()", status)

frame1 = tof.Frame()
frame2 = tof.Frame()

status = camera1.requestFrame(frame1)
print("camera1.requestFrame()", status)

status = camera2.requestFrame(frame2)
print("camera1.requestFrame()", status)

frameDataDetails1 = tof.FrameDataDetails()
status = frame1.getDataDetails("depth", frameDataDetails1)
print("frame.getDataDetails()", status)
print("depth frame details:", "width:", frameDataDetails1.width, "height:", frameDataDetails1.height, "type:", frameDataDetails1.type)

frameDataDetails2 = tof.FrameDataDetails()
status = frame2.getDataDetails("depth", frameDataDetails2)
print("frame.getDataDetails()", status)
print("depth frame details:", "width:", frameDataDetails2.width, "height:", frameDataDetails2.height, "type:", frameDataDetails2.type)

status = camera1.stop()
print("camera1.stop()", status)
status = camera2.stop()
print("camera1.stop()", status)

image1 = np.array(frame1.getData("depth"), copy=False)
image2 = np.array(frame2.getData("depth"), copy=False)

plt.figure(10)
plt.imshow(image1, cmap = 'jet')
plt.colorbar()

plt.figure(20)
plt.imshow(image2, cmap = 'jet')
plt.colorbar()
plt.show()
