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

def help():
    print(f"{sys.argv[0]} usage:")
    print(f"USB: {sys.argv[0]} <mode number> <config>")
    print(f"Network connection: {sys.argv[0]} <mode number> <ip> <config>")
    print()
    print("For example:")
    print(f"python {sys.argv[0]} 0 10.43.0.1 config\config_adsd3500_adsd3100.json")
    exit(1)

if len(sys.argv) < 3 or len(sys.argv) > 4 or sys.argv[1] == "--help" or sys.argv[1] == "-h" :
    help()
    exit(-1)

system = tof.System()

print("SDK version: ", tof.getApiVersion(), " | branch: ", tof.getBranchVersion(), " | commit: ", tof.getCommitVersion())

mode = 0
cameras = []
ip = ""
if len(sys.argv) == 4:
    mode = sys.argv[1]
    ip = sys.argv[2]
    config = sys.argv[3]
    print (f"Looking for camera on network @ {ip}. Will use {config}.")
    ip = "ip:" + ip
elif len(sys.argv) == 3:
    mode = sys.argv[1]
    config = sys.argv[2]
    print (f"Looking for camera on UVC. Will use {config}.")
else :
    print("Too many arguments provided!")
    exit(-2)

#Checks on input
if os.path.exists(config) == False:
    print(f"Error: Config file cannot be found - {config}")
    help()
    exit(-4)

status = system.getCameraList(cameras, ip)
print("system.getCameraList()", status)

camera1 = cameras[0]

#create callback and register it to the interrupt routine
def callbackFunction(callbackStatus):
    print("Running the python callback for which the status of ADSD3500 has been forwarded. ADSD3500 status = ", callbackStatus)

sensor = camera1.getSensor()
status = sensor.adsd3500_register_interrupt_callback(callbackFunction)

status = camera1.initialize(config)
print("camera1.initialize()", status)

modes = []
status = camera1.getAvailableModes(modes)
print("camera1.getAvailableModes()", status)
print(modes)

if int(mode) not in modes:
    print(f"Error: Unknown mode - {mode}")
    exit(-3)

camDetails = tof.CameraDetails()
status = camera1.getDetails(camDetails)
print("camera1.getDetails()", status)
print("camera1 details:", "id:", camDetails.cameraId, "connection:", camDetails.connection)

status = camera1.setMode(int(mode))
print("camera1.setMode(",mode,")", status)

# Example of getting/modifying/setting the current ADSD3500 parameters
# status, currentFrameProcessParams = camera1.getFrameProcessParams()
# currentFrameProcessParams["ab_thresh_min"] = "4"
# camera1.setFrameProcessParams(currentFrameProcessParams)

# Example of configuring the Dynamic Mode Switching
# The expected sequence of frame is: mode2, mode2, mode2, mode3, mode2, ...
# camera1.adsd3500setEnableDynamicModeSwitching(True)
# camera1.adsds3500setDynamicModeSwitchingSequence([(2, 3), (3, 1)])

status = camera1.start()
print("camera1.start()", status)

frame = tof.Frame()

status = camera1.requestFrame(frame)
print("camera1.requestFrame()", status)

frameDataDetails = tof.FrameDataDetails()
status = frame.getDataDetails("depth", frameDataDetails)
print("frame.getDataDetails()", status)
print("depth frame details:", "width:", frameDataDetails.width, "height:", frameDataDetails.height, "type:", frameDataDetails.type)

status = camera1.stop()
print("camera1.stop()", status)

image = np.array(frame.getData("depth"), copy=False)

metadata = tof.Metadata
status, metadata = frame.getMetadataStruct()

print("Sensor temperature from metadata: ", metadata.sensorTemperature)
print("Laser temperature from metadata: ", metadata.laserTemperature)
print("Frame number from metadata: ", metadata.frameNumber)
print("Mode from metadata: ", metadata.imagerMode)

#Unregister callback
status = sensor.adsd3500_unregister_interrupt_callback(callbackFunction)

plt.figure()
plt.imshow(image)
plt.colorbar()
plt.show()


