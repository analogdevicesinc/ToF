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
import pygame
import sys
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap

modemapping = {
"lr-native": {"width":1024, "height":1024},
"lr-qnative": {"width":512, "height":512},
"sr-native": {"width":1024, "height":1024},
"sr-qnative": {"width":512, "height":512}
}

mode = "lr-qnative"

if len(sys.argv) < 2  or sys.argv[1] == "--help" or sys.argv[1] == "-h" :
    print(f"{sys.argv[0]} usage:")
    print(f"USB: {sys.argv[0]} <config>")
    print(f"Network connection: {sys.argv[0]} <ip> <config>")
    exit(1)

viridis = plt.get_cmap('viridis')

system = tof.System()

cameras = []
if len(sys.argv) == 3 :
    status = system.getCameraListAtIp(cameras,sys.argv[1])
    config = sys.argv[2]
elif len(sys.argv) == 2 :
    status = system.getCameraList(cameras)
    config = sys.argv[1]
else :
    print("Too many arguments provided!")
    exit(1)

print("system.getCameraList()", status)

camera1 = cameras[0]

status = camera1.setControl("initialization_config", config)
print("camera1.setControl()", status)

status = camera1.initialize()
print("camera1.initialize()", status)

types = []
status = camera1.getAvailableFrameTypes(types)
print("camera1.getAvailableFrameTypes()", status)
print(types)

camDetails = tof.CameraDetails()
status = camera1.getDetails(camDetails)
print("camera1.getDetails()", status)
print("camera1 details:", "id:", camDetails.cameraId, "connection:", camDetails.connection)

status = camera1.setFrameType(mode)
print("camera1.setFrameType()", status)

status = camera1.start()
print("camera1.start()", status)
    
def normalize(image_scalar, width, height):
    image_scalar_norm = image_scalar / image_scalar.max()

    # Apply the colormap to the scalar image to obtain an RGB image
    image_rgb = viridis(image_scalar_norm)
        
    surface = (image_rgb[:, :, :3] * 255).astype(np.uint8)
    return surface

def animate():
    frame = tof.Frame()
    status = camera1.requestFrame(frame)
    frameDataDetails = tof.FrameDataDetails()
    status = frame.getDataDetails("depth", frameDataDetails)
    image = np.array(frame.getData("depth"), copy=False)
    image = np.rot90(image)
    return pygame.surfarray.make_surface(normalize(image, frameDataDetails.width, frameDataDetails.height))
    
def main():
    pygame.init()
    window_size = (modemapping[mode]["width"], modemapping[mode]["height"])
    screen = pygame.display.set_mode(window_size)

    # display the animation
    done = False
    i = 0
    while not done:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                done = True

        screen.blit(animate(), (0, 0))
        pygame.display.flip()

    # quit Pygame
    pygame.quit()

    status = camera1.stop()
    
main()