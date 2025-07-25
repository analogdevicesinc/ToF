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
import cv2 as cv
import argparse
from enum import Enum
import sys
import keyboard

ip = "192.168.56.1" # Set to "192.168.56.1" if networking is used.
mode = 3

inWidth = 300
inHeight = 300
WHRatio = inWidth / float(inHeight)
inScaleFactor = 0.007843
meanVal = 127.5
thr = 0.2
WINDOW_NAME = "Display Objects"
WINDOW_NAME_DEPTH = "Display Objects Depth"


class ModesEnum(Enum):
    MODE_NEAR = 0
    MODE_MEDIUM = 1
    MODE_FAR = 2


if __name__ == "__main__":

    parser = argparse.ArgumentParser(
        description='Script to run MobileNet-SSD object detection network ')
    parser.add_argument("--ip", default=ip, help="IP address of ToF Device")
    parser.add_argument("--prototxt", default="MobileNetSSD_deploy.prototxt",
                        help='Path to text network file: '
                             'MobileNetSSD_deploy.prototxt')
    parser.add_argument("--weights", default="MobileNetSSD_deploy.caffemodel",
                        help='Path to weights: '
                             'MobileNetSSD_deploy.caffemodel')
    args = parser.parse_args()
    try:
        net = cv.dnn.readNetFromCaffe(args.prototxt, args.weights)
    except:
        print("Error: Please give the correct location of the prototxt and caffemodel")
        sys.exit(1)

    swapRB = False
    classNames = {0: 'background',
                  1: 'aeroplane', 2: 'bicycle', 3: 'bird', 4: 'boat',
                  5: 'bottle', 6: 'bus', 7: 'car', 8: 'cat', 9: 'chair',
                  10: 'cow', 11: 'diningtable', 12: 'dog', 13: 'horse',
                  14: 'motorbike', 15: 'person', 16: 'pottedplant',
                  17: 'sheep', 18: 'sofa', 19: 'train', 20: 'tvmonitor'}

    system = tof.System()

    print("SDK version: ", tof.getApiVersion(), " | branch: ", tof.getBranchVersion(), " | commit: ", tof.getCommitVersion())

    cameras = []
    status = system.getCameraList(cameras, "ip:"+args.ip)
    if not status:
        print("system.getCameraList(): ", status)

    camera1 = cameras[0]

    status = camera1.initialize()
    if not status:
        print("camera1.initialize() failed with status: ", status)

    modes = []
    status = camera1.getAvailableModes(modes)
    if not status:
        print("system.getAvailableModes() failed with status: ", status)
        
    status = camera1.setMode(mode)
    if not status:
        print("camera1.setSetMode() failed with status:", status)
    
    status = camera1.start()
    if not status:
        print("camera1.start() failed with status:", status)
   
    camDetails = tof.CameraDetails()
    status = camera1.getDetails(camDetails)
    if not status:
        print("system.getDetails() failed with status: ", status)

    # Enable noise reduction for better results
    smallSignalThreshold = 100
    camera1.setControl("noise_reduction_threshold", str(smallSignalThreshold))

    camera_range = 5000
    bitCount = 9
    frame = tof.Frame()

    max_value_of_AB_pixel = 2 ** bitCount - 1
    distance_scale_ab = 255.0 / max_value_of_AB_pixel
    distance_scale = 255.0 / camera_range

    while True:
    
        if keyboard.is_pressed('q'):
            print("'q' key pressed. Exiting loop.")
            break
            
        # Capture frame-by-frame
        status = camera1.requestFrame(frame)
        if not status:
            print("camera1.requestFrame() failed with status: ", status)

        depth_map = np.array(frame.getData("depth"), dtype="uint16", copy=False)
        ab_map = np.array(frame.getData("ab"), dtype="uint16", copy=False)

        # Creation of the AB image
        ab_map = ab_map[0: int(ab_map.shape[0] / 2), :]
        ab_map = distance_scale_ab * ab_map
        ab_map = np.uint8(ab_map)
        ab_map = cv.flip(ab_map, 1)
        ab_map = cv.cvtColor(ab_map, cv.COLOR_GRAY2RGB)

        # Creation of the Depth image
        new_shape = (int(depth_map.shape[0] / 2), depth_map.shape[1])
        depth_map = np.resize(depth_map, new_shape)
        depth_map = cv.flip(depth_map, 1)
        distance_map = depth_map
        depth_map = distance_scale * depth_map
        depth_map = np.uint8(depth_map)
        depth_map = cv.applyColorMap(depth_map, cv.COLORMAP_RAINBOW)

        # Combine depth and AB for more accurate results
        result = cv.addWeighted(ab_map, 0.4, depth_map, 0.6, 0)

        # Start the computations for object detection using DNN
        blob = cv.dnn.blobFromImage(result, inScaleFactor, (inWidth, inHeight), (meanVal, meanVal, meanVal), swapRB)
        
        net.setInput(blob)
        detections = net.forward()

        cols = result.shape[1]
        rows = result.shape[0]

        if cols / float(rows) > WHRatio:
            cropSize = (int(rows * WHRatio), rows)
        else:
            cropSize = (cols, int(cols / WHRatio))

        y1 = int((rows - cropSize[1]) / 2)
        y2 = y1 + cropSize[1]
        x1 = int((cols - cropSize[0]) / 2)
        x2 = x1 + cropSize[0]
        result = result[y1:y2, x1:x2]
        depth_map = depth_map[y1:y2, x1:x2]
        distance_map = distance_map[y1:y2, x1:x2]

        cols = result.shape[1]
        rows = result.shape[0]

        for i in range(detections.shape[2]):
            confidence = detections[0, 0, i, 2]
            if confidence > thr:
                class_id = int(detections[0, 0, i, 1])
                xLeftBottom = int(detections[0, 0, i, 3] * cols)
                yLeftBottom = int(detections[0, 0, i, 4] * rows)
                xRightTop = int(detections[0, 0, i, 5] * cols)
                yRightTop = int(detections[0, 0, i, 6] * rows)

                cv.rectangle(result, (xLeftBottom, yLeftBottom), (xRightTop, yRightTop),
                             (0, 255, 0))
                cv.rectangle(depth_map, (xLeftBottom, yLeftBottom), (xRightTop, yRightTop),
                             (0, 255, 0))
                center = ((xLeftBottom + xRightTop) * 0.5, (yLeftBottom + yRightTop) * 0.5)

                value_x = int(center[0])
                value_y = int(center[1])
                cv.drawMarker(result, (value_x, value_y), (0, 0, 0), cv.MARKER_CROSS)
                cv.drawMarker(depth_map, (value_x, value_y), (0, 0, 0), cv.MARKER_CROSS)

                if class_id in classNames:
                    label_depth = classNames[class_id] + ": " + \
                            "{0:.3f}".format(distance_map[value_x, value_y] / 1000.0) + " meters"
                    label_conf = "Confidence: " + "{0:.4f}".format(confidence)
                    labelSize_depth, baseLine = cv.getTextSize(label_depth, cv.FONT_HERSHEY_SIMPLEX, 0.5, 1)
                    labelSize_conf = cv.getTextSize(label_conf, cv.FONT_HERSHEY_SIMPLEX, 0.5, 1)

                    if labelSize_depth[1] > labelSize_conf[1]:
                        labelSize = labelSize_depth
                    else:
                        labelSize = labelSize_conf
                    yLeftBottom = max(yLeftBottom, labelSize[1])
                    cv.rectangle(result, (value_x - int(labelSize[0] * 0.5), yLeftBottom),
                                 (value_x + int(labelSize[0] * 0.5), yLeftBottom + 2 * labelSize[1] + 2 * baseLine),
                                 (255, 255, 255), cv.FILLED)
                    cv.putText(result, label_depth, (value_x - int(labelSize[0] * 0.5), yLeftBottom + labelSize[1]),
                               cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0))
                    cv.putText(result, label_conf, (value_x - int(labelSize[0] * 0.5), yLeftBottom + 2 * labelSize[1]
                                                    + baseLine),
                               cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0))

                    cv.rectangle(depth_map, (value_x - int(labelSize[0] * 0.5), yLeftBottom),
                                 (value_x + int(labelSize[0] * 0.5), yLeftBottom + 2 * labelSize[1] + 2 * baseLine),
                                 (255, 255, 255), cv.FILLED)
                    cv.putText(depth_map, label_depth, (value_x - int(labelSize[0] * 0.5),  yLeftBottom + labelSize[1]),
                               cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0))
                    cv.putText(depth_map, label_conf, (value_x - int(labelSize[0] * 0.5), yLeftBottom + 2 * labelSize[1]
                                                       + baseLine),
                               cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0))

        # Show image with object detection
        cv.namedWindow(WINDOW_NAME, cv.WINDOW_AUTOSIZE)
        cv.imshow(WINDOW_NAME, result)

        # Show Depth map
        cv.namedWindow(WINDOW_NAME_DEPTH, cv.WINDOW_AUTOSIZE)
        cv.imshow(WINDOW_NAME_DEPTH, depth_map)

        if cv.waitKey(1) >= 0:
            break
            
    camera1.stop()
