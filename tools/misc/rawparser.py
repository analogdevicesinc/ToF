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
#import aditofpython as tof
import numpy as np
import sys
import os
import time
import argparse
import cv2 as cv
import open3d as o3d
import struct

width = 0
height = 0
fps = 0
bytePerPx = 10
startOfFrame = 0
metadataLength = 128
logImage = False
raw_img_dir = '\\sample_raw\\'
fileName = 'frames202309111819'
rawFileType = '.RAW'
binFileType = '.bin'
pngFileType = '.png'
plyFileType = '.ply'

def visualize_ab(filename, directory, index):
    ab_frame = np.zeros([height,width])
    with open ('%s' % filename) as file:
        #parse the AB data from binary file 
        byte_array = np.fromfile(file, dtype=np.uint16)
        ab_frame = np.reshape(byte_array[height*width*0:height*width*1], [height,width])

        #normalize ab data to 8bit image
        norm_ab_frame = cv.normalize(ab_frame, None, 0, 255, cv.NORM_MINMAX, dtype=cv.CV_8U)
        #Apply log transform if logImage is true
        if logImage == True:
            c = 255 / np.log(1 + np.max(norm_ab_frame)) 
            norm_ab_frame = c * (np.log(norm_ab_frame + 1)) 
            norm_ab_frame = np.array(norm_ab_frame, dtype = np.uint8)
        else:
            norm_ab_frame = np.uint8(norm_ab_frame)
        norm_ab_frame = cv.cvtColor(norm_ab_frame, cv.COLOR_GRAY2RGB)
            
        #save ab frame as PNG
        img = o3d.geometry.Image(norm_ab_frame)
        #Save the image to a file
        o3d.io.write_image(directory + 'ab_' + args.filename + '_' + index + pngFileType, img)

def visualize_depth(filename, directory, index):
    depth_frame = np.zeros([height,width])
    with open ('%s' % filename) as file:
        #parse the depth data from binary file 
        byte_array = np.fromfile(file, dtype=np.uint16)
        depth_frame = np.reshape(byte_array[height*width*1:height*width*2], [height,width])

        #normalize depth data to 8bit image
        norm_depth_frame = cv.normalize(depth_frame, None, 0, 255, cv.NORM_MINMAX, dtype=cv.CV_8U)
        norm_depth_frame = np.uint8(norm_depth_frame)
        norm_depth_frame = cv.applyColorMap(norm_depth_frame,cv.COLORMAP_TURBO)
        
        #save depth frame as PNG
        img = o3d.geometry.Image(norm_depth_frame)
        #Save the image to a file
        o3d.io.write_image(directory + 'depth_' + args.filename + '_' + index + pngFileType, img)

def visualize_confidence(filename, directory, index):
    conf_frame = np.zeros([height,width])
    with open ('%s' % filename) as file:
        #parse the confidence data from binary file 
        byte_array = np.fromfile(file, dtype=np.int16)
        conf_frame = np.reshape(byte_array[height*width*5:height*width*6], [height,width])
        conf_frame = cv.applyColorMap(conf_frame, cv2.COLORMAP_VIRIDIS)
            
        #save depth frame as
        img = o3d.geometry.Image(conf_frame)
        #Save the image to a file
        o3d.io.write_image(directory + 'conf_' + args.filename + '_' + index + pngFileType, img)

def visualize_pcloud(filename, directory, index):
    # Create visualizer
    vis = o3d.visualization.Visualizer()
    vis.create_window("PointCloud", 1200, 1200)

    point_cloud = o3d.geometry.PointCloud()
    xyz_frame = np.zeros([height*width,3])
    with open ('%s' % filename) as file:
        byte_array = np.fromfile(file, dtype=np.int16)
        xyz_frame = np.resize(byte_array[height*width*2:height*width*5], (int(len(byte_array[height*width*2:height*width*3])),3))

        point_cloud.points = o3d.utility.Vector3dVector(xyz_frame)
        point_cloud.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
        o3d.io.write_point_cloud(directory + 'pointcloud_' + args.filename + '_' + index + plyFileType,point_cloud)
        vis.add_geometry(point_cloud)
        vis.update_geometry(point_cloud)
        vis.run()
        vis.poll_events()
        vis.update_renderer()
        
def parse_metadata(filename, directory, index):
    elemsList = [
        ('frameWidth',             'H'),
        ('frameHeight',            'H'),
        ('outputconfig',           'B'),
        ('depthPhaseBits',         'B'),
        ('ABBits',                 'B'),
        ('confidenceBits',         'B'),
        ('invalidPhaseValue',      'H'),
        ('frequencyIndex',         'B'),
        ('ABFrequencyIndex',       'B'),
        ('frameNumber',            'L'),
        ('imagerMode',             'B'),
        ('noOfPhases',             'B'),
        ('noOfFrequencies',        'H'),
        ('ElapsedTimeinFrac',      'L'),
        ('ElapsedTimeinSec',       'L'),
        ('sensorTemp',             'L'),
        ('laserTemp',              'L'),
        ('paddingBytes',         '92x'),
    ]
    format_string = "".join(fmt for name, fmt in elemsList)
    #with open ('%s' % filename) as file:
    with open(filename, "rb") as file:
        file.seek(-(metadataLength),2)
        data = file.read(metadataLength)
        values = struct.unpack(format_string, data)
        
        # Create a list of elements with names and values
        elements = [(name, value) for (name, fmt), value in zip(elemsList, values)]
        
        # Save the elements list
        with open(directory + 'metadata_' + args.filename + '_' + index +'.txt', 'w') as outfile:
            outfile.writelines([str(i)+'\n' for i in elements])

def generate_vid(mainDir,numberOfFrames,width,height):
    #create video directory
    vidDir = new_dir + '\\vid_' + args.filename
    os.mkdir(vidDir)
    
    #Create a video writer object
    video = cv.VideoWriter(vidDir + '\\vid_' + args.filename + '.mp4', cv.VideoWriter_fourcc(*"mp4v"), 10, (width*2, height))
    
    #Loop over the AB and depth images and and write them to video
    for i in range(0,numberOfFrames):
        binDir = mainDir +  '\\' + args.filename + '_' + str(i) +'\\' 
        depth_img = cv.imread(binDir + 'depth_' + args.filename + '_' + str(i) + pngFileType)
        ab_img = cv.imread(binDir + 'ab_' + args.filename + '_' + str(i) + pngFileType)

        # concatenate the images horizontally
        new_img = cv.hconcat([depth_img, ab_img])
        new_img = cv.resize(new_img, (width*2,height))
        # save and show the new image 
        video.write(new_img)
 
    

if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description='Scrip to parse a raw file and extract different frame data ')
    parser.add_argument("--filename", default = 'frames202309111819', help="filename to parse")
    args = parser.parse_args()
    print(f"filename: {args.filename}")
    dir_path = os.path.dirname( os.path.abspath(__file__)) + raw_img_dir
    if os.path.exists(dir_path):
        print(f"The directory {dir_path} already exists.")
    else:
        # Create the directory
        os.makedirs(dir_path)
        print(f"The directory {dir_path} was created.")
    
    #identify width, height and number of frames from raw file
    with open(os.path.dirname( os.path.abspath(__file__))+ '\\' + args.filename + rawFileType, 'rb') as f:
        data = f.read(12)
        width = int.from_bytes(data[:4], 'little')
        height = int.from_bytes(data[4:8], 'little')
        fps = int.from_bytes(data[8:12], 'little')
        print(f"width: {width} height: {height} # of frames:  {fps}")
        #show frame details
        file_size = os.path.getsize(os.path.dirname( os.path.abspath(__file__)) + '\\' + args.filename + rawFileType)
        print("file size: " + str(file_size))
        sizeOfHeader = 12;
        sizeOfFrame = (bytePerPx * height * width)+ metadataLength;
        print("frame size: " + str(sizeOfFrame))
        m_numberOfFrames = (file_size - sizeOfHeader) / sizeOfFrame;
        print("number of frames: " + str(m_numberOfFrames))
        f.seek(12)
        data = f.read(file_size - sizeOfHeader)
        m_frameData = np.frombuffer(data, dtype=np.uint8)

    #create directory for output frames
    new_dir = os.path.dirname( os.path.abspath(__file__)) + raw_img_dir + args.filename
    print(new_dir)
    os.mkdir(new_dir)
    first_time_render_pc = 1
    
    for i in range(0, fps):
        # Create frame folders
        frameDir = new_dir + '\\' + args.filename + '_' + str(i) +'\\'
        os.mkdir(frameDir)
        binFileName = frameDir + args.filename + '_' + str(i) + binFileType
        print(binFileName)
        # Open the file in binary mode
        with open(binFileName , "wb") as f:
            endOfFrame = startOfFrame + sizeOfFrame
            f.write(m_frameData[startOfFrame : endOfFrame])
            startOfFrame = endOfFrame
        visualize_ab(binFileName,frameDir,str(i))
        visualize_depth(binFileName,frameDir,str(i))
        if first_time_render_pc:
            visualize_pcloud(binFileName,frameDir,str(i))
            first_time_render_pc = 0
        parse_metadata(binFileName,frameDir,str(i))
    generate_vid(new_dir,fps,width,height)
   