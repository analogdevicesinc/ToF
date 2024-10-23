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

MegaPixel = 1024
qMegaPixel = [512, 640, 256, 320]
confBytesPerPx = 4 
abBytesPerPx = 2
xyzBytesPerPx = 6
depthPerPx = 2
#BIT_MAP = {'confBytesPerPx': 4,'abBytesPerPx': 2, 'depthPerPx': 2,'xyzBytesPerPx':6}

startOfFrame = 0
metadataLength = 128
logImage = False
pngFileType = '.png'
plyFileType = '.ply'

rawParserVersion = "1.0.0"
TOFEvalVersion = "5.0.0"
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
    format_string ='=' +"".join(fmt for name, fmt in elemsList)
    #with open ('%s' % filename) as file:
    with open(filename, "rb") as file:
        file.seek(0)
        data = file.read(metadataLength)
        values = struct.unpack(format_string, data)
        
        # Create a list of elements with names and values
        elements = [(name, value) for (name, fmt), value in zip(elemsList, values)]
        # depthbitsPerPx, abbitsPerPx, confbitsPerPx = elements[3:6]
                
        # print('Bits in depth: %s, AB: %s and Conf: %s' %(depthPerPx, abBytesPerPx, confBytesPerPx))
        
        # Save the elements list
        with open(directory + 'metadata_' + base_filename + '_' + index +'.txt', 'w') as outfile:
            outfile.writelines([str(i)+'\n' for i in elements])

def visualize_depth(filename, directory, index):
    depth_frame = np.zeros([height,width])
    with open ('%s' % filename) as file:
        #parse the depth data from binary file 
        byte_array = np.fromfile(file, dtype=np.uint16, offset = metadataLength, count = height*width)
        depth_frame = np.reshape(byte_array, [height,width])

        #normalize depth data to 8bit image
        norm_depth_frame = cv.normalize(depth_frame, None, 0, 255, cv.NORM_MINMAX, dtype=cv.CV_8U)
        norm_depth_frame = np.uint8(norm_depth_frame)
        norm_depth_frame = cv.applyColorMap(norm_depth_frame,cv.COLORMAP_TURBO)
        
        #save depth frame as
        img = o3d.geometry.Image(norm_depth_frame)
        #Save the image to a file
        o3d.io.write_image(directory + 'depth_' + base_filename + '_' + index + pngFileType, img)

def visualize_ab(filename, directory, index):
    ab_frame = np.zeros([height,width])
    with open ('%s' % filename) as file:
        #parse the AB data from binary file 
        byte_array = np.fromfile(file, dtype=np.uint16,
            offset = depthPerPx * height * width + metadataLength, count = height*width)
        ab_frame = np.reshape(byte_array, [height,width])

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
            
        #save depth frame as
        img = o3d.geometry.Image(norm_ab_frame)
        #Save the image to a file
        o3d.io.write_image(directory + 'ab_' + base_filename + '_' + index + pngFileType, img)

def visualize_confidence(filename, directory, index):
    conf_frame = np.zeros([height,width])
    with open ('%s' % filename) as file:
        #parse the confidence data from binary file 
        byte_array = np.fromfile(file, dtype=np.int16, offset = (depthPerPx + 
                abBytesPerPx)*height*width+metadataLength, count = height*width)
        conf_frame = np.reshape(byte_array, [height,width])
        
        norm_conf_frame = cv.normalize(conf_frame, None, 0, 255, cv.NORM_MINMAX, dtype=cv.CV_8U)
        norm_conf_frame = np.uint8(norm_conf_frame)
        norm_conf_frame = 255 - norm_conf_frame
        
        #save depth frame as
        img = o3d.geometry.Image(norm_conf_frame)
        #Save the image to a file
        o3d.io.write_image(directory + 'conf_' + base_filename + '_' + index + pngFileType, img)

def visualize_pcloud(filename, directory, index):
    # Create visualizer
    vis = o3d.visualization.Visualizer()
    vis.create_window("PointCloud", 1200, 1200)

    point_cloud = o3d.geometry.PointCloud()
    xyz_frame = np.zeros([height*width,3])
    with open ('%s' % filename) as file:
        
        if width in qMegaPixel and height in qMegaPixel:
            byte_array = np.fromfile(file, dtype=np.int16, offset = (confBytesPerPx + depthPerPx + 
                abBytesPerPx)*height*width+metadataLength, count = height*width*3)
        elif width == MegaPixel and height == MegaPixel:
            byte_array = np.fromfile(file, dtype=np.int16, offset = (depthPerPx + 
                abBytesPerPx)*height*width+metadataLength, count = height*width*3)
        
        
        xyz_frame = np.resize(byte_array, (height*width,3))

        point_cloud.points = o3d.utility.Vector3dVector(xyz_frame)
        point_cloud.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
        o3d.io.write_point_cloud(directory + 'pointcloud_' + base_filename 
            + '_' + index + plyFileType,point_cloud)
        vis.add_geometry(point_cloud)
        vis.update_geometry(point_cloud)
        vis.run()
        vis.poll_events()
        vis.update_renderer()
        
def generate_vid(mainDir,numberOfFrames,width,height):
    #create video directory
    vidDir = mainDir + '/vid_' + base_filename
    if not os.path.exists(vidDir):
        os.makedirs(vidDir)
        
    #Create a video writer object
    video = cv.VideoWriter(vidDir + '/vid_' + base_filename + '.mp4', cv.VideoWriter_fourcc(*"mp4v"), 10, (width*2, height))
    
    #Loop over the AB and depth images and and write them to video
    for i in range(0,numberOfFrames):
        binDir = mainDir +  '/' + base_filename + '_' + str(i) +'/' 
        depth_img = cv.imread(binDir + 'depth_' + base_filename + '_' + str(i) + pngFileType)
        ab_img = cv.imread(binDir + 'ab_' + base_filename + '_' + str(i) + pngFileType)

        # concatenate the images horizontally
        new_img = cv.hconcat([depth_img, ab_img])
        new_img = cv.resize(new_img, (width*2,height))
        # save and show the new image 
        video.write(new_img)
 
    
if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description='Script to parse a raw file and extract different frame data ')
    parser.add_argument("filename", type=str ,help="bin filename to parse")
    parser.add_argument("--no_xyz", dest='no_xyz', action='store_true', help="Provide false if input file don't have XYZ. Default assuming false")

   
    args = parser.parse_args()
    #print(args)
    #set to identify  for rendering xyz at first frame
    first_time_render_pc = 1
    if args.no_xyz:
        xyzBytesPerPx = 0
        first_time_render_pc = 0
    
    
    #check if file exist
    if not os.path.exists(args.filename):
        sys.exit(f"Error: {args.filename} does not exist")
    
    print("rawparser version: ", rawParserVersion)
    print("TOF SDK Version: ", TOFEvalVersion)
    
    print(f"filename: {args.filename}")
    base_dir, _ = os.path.splitext(args.filename)
    dir_path = base_dir + "_parsed"
    if os.path.exists(dir_path):
        print(f"The directory {dir_path} already exists.")
    else:
        # Create the directory
        os.makedirs(dir_path)
        print(f"The directory {dir_path} was created.")

    #identify width, height and number of frames from raw file
    with open(args.filename , 'rb') as f:
        data = f.read(5)
        width = int.from_bytes(data[:2], 'little')
        height = int.from_bytes(data[2:4], 'little')
        print(f"width: {width} height: {height}")

        

        #bitsperpixcel
        f.seek(0)
        bitsperpixcel = f.read(8)
        print(bitsperpixcel) 
        
        if not (bitsperpixcel[5]):            
            depthPerPx=0
           
        if not (bitsperpixcel[6]):
            abBytesPerPx=0
            
        if not (bitsperpixcel[7]):
            confBytesPerPx=0
        
          
        print('Bits in depth: %s, AB: %s and Conf: %s' %(depthPerPx, abBytesPerPx, confBytesPerPx))

        
        #show frame details
        file_size = os.path.getsize(args.filename)
        print("file size: " + str(file_size))

        #identify the image size
        if width in qMegaPixel and height in qMegaPixel:
            bytePerPx = abBytesPerPx + depthPerPx + confBytesPerPx + xyzBytesPerPx
        elif width == MegaPixel and height == MegaPixel:
            bytePerPx = abBytesPerPx + depthPerPx + xyzBytesPerPx
        else:
            sys.exit("Error: no byte per pixel data on this file, cannot parse.")
            
        sizeOfFrame = (bytePerPx * height * width)+ metadataLength;
        print("frame size: " + str(sizeOfFrame))
        m_numberOfFrames = int((file_size) / sizeOfFrame);
        print("number of frames: " + str(m_numberOfFrames))
        
        f.seek(0)
        data = f.read(file_size)
        m_frameData = np.frombuffer(data, dtype=np.uint8)
      
    
    
    for i in range(0, m_numberOfFrames):
        # Create frame folders
        base_filename, _ = os.path.splitext(os.path.basename(args.filename))
        frameDir = dir_path + '/' + base_filename + '_' + str(i) +'/'
        if not os.path.exists(frameDir):
            os.makedirs(frameDir)
        binFileName = frameDir + base_filename + '_' + str(i) + ".bin"
        print(binFileName)
     
        # Open the file in binary mode
        with open(binFileName , "wb") as f:
            endOfFrame = startOfFrame + sizeOfFrame
            f.write(m_frameData[startOfFrame : endOfFrame])
            startOfFrame = endOfFrame
        parse_metadata(binFileName,frameDir,str(i))   
        if depthPerPx: 
            visualize_depth(binFileName,frameDir,str(i))
        if abBytesPerPx:
            visualize_ab(binFileName,frameDir,str(i))
        
        #check if confidence data is included
        if (width in qMegaPixel and height in qMegaPixel) and confBytesPerPx:
            visualize_confidence(binFileName,frameDir,str(i))

        #render and visualize point cloud at first frame only
        if first_time_render_pc:
            visualize_pcloud(binFileName,frameDir,str(i))
            first_time_render_pc = 0
            
    #generate video stitched from ab and depth    
    generate_vid(dir_path, m_numberOfFrames, width, height)
    
