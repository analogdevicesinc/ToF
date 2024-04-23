/*****************************************************************************
* Copyright (c) 2023 - Analog Devices Inc. All Rights Reserved.
* This software is proprietary & confidential to Analog Devices, Inc.
* and its licensors.
*******************************************************************************
*******************************************************************************
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
* http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.*/

#include <iostream>
#include "../include/adsd3500_util.h"

int main(int argc, char *argv[]) {

    int ret;
    auto adsd3500 = Adsd3500();

    // 1. Reset ADSD3500
    ret = adsd3500.ResetAdsd3500();
    if (ret < 0) {
        printf("Unable to reset Adsd3500.\n");
    }

    // 2. Configure the ADSD3500 and depth compute library with the ini file. 
    ret = adsd3500.OpenAdsd3500();
    if (ret < 0) {
        printf("Unable to open Adsd3500.\n");
    }

    // Set Image mode number
    adsd3500.mode_num = 3; //lr-qnative mode

    // Read .ini config file
    const char*  iniFileName = "config/RawToDepthAdsd3030_lr-qnative.ini";
    ret = adsd3500.GetIniKeyValuePairFromConfig(iniFileName);
    if (ret < 0) {
        printf("Unable to read ini parameters from the Config file.\n");
    }

    // Read Camera Intrinsic and Dealias Parameters from Adsd3500.
    ret = adsd3500.GetIntrinsicsAndDealiasParams();
    if (ret < 0) {
      printf("Unable to get Intrinsic and Dealias parameters from Adsd3500.\n");
    }
    
    // Configure Adsd3500 with .ini file
    ret = adsd3500.ConfigureAdsd3500WithIniParams();
    if (ret < 0) {
      printf("Unable to configure Adsd3500 with ini file.\n");
    }

    // Configure Depth Compute library with Ini Params
    ret = adsd3500.ConfigureDepthComputeLibraryWithIniParams();
    if (ret < 0) {
        printf("Unable to configure Depth Compute Library.\n");
    } 
    
    // Configure V4L2 MIPI Capture Driver and V4L2 Capture Sensor Driver.
    ret = adsd3500.ConfigureDeviceDrivers();
    if (ret < 0) {
        printf("Unable to open Adsd3500.\n");
    }    

    // 3. Complete configuration of depth compute library with CCB parameters from the ADSD3500.
    ret = adsd3500.GetImagerTypeAndCCB(); 
    if (ret < 0) {
        std::cout << "Unable to get the Imager type and CCB." << std::endl;
        return ret;
    }

    // 4. Set up Interrupt Support.


    // 5. Set the Imager Mode.
    usleep(1000 * 5000); // Wait for a period for 5 seconds.

    ret = adsd3500.SetFrameType();
    if (ret < 0) {
        printf("Unable to set frame type Adsd3500.\n");
    }  

    // 6. Set the Stream on
    ret = adsd3500.StartStream();
    if (ret < 0) {
        printf("Unable to start stream.\n");
    }

    // 7. Receive Frames
    int num_frames = 1;
    int buffer_height = adsd3500.xyzDealiasData.n_rows; // 256
    int buffer_width = adsd3500.xyzDealiasData.n_cols; //320
    float totalBits = adsd3500.depthBits + adsd3500.abBits + adsd3500.confBits;
    int buffer_size = buffer_height*buffer_width*totalBits;  // 256*320*5 bytes

    uint16_t* buffer = new uint16_t[buffer_size];
    for (int i = 0; i < num_frames; i++) {   
        ret = adsd3500.RequestFrame(buffer);
        if (ret < 0 || buffer == nullptr) {
            std::cout << "Unable to receive frames from Adsd3500" << std::endl;
        }
    }

    // 8. Get Depth, AB, Confidence Data using Depth Compute Library and store them as .bin file.
    adsd3500.ParseRawDataWithDCL(buffer);
    if (ret < 0) {
        std::cout << "Unable to parse raw frames." << std::endl;
    }

    return 0;
}