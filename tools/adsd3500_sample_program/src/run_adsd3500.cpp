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

#include "../include/adsd3500_util.h"
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>
#include <math.h>

int main(int argc, char *argv[]) {

    int ret;
    auto adsd3500 = Adsd3500();

    // Default Arguments.
    adsd3500.mode_num = 2;      // Image mode number.
    int num_frames = 1;         // Number of frames
    adsd3500.ccb_as_master = 1; // Enables/Disbales CCB as master.

    // Parse Arguments from the Command line.
    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "-m") == 0 && i + 1 < argc) {
            adsd3500.mode_num = std::atoi(argv[++i]);
        } else if (std::strcmp(argv[i], "-n") == 0 && i + 1 < argc) {
            num_frames = std::atoi(argv[++i]);
        } else {
            std::cerr << "Usage: " << argv[0]
                      << " [-m mode_num] [-n num_frames]\n";
            return -1;
        }
    }

    // Reset ADSD3500
    ret = adsd3500.ResetAdsd3500();
    if (ret < 0) {
        printf("Unable to reset Adsd3500.\n");
        return ret;
    }

    // Configure the ADSD3500 and depth compute library with the ini file.
    ret = adsd3500.OpenAdsd3500();
    if (ret < 0) {
        printf("Unable to open Adsd3500.\n");
        return ret;
    }

    // Get the Imager Type and the CCB.
    ret = adsd3500.GetImagerTypeAndCCB();
    if (ret < 0) {
        std::cout << "Unable to get the Imager type and CCB." << std::endl;
        return ret;
    }

    // Set Ini file path.
    const char *iniFileName = nullptr;
    if (adsd3500.imagerType == ImagerType::IMAGER_ADSD3030) {
        iniFileName =
            IniFilePath::adsd3030ModeToConfigFileMap[adsd3500.mode_num].c_str();
    } else if (adsd3500.imagerType == ImagerType::IMAGER_ADSD3100) {
        iniFileName =
            IniFilePath::adsd3100ModeToConfigFileMap[adsd3500.mode_num].c_str();
    } else {
        std::cout << "Unsupported Imager.. Exiting!" << std::endl;
        return 0;
    }

    // Get Ini Key value pairs.
    if (iniFileName == nullptr) {
        std::cout << "Ini File not found.. Exiting!" << std::endl;
        return 0;
    }
    std::cout << "Ini File Path: " << iniFileName << std::endl;
    ret = adsd3500.GetIniKeyValuePair(iniFileName);
    if (ret < 0) {
        printf("Unable to read ini parameters from the Config file.\n");
        return ret;
    }

    // Read Camera Intrinsic and Dealias Parameters from Adsd3500.
    ret = adsd3500.GetIntrinsicsAndDealiasParams();
    if (ret < 0) {
        printf(
            "Unable to get Intrinsic and Dealias parameters from Adsd3500.\n");
        return ret;
    }

    // Configure Adsd3500 with .ini file
    ret = adsd3500.ConfigureAdsd3500WithIniParams();
    if (ret < 0) {
        printf("Unable to configure Adsd3500 with ini file.\n");
        return ret;
    }

    // Configure Depth Compute library with Ini Params
    ret = adsd3500.ConfigureDepthComputeLibraryWithIniParams();
    if (ret < 0) {
        printf("Unable to configure Depth Compute Library.\n");
        return ret;
    }

    // Configure V4L2 MIPI Capture Driver and V4L2 Capture Sensor Driver.
    ret = adsd3500.ConfigureDeviceDrivers();
    if (ret < 0) {
        printf("Unable to open Adsd3500.\n");
        return ret;
    }

    // Set up Interrupt Support.
    // TODO.

    // Set the Imager Mode.
    usleep(1000 * 5000); // Wait for a period for 5 seconds.

    ret = adsd3500.SetFrameType();
    if (ret < 0) {
        printf("Unable to set frame type Adsd3500.\n");
        return ret;
    }

    // Set the Stream on
    ret = adsd3500.StartStream();
    if (ret < 0) {
        printf("Unable to start stream.\n");
        return ret;
    }

    int buffer_height, buffer_width, total_pixels, buffer_size;
    buffer_height = adsd3500.xyzDealiasData.n_rows;
    buffer_width = adsd3500.xyzDealiasData.n_cols;
    total_pixels = buffer_height * buffer_width;
    if (adsd3500.inputFormat == "raw8") { // For QMP modes.
        float totalBits =
            adsd3500.depthBits + adsd3500.abBits + adsd3500.confBits;
        buffer_size = total_pixels * ceil(totalBits / 16);
    } else if (adsd3500.inputFormat == "mipiRaw12_8") {
        if (adsd3500.mode_num == 0 || adsd3500.mode_num == 1) { // For MP modes
            buffer_size =
                adsd3500.frame.frameHeight * adsd3500.frame.frameWidth;
        }
    }

    uint16_t *depth_buffer = new uint16_t[total_pixels * num_frames];
    uint16_t *ab_buffer = new uint16_t[total_pixels * num_frames];
    uint8_t *conf_buffer = new uint8_t[total_pixels * num_frames];

    std::ofstream ab("out_ab.bin", std::ios::binary);
    std::ofstream depth("out_depth.bin", std::ios::binary);
    std::ofstream conf("out_conf.bin", std::ios::binary);

    std::cout << "Number of Frames requested: " << num_frames << std::endl;

    for (int i = 0; i < num_frames; i++) {
        // Receive Frames
        uint16_t *buffer = new uint16_t[buffer_size];
        ret = adsd3500.RequestFrame(buffer);
        if (ret < 0 || buffer == nullptr) {
            std::cout << "Unable to receive frames from Adsd3500" << std::endl;
        }

        // Get Depth, AB, Confidence Data using Depth Compute Library and store them as .bin file.
        adsd3500.ParseRawDataWithDCL(buffer);
        if (ret < 0) {
            std::cout << "Unable to parse raw frames." << std::endl;
        }

        memcpy(ab_buffer + i * total_pixels,
               adsd3500.tofi_compute_context->p_ab_frame,
               total_pixels * sizeof(uint16_t));
        memcpy(depth_buffer + i * total_pixels,
               adsd3500.tofi_compute_context->p_depth_frame,
               total_pixels * sizeof(uint16_t));
        memcpy(conf_buffer + i * total_pixels,
               adsd3500.tofi_compute_context->p_conf_frame,
               total_pixels * sizeof(uint8_t));
    }

    // Store AB, Depth and Confidence frames on to a .bin files.
    ab.write((char *)ab_buffer, total_pixels * num_frames * sizeof(uint16_t));
    ab.close();

    // Store Depth frame to a .bin file.
    depth.write((char *)depth_buffer,
                total_pixels * num_frames * sizeof(uint16_t));
    depth.close();

    // Store Confidence frame to a .bin file.
    conf.write((char *)conf_buffer,
               total_pixels * num_frames * sizeof(uint8_t));
    conf.close();

    delete[] ab_buffer;
    delete[] depth_buffer;
    delete[] conf_buffer;

    // Stop Stream and Close Camera
    // Handled by the ADSD3500 class destructor.

    return 0;
}