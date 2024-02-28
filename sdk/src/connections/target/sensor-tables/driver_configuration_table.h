/*
 * BSD 3-Clause License
 *
 * Copyright (c) 2019, Analog Devices, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <aditof/depth_sensor_interface.h>
#include <vector>

struct DriverConfiguration {
    std::string depthBits;
    std::string abBits;
    std::string confBits;
    std::string pixelFormat;
    int driverWidth;
    int driverHeigth;
    int pixelFormatIndex;
};

const std::vector<DriverConfiguration> m_adsd3100Mode0 = {
    /* imagerType  mode depth  ab   conf  pixelF dWidth dHeight pixFIndex
   sr-native */
    {"16", "16", "0", "raw16", 2048, 3072, 0},
    {"12", "12", "0", "raw16_bits12_shift4", 1024, 3072, 1},
    {"12", "16", "0", "mipiRaw12_8", 2048, 2560, 0},
};
const std::vector<DriverConfiguration> m_adsd3100Mode1 = {
    /* imagerType  mode depth  ab   conf  pixelF dWidth dHeight pixFIndex
   sr-native */
    {"16", "16", "0", "raw16", 2048, 3072, 0},
    {"12", "12", "0", "raw16_bits12_shift4", 1024, 3072, 1},
    {"12", "16", "0", "mipiRaw12_8", 2048, 2560, 0},
};
const std::vector<DriverConfiguration> m_adsd3100Mode2_3_4_6 = {
    /* imagerType  mode depth  ab   conf  pixelF dWidth dHeight pixFIndex
   sr-native */
    {"16", "16", "0", "raw16", 2048, 3072, 0},
    {"12", "12", "0", "raw16_bits12_shift4", 1024, 3072, 1},
    {"12", "16", "0", "mipiRaw12_8", 2048, 2560, 0},
};

const std::vector<DriverConfiguration> m_adsd3030Mode0 = {
    /* imagerType  mode depth  ab   conf  pixelF dWidth dHeight pixFIndex
   sr-native */
    {"16", "16", "0", "raw16", 2048, 3072, 0},
    {"12", "12", "0", "raw16_bits12_shift4", 1024, 3072, 1},
    {"12", "16", "0", "mipiRaw12_8", 2048, 2560, 0},
};
const std::vector<DriverConfiguration> m_adsd3030Mode1 = {
    /* imagerType  mode depth  ab   conf  pixelF dWidth dHeight pixFIndex
   sr-native */
    {"16", "16", "0", "raw16", 2048, 3072, 0},
    {"12", "12", "0", "raw16_bits12_shift4", 1024, 3072, 1},
    {"12", "16", "0", "mipiRaw12_8", 2048, 2560, 0},
};
const std::vector<DriverConfiguration> m_adsd3030Mode2_3_4_6 = {
    /* imagerType  mode depth  ab   conf  pixelF dWidth dHeight pixFIndex
   sr-native */
    {"16", "16", "0", "raw16", 2048, 3072, 0},
    {"12", "12", "0", "raw16_bits12_shift4", 1024, 3072, 1},
    {"12", "16", "0", "mipiRaw12_8", 2048, 2560, 0},
};

//defined locally. To be updated
struct DepthSensorFrameTypeUpdated {
    std::string mode;
    std::vector<std::string> frameContent;
    uint8_t modeNumer;
    int pixelFormatIndex;

    //driver width/height. Can be used for both chipRaw and imagerRaw.
    int frameWidthInBytes;
    int frameHeightInBytes;

    //processed data witdh/height
    int baseResolutionWidth;
    int baseResolutionHeight;

    int metadataSize;
    std::vector<DriverConfiguration>
};

std::vector<DepthSensorFrameTypeUpdated> adsd3100_standardModes = {
    {"sr-native",
     {"raw", "depth", "ab", "conf", "xyz", "metadata"},
     0,
     0,
     0,
     0,
     1024,
     1024,
     128,
     m_adsd3100Mode0},
    {"lr-native",
     {"raw", "depth", "ab", "conf", "xyz", "metadata"},
     1,
     0,
     0,
     0,
     1024,
     1024,
     128,
     m_adsd3100Mode1}};
