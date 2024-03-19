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
#ifndef SENSOR_DEFINITIONS_H
#define SENSOR_DEFINITIONS_H

#include "aditof/connections.h"

#include <iostream>
#include <string>
#include <vector>

/**
 * @brief Namespace aditof
 */
namespace aditof {

/**
 * @struct SensorDetails
 * @brief Provides details about the device
 */
struct SensorDetails {
    /**
     * @brief The sensor identification data to be used to differentiate between sensors.
     * When on target, id is set to video driver path. When on network, id is set to the IP of the target.
     */
    std::string id;

    /**
     * @brief The type of connection with the sensor
     */
    ConnectionType connectionType;
};

/**
 * @struct DepthSensorFrameContent
 * @brief Describes the content of a frame. For example: A frame could contain
 * depth and IR data.
 */
struct DriverConfiguration {
    std::string depthBits;
    std::string abBits;
    std::string confBits;
    std::string pixelFormat;
    int driverWidth;
    int driverHeigth;
    int pixelFormatIndex;
};

/**
 * @struct DepthSensorFrameType
 * @brief Describes the type of entire frame that a depth sensor can capture and transmit
 */
struct DepthSensorFrameType {
    std::string mode;
    std::vector<std::string> frameContent;
    uint8_t modeNumber;
    int pixelFormatIndex;

    //driver width/height. Can be used for both chipRaw and imagerRaw.
    int frameWidthInBytes;
    int frameHeightInBytes;

    //processed data witdh/height
    int baseResolutionWidth;
    int baseResolutionHeight;

    int metadataSize;
    std::vector<DriverConfiguration> driverConfiguration;
};

/**
 * @brief prints human readable frame details
 */
inline std::ostream &operator<<(std::ostream &o,
                                const DepthSensorFrameType &a) {
    o << "DepthSensorFrame: T: " << a.mode << "\tW: " << a.baseResolutionWidth
      << "\tH: " << a.baseResolutionHeight << " contains:\n";
    for (auto &content : a.frameContent) {
        o << "\t" << content;
    }
    return o;
}
} // namespace aditof

#endif // SENSOR_DEFINITIONS_H
