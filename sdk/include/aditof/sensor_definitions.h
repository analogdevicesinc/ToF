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
 * @struct DriverConfiguration
 * @brief Describes the configuration of the used driver
 */
struct DriverConfiguration {
    /**
     * @brief Base width value of the frame
    */
    std::string baseWidth;
    /**
     * @brief Base heigth value of the frame
    */
    std::string baseHeigth;
    /**
     * @brief Number of phases
    */
    std::string noOfPhases;
    /**
     * @brief Stores depth data
    */
    std::string depthBits;

    /**
     * @brief Stores ab data
    */
    std::string abBits;

    /**
     * @brief Stores conf data
    */
    std::string confBits;

    /**
     * @brief Stores value from driver
    */
    std::string pixelFormat;

    /**
     * @brief Stores driver width
    */
    int driverWidth;

    /**
     * @brief Stores driver height
    */
    int driverHeigth;

    /**
     * @brief Index of two possbile values sensor values (8bit, 12/16bit)
    */
    int pixelFormatIndex;
};

/**
 * @struct DepthSensorFrameType
 * @brief Describes the type of entire frame that a depth sensor can capture and transmit
 */
struct DepthSensorFrameType {
    
    /**
     * @brief Number associated with the mode
    */
    uint8_t modeNumber;

    /**
     * @brief Stores the content of each frame
    */
    std::vector<std::string> frameContent;

    /**
     * @brief Number of phases
    */
    uint8_t numberOfPhases;

    /**
     * @brief Index of two possbile values sensor values (8bit, 12/16bit) 
    */
    int pixelFormatIndex;

    /**
     * @brief Driver width, can be used for both chipRaw and imagerRaw.
    */
    int frameWidthInBytes;

    /**
     * @brief Driver height, can be used for both chipRaw and imagerRaw.
    */
    int frameHeightInBytes;

    /**
     * @brief Processed data witdh.
    */
    int baseResolutionWidth;

    /**
     * @brief Processed data height.
    */
    int baseResolutionHeight;

    /**
     * @brief Stores the size of metadata
    */
    int metadataSize;

    /**
     * @brief Stores the content of a frame
    */
    DriverConfiguration driverConfiguration;
};

/**
 * @brief prints human readable frame details
 */
inline std::ostream &operator<<(std::ostream &o,
                                const DepthSensorFrameType &a) {
    o << "DepthSensorFrame: T: " << a.modeNumber << "\tW: " << a.baseResolutionWidth
      << "\tH: " << a.baseResolutionHeight << " contains:\n";
    for (auto &content : a.frameContent) {
        o << "\t" << content;
    }
    return o;
}
} // namespace aditof

#endif // SENSOR_DEFINITIONS_H
