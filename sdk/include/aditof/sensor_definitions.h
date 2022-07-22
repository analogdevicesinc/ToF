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

#include <string>
#include <vector>
#include <iostream>

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
     * @brief The sensor's name
     */
    std::string sensorName;

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
struct DepthSensorFrameContent {
    /**
     * @brief The type of frame content supported by the sensor
     */
    std::string type;

    /**
     * @brief The width of the frame content that the sensor can capture
     */
    unsigned int width;

    /**
     * @brief The height of the frame content that the sensor can capture
     */
    unsigned int height;
};

/**
 * @struct DepthSensorFrameFormat
 * @brief Describes the type of entire frame that a depth sensor can capture and transmit
 */
struct DepthSensorFrameType {
    /**
     * @brief The type of entire frame supported by the sensor
     */
    std::string type;

    /**
     * @brief Describes the content of the entire frame
     */
    std::vector<struct DepthSensorFrameContent> content;

    /**
     * @brief The width of the entire frame that the sensor can capture
     */
    unsigned int width;

    /**
     * @brief The height of the entire frame that the sensor can capture
     */
    unsigned int height;


};
inline std::ostream& operator << (std::ostream& o, const DepthSensorFrameContent& a)
{
    o << "T: " << a.type << "\tW: " << a.width << "\tH: "<< a.height << "\n";
    return o;
}

inline std::ostream& operator << (std::ostream& o, const DepthSensorFrameType& a)
{
    o << "DepthSensorFrame: T: " << a.type << "\tW: " << a.width << "\tH: "<< a.height << " contains:\n";
        for (const DepthSensorFrameContent content : a.content){
            o << "\t" <<content;
        }
    return o;
}
} // namespace aditof



#endif // SENSOR_DEFINITIONS_H
