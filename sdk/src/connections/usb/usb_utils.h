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
#ifndef USB_UTILS_H
#define USB_UTILS_H

#include <aditof/frame_definitions.h>
#include <aditof/sensor_definitions.h>
#include <aditof/status_definitions.h>
#include <string>
#include <vector>

namespace usb_payload {
  class DepthSensorFrameTypeVector;
  class DepthSensorFrameType;
}

class UsbUtils {
  public:

    /**
     * @brief Converts from protobuf message to aditof type (vector of DepthSensorFrameType)
     * @param[out] depthSensorFrameTypes - the vector obtained from conversion
     * @param protoMsg - The protobuf message to be converted
     */
    static void protoMsgToDepthSensorFrameTypes(
        std::vector<aditof::DepthSensorFrameType> &depthSensorFrameTypesVector,
        const usb_payload::DepthSensorFrameTypeVector &protoMsg);

    /**
     * @brief Converts a DepthSensorFrameType to a protobuf message.
     * @param depthSensorFrameType - the structure to convert.
     * @param[out] protoMsg - the buffer cotaining the serialized data.
     */
    static void depthSensorFrameTypeToProtoMsg(
        const aditof::DepthSensorFrameType &depthSensorFrameType,
        usb_payload::DepthSensorFrameType *protoMsg);
};

#endif // USB_UTILS_H
