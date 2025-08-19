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

#include "connections/usb/usb_utils.h"
#include "usb_buffer.pb.h"
#include "utils.h"

using namespace std;
using namespace aditof;

void UsbUtils::protoMsgToDepthSensorFrameTypes(
    std::vector<aditof::DepthSensorFrameType> &depthSensorFrameTypes,
    const usb_payload::DepthSensorFrameTypeVector &protoMsg) {
    using namespace google::protobuf::io;

    depthSensorFrameTypes.clear();

    for (const auto &frameTypeMsg : protoMsg.depthsensorframetypes()) {
        aditof::DepthSensorFrameType depthSensorFrameType;

        depthSensorFrameType.type = frameTypeMsg.type();
        depthSensorFrameType.width = frameTypeMsg.width();
        depthSensorFrameType.height = frameTypeMsg.height();

        for (const auto &frameContentMsg : frameTypeMsg.depthsensorframecontent()) {
            DepthSensorFrameContent depthSensorFrameContent;

            depthSensorFrameContent.width = frameContentMsg.width();
            depthSensorFrameContent.height = frameContentMsg.height();
            depthSensorFrameContent.type = frameContentMsg.type();
            depthSensorFrameType.content.push_back(depthSensorFrameContent);
        }

        depthSensorFrameTypes.push_back(depthSensorFrameType);
    }
}

void UsbUtils::depthSensorFrameTypeToProtoMsg(
    const aditof::DepthSensorFrameType &depthSensorFrameType,
    usb_payload::DepthSensorFrameType *protoMsg) {

    protoMsg->set_type(depthSensorFrameType.type);
    protoMsg->set_width(depthSensorFrameType.width);
    protoMsg->set_height(depthSensorFrameType.height);

    for (const auto &depthSensorFrameContent : depthSensorFrameType.content) {

        auto depthSensorFrameContentMsg =
            protoMsg->add_depthsensorframecontent();
        depthSensorFrameContentMsg->set_type(depthSensorFrameContent.type);
        depthSensorFrameContentMsg->set_width(depthSensorFrameContent.width);
        depthSensorFrameContentMsg->set_height(depthSensorFrameContent.height);
    }
}
