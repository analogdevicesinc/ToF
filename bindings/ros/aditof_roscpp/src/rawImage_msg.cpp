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
 * FOR ANY DRAWECT, INDRAWECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#include "rawImage_msg.h"
using namespace aditof;

RAWImageMsg::RAWImageMsg() {}

RAWImageMsg::RAWImageMsg(const std::shared_ptr<aditof::Camera> &camera,
                         aditof::Frame **frame, std::string encoding,
                         ros::Time tStamp) {

    msg.encoding = encoding;
    //FrameDataToMsg(camera, frame, tStamp);
}

void RAWImageMsg::FrameDataToMsg(const std::shared_ptr<Camera> &camera,
                                 aditof::Frame **frame, ros::Time tStamp) {

    aditof::CameraDetails *details_tmp = new aditof::CameraDetails;
    getCameraDataDetails(camera, *details_tmp);
    for (auto iter : (*details_tmp).frameType.dataDetails) {
        if (!std::strcmp(iter.type.c_str(), "raw")) {
            msg.width = iter.width;
            msg.height = iter.height;
        }
    }
    setMetadataMembers(msg.width, msg.height, tStamp);

    uint16_t *frameData = getFrameData(frame, "raw");
    if (!frameData) {
        LOG(ERROR) << "getFrameData call failed";
        return;
    }

    setDataMembers(camera, frameData);
}

void RAWImageMsg::setMetadataMembers(int width, int height, ros::Time tStamp) {
    msg.header.stamp = tStamp;
    msg.header.frame_id = "aditof_raw_img";

    msg.encoding = msg.encoding;
    msg.is_bigendian = false;

    int pixelByteCnt = sensor_msgs::image_encodings::bitDepth(msg.encoding) /
                       8 *
                       sensor_msgs::image_encodings::numChannels(msg.encoding);
    msg.step = width * pixelByteCnt;

    msg.data.resize(msg.step * height);
}

void RAWImageMsg::setDataMembers(const std::shared_ptr<Camera> &camera,
                                 uint16_t *frameData) {
    if (msg.encoding.compare(sensor_msgs::image_encodings::MONO16) == 0) {

        uint8_t *msgDataPtr = msg.data.data();
        std::memcpy(msgDataPtr, frameData, msg.step * msg.height);
    } else
        ROS_ERROR("Image encoding invalid or not available");
}

void RAWImageMsg::publishMsg(const ros::Publisher &pub) { pub.publish(msg); }
