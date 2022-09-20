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
#include "depthImage_msg.h"
using namespace aditof;

DepthImageMsg::DepthImageMsg() {}

DepthImageMsg::DepthImageMsg(const std::shared_ptr<aditof::Camera> &camera,
                             aditof::Frame **frame, std::string encoding)
{
    imgEncoding = encoding;
    FrameDataToMsg(camera, frame);
}

void DepthImageMsg::FrameDataToMsg(const std::shared_ptr<Camera> &camera,
                                   aditof::Frame **frame)
{
    FrameDetails fDetails;
    (*frame)->getDetails(fDetails);

    setMetadataMembers(fDetails.width, fDetails.height);

    uint16_t *frameData = getFrameData(frame, "depth");
    if (!frameData)
    {
        LOG(ERROR) << "getFrameData call failed";
        return;
    }

    setDataMembers(camera, frameData);
}

void DepthImageMsg::setMetadataMembers(int width, int height)
{
    // message.header.stamp = tStamp;
    message.header.frame_id = "aditof_depth_img";

    message.width = width;
    message.height = height;
    message.encoding = imgEncoding;
    message.is_bigendian = false;

    int pixelByteCnt = sensor_msgs::image_encodings::bitDepth(imgEncoding) / 8 *
                       sensor_msgs::image_encodings::numChannels(imgEncoding);
    message.step = width * pixelByteCnt;

    message.data.resize(message.step * height);
}

void DepthImageMsg::setDataMembers(const std::shared_ptr<Camera> &camera,
                                   uint16_t *frameData)
{

    if (message.encoding.compare(sensor_msgs::image_encodings::RGBA8) == 0)
    {
        std::vector<uint16_t> depthData(frameData,
                                        frameData + message.width * message.height);
        dataToRGBA8(0, 0x0fff, frameData);
    }
    else if (message.encoding.compare(sensor_msgs::image_encodings::MONO16) ==
             0)
    {
        memcpy(message.data.data(), frameData, 2 * message.width * message.height);
    }
    else
        LOG(ERROR) << "Image encoding invalid or not available";
}

void DepthImageMsg::dataToRGBA8(uint16_t min_range, uint16_t max_range,
                                uint16_t *data)
{
    uint8_t *msgDataPtr = message.data.data();
    int32_t delta = static_cast<uint32_t>(max_range - min_range);

    for (unsigned int i = 0; i < message.width * message.height; i++)
    {
        // normalized value
        double norm_val = static_cast<double>(
            static_cast<double>(data[i] - min_range) / delta);
        double hue = norm_val * INDIGO + (1.0f - norm_val) * RED;

        Rgba8Color color = HSVtoRGBA8(hue, SAT, VAL);
        memcpy(msgDataPtr, &color, 4);
        msgDataPtr += 4;
    }
}

Rgba8Color DepthImageMsg::HSVtoRGBA8(double hue, double sat, double val)
{
    double c = 0.0, m = 0.0, x = 0.0;
    double h = hue / 60.0;

    c = sat * val;
    x = c * (1.0 - std::abs(std::fmod(h, 2) - 1.0));
    m = val - c;

    Rgb32Color rgb32;

    rgb32.r = m;
    rgb32.g = m;
    rgb32.b = m;

    if (h <= 1.0)
    {
        rgb32.r += c;
        rgb32.g += x;
    }
    else if (h <= 2.0)
    {
        rgb32.r += x;
        rgb32.g += c;
    }
    else if (h <= 3.0)
    {
        rgb32.g += c;
        rgb32.b += x;
    }
    else if (h <= 4.0)
    {
        rgb32.g += x;
        rgb32.b += c;
    }
    else if (h <= 5.0)
    {
        rgb32.r += x;
        rgb32.b += c;
    }
    else if (h <= 6.0)
    {
        rgb32.r += c;
        rgb32.b += x;
    }

    Rgba8Color rgba8;

    rgba8.r = floor(rgb32.r >= 1.0 ? 255 : rgb32.r * 256.0);
    rgba8.g = floor(rgb32.g >= 1.0 ? 255 : rgb32.g * 256.0);
    rgba8.b = floor(rgb32.b >= 1.0 ? 255 : rgb32.b * 256.0);
    rgba8.a = 0XFF;

    return rgba8;
}

// void DepthImageMsg::publishMsg(const rclcpp::Publisher<sensor_msgs::msg::Image> &pub) { pub.publish(message); }

void DepthImageMsg::setDepthDataFormat(int value)
{
    message.encoding = (value == 0) ? sensor_msgs::image_encodings::RGBA8
                                    : sensor_msgs::image_encodings::MONO16;
    imgEncoding = (value == 0) ? sensor_msgs::image_encodings::RGBA8
                               : sensor_msgs::image_encodings::MONO16;
}

sensor_msgs::msg::Image DepthImageMsg::getMessage()
{
    return message;
}
