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

#include "publisher_factory.h"

PublisherFactory::PublisherFactory(){};

void PublisherFactory::createNew(const rclcpp::Node::SharedPtr &node, image_transport::ImageTransport &it,
                                 const std::shared_ptr<aditof::Camera> &camera,
                                 aditof::Frame **frame, bool enableDepthCompute)
{
    // Get frame types
    aditof::CameraDetails *details_tmp = new aditof::CameraDetails;
    getCameraDataDetails(camera, *details_tmp);

    for (auto iter : (*details_tmp).frameType.dataDetails)
    {
        if (!strcmp(iter.type.c_str(), "ir") && enableDepthCompute == true)
        {
            img_publishers.emplace_back(it.advertise("tof_camera/ir", 2));
            imgMsgs.emplace_back(new IRImageMsg(camera, frame, sensor_msgs::image_encodings::MONO16));
            LOG(INFO) << "Added ir publisher";
        }
        else if (!strcmp(iter.type.c_str(), "depth") && enableDepthCompute == true)
        {
            img_publishers.emplace_back(it.advertise("tof_camera/depth", 2));
            imgMsgs.emplace_back(new DepthImageMsg(camera, frame, sensor_msgs::image_encodings::RGBA8));
            LOG(INFO) << "Added depth publisher";
        }
        else if (!strcmp(iter.type.c_str(), "raw") && enableDepthCompute == false)
        {
            img_publishers.emplace_back(it.advertise("tof_camera/raw", 2));
            imgMsgs.emplace_back(new RAWImageMsg(
                camera, frame, sensor_msgs::image_encodings::MONO16));
            LOG(INFO) << "Added raw data publisher";
        }
    }
    startCamera(camera);
}
void PublisherFactory::updatePublishers(
    const std::shared_ptr<aditof::Camera> &camera, aditof::Frame **frame)
{
    for (unsigned int i = 0; i < imgMsgs.size(); ++i)
    {
        imgMsgs.at(i)->FrameDataToMsg(camera, frame);
        img_publishers.at(i).publish(imgMsgs.at(i)->getMessage());
    }
}
void PublisherFactory::deletePublishers(
    const std::shared_ptr<aditof::Camera> &camera)
{
    stopCamera(camera);
    img_publishers.clear();
    imgMsgs.clear();
}
void PublisherFactory::setDepthFormat(const int val)
{
    for (unsigned int i = 0; i < imgMsgs.size(); ++i)
    {
        if (std::dynamic_pointer_cast<DepthImageMsg>(imgMsgs[i]))
        {
            std::dynamic_pointer_cast<DepthImageMsg>(imgMsgs[i])
                .get()
                ->setDepthDataFormat(val);
        }
    }
}
