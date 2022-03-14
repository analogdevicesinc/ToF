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

PublisherFactory::PublisherFactory() { m_currentMode = ModeTypes::NONE; };

void PublisherFactory::createNew(ModeTypes mode, ros::NodeHandle nHandle,
                                 const std::shared_ptr<aditof::Camera> &camera,
                                 aditof::Frame **frame) {

    ros::Time timeStamp = ros::Time::now();

    if (*frame != nullptr)
        (*frame)->~Frame();

    deletePublishers(camera);
    switch (mode) {
    case ModeTypes::mode7: {
        setFrameType(camera, "qmp");
        break;
    }
    case ModeTypes::mode10: {
        setFrameType(camera, "mp");
        break;
    }
    default:
        break;
    }
    *frame = new aditof::Frame();

    // Get frame types
    aditof::CameraDetails *details_tmp = new aditof::CameraDetails;
    getCameraDataDetails(camera, *details_tmp);

    for (auto iter : (*details_tmp).frameType.dataDetails) {
        if (!std::strcmp(iter.type.c_str(), "ir")) {
            img_publishers.emplace_back(
                nHandle.advertise<sensor_msgs::Image>("aditof_ir", 5));
            imgMsgs.emplace_back(new IRImageMsg(
                camera, frame, sensor_msgs::image_encodings::MONO16,
                timeStamp));
            LOG(INFO) << "Added ir publisher";
        } else if (!std::strcmp(iter.type.c_str(), "depth")) {
            img_publishers.emplace_back(
                nHandle.advertise<sensor_msgs::Image>("aditof_depth", 5));
            imgMsgs.emplace_back(new DepthImageMsg(
                camera, frame, sensor_msgs::image_encodings::RGBA8, timeStamp));
            LOG(INFO) << "Added depth publisher";

        } else if (!std::strcmp(iter.type.c_str(), "xyz")) {
            img_publishers.emplace_back(
                nHandle.advertise<sensor_msgs::PointCloud2>("aditof_pcloud",
                                                            5));
            imgMsgs.emplace_back(new PointCloud2Msg(camera, frame, timeStamp));
            LOG(INFO) << "Added point_cloud publisher";
        } else if (!std::strcmp(iter.type.c_str(), "embedded_header")) {
            //add embedded header publisher
        }
    }

    startCamera(camera);
    m_currentMode = mode;
}
void PublisherFactory::updatePublishers(
    const std::shared_ptr<aditof::Camera> &camera, aditof::Frame **frame) {
    ros::Time timeStamp = ros::Time::now();
    for (unsigned int i = 0; i < imgMsgs.size(); ++i) {
        imgMsgs.at(i)->FrameDataToMsg(camera, frame, timeStamp);
        imgMsgs.at(i)->publishMsg(img_publishers[i]);
    }
}
void PublisherFactory::deletePublishers(
    const std::shared_ptr<aditof::Camera> &camera) {
    stopCamera(camera);
    img_publishers.clear();
    imgMsgs.clear();
}
void PublisherFactory::setDepthFormat(const int val) {
    for (unsigned int i = 0; i < imgMsgs.size(); ++i) {
        if (std::dynamic_pointer_cast<DepthImageMsg>(imgMsgs[i])) {
            std::dynamic_pointer_cast<DepthImageMsg>(imgMsgs[i])
                .get()
                ->setDepthDataFormat(val);
        }
    }
}
