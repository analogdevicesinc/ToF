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

void PublisherFactory::create(ModeTypes mode, ros::NodeHandle nHandle,
                              const std::shared_ptr<aditof::Camera> &camera,
                              aditof::Frame *frame) {
    ros::Time timeStamp = ros::Time::now();
    switch (mode) {
    case ModeTypes::mode7:
        //ir
        img_publishers.emplace_back(
            nHandle.advertise<sensor_msgs::Image>("aditof_ir", 5));
        imgMsgs.emplace_back(new IRImageMsg(
            camera, frame, sensor_msgs::image_encodings::MONO16, timeStamp));

        //depth
        img_publishers.emplace_back();
        imgMsgs.emplace_back();

        //pointcloud
        img_publishers.emplace_back();
        imgMsgs.emplace_back();

        //camera info
        img_publishers.emplace_back();
        imgMsgs.emplace_back();

        break;
    case ModeTypes::mode10:
        //ir
        img_publishers.emplace_back(
            nHandle.advertise<sensor_msgs::Image>("aditof_ir", 5));
        imgMsgs.emplace_back(new IRImageMsg(
            camera, frame, sensor_msgs::image_encodings::MONO16, timeStamp));

        //depth
        img_publishers.emplace_back();
        imgMsgs.emplace_back();

        //pointcloud
        img_publishers.emplace_back();
        imgMsgs.emplace_back();

        //camera info
        img_publishers.emplace_back();
        imgMsgs.emplace_back();
        
        break;
    default:
        break;
    }
}
void PublisherFactory::update_publishers(
    const std::shared_ptr<aditof::Camera> &camera, aditof::Frame *frame) {
    ros::Time timeStamp = ros::Time::now();
    for (unsigned int i = 0; i < imgMsgs.size(); ++i) {
        imgMsgs.at(i)->FrameDataToMsg(camera, frame, timeStamp);
        imgMsgs.at(i)->publishMsg(img_publishers[i]);
    }
}
void PublisherFactory::delete_publishers() {
    img_publishers.clear();
    imgMsgs.clear();
}
