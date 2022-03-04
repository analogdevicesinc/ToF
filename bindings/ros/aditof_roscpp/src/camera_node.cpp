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
#include "aditof_roscpp/Aditof_roscppConfig.h"
#include "message_factory.h"
#include "publisher_factory.h"
#include <aditof_utils.h>
#include <dynamic_reconfigure/server.h>
#include <ros/ros.h>

#include "../../../../sdk/include/aditof/camera.h"

using namespace aditof;

std::mutex m_mtxDynamicRec;
std::mutex m_mtxDynamicRec2;

void callback(aditof_roscpp::Aditof_roscppConfig &config,
              PublisherFactory *publisher,
              ros::NodeHandle *nHandle,
              const std::shared_ptr<aditof::Camera> &camera,
              aditof::Frame *frame) {

        //aquire two mutexes
        while(m_mtxDynamicRec.try_lock());
        while(m_mtxDynamicRec2.try_lock());


        switch (config.camera_mode) {
        case 0:
            //clear current publishers
            publisher->delete_publishers();
            //create new publishers
            publisher->create(ModeTypes::mode7, *nHandle, camera, frame);
            LOG(INFO) << "Mode 7 selected";
            break;
        case 1: //mode 10
            //clear current publishers
            publisher->delete_publishers();
            //create new publishers
            publisher->create(ModeTypes::mode10, *nHandle, camera, frame);
            LOG(INFO) << "Mode 10 selected";
            break;
        }
        m_mtxDynamicRec.unlock();
        m_mtxDynamicRec2.unlock();
        //camera->start();
}

int main(int argc, char **argv) {

    std::shared_ptr<Camera> camera = initCamera(argc, argv);
    ROS_ASSERT_MSG(camera, "initCamera call failed");


    setFrameType(camera, "pcm");
    startCamera(camera);

    ros::init(argc, argv, "aditof_camera_node");
    dynamic_reconfigure::Server<aditof_roscpp::Aditof_roscppConfig> server;
    dynamic_reconfigure::Server<
        aditof_roscpp::Aditof_roscppConfig>::CallbackType f;

    //create publishers
    ros::NodeHandle nHandle("aditof_roscpp");
    //create frame for general access
    aditof::Frame frame;
    //request new frame
    getNewFrame(camera, &frame);
    //generate publisherFactory
    PublisherFactory publishers;

    publishers.create(ModeTypes::mode7, nHandle, camera, &frame);
    
    f = boost::bind(&callback, _1, &publishers, &nHandle, camera, &frame);
    server.setCallback(f);

    while ( ros::ok()) {
        while(m_mtxDynamicRec.try_lock());
        while(m_mtxDynamicRec2.try_lock());

        m_mtxDynamicRec.unlock();

        ros::Time tStamp = ros::Time::now();
        getNewFrame(camera, &frame);
        publishers.update_publishers(camera, &frame);
        ros::spinOnce();
        
        m_mtxDynamicRec2.unlock();
    }
    publishers.delete_publishers();
    return 0;
}
