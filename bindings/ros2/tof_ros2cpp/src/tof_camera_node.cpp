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
#include <aditof_utils.h>
#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include "aditof/camera.h"
#include <aditof_sensor_msg.h>
#include <publisher_factory.h>

// #include "image_transport/image_transport.hpp"
// #include "rclcpp/rclcpp.hpp"

using namespace aditof;

std::mutex m_mtxDynamicRec;
std::mutex m_mtxDynamicRec2;

using namespace std::chrono_literals;

int main(int argc, char **argv)
{
    /*
    pos 0 - ip
    pos 1 - config_path
    pos 2 - use_depthCompute
    pos 3 - mode
    */
    std::string *arguments = parseArgs(argc, argv);

    // Initializing camera and establishing connection
    std::shared_ptr<Camera> camera = initCamera(arguments);
    // Setting camera parameters
    (arguments[2] == "true") ? enableCameraDepthCompute(camera, true) : enableCameraDepthCompute(camera, false);
    (arguments[3] == "1") ? setFrameType(camera, "qmp") : setFrameType(camera, "mp");

    // Creating camera frame for the API
    auto tmp = new Frame;
    aditof::Frame **frame = &tmp;
    startCamera(camera);

    // Creating camera node
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("tof_camera_publisher", options);

    // Creating Image transporter
    image_transport::ImageTransport it(node);

    // Creating publisher
    PublisherFactory publishers;
    publishers.createNew(node, it, camera, frame, (arguments[2] == "true") ? true : false);


    while (rclcpp::ok())
    {
        getNewFrame(camera, frame);
        publishers.updatePublishers(camera, frame);
        rclcpp::spin_some(node);
    }
    return 0;
}
