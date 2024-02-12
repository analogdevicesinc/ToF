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
#include <aditof/camera.h>
#include <aditof/frame.h>
#include <aditof/system.h>
#include <aditof/version.h>
#ifdef USE_GLOG
#include <glog/logging.h>
#else
#include <aditof/log.h>
#endif

#include "../aditof_open3d.h"
#include <command_parser.h>

static const uint8_t colormap[3 * 256] = {
#include "colormap.txt"
};

using namespace aditof;

static const char Help_Menu[] =
    R"(PointCloud usage:
    PointCloud CONFIG
    PointCloud (-h | --help)
    PointCloud [-ip | --ip <ip>] [-m | --m <mode>] CONFIG

    Arguments:
      CONFIG            Input config_default.json file (which has *.ccb and *.cfg)

    Options:
      -h --help          Show this screen.
      -m --m <mode>      Mode to capture data in. [default: 0]

    NOTE: -m | --m argument supports both index and string (0/sr-native)

    Valid mode (-m | --m) options are:
        0: short-range native
        1: long-range native
        2: short-range Qnative
        3: long-range Qnative
        4: pcm-native
        5: long-range mixed
        6: short-range mixed
)";

int main(int argc, char *argv[]) {
    std::map<std::string, struct Argument> command_map = {
        {"-h", {"--help", false, "", "", false}},
        {"-ip", {"--ip", false, "", "", true}},
        {"-m", {"--m", false, "", "0", true}},
        {"config", {"CONFIG", true, "last", "", true}}};

    CommandParser command;
    std::string arg_error;
    google::InitGoogleLogging(argv[0]);
    FLAGS_alsologtostderr = 1;

    command.parseArguments(argc, argv, command_map);
    int result = command.checkArgumentExist(command_map, arg_error);
    if (result != 0) {
        LOG(ERROR) << "Argument " << arg_error << " doesn't exist!";
        std::cout << Help_Menu;
        return -1;
    }

    result = command.helpMenu();
    if (result == 1) {
        std::cout << Help_Menu;
        return 0;
    } else if (result == -1) {
        LOG(ERROR) << "Usage of argument -h/--help"
                   << " is incorrect! Help argument should be used alone!";
        std::cout << Help_Menu;
        return -1;
    }

    result = command.checkValue(command_map, arg_error);
    if (result != 0) {
        LOG(ERROR) << "Argument: " << command_map[arg_error].long_option
                   << " doesn't have assigned or default value!";
        std::cout << Help_Menu;
        return -1;
    }

    result = command.checkMandatoryArguments(command_map, arg_error);
    if (result != 0) {
        std::string argName = (arg_error == "-config")
                                  ? "CONFIG"
                                  : command_map[arg_error].long_option;

        LOG(ERROR) << "Mandatory argument: " << argName << " missing";
        std::cout << Help_Menu;
        return -1;
    }

    result = command.checkMandatoryPosition(command_map, arg_error);
    if (result != 0) {
        std::string argName = (arg_error == "-config")
                                  ? "CONFIG"
                                  : command_map[arg_error].long_option;

        LOG(ERROR) << "Mandatory argument " << argName
                   << " is not on its correct position ("
                   << command_map[arg_error].position << ").";
        std::cout << Help_Menu;
        return -1;
    }

    LOG(INFO) << "SDK version: " << aditof::getApiVersion()
              << " | branch: " << aditof::getBranchVersion()
              << " | commit: " << aditof::getCommitVersion();

    Status status = Status::OK;
    std::string configFile;
    std::string ip;
    uint32_t mode = 0;

    // Parsing mode type
    std::string modeName;
    try {
        std::size_t counter;
        mode = std::stoi(command_map["-m"].value, &counter);
        if (counter != command_map["-m"].value.size()) {
            throw command_map["-m"].value.c_str();
        }
    } catch (const char *name) {
        modeName = name;
    } catch (const std::exception &) {
        modeName = command_map["-m"].value;
    }

    configFile = command_map["config"].value;

    if (!command_map["-ip"].value.empty()) {
        ip = "ip:" + command_map["-ip"].value;
    }

    System system;

    std::vector<std::shared_ptr<Camera>> cameras;
    system.getCameraList(cameras);
    if (!ip.empty()) {
        system.getCameraList(cameras, ip);
    } else {
        system.getCameraList(cameras);
    }

    if (cameras.empty()) {
        LOG(WARNING) << "No cameras found";
        return 0;
    }

    auto camera = cameras.front();
    status = camera->initialize(configFile);
    if (status != Status::OK) {
        LOG(ERROR) << "Could not initialize camera!";
        return 0;
    }

    std::vector<std::string> frameTypes;
    camera->getAvailableFrameTypes(frameTypes);
    if (frameTypes.empty()) {
        LOG(ERROR) << "No frame type available!";
        return 0;
    }

    status = camera->setFrameType(frameTypes.front());
    if (status != Status::OK) {
        LOG(ERROR) << "Could not set camera frame type!";
        return 0;
    }

    std::vector<std::string> modes;
    camera->getAvailableModes(modes);
    if (modes.empty()) {
        LOG(ERROR) << "No camera modes available!";
        return 0;
    }

    status = camera->setMode(modes[0]);
    if (status != Status::OK) {
        LOG(ERROR) << "Could not set camera mode!";
        return 0;
    }

    aditof::Frame frame;

    /* Get the camera details */
    aditof::CameraDetails cameraDetails;
    camera->getDetails(cameraDetails);
    int camera_rangeMax = cameraDetails.maxDepth;
    int camera_rangeMin = cameraDetails.minDepth;
    int bitCount = cameraDetails.bitCount;

    aditof::IntrinsicParameters intrinsics = cameraDetails.intrinsics;
    double fx = intrinsics.fx;
    double fy = intrinsics.fy;
    double cx = intrinsics.cx;
    double cy = intrinsics.cy;

    /* Enable noise reduction for better results */
    const int smallSignalThreshold = 100;
    camera->setControl("noise_reduction_threshold",
                       std::to_string(smallSignalThreshold));

    /* Request frame from camera */
    status = camera->requestFrame(&frame);
    if (status != Status::OK) {
        LOG(ERROR) << "Could not request frame!";
        return 0;
    }
    aditof::FrameDataDetails frameDetails;
    frame.getDataDetails("depth", frameDetails);
    int frameHeight = static_cast<int>(frameDetails.height);
    int frameWidth = static_cast<int>(frameDetails.width);

    /* Create visualizer for depth and AB images */
    auto visualized_ab_img = std::make_shared<geometry::Image>();
    visualized_ab_img->Prepare(frameWidth, frameHeight, 1, 1);
    visualization::Visualizer ab_vis;
    ab_vis.CreateVisualizerWindow("AB Image", 2 * frameWidth, 2 * frameHeight);
    bool is_geometry_added_ab = false;

    auto visualized_depth_img = std::make_shared<geometry::Image>();
    visualized_depth_img->Prepare(frameWidth, frameHeight, 3, 1);
    visualization::Visualizer depth_vis;
    depth_vis.CreateVisualizerWindow("Depth Image", 2 * frameWidth,
                                     2 * frameHeight);
    bool is_geometry_added_depth = false;

    /* Create visualizer for pointcloud */
    visualization::Visualizer pointcloud_vis;
    pointcloud_vis.CreateVisualizerWindow("Pointcloud", 1200, 1200);
    bool is_geometry_added_pointcloud = false;

    const float PI_VALUE = 3.14;
    std::shared_ptr<geometry::PointCloud> pointcloud_ptr = nullptr;
    Eigen::Matrix4d transformation = Eigen::Matrix4d::Identity();
    transformation.block<3, 3>(0, 0) = static_cast<Eigen::Matrix3d>(
        Eigen::AngleAxisd(PI_VALUE, Eigen::Vector3d::UnitX()));
    camera::PinholeCameraIntrinsic intrinsicParameters(frameWidth, frameHeight,
                                                       fx, fy, cx, cy);

    bool is_window_closed = true;
    while (true) {
        /* Request frame from camera */
        status = camera->requestFrame(&frame);
        if (status != Status::OK) {
            LOG(ERROR) << "Could not request frame!";
            return 0;
        }
        geometry::Image depth_image;
        status = fromFrameToDepthImg(frame, camera_rangeMin, camera_rangeMax,
                                     depth_image);
        if (status != Status::OK) {
            LOG(ERROR) << "Could not convert from frame to Image!";
        }

        geometry::Image depth16bits_image;
        status = fromFrameTo16bitsDepth(frame, depth16bits_image);
        if (status != Status::OK) {
            LOG(ERROR) << "Could not convert from frame to Image!";
        }

        geometry::Image ab_image;
        status = fromFrameToABImg(frame, bitCount, ab_image);
        if (status != Status::OK) {
            LOG(ERROR) << "Could not convert from frame to Image!";
        }

        geometry::Image depth_color;
        depth_color.Prepare(frameWidth, frameHeight, 3, 1);
        for (int i = 0; i < frameHeight * frameWidth; i++) {
            memcpy(depth_color.data_.data() + i * 3,
                   &colormap[depth_image.data_[i] * 3], 3);
        }

        geometry::Image ab_color;
        ab_color.Prepare(frameWidth, frameHeight, 3, 1);
        for (int i = 0, j = 0; i < frameHeight * frameWidth; i++, j = j + 3) {
            ab_color.data_[j] = ab_image.data_[i];
            ab_color.data_[j + 1] = ab_image.data_[i];
            ab_color.data_[j + 2] = ab_image.data_[i];
        }

        geometry::Image color_image;
        color_image.Prepare(frameWidth, frameHeight, 3, 1);
        for (int i = 0; i < frameHeight * frameWidth * 3; i = i + 3) {
            color_image.data_[i] =
                ab_color.data_[i] * 0.5 + depth_color.data_[i] * 0.5;
            color_image.data_[i + 1] =
                ab_color.data_[i + 1] * 0.5 + depth_color.data_[i + 1] * 0.5;
            color_image.data_[i + 2] =
                ab_color.data_[i + 2] * 0.5 + depth_color.data_[i + 2] * 0.5;
        }

        visualized_ab_img->data_ = ab_image.data_;
        if (!is_geometry_added_ab) {
            ab_vis.AddGeometry(visualized_ab_img);
            is_geometry_added_ab = true;
        }

        visualized_depth_img->data_ = depth_color.data_;
        if (!is_geometry_added_depth) {
            depth_vis.AddGeometry(visualized_depth_img);
            is_geometry_added_depth = true;
        }
        ab_vis.UpdateGeometry();
        ab_vis.PollEvents();
        ab_vis.UpdateRender();

        depth_vis.UpdateGeometry();
        depth_vis.PollEvents();
        depth_vis.UpdateRender();

        /* create and show pointcloud */
        auto rgbd_ptr = geometry::RGBDImage::CreateFromColorAndDepth(
            color_image, depth16bits_image, 1000.0, 3.0, false);
        if (!pointcloud_ptr) {
            pointcloud_ptr = geometry::PointCloud::CreateFromRGBDImage(
                *rgbd_ptr, intrinsicParameters);

            auto bounding_box = pointcloud_ptr->GetAxisAlignedBoundingBox();
            Eigen::Matrix4d trans_to_origin = Eigen::Matrix4d::Identity();
            trans_to_origin.block<3, 1>(0, 3) = bounding_box.GetCenter() * -1.0;

            pointcloud_ptr->Transform(trans_to_origin.inverse() *
                                      transformation * trans_to_origin);
        } else {
            auto temp = geometry::PointCloud::CreateFromRGBDImage(
                *rgbd_ptr, intrinsicParameters);

            auto bounding_box = temp->GetAxisAlignedBoundingBox();
            Eigen::Matrix4d trans_to_origin = Eigen::Matrix4d::Identity();
            trans_to_origin.block<3, 1>(0, 3) = bounding_box.GetCenter() * -1.0;

            temp->Transform(trans_to_origin.inverse() * transformation *
                            trans_to_origin);

            pointcloud_ptr->points_ = temp->points_;
            pointcloud_ptr->normals_ = temp->normals_;
            pointcloud_ptr->colors_ = temp->colors_;
        }

        if (!is_geometry_added_pointcloud) {
            pointcloud_vis.AddGeometry(pointcloud_ptr);
            is_geometry_added_pointcloud = true;
        }

        if (is_window_closed == false) {
            pointcloud_vis.DestroyVisualizerWindow();
            ab_vis.DestroyVisualizerWindow();
            depth_vis.DestroyVisualizerWindow();
            break;
        } else {
            pointcloud_vis.UpdateGeometry();
            is_window_closed = pointcloud_vis.PollEvents();
            pointcloud_vis.UpdateRender();
        }
    }

    return 0;
}
