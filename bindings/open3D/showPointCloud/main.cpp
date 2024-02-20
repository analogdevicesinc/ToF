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
#include <aditof/depth_sensor_interface.h>
#include <aditof/frame.h>
#include <aditof/system.h>
#include <aditof/version.h>
#ifdef USE_GLOG
#include <glog/logging.h>
#else
#include <aditof/log.h>
#endif

#include <command_parser.h>
#include <open3d/Open3D.h>

using namespace aditof;
using namespace std::chrono;
using namespace open3d;

uint16_t MAX_DISTANCE = 1000;
uint16_t MIN_DISTANCE = 200;
bool AB_LOG10 = false;

static const char Help_Menu[] =
    R"(PointCloud-Open3D usage:
    PointCloud-Open3D CONFIG
    PointCloud-Open3D (-h | --help)
    PointCloud-Open3D [-ip | --ip <ip>] [-m | --m <mode>] CONFIG

    Arguments:
      CONFIG            Input config_default.json file (which has *.ccb and *.cfg)

    Options:
      -h --help              Show this screen.
      -m --m <mode>          Mode to capture data in. [default: 0]
      -colormap --colormap   Disable the overlay of ab image on top of the colormap. [default: 0]
                             Open3D visualiser supports multiple colormap renderings at runtime. Please check:
                             https://www.open3d.org/docs/latest/tutorial/Basic/visualization.html#Rendering-styles
                             For more details.

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
        {"-colormap", {"--colormap", false, "", "0", false}},
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
    bool overlayColormap = 0;

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

    if (!command_map["-colormap"].value.empty()) {
        overlayColormap = std::stoi(command_map["-colormap"].value);
    }

    System system;

    std::vector<std::shared_ptr<Camera>> cameras;
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

    // Registering a callback to be executed when ADSD3500 issues an interrupt
    std::shared_ptr<DepthSensorInterface> sensor = camera->getSensor();
    aditof::SensorInterruptCallback callback = [](Adsd3500Status status) {
        LOG(INFO) << "Running the callback for which the status of ADSD3500 "
                     "has been "
                     "forwarded. ADSD3500 status = "
                  << status;
    };
    Status registerCbStatus =
        sensor->adsd3500_register_interrupt_callback(callback);
    if (status != Status::OK) {
        LOG(WARNING) << "Could not register callback";
    }

    status = camera->initialize(configFile);
    if (status != Status::OK) {
        LOG(ERROR) << "Could not initialize camera!";
        return 0;
    }

    aditof::CameraDetails cameraDetails;
    camera->getDetails(cameraDetails);

    LOG(INFO) << "SD card image version: " << cameraDetails.sdCardImageVersion;
    LOG(INFO) << "Kernel version: " << cameraDetails.kernelVersion;
    LOG(INFO) << "U-Boot version: " << cameraDetails.uBootVersion;

    std::vector<std::string> frameTypes;
    camera->getAvailableFrameTypes(frameTypes);
    if (frameTypes.empty()) {
        std::cout << "no frame type avaialble!";
        return 0;
    }

    if (modeName.empty()) {
        status = camera->getFrameTypeNameFromId(mode, modeName);
        if (status != Status::OK) {
            LOG(ERROR) << "Mode: " << mode
                       << " is invalid for this type of camera!";
            return 0;
        }
    }

    status = camera->setFrameType(modeName);
    if (status != Status::OK) {
        LOG(ERROR) << "Could not set camera frame type!";
        return 0;
    }

    status = camera->start();
    if (status != Status::OK) {
        LOG(ERROR) << "Could not start the camera!";
        return 0;
    }

    bool is_geometry_added_pointcloud = false;
    visualization::Visualizer pointcloud_vis;
    pointcloud_vis.CreateVisualizerWindow(
        "ADTF3175D Eval Kit Open3d Point Cloud Example - ESC to Exit");

    auto pointcloud_ptr = std::make_shared<geometry::PointCloud>();
    std::chrono::time_point<std::chrono::high_resolution_clock> start, end;
    uint16_t frcnt = 0;

    while (pointcloud_vis.PollEvents()) {

        if (frcnt == 0) {
            start = std::chrono::high_resolution_clock::now();
        }

        aditof::Frame frame;
        /* Request frame from camera */
        status = camera->requestFrame(&frame);
        if (status != Status::OK) {
            LOG(ERROR) << "Could not request frame!";
            return 0;
        }
        frcnt++;

        int16_t *pcloud;
        frame.getData("xyz", (uint16_t **)&pcloud);
        uint16_t *pab;
        frame.getData("ab", (uint16_t **)&pab);

        FrameDataDetails fDetails;
        frame.getDataDetails("xyz", fDetails);
        const uint32_t fsize = fDetails.width * fDetails.height;

        pointcloud_ptr->points_.clear();
        pointcloud_ptr->colors_.clear();

        uint16_t ab_max = 0;
        uint16_t ab_min = static_cast<uint16_t>(-1);

        uint32_t cnt_valid_pts = 0;
        for (uint32_t cnt = 0; cnt < fsize; cnt++) {
            int16_t _x = pcloud[3 * cnt];
            int16_t _y = pcloud[3 * cnt + 1];
            int16_t _z = pcloud[3 * cnt + 2];

            if (_z != 0 && _z > MIN_DISTANCE && _z < MAX_DISTANCE) {
                pointcloud_ptr->points_.emplace_back(Eigen::Vector3d(
                    static_cast<double>(_x), static_cast<double>(_y),
                    static_cast<double>(_z)));
                cnt_valid_pts++;

                if (pab[cnt] > ab_max) {
                    ab_max = pab[cnt];
                }
                if (pab[cnt] < ab_min) {
                    ab_min = pab[cnt];
                }
            }
        }

        pointcloud_ptr->colors_.resize(cnt_valid_pts);

        const double rangeZ = ab_max - ab_min;
        const double c = (AB_LOG10) ? (255.0 / std::log10(1 + ab_max)) : 0;

        uint32_t idx = 0;
        if (!overlayColormap) {
            for (uint32_t cnt = 0; cnt < fsize; cnt++) {
                int16_t _z = pcloud[3 * cnt + 2];

                if (_z != 0 && _z > MIN_DISTANCE && _z < MAX_DISTANCE) {
                    double _clr =
                        (static_cast<double>(pab[cnt]) - ab_min) / rangeZ;
                    if (AB_LOG10) {
                        _clr = c * std::log10(_clr + 1);
                    }
                    pointcloud_ptr->colors_[idx++] =
                        Eigen::Vector3d(_clr, _clr, _clr);
                }
            }
        }

        if (is_geometry_added_pointcloud == false) {
            pointcloud_vis.AddGeometry(pointcloud_ptr);
            is_geometry_added_pointcloud = true;
        } else {
            pointcloud_vis.UpdateGeometry(pointcloud_ptr);
            pointcloud_vis.UpdateRender();
        }

        if (frcnt == 10) {
            end = std::chrono::high_resolution_clock::now();
            double seconds =
                std::chrono::duration_cast<duration<double>>(end - start)
                    .count();
            LOG(INFO) << "fps = " << ((double)frcnt / seconds)
                      << " voxel cnt = " << idx;
            frcnt = 0;
        }
    }
    pointcloud_vis.DestroyVisualizerWindow();

    status = camera->stop();
    if (status != Status::OK) {
        LOG(ERROR) << "Could not stop the camera!";
        return 0;
    }

    // Example on how to unregister a callback from ADSD3500 interupts
    if (registerCbStatus == Status::OK) {
        sensor->adsd3500_unregister_interrupt_callback(callback);
    }

    return 0;
}
