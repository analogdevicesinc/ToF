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
#include "connections/usb/usb_sensor_enumerator.h"
#include "connections/usb/linux/usb_linux_utils.h"
#include "connections/usb/usb_depth_sensor.h"
#include "connections/usb/usb_storage.h"
#include "connections/usb/usb_temperature_sensor.h"
#include "connections/usb/usb_utils.h"
#include "usb_buffer.pb.h"
#include "utils.h"

#include <dirent.h>
#include <fcntl.h>
#include <glog/logging.h>
#include <linux/videodev2.h>
#include <memory>
#include <sys/stat.h>

using namespace std;
using namespace aditof;

UsbSensorEnumerator::~UsbSensorEnumerator() = default;

Status UsbSensorEnumerator::searchSensors() {
    Status status = Status::OK;

    LOG(INFO) << "Looking for USB connected sensors";

    const char *path = "/dev/";
    DIR *d;

    d = opendir(path);
    if (!d) {
        LOG(WARNING) << "Failed to open dir at path: " << path;
        return Status::UNREACHABLE;
    }

    struct dirent *dir;
    char sset[] = "video";
    while ((dir = readdir(d)) != nullptr) {
        if (strspn(sset, (dir->d_name)) != 5) {
            continue;
        }
        string driverPath(path);
        driverPath += dir->d_name;

        struct stat st;
        if (-1 == stat(driverPath.c_str(), &st)) {
            LOG(WARNING) << "Cannot identify '" << driverPath
                         << "' error: " << errno << "(" << strerror(errno)
                         << ")";
            continue;
        }

        if (!S_ISCHR(st.st_mode)) {
            LOG(WARNING) << driverPath << " is no device";
            continue;
        }

        int fd = open(driverPath.c_str(), O_RDWR | O_NONBLOCK, 0);
        if (-1 == fd) {
            LOG(WARNING) << "Cannot open '" << driverPath
                         << "' error: " << errno << "(" << strerror(errno)
                         << ")";
            continue;
        }

        struct v4l2_capability cap;
        if (-1 == UsbLinuxUtils::xioctl(fd, VIDIOC_QUERYCAP, &cap)) {
            if (EINVAL == errno) {
                LOG(WARNING) << driverPath << " is not V4L2 device";
                close(fd);
                continue;
            } else {
                LOG(WARNING) << "VIDIOC_QUERYCAP";
            }
        }

        // Skip device which does not support VIDIOC_G_FMT
        struct v4l2_format fmt;
        CLEAR(fmt);
        fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        if (-1 == UsbLinuxUtils::xioctl(fd, VIDIOC_G_FMT, &fmt)) {
            close(fd);
            continue;
        }

#if defined(ITOF)
        std::string devName("ADI CMOS TOF UVC Gadget");
#else
        std::string devName("ADI TOF DEPTH SENSOR");
#endif

        if (strncmp(reinterpret_cast<const char *>(cap.card), devName.c_str(),
                    devName.length()) != 0) {
            close(fd);
            continue;
        }

        if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE)) {
            LOG(WARNING) << driverPath << " is no video capture device";
            close(fd);
            continue;
        }

        if (!(cap.capabilities & V4L2_CAP_STREAMING)) {
            LOG(WARNING) << driverPath << " does not support streaming i/o";
            close(fd);
            continue;
        }

        SensorInfo sInfo;
        sInfo.driverPath = driverPath;

        DLOG(INFO) << "Found USB capture device at: " << driverPath;

        // Query the sensors that are available on target

        // Send request
        usb_payload::ClientRequest requestMsg;
        requestMsg.set_func_name(usb_payload::FunctionName::SEARCH_SENSORS);
        requestMsg.add_func_int32_param(
            1); // TO DO: Check if this is needed. Without it, the serialized string will be empty.

        std::string requestStr;
        requestMsg.SerializeToString(&requestStr);
        status = UsbLinuxUtils::uvcExUnitSendRequest(fd, requestStr);
        if (status != Status::OK) {
            LOG(ERROR) << "Request to search for sensors failed";
            return status;
        }

        // Read UVC gadget response
        std::string responseStr;
        status = UsbLinuxUtils::uvcExUnitGetResponse(fd, responseStr);
        if (status != Status::OK) {
            LOG(ERROR) << "Request to search for sensors failed";
            return status;
        }
        usb_payload::ServerResponse responseMsg;
        bool parsed = responseMsg.ParseFromString(responseStr);
        if (!parsed) {
            LOG(ERROR) << "Failed to deserialize string containing UVC gadget "
                          "response";
            return Status::INVALID_ARGUMENT;
        }

        DLOG(INFO) << "Received the following message with "
                      "available sensors from target: "
                   << responseMsg.DebugString();

        if (responseMsg.status() != usb_payload::Status::OK) {
            LOG(ERROR) << "Search for sensors operation failed on UVC gadget";
            return static_cast<Status>(responseMsg.status());
        }

        // If request and response went well, extract data from response
        m_sensorsInfo.emplace_back(sInfo);

        m_storagesInfo.clear();
        for (int i = 0; i < responseMsg.sensors_info().storages_size(); ++i) {
            auto storage = responseMsg.sensors_info().storages(i);
            m_storagesInfo.emplace_back(
                std::make_pair(storage.name(), storage.id()));
        }

        m_temperatureSensorsInfo.clear();
        for (int i = 0; i < responseMsg.sensors_info().temp_sensors_size();
             ++i) {
            auto tempSensor = responseMsg.sensors_info().temp_sensors(i);
            m_storagesInfo.emplace_back(
                std::make_pair(tempSensor.name(), tempSensor.id()));
        }
    }

    closedir(d);

    return status;
}

Status UsbSensorEnumerator::getDepthSensors(
    std::vector<std::shared_ptr<DepthSensorInterface>> &depthSensors) {

    depthSensors.clear();

    for (const auto &sInfo : m_sensorsInfo) {
        auto sensor = std::make_shared<UsbDepthSensor>(sInfo.driverPath);
        depthSensors.emplace_back(sensor);
    }

    return Status::OK;
}

Status UsbSensorEnumerator::getStorages(
    std::vector<std::shared_ptr<StorageInterface>> &storages) {

    storages.clear();

    for (const auto &nameAndId : m_storagesInfo) {
        auto storage =
            std::make_shared<UsbStorage>(nameAndId.first, nameAndId.second);
        storages.emplace_back(storage);
    }

    return Status::OK;
}

Status UsbSensorEnumerator::getTemperatureSensors(
    std::vector<std::shared_ptr<TemperatureSensorInterface>>
        &temperatureSensors) {

    temperatureSensors.clear();

    for (const auto &nameAndId : m_temperatureSensorsInfo) {
        auto tSensor = std::make_shared<UsbTemperatureSensor>(nameAndId.first,
                                                              nameAndId.second);
        temperatureSensors.emplace_back(tSensor);
    }

    return Status::OK;
}
