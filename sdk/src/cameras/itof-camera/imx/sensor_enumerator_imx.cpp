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

#include "connections/target/target_sensor_enumerator.h"
#include "sensor_names.h"
#include "target_definitions.h"

#include <dirent.h>
#include <glog/logging.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <string>
#include <sys/stat.h>
#include <unistd.h>

using namespace aditof;

namespace local {

aditof::Status findDevicePathsAtVideo(const std::string &video,
                                      std::string &dev_name,
                                      std::string &subdev_name) {
    using namespace aditof;
    using namespace std;

    char *buf;
    int size = 0;

    /* Run media-ctl to get the video processing pipes */
    char cmd[64];
    sprintf(cmd, "sudo media-ctl -d %s --print-dot", video.c_str());
    FILE *fp = popen(cmd, "r");
    if (!fp) {
        LOG(WARNING) << "Error running media-ctl";
        return Status::GENERIC_ERROR;
    }

    /* Read the media-ctl output stream */
    buf = (char *)malloc(128 * 1024);
    while (!feof(fp)) {
        fread(&buf[size], 1, 1, fp);
        size++;
    }
    pclose(fp);
    buf[size] = '\0';

    /* Search command media-ctl for device/subdevice name */
    string str(buf);
    free(buf);

    size_t pos = str.find("mxc_isi.0.capture");
    if (pos != string::npos) {
        dev_name = str.substr(pos + strlen("mxc_isi.0.capture") + 2, strlen("/dev/mediaX"));
    }
    else {
        return Status::GENERIC_ERROR;
    } 

    pos = str.find("addicmos spi0.0");
    if (pos != string::npos) {
        subdev_name = str.substr(pos + strlen("addicmos spi0.0") + 2, strlen("/dev/v4l-subdevX"));
    }
    else {
        return Status::GENERIC_ERROR;
    } 

    return Status::OK;
}

}; // namespace local

Status TargetSensorEnumerator::searchSensors() {
    Status status = Status::OK;

    LOG(INFO) << "Looking for sensors on the target";

    // Find all video device paths
    std::vector<std::string> videoPaths;
    const std::string videoDirPath("/dev/");
    const std::string videoBaseName("media");

    DIR *dirp = opendir(videoDirPath.c_str());
    struct dirent *dp;
    while ((dp = readdir(dirp))) {
        if (!strncmp(dp->d_name, videoBaseName.c_str(),
                     videoBaseName.length())) {
            std::string fullvideoPath = videoDirPath + std::string(dp->d_name);
            videoPaths.emplace_back(fullvideoPath);
        }
    }
    closedir(dirp);

    // Identify any eligible time of flight cameras
    for (const auto &video : videoPaths) {
        DLOG(INFO) << "Looking at: " << video << " for an eligible TOF camera";

        std::string devPath;
        std::string subdevPath;

        status = local::findDevicePathsAtVideo(video, devPath, subdevPath);
        if (status != Status::OK) {
            LOG(WARNING) << "failed to find device paths at video: " << video;
            return status;
        }

        if (devPath.empty() || subdevPath.empty()) {
            continue;
        }
        DLOG(INFO) << "Considering: " << video << " an eligible TOF camera";

        SensorInfo sInfo;
        sInfo.sensorType = SensorType::SENSOR_ADSD3100;
        sInfo.driverPath = devPath;
        sInfo.subDevPath = subdevPath;
        sInfo.captureDev = CAPTURE_DEVICE_NAME;
        m_sensorsInfo.emplace_back(sInfo);
    }

    return status;
}

