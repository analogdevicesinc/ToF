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
#include "aditof/sensor_enumerator_factory.h"
#include "aditof/sensor_enumerator_interface.h"
#include <aditof/version.h>
#include <fstream>
#ifdef USE_GLOG
#include <glog/logging.h>
#else
#include <aditof/log.h>
#endif

#include <ios>
#include <iostream>
#include <map>
#include <windows.h>

using namespace aditof;

int main(int argc, char *argv[]) {
    std::string ip;

    if (argc > 1) {
        ip = argv[1];
    } else {
        ip = "10.43.0.1";
    }
    Status status = Status::OK;

    std::unique_ptr<SensorEnumeratorInterface> enumerator =
        SensorEnumeratorFactory::buildNetworkSensorEnumerator(ip);

    std::vector<std::shared_ptr<DepthSensorInterface>> depthSensors;
    std::string uboot;
    std::string kernel;
    std::string sd_ver;

    status = enumerator->searchSensors();
    if (status != Status::OK) {
        LOG(ERROR) << "Could not search sensor!";
        return 1;
    }

    status = enumerator->getUbootVersion(uboot);
    if (status != Status::OK) {
        LOG(ERROR) << "Could not get Uboot version!";
        return 1;
    }
    LOG(INFO) << "Uboot Version: " << uboot;

    status = enumerator->getKernelVersion(kernel);
    if (status != Status::OK) {
        LOG(ERROR) << "Could not get kernel version!";
        return 1;
    }
    LOG(INFO) << "KernelVersion: " << kernel;

    status = enumerator->getSdVersion(sd_ver);
    if (status != Status::OK) {
        LOG(ERROR) << "Could not get sd version!";
        return 1;
    }
    LOG(INFO) << "sd card version: " << sd_ver;

    status = enumerator->getDepthSensors(depthSensors);
    if (status != Status::OK) {
        LOG(ERROR) << "Could not get depthSensors";
        return 1;
    }
    auto depthSensor = depthSensors.front();
    std::string sensorName;
    status = depthSensor->getName(sensorName);
    LOG(INFO) << "sensorname: " << sensorName;
    return 0;
}