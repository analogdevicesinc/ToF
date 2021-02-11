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
#include "camera_cmos.h"

#include <algorithm>
#include <array>
#include <glog/logging.h>

CameraCmos::CameraCmos(
    std::shared_ptr<aditof::DepthSensorInterface> depthSensor,
    std::vector<std::shared_ptr<aditof::StorageInterface>> &eeproms,
    std::vector<std::shared_ptr<aditof::TemperatureSensorInterface>> &tSensors)
    : m_depthSensor(depthSensor) {

    // Check Depth Sensor
    if (!depthSensor) {
        LOG(WARNING) << "Invalid instance of a depth sensor";
        return;
    }
    aditof::SensorDetails sDetails;
    m_depthSensor->getDetails(sDetails);
    m_details.connection = sDetails.connectionType;
}

CameraCmos::~CameraCmos() {}

aditof::Status CameraCmos::initialize() { return aditof::Status::OK; }

aditof::Status CameraCmos::start() { return aditof::Status::OK; }

aditof::Status CameraCmos::stop() { return aditof::Status::OK; }

aditof::Status CameraCmos::setMode(const std::string &mode,
                                   const std::string &modeFilename) {
    return aditof::Status::OK;
}

aditof::Status
CameraCmos::getAvailableModes(std::vector<std::string> &availableModes) const {
    return aditof::Status::OK;
}

aditof::Status CameraCmos::setFrameType(const std::string &frameType) {
    return aditof::Status::OK;
}

aditof::Status CameraCmos::getAvailableFrameTypes(
    std::vector<std::string> &availableFrameTypes) const {
    return aditof::Status::OK;
}

aditof::Status CameraCmos::requestFrame(aditof::Frame *frame,
                                        aditof::FrameUpdateCallback /*cb*/) {
    return aditof::Status::OK;
}

aditof::Status CameraCmos::getDetails(aditof::CameraDetails &details) const {
    using namespace aditof;
    Status status = Status::OK;

    details = m_details;

    return status;
}

std::shared_ptr<aditof::DepthSensorInterface> CameraCmos::getSensor() {
    return m_depthSensor;
}

aditof::Status CameraCmos::getEeproms(
    std::vector<std::shared_ptr<aditof::StorageInterface>> &eeproms) {
    eeproms.clear();

    return aditof::Status::OK;
}

aditof::Status CameraCmos::getTemperatureSensors(
    std::vector<std::shared_ptr<aditof::TemperatureSensorInterface>> &sensors) {
    sensors.clear();

    return aditof::Status::OK;
}

aditof::Status
CameraCmos::getAvailableControls(std::vector<std::string> &controls) const {
    using namespace aditof;
    Status status = Status::OK;

    controls = m_availableControls;

    return status;
}

aditof::Status CameraCmos::setControl(const std::string &control,
                                      const std::string &value) {
    using namespace aditof;
    Status status = Status::OK;

    auto it = std::find(m_availableControls.begin(), m_availableControls.end(),
                        control);
    if (it == m_availableControls.end()) {
        LOG(WARNING) << "Unsupported control";
        return Status::INVALID_ARGUMENT;
    }

    return status;
}

aditof::Status CameraCmos::getControl(const std::string &control,
                                      std::string &value) const {
    using namespace aditof;
    Status status = Status::OK;

    auto it = std::find(m_availableControls.begin(), m_availableControls.end(),
                        control);
    if (it == m_availableControls.end()) {
        LOG(WARNING) << "Unsupported control";
        return Status::INVALID_ARGUMENT;
    }

    return status;
}

aditof::Status CameraCmos::initComputeLibrary(void) {
    // TO DO

    return aditof::Status::UNAVAILABLE;
}

aditof::Status CameraCmos::freeComputeLibrary(void) {
    // TO DO

    return aditof::Status::UNAVAILABLE;
}

std::tuple<aditof::Status, int, int, int> CameraCmos::loadConfigData(void) {
    // TO DO

    return std::make_tuple<aditof::Status, int, int>(
        aditof::Status::UNAVAILABLE, 0, 0, 0);
}

void CameraCmos::freeConfigData(void) {
    // TO DO
}

aditof::Status CameraCmos::isValidFrame(const int /*numTotalFrames*/) {
    // TO DO

    return aditof::Status::UNAVAILABLE;
}

aditof::Status CameraCmos::isValidMode(const uint8_t /*hdr_mode*/) {
    // TO DO

    return aditof::Status::UNAVAILABLE;
}

aditof::Status
CameraCmos::processFrame(uint8_t * /*rawFrame*/, uint16_t * /*captureData*/,
                         uint8_t * /*head*/, const uint16_t /*embed_height*/,
                         const uint16_t /*embed_width*/,
                         aditof::FrameDetails & /*frameDetails*/) {
    // TO DO

    return aditof::Status::UNAVAILABLE;
}

aditof::Status CameraCmos::getCurrentModeInfo(ModeInfo::modeInfo & /*info*/) {
    // TO DO

    return aditof::Status::UNAVAILABLE;
}

aditof::Status CameraCmos::cleanupTempFiles() {
    // TO DO

    return aditof::Status::UNAVAILABLE;
}
