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
#include "network_depth_sensor.h"
#include "connections/network/network.h"

#ifdef USE_GLOG
#include <glog/logging.h>
#else
#include <aditof/log.h>
#endif
#include <chrono>
#include <unordered_map>

struct CalibrationData {
    std::string mode;
    float gain;
    float offset;
    uint16_t *cache;
};
struct NetworkDepthSensor::ImplData {
    NetworkHandle handle;
    std::string ip;
    aditof::DepthSensorFrameType frameTypeCache;
    std::unordered_map<std::string, CalibrationData> calibration_cache;
    bool opened;
    Network::InterruptNotificationCallback cb;
};

NetworkDepthSensor::NetworkDepthSensor(const std::string &name,
                                       const std::string &ip)
    : m_implData(new NetworkDepthSensor::ImplData), m_stopServerCheck(false) {

    m_implData->cb = [this]() {
        Network *net = m_implData->handle.net;

        if (!net->isServer_Connected()) {
            LOG(WARNING) << "Not connected to server";
            return;
        }

        net->send_buff[m_sensorIndex].set_func_name("GetInterrupts");
        net->send_buff[m_sensorIndex].set_expect_reply(true);

        if (net->SendCommand() != 0) {
            LOG(WARNING) << "Send Command Failed";
            return;
        }

        if (net->recv_server_data() != 0) {
            LOG(WARNING) << "Receive Data Failed";
            return;
        }

        if (net->recv_buff[m_sensorIndex].server_status() !=
            payload::ServerStatus::REQUEST_ACCEPTED) {
            LOG(WARNING) << "API execution on Target Failed";
            return;
        }

        for (int32_t i = 0;
             i < net->recv_buff[m_sensorIndex].int32_payload_size(); ++i) {
            for (auto m_interruptCallback : m_interruptCallbackMap) {
                m_interruptCallback.second(
                    (aditof::Adsd3500Status)net->recv_buff[m_sensorIndex]
                        .int32_payload(i));
            }
        }
    };

    int m_sensorCounter = 0;
    m_sensorIndex = m_sensorCounter;
    m_sensorCounter++;

    Network *net = new Network(m_sensorIndex);
    m_implData->handle.net = net;
    m_implData->handle.net->registerInterruptCallback(m_implData->cb);
    m_implData->ip = ip;
    m_implData->opened = false;
    m_sensorDetails.connectionType = aditof::ConnectionType::NETWORK;
    m_sensorDetails.id = ip;
    m_sensorName = name;
}

NetworkDepthSensor::~NetworkDepthSensor() {
    std::unique_lock<std::mutex> mutex_lock(m_implData->handle.net_mutex);

    // If channel communication has been opened, let the server know we're hanging up
    if (m_implData->opened) {
        if (!m_implData->handle.net->isServer_Connected()) {
            LOG(WARNING) << "Not connected to server";
        }

        m_implData->handle.net->send_buff[m_sensorIndex].set_func_name(
            "HangUp");
        m_implData->handle.net->send_buff[m_sensorIndex].set_expect_reply(
            false);

        if (m_implData->handle.net->SendCommand() != 0) {
            LOG(WARNING) << "Send Command Failed";
        }

        if (!m_stopServerCheck) {
            m_stopServerCheck = true;
            m_activityCheckThread.join();
        }
    }

    delete m_implData->handle.net;

    for (auto it = m_implData->calibration_cache.begin();
         it != m_implData->calibration_cache.begin(); ++it) {
        delete[] it->second.cache;
        it->second.cache = nullptr;
    }
}

aditof::Status
NetworkDepthSensor::getIniParams(std::map<std::string, float> &params) {
    using namespace aditof;
    Network *net = m_implData->handle.net;
    std::unique_lock<std::mutex> mutex_lock(m_implData->handle.net_mutex);

    if (!net->isServer_Connected()) {
        LOG(WARNING) << "Not connected to server";
        return Status::UNREACHABLE;
    }

    net->send_buff[m_sensorIndex].set_func_name("GetIniParam");
    net->send_buff[m_sensorIndex].set_expect_reply(true);

    if (net->SendCommand() != 0) {
        LOG(WARNING) << "Send Command Failed";
        return Status::INVALID_ARGUMENT;
    }

    if (net->recv_server_data() != 0) {
        LOG(WARNING) << "Receive Data Failed";
        return Status::GENERIC_ERROR;
    }

    if (net->recv_buff[m_sensorIndex].server_status() !=
        payload::ServerStatus::REQUEST_ACCEPTED) {
        LOG(WARNING) << "API execution on Target Failed";
        return Status::GENERIC_ERROR;
    }

    Status status = static_cast<Status>(net->recv_buff[m_sensorIndex].status());

    if (status == Status::OK) {
        params["ab_thresh_min"] =
            static_cast<float>(net->recv_buff[m_sensorIndex].float_payload(0));
        params["ab_sum_thresh"] =
            static_cast<float>(net->recv_buff[m_sensorIndex].float_payload(1));
        params["conf_thresh"] =
            static_cast<float>(net->recv_buff[m_sensorIndex].float_payload(2));
        params["radial_thresh_min"] =
            static_cast<float>(net->recv_buff[m_sensorIndex].float_payload(3));
        params["radial_thresh_max"] =
            static_cast<float>(net->recv_buff[m_sensorIndex].float_payload(4));
        params["jblf_apply_flag"] =
            static_cast<float>(net->recv_buff[m_sensorIndex].float_payload(5));
        params["jblf_window_size"] =
            static_cast<float>(net->recv_buff[m_sensorIndex].float_payload(6));
        params["jblf_gaussian_sigma"] =
            static_cast<float>(net->recv_buff[m_sensorIndex].float_payload(7));
        params["jblf_exponential_term"] =
            static_cast<float>(net->recv_buff[m_sensorIndex].float_payload(8));
        params["jblf_max_edge"] =
            static_cast<float>(net->recv_buff[m_sensorIndex].float_payload(9));
        params["jblf_ab_threshold"] =
            static_cast<float>(net->recv_buff[m_sensorIndex].float_payload(10));
        params["headerSize"] =
            static_cast<float>(net->recv_buff[m_sensorIndex].float_payload(11));
    }

    return status;
}

aditof::Status
NetworkDepthSensor::setIniParams(const std::map<std::string, float> &params) {
    using namespace aditof;
    Network *net = m_implData->handle.net;
    std::unique_lock<std::mutex> mutex_lock(m_implData->handle.net_mutex);

    if (!net->isServer_Connected()) {
        LOG(WARNING) << "Not connected to server";
        return Status::UNREACHABLE;
    }

    net->send_buff[m_sensorIndex].set_func_name("SetIniParam");
    net->send_buff[m_sensorIndex].set_expect_reply(true);
    net->send_buff[m_sensorIndex].add_func_float_param(
        params.at("ab_thresh_min"));
    net->send_buff[m_sensorIndex].add_func_float_param(
        params.at("ab_sum_thresh"));
    net->send_buff[m_sensorIndex].add_func_float_param(
        params.at("conf_thresh"));
    net->send_buff[m_sensorIndex].add_func_float_param(
        params.at("radial_thresh_min"));
    net->send_buff[m_sensorIndex].add_func_float_param(
        params.at("radial_thresh_max"));
    net->send_buff[m_sensorIndex].add_func_float_param(
        params.at("jblf_apply_flag"));
    net->send_buff[m_sensorIndex].add_func_float_param(
        params.at("jblf_window_size"));
    net->send_buff[m_sensorIndex].add_func_float_param(
        params.at("jblf_gaussian_sigma"));
    net->send_buff[m_sensorIndex].add_func_float_param(
        params.at("jblf_exponential_term"));
    net->send_buff[m_sensorIndex].add_func_float_param(
        params.at("jblf_max_edge"));
    net->send_buff[m_sensorIndex].add_func_float_param(
        params.at("jblf_ab_threshold"));

    if (net->SendCommand() != 0) {
        LOG(WARNING) << "Send Command Failed";
        return Status::INVALID_ARGUMENT;
    }

    if (net->recv_server_data() != 0) {
        LOG(WARNING) << "Receive Data Failed";
        return Status::GENERIC_ERROR;
    }

    if (net->recv_buff[m_sensorIndex].server_status() !=
        payload::ServerStatus::REQUEST_ACCEPTED) {
        LOG(WARNING) << "API execution on Target Failed";
        return Status::GENERIC_ERROR;
    }

    Status status = static_cast<Status>(net->recv_buff[m_sensorIndex].status());
    return status;
}

aditof::Status NetworkDepthSensor::open() {
    using namespace aditof;

    Network *net = m_implData->handle.net;
    std::unique_lock<std::mutex> mutex_lock(m_implData->handle.net_mutex);

    if (net->ServerConnect(m_implData->ip) != 0) {
        LOG(WARNING) << "Server Connect Failed";
        return Status::UNREACHABLE;
    }

    if (!net->isServer_Connected()) {
        LOG(WARNING) << "Not connected to server";
        return Status::UNREACHABLE;
    }

    net->send_buff[m_sensorIndex].set_func_name("Open");
    net->send_buff[m_sensorIndex].set_expect_reply(true);

    if (net->SendCommand() != 0) {
        LOG(WARNING) << "Send Command Failed";
        return Status::INVALID_ARGUMENT;
    }

    if (net->recv_server_data() != 0) {
        LOG(WARNING) << "Receive Data Failed";
        return Status::GENERIC_ERROR;
    }

    if (net->recv_buff[m_sensorIndex].server_status() !=
        payload::ServerStatus::REQUEST_ACCEPTED) {
        LOG(WARNING) << "API execution on Target Failed";
        return Status::GENERIC_ERROR;
    }

    Status status = static_cast<Status>(net->recv_buff[m_sensorIndex].status());

    if (status == Status::OK) {
        m_implData->opened = true;

        // Create a new thread that periodically checks for inactivity on client-network then goes back to sleep
        m_activityCheckThread =
            std::thread(&NetworkDepthSensor::checkForServerUpdates, this);
    }

    return status;
}

aditof::Status NetworkDepthSensor::start() {
    using namespace aditof;

    Network *net = m_implData->handle.net;
    std::unique_lock<std::mutex> mutex_lock(m_implData->handle.net_mutex);

    if (!net->isServer_Connected()) {
        LOG(WARNING) << "Not connected to server";
        return Status::UNREACHABLE;
    }

    net->send_buff[m_sensorIndex].set_func_name("Start");
    net->send_buff[m_sensorIndex].set_expect_reply(true);

    if (net->SendCommand() != 0) {
        LOG(WARNING) << "Send Command Failed";
        return Status::INVALID_ARGUMENT;
    }

    if (net->recv_server_data() != 0) {
        LOG(WARNING) << "Receive Data Failed";
        return Status::GENERIC_ERROR;
    }

    if (net->recv_buff[m_sensorIndex].server_status() !=
        payload::ServerStatus::REQUEST_ACCEPTED) {
        LOG(WARNING) << "API execution on Target Failed";
        return Status::GENERIC_ERROR;
    }

    Status status = static_cast<Status>(net->recv_buff[m_sensorIndex].status());

    return status;
}

aditof::Status NetworkDepthSensor::stop() {
    using namespace aditof;

    Network *net = m_implData->handle.net;
    std::unique_lock<std::mutex> mutex_lock(m_implData->handle.net_mutex);

    LOG(INFO) << "Stopping device";

    if (!net->isServer_Connected()) {
        LOG(WARNING) << "Not connected to server";
        return Status::UNREACHABLE;
    }

    net->send_buff[m_sensorIndex].set_func_name("Stop");
    net->send_buff[m_sensorIndex].set_expect_reply(true);

    if (net->SendCommand() != 0) {
        LOG(WARNING) << "Send Command Failed";
        return Status::INVALID_ARGUMENT;
    }

    if (net->recv_server_data() != 0) {
        LOG(WARNING) << "Receive Data Failed";
        return Status::GENERIC_ERROR;
    }

    if (net->recv_buff[m_sensorIndex].server_status() !=
        payload::ServerStatus::REQUEST_ACCEPTED) {
        LOG(WARNING) << "API execution on Target Failed";
        return Status::GENERIC_ERROR;
    }

    Status status = static_cast<Status>(net->recv_buff[m_sensorIndex].status());

    return status;
}

aditof::Status
NetworkDepthSensor::getAvailableFrameTypes(std::vector<std::string> &types) {
    using namespace aditof;

    Network *net = m_implData->handle.net;
    std::unique_lock<std::mutex> mutex_lock(m_implData->handle.net_mutex);

    if (!net->isServer_Connected()) {
        LOG(WARNING) << "Not connected to server";
        return Status::UNREACHABLE;
    }

    net->send_buff[m_sensorIndex].set_func_name("GetAvailableFrameTypes");
    net->send_buff[m_sensorIndex].set_expect_reply(true);

    if (net->SendCommand() != 0) {
        LOG(WARNING) << "Send Command Failed";
        return Status::INVALID_ARGUMENT;
    }

    if (net->recv_server_data() != 0) {
        LOG(WARNING) << "Receive Data Failed";
        return Status::GENERIC_ERROR;
    }

    if (net->recv_buff[m_sensorIndex].server_status() !=
        payload::ServerStatus::REQUEST_ACCEPTED) {
        LOG(WARNING) << "API execution on Target Failed";
        return Status::GENERIC_ERROR;
    }

    // Cleanup array (if required) before filling it with the available types
    if (types.size() != 0) {
        types.clear();
    }

    for (int i = 0; i < net->recv_buff[m_sensorIndex].strings_payload_size();
         i++) {
        types.emplace_back(net->recv_buff[m_sensorIndex].strings_payload(i));
    }

    Status status = static_cast<Status>(net->recv_buff[m_sensorIndex].status());

    return status;
}

aditof::Status
NetworkDepthSensor::getFrameTypeDetails(const std::string &frameName,
                                        aditof::DepthSensorFrameType &details) {
    using namespace aditof;
    Network *net = m_implData->handle.net;
    std::unique_lock<std::mutex> mutex_lock(m_implData->handle.net_mutex);

    if (!net->isServer_Connected()) {
        LOG(WARNING) << "Not connected to server";
        return Status::UNREACHABLE;
    }

    net->send_buff[m_sensorIndex].set_func_name("GetFrameTypeDetails");
    net->send_buff[m_sensorIndex].add_func_strings_param(frameName);
    net->send_buff[m_sensorIndex].set_expect_reply(true);

    if (net->SendCommand() != 0) {
        LOG(WARNING) << "Send Command Failed";
        return Status::INVALID_ARGUMENT;
    }

    if (net->recv_server_data() != 0) {
        LOG(WARNING) << "Receive Data Failed";
        return Status::GENERIC_ERROR;
    }

    if (net->recv_buff[m_sensorIndex].server_status() !=
        payload::ServerStatus::REQUEST_ACCEPTED) {
        LOG(WARNING) << "API execution on Target Failed";
        return Status::GENERIC_ERROR;
    }
    details.mode = frameName;
    details.frameContent =
        net->recv_buff[m_sensorIndex].depthSensorFrameType().frameContent;
    details.modeNumber =
        net->recv_buff[m_sensorIndex].depthSensorFrameType().modeNumber();
    details.pixelFormatIndex =
        net->recv_buff[m_sensorIndex].depthSensorFrameType().pixelFormatIndex();
    details.frameWidthInBytes = net->recv_buff[m_sensorIndex]
                                    .depthSensorFrameType()
                                    .frameWidthInBytes();
    details.frameHeightInBytes = net->recv_buff[m_sensorIndex]
                                     .depthSensorFrameType()
                                     .frameHeightInBytes();
    details.baseResolutionWidth = net->recv_buff[m_sensorIndex]
                                      .depthSensorFrameType()
                                      .baseResolutionWidth();
    details.baseResolutionHeight = net->recv_buff[m_sensorIndex]
                                       .depthSensorFrameType()
                                       .baseResolutionHeight();
    details.metadataSize =
        net->recv_buff[m_sensorIndex].depthSensorFrameType().metadataSize();
    Status status = static_cast<Status>(net->recv_buff[m_sensorIndex].status());
    return status;
}

aditof::Status
NetworkDepthSensor::setFrameType(const aditof::DepthSensorFrameType &type) {
    using namespace aditof;

    Network *net = m_implData->handle.net;
    std::unique_lock<std::mutex> mutex_lock(m_implData->handle.net_mutex);

    if (!net->isServer_Connected()) {
        LOG(WARNING) << "Not connected to server";
        return Status::UNREACHABLE;
    }

    net->send_buff[m_sensorIndex].set_func_name("SetFrameType");
    auto protoFrameContent =
        net->send_buff[m_sensorIndex].add_depthsensorframecontent();
    protoFrameContent->set_type(type.type);
    protoFrameContent->set_width(type.width);
    protoFrameContent->set_height(type.height);

    net->send_buff[m_sensorIndex].set_expect_reply(true);

    if (net->SendCommand() != 0) {
        LOG(WARNING) << "Send Command Failed";
        return Status::INVALID_ARGUMENT;
    }

    if (net->recv_server_data() != 0) {
        LOG(WARNING) << "Receive Data Failed";
        return Status::GENERIC_ERROR;
    }

    if (net->recv_buff[m_sensorIndex].server_status() !=
        payload::ServerStatus::REQUEST_ACCEPTED) {
        LOG(WARNING) << "API execution on Target Failed";
        return Status::GENERIC_ERROR;
    }

    Status status = static_cast<Status>(net->recv_buff[m_sensorIndex].status());

    if (status == Status::OK) {
        m_implData->frameTypeCache = type;
    }

    return status;
}

aditof::Status NetworkDepthSensor::getFrame(uint16_t *buffer) {
    using namespace aditof;

    Network *net = m_implData->handle.net;
    std::unique_lock<std::mutex> mutex_lock(m_implData->handle.net_mutex);

    if (!net->isServer_Connected()) {
        LOG(WARNING) << "Not connected to server";
        return Status::UNREACHABLE;
    }

    net->send_buff[m_sensorIndex].set_func_name("GetFrame");
    net->send_buff[m_sensorIndex].set_expect_reply(true);

    if (net->SendCommand(static_cast<void *>(buffer)) != 0) {
        LOG(WARNING) << "Send Command Failed";
        return Status::INVALID_ARGUMENT;
    }

    if (net->recv_server_data() != 0) {
        LOG(WARNING) << "Receive Data Failed";
        return Status::GENERIC_ERROR;
    }

    if (net->recv_buff[m_sensorIndex].server_status() !=
        payload::ServerStatus::REQUEST_ACCEPTED) {
        LOG(WARNING) << "API execution on Target Failed";
        return Status::GENERIC_ERROR;
    }

    Status status = static_cast<Status>(net->recv_buff[m_sensorIndex].status());
    if (status != Status::OK) {
        LOG(WARNING) << "getFrame() failed on target";
        return status;
    }

    return status;
}

aditof::Status NetworkDepthSensor::getAvailableControls(
    std::vector<std::string> &controls) const {
    using namespace aditof;

    Network *net = m_implData->handle.net;
    std::unique_lock<std::mutex> mutex_lock(m_implData->handle.net_mutex);

    if (!net->isServer_Connected()) {
        LOG(WARNING) << "Not connected to server";
        return Status::UNREACHABLE;
    }

    net->send_buff[m_sensorIndex].set_func_name("GetAvailableControls");
    net->send_buff[m_sensorIndex].set_expect_reply(true);

    if (net->SendCommand() != 0) {
        LOG(WARNING) << "Send Command Failed";
        return Status::INVALID_ARGUMENT;
    }

    if (net->recv_server_data() != 0) {
        LOG(WARNING) << "Receive Data Failed";
        return Status::GENERIC_ERROR;
    }

    if (net->recv_buff[m_sensorIndex].server_status() !=
        payload::ServerStatus::REQUEST_ACCEPTED) {
        LOG(WARNING) << "API execution on Target Failed";
        return Status::GENERIC_ERROR;
    }

    Status status = static_cast<Status>(net->recv_buff[m_sensorIndex].status());

    if (status == Status::OK) {
        controls.clear();

        for (int i = 0;
             i < net->recv_buff[m_sensorIndex].strings_payload_size(); i++) {
            std::string controlName =
                net->recv_buff[m_sensorIndex].strings_payload(i);
            controls.push_back(controlName);
        }
    }

    return status;
}

aditof::Status NetworkDepthSensor::setControl(const std::string &control,
                                              const std::string &value) {
    using namespace aditof;

    Network *net = m_implData->handle.net;
    std::unique_lock<std::mutex> mutex_lock(m_implData->handle.net_mutex);

    if (!net->isServer_Connected()) {
        LOG(WARNING) << "Not connected to server";
        return Status::UNREACHABLE;
    }

    net->send_buff[m_sensorIndex].set_func_name("SetControl");
    net->send_buff[m_sensorIndex].add_func_strings_param(control);
    net->send_buff[m_sensorIndex].add_func_strings_param(value);
    net->send_buff[m_sensorIndex].set_expect_reply(true);

    if (net->SendCommand() != 0) {
        LOG(WARNING) << "Send Command Failed";
        return Status::INVALID_ARGUMENT;
    }

    if (net->recv_server_data() != 0) {
        LOG(WARNING) << "Receive Data Failed";
        return Status::GENERIC_ERROR;
    }

    if (net->recv_buff[m_sensorIndex].server_status() !=
        payload::ServerStatus::REQUEST_ACCEPTED) {
        LOG(WARNING) << "API execution on Target Failed";
        return Status::GENERIC_ERROR;
    }

    Status status = static_cast<Status>(net->recv_buff[m_sensorIndex].status());

    return status;
}

aditof::Status NetworkDepthSensor::getControl(const std::string &control,
                                              std::string &value) const {
    using namespace aditof;

    Network *net = m_implData->handle.net;
    std::unique_lock<std::mutex> mutex_lock(m_implData->handle.net_mutex);

    if (!net->isServer_Connected()) {
        LOG(WARNING) << "Not connected to server";
        return Status::UNREACHABLE;
    }

    net->send_buff[m_sensorIndex].set_func_name("GetControl");
    net->send_buff[m_sensorIndex].add_func_strings_param(control);
    net->send_buff[m_sensorIndex].set_expect_reply(true);

    if (net->SendCommand() != 0) {
        LOG(WARNING) << "Send Command Failed";
        return Status::INVALID_ARGUMENT;
    }

    if (net->recv_server_data() != 0) {
        LOG(WARNING) << "Receive Data Failed";
        return Status::GENERIC_ERROR;
    }

    if (net->recv_buff[m_sensorIndex].server_status() !=
        payload::ServerStatus::REQUEST_ACCEPTED) {
        LOG(WARNING) << "API execution on Target Failed";
        return Status::GENERIC_ERROR;
    }

    Status status = static_cast<Status>(net->recv_buff[m_sensorIndex].status());

    if (status == Status::OK) {
        value = net->recv_buff[m_sensorIndex].strings_payload(0);
    }

    return status;
}

aditof::Status
NetworkDepthSensor::getDetails(aditof::SensorDetails &details) const {
    details = m_sensorDetails;
    return aditof::Status::OK;
}

aditof::Status NetworkDepthSensor::getHandle(void **handle) {
    if (m_implData->opened) {
        *handle = &m_implData->handle;
        return aditof::Status::OK;
    } else {
        *handle = nullptr;
        LOG(ERROR) << "Won't return the handle. Device hasn't been opened yet.";
        return aditof::Status::UNAVAILABLE;
    }
    return aditof::Status::OK;
}

aditof::Status NetworkDepthSensor::getName(std::string &name) const {
    name = m_sensorName;
    return aditof::Status::OK;
}

aditof::Status
NetworkDepthSensor::setHostConnectionType(std::string &connectionType) {
    LOG(INFO) << "Function used only on target!";
    return aditof::Status::OK;
}

aditof::Status NetworkDepthSensor::adsd3500_read_cmd(uint16_t cmd,
                                                     uint16_t *data,
                                                     unsigned int usDelay) {
    using namespace aditof;

    Network *net = m_implData->handle.net;
    std::unique_lock<std::mutex> mutex_lock(m_implData->handle.net_mutex);

    if (!net->isServer_Connected()) {
        LOG(WARNING) << "Not connected to server";
        return Status::UNREACHABLE;
    }

    net->send_buff[m_sensorIndex].set_func_name("Adsd3500ReadCmd");
    net->send_buff[m_sensorIndex].add_func_int32_param(
        static_cast<::google::int32>(cmd));
    net->send_buff[m_sensorIndex].add_func_int32_param(
        static_cast<::google::int32>(usDelay));
    net->send_buff[m_sensorIndex].set_expect_reply(true);

    if (net->SendCommand() != 0) {
        LOG(WARNING) << "Send Command Failed";
        return Status::INVALID_ARGUMENT;
    }

    if (net->recv_server_data() != 0) {
        LOG(WARNING) << "Receive Data Failed";
        return Status::GENERIC_ERROR;
    }

    if (net->recv_buff[m_sensorIndex].server_status() !=
        payload::ServerStatus::REQUEST_ACCEPTED) {
        LOG(WARNING) << "API execution on Target Failed";
        return Status::GENERIC_ERROR;
    }

    Status status = static_cast<Status>(net->recv_buff[m_sensorIndex].status());

    if (status == Status::OK) {
        *data = static_cast<uint16_t>(
            net->recv_buff[m_sensorIndex].int32_payload(0));
    }

    return status;
}

aditof::Status NetworkDepthSensor::adsd3500_write_cmd(uint16_t cmd,
                                                      uint16_t data) {
    using namespace aditof;

    Network *net = m_implData->handle.net;
    std::unique_lock<std::mutex> mutex_lock(m_implData->handle.net_mutex);

    if (!net->isServer_Connected()) {
        LOG(WARNING) << "Not connected to server";
        return Status::UNREACHABLE;
    }

    net->send_buff[m_sensorIndex].set_func_name("Adsd3500WriteCmd");
    net->send_buff[m_sensorIndex].add_func_int32_param(
        static_cast<::google::int32>(cmd));
    net->send_buff[m_sensorIndex].add_func_int32_param(
        static_cast<::google::int32>(data));
    net->send_buff[m_sensorIndex].set_expect_reply(true);

    if (net->SendCommand() != 0) {
        LOG(WARNING) << "Send Command Failed";
        return Status::INVALID_ARGUMENT;
    }

    if (net->recv_server_data() != 0) {
        LOG(WARNING) << "Receive Data Failed";
        return Status::GENERIC_ERROR;
    }

    if (net->recv_buff[m_sensorIndex].server_status() !=
        payload::ServerStatus::REQUEST_ACCEPTED) {
        LOG(WARNING) << "API execution on Target Failed";
        return Status::GENERIC_ERROR;
    }

    Status status = static_cast<Status>(net->recv_buff[m_sensorIndex].status());

    return status;
}

aditof::Status NetworkDepthSensor::adsd3500_read_payload_cmd(
    uint32_t cmd, uint8_t *readback_data, uint16_t payload_len) {
    using namespace aditof;

    Network *net = m_implData->handle.net;
    std::unique_lock<std::mutex> mutex_lock(m_implData->handle.net_mutex);

    if (!net->isServer_Connected()) {
        LOG(WARNING) << "Not connected to server";
        return Status::UNREACHABLE;
    }

    net->send_buff[m_sensorIndex].set_func_name("Adsd3500ReadPayloadCmd");
    net->send_buff[m_sensorIndex].add_func_int32_param(
        static_cast<::google::int32>(cmd));
    net->send_buff[m_sensorIndex].add_func_int32_param(
        static_cast<::google::int32>(payload_len));
    net->send_buff[m_sensorIndex].add_func_bytes_param(readback_data,
                                                       4 * sizeof(uint8_t));
    net->send_buff[m_sensorIndex].set_expect_reply(true);

    if (net->SendCommand() != 0) {
        LOG(WARNING) << "Send Command Failed";
        return Status::INVALID_ARGUMENT;
    }

    if (net->recv_server_data() != 0) {
        LOG(WARNING) << "Receive Data Failed";
        return Status::GENERIC_ERROR;
    }

    if (net->recv_buff[m_sensorIndex].server_status() !=
        payload::ServerStatus::REQUEST_ACCEPTED) {
        LOG(WARNING) << "API execution on Target Failed";
        return Status::GENERIC_ERROR;
    }

    Status status = static_cast<Status>(net->recv_buff[m_sensorIndex].status());

    if (status == Status::OK) {
        memcpy(readback_data,
               net->recv_buff[m_sensorIndex].bytes_payload(0).c_str(),
               net->recv_buff[m_sensorIndex].bytes_payload(0).length());
    }

    return status;
}

aditof::Status NetworkDepthSensor::adsd3500_read_payload(uint8_t *payload,
                                                         uint16_t payload_len) {
    using namespace aditof;

    Network *net = m_implData->handle.net;
    std::unique_lock<std::mutex> mutex_lock(m_implData->handle.net_mutex);

    if (!net->isServer_Connected()) {
        LOG(WARNING) << "Not connected to server";
        return Status::UNREACHABLE;
    }

    net->send_buff[m_sensorIndex].set_func_name("Adsd3500ReadPayload");
    net->send_buff[m_sensorIndex].add_func_int32_param(
        static_cast<::google::int32>(payload_len));
    net->send_buff[m_sensorIndex].set_expect_reply(true);

    if (net->SendCommand() != 0) {
        LOG(WARNING) << "Send Command Failed";
        return Status::INVALID_ARGUMENT;
    }

    if (net->recv_server_data() != 0) {
        LOG(WARNING) << "Receive Data Failed";
        return Status::GENERIC_ERROR;
    }

    if (net->recv_buff[m_sensorIndex].server_status() !=
        payload::ServerStatus::REQUEST_ACCEPTED) {
        LOG(WARNING) << "API execution on Target Failed";
        return Status::GENERIC_ERROR;
    }

    Status status = static_cast<Status>(net->recv_buff[m_sensorIndex].status());

    if (status == Status::OK) {
        memcpy(payload, net->recv_buff[m_sensorIndex].bytes_payload(0).c_str(),
               net->recv_buff[m_sensorIndex].bytes_payload(0).length());
    }

    return status;
}

aditof::Status
NetworkDepthSensor::adsd3500_write_payload_cmd(uint32_t cmd, uint8_t *payload,
                                               uint16_t payload_len) {
    using namespace aditof;

    Network *net = m_implData->handle.net;
    std::unique_lock<std::mutex> mutex_lock(m_implData->handle.net_mutex);

    if (!net->isServer_Connected()) {
        LOG(WARNING) << "Not connected to server";
        return Status::UNREACHABLE;
    }

    net->send_buff[m_sensorIndex].set_func_name("Adsd3500WritePayloadCmd");
    net->send_buff[m_sensorIndex].add_func_int32_param(
        static_cast<::google::int32>(cmd));
    net->send_buff[m_sensorIndex].add_func_int32_param(
        static_cast<::google::int32>(payload_len));
    net->send_buff[m_sensorIndex].add_func_bytes_param(payload, payload_len);
    net->send_buff[m_sensorIndex].set_expect_reply(true);

    if (net->SendCommand() != 0) {
        LOG(WARNING) << "Send Command Failed";
        return Status::INVALID_ARGUMENT;
    }

    if (net->recv_server_data() != 0) {
        LOG(WARNING) << "Receive Data Failed";
        return Status::GENERIC_ERROR;
    }

    if (net->recv_buff[m_sensorIndex].server_status() !=
        payload::ServerStatus::REQUEST_ACCEPTED) {
        LOG(WARNING) << "API execution on Target Failed";
        return Status::GENERIC_ERROR;
    }

    Status status = static_cast<Status>(net->recv_buff[m_sensorIndex].status());

    return status;
}

aditof::Status
NetworkDepthSensor::adsd3500_write_payload(uint8_t *payload,
                                           uint16_t payload_len) {
    using namespace aditof;

    Network *net = m_implData->handle.net;
    std::unique_lock<std::mutex> mutex_lock(m_implData->handle.net_mutex);

    if (!net->isServer_Connected()) {
        LOG(WARNING) << "Not connected to server";
        return Status::UNREACHABLE;
    }

    net->send_buff[m_sensorIndex].set_func_name("Adsd3500WritePayload");
    net->send_buff[m_sensorIndex].add_func_int32_param(
        static_cast<::google::int32>(payload_len));
    net->send_buff[m_sensorIndex].add_func_bytes_param(payload, payload_len);
    net->send_buff[m_sensorIndex].set_expect_reply(true);

    if (net->SendCommand() != 0) {
        LOG(WARNING) << "Send Command Failed";
        return Status::INVALID_ARGUMENT;
    }

    if (net->recv_server_data() != 0) {
        LOG(WARNING) << "Receive Data Failed";
        return Status::GENERIC_ERROR;
    }

    if (net->recv_buff[m_sensorIndex].server_status() !=
        payload::ServerStatus::REQUEST_ACCEPTED) {
        LOG(WARNING) << "API execution on Target Failed";
        return Status::GENERIC_ERROR;
    }

    Status status = static_cast<Status>(net->recv_buff[m_sensorIndex].status());

    return status;
}

aditof::Status NetworkDepthSensor::adsd3500_reset() {
    using namespace aditof;

    Network *net = m_implData->handle.net;
    std::unique_lock<std::mutex> mutex_lock(m_implData->handle.net_mutex);

    if (!net->isServer_Connected()) {
        LOG(WARNING) << "Not connected to server";
        return Status::UNREACHABLE;
    }

    net->send_buff[m_sensorIndex].set_func_name("Adsd3500Reset");
    net->send_buff[m_sensorIndex].set_expect_reply(true);

    if (net->SendCommand() != 0) {
        LOG(WARNING) << "Send Command Failed";
        return Status::INVALID_ARGUMENT;
    }

    if (net->recv_server_data() != 0) {
        LOG(WARNING) << "Receive Data Failed";
        return Status::GENERIC_ERROR;
    }

    if (net->recv_buff[m_sensorIndex].server_status() !=
        payload::ServerStatus::REQUEST_ACCEPTED) {
        LOG(WARNING) << "API execution on Target Failed";
        return Status::GENERIC_ERROR;
    }

    Status status = static_cast<Status>(net->recv_buff[m_sensorIndex].status());

    return status;
}

aditof::Status NetworkDepthSensor::adsd3500_register_interrupt_callback(
    aditof::SensorInterruptCallback &cb) {
    m_interruptCallbackMap.insert({&cb, cb});
    return aditof::Status::OK;
}

aditof::Status NetworkDepthSensor::adsd3500_unregister_interrupt_callback(
    aditof::SensorInterruptCallback &cb) {

    m_interruptCallbackMap.erase(&cb);

    return aditof::Status::OK;
}

aditof::Status NetworkDepthSensor::adsd3500_get_status(int &chipStatus,
                                                       int &imagerStatus) {
    using namespace aditof;

    Network *net = m_implData->handle.net;
    std::unique_lock<std::mutex> mutex_lock(m_implData->handle.net_mutex);

    if (!net->isServer_Connected()) {
        LOG(WARNING) << "Not connected to server";
        return Status::UNREACHABLE;
    }

    net->send_buff[m_sensorIndex].set_func_name("Adsd3500GetStatus");
    net->send_buff[m_sensorIndex].set_expect_reply(true);

    if (net->SendCommand() != 0) {
        LOG(WARNING) << "Send Command Failed";
        return Status::INVALID_ARGUMENT;
    }

    if (net->recv_server_data() != 0) {
        LOG(WARNING) << "Receive Data Failed";
        return Status::GENERIC_ERROR;
    }

    if (net->recv_buff[m_sensorIndex].server_status() !=
        payload::ServerStatus::REQUEST_ACCEPTED) {
        LOG(WARNING) << "API execution on Target Failed";
        return Status::GENERIC_ERROR;
    }

    chipStatus = net->recv_buff[m_sensorIndex].int32_payload(0);
    imagerStatus = net->recv_buff[m_sensorIndex].int32_payload(1);

    Status status = static_cast<Status>(net->recv_buff[m_sensorIndex].status());

    return status;
}

aditof::Status NetworkDepthSensor::initTargetDepthCompute(
    uint8_t *iniFile, uint16_t iniFileLength, uint8_t *calData,
    uint16_t calDataLength) {
    using namespace aditof;

    Network *net = m_implData->handle.net;
    std::unique_lock<std::mutex> mutex_lock(m_implData->handle.net_mutex);

    if (!net->isServer_Connected()) {
        LOG(WARNING) << "Not connected to server";
        return Status::UNREACHABLE;
    }

    net->send_buff[m_sensorIndex].set_func_name("InitTargetDepthCompute");
    net->send_buff[m_sensorIndex].add_func_int32_param(
        static_cast<::google::int32>(iniFileLength));
    net->send_buff[m_sensorIndex].add_func_int32_param(
        static_cast<::google::int32>(calDataLength));
    net->send_buff[m_sensorIndex].add_func_bytes_param(iniFile, iniFileLength);
    net->send_buff[m_sensorIndex].add_func_bytes_param(calData, calDataLength);
    net->send_buff[m_sensorIndex].set_expect_reply(true);

    if (net->SendCommand() != 0) {
        LOG(WARNING) << "Send Command Failed";
        return Status::INVALID_ARGUMENT;
    }

    if (net->recv_server_data() != 0) {
        LOG(WARNING) << "Receive Data Failed";
        return Status::GENERIC_ERROR;
    }

    if (net->recv_buff[m_sensorIndex].server_status() !=
        payload::ServerStatus::REQUEST_ACCEPTED) {
        LOG(WARNING) << "API execution on Target Failed";
        return Status::GENERIC_ERROR;
    }

    Status status = static_cast<Status>(net->recv_buff[m_sensorIndex].status());

    return status;
}

void NetworkDepthSensor::checkForServerUpdates() {
    using namespace std::chrono;

    while (!m_stopServerCheck) {
        // get latest timestamp from Network object
        steady_clock::time_point latestActivityTimestamp =
            m_implData->handle.net->getLatestActivityTimestamp();

        // get current timestamp
        steady_clock::time_point now = steady_clock::now();

        // decide if it is required to check for server updates
        duration<double> inactivityDuration =
            duration_cast<duration<double>>(now - latestActivityTimestamp);
        if (inactivityDuration.count() > 1.0) {
            // check server for interrupts
            std::unique_lock<std::mutex> mutex_lock(
                m_implData->handle.net_mutex);
            if (m_implData->handle.net->isServer_Connected()) {
                m_implData->cb();
            } else {
                m_stopServerCheck = true;
                break;
            }
        }
        // go back to sleep
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }
}

aditof::Status
NetworkDepthSensor::setSensorConfiguration(const std::string &sensorConf) {
    using namespace aditof;

    Network *net = m_implData->handle.net;
    std::unique_lock<std::mutex> mutex_lock(m_implData->handle.net_mutex);

    if (!net->isServer_Connected()) {
        LOG(WARNING) << "Not connected to server";
        return Status::UNREACHABLE;
    }

    net->send_buff[m_sensorIndex].set_func_name("SetSensorConfiguration");
    net->send_buff[m_sensorIndex].add_func_strings_param(sensorConf);
    net->send_buff[m_sensorIndex].set_expect_reply(true);

    if (net->SendCommand() != 0) {
        LOG(WARNING) << "Send Command Failed";
        return Status::INVALID_ARGUMENT;
    }

    if (net->recv_server_data() != 0) {
        LOG(WARNING) << "Receive Data Failed";
        return Status::GENERIC_ERROR;
    }

    if (net->recv_buff[m_sensorIndex].server_status() !=
        payload::ServerStatus::REQUEST_ACCEPTED) {
        LOG(WARNING) << "API execution on Target Failed";
        return Status::GENERIC_ERROR;
    }

    Status status = static_cast<Status>(net->recv_buff[m_sensorIndex].status());

    return status;
}
