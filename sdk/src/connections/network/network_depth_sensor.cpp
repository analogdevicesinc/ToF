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
#include "device_utils.h"

#include <glog/logging.h>
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
};

NetworkDepthSensor::NetworkDepthSensor(const std::string &name, const std::string &ip)
    : m_implData(new NetworkDepthSensor::ImplData) {

    static int m_sensorCounter = 0;
    m_sensorIndex = m_sensorCounter;
    m_sensorCounter++;

    Network *net = new Network(m_sensorIndex);
    m_implData->handle.net = net;
    m_implData->ip = ip;
    m_implData->opened = false;
    m_sensorDetails.connectionType = aditof::ConnectionType::NETWORK;
    m_sensorName = name;
}

NetworkDepthSensor::~NetworkDepthSensor() {
    std::unique_lock<std::mutex> mutex_lock(m_implData->handle.net_mutex);

    // If channel communication has been opened, let the server know we're hanging up
    if (m_implData->opened) {
        if (!m_implData->handle.net->isServer_Connected()) {
            LOG(WARNING) << "Not connected to server";
        }

        m_implData->handle.net->send_buff[m_sensorIndex].set_func_name("HangUp");
        m_implData->handle.net->send_buff[m_sensorIndex].set_expect_reply(false);

        if (m_implData->handle.net->SendCommand() != 0) {
            LOG(WARNING) << "Send Command Failed";
        }
    }

    delete m_implData->handle.net;

    for (auto it = m_implData->calibration_cache.begin();
         it != m_implData->calibration_cache.begin(); ++it) {
        delete[] it->second.cache;
        it->second.cache = nullptr;
    }
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

aditof::Status NetworkDepthSensor::getAvailableFrameTypes(
    std::vector<aditof::DepthSensorFrameType> &types) {
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

    for (int i = 0; i < net->recv_buff[m_sensorIndex].available_frame_types_size(); i++) {
        payload::DepthSensorFrameType protoFrameType =
            net->recv_buff[m_sensorIndex].available_frame_types(i);
        aditof::DepthSensorFrameType aditofFrameType;

        aditofFrameType.type = protoFrameType.type();
        for (int j = 0; j < protoFrameType.depthsensorframecontent_size();
             ++j) {
            payload::DepthSensorFrameContent protoFrameContent =
                protoFrameType.depthsensorframecontent(j);
            aditof::DepthSensorFrameContent aditofFrameContent;

            aditofFrameContent.type = protoFrameContent.type();
            aditofFrameContent.width = protoFrameContent.width();
            aditofFrameContent.height = protoFrameContent.height();
            aditofFrameType.content.emplace_back(aditofFrameContent);
        }
        aditofFrameType.width = protoFrameType.width();
        aditofFrameType.height = protoFrameType.height();

        types.push_back(aditofFrameType);
    }

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
    net->send_buff[m_sensorIndex].mutable_frame_type()->set_width(type.width);
    net->send_buff[m_sensorIndex].mutable_frame_type()->set_height(type.height);
    net->send_buff[m_sensorIndex].mutable_frame_type()->set_type(type.type);
    for (const auto &content : type.content) {
        auto protoFameContent =
            net->send_buff[m_sensorIndex].mutable_frame_type()->add_depthsensorframecontent();

        protoFameContent->set_type(content.type);
        protoFameContent->set_width(content.width);
        protoFameContent->set_height(content.height);
    }
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

aditof::Status NetworkDepthSensor::program(const uint8_t *firmware,
                                           size_t size) {
    using namespace aditof;

    Network *net = m_implData->handle.net;
    std::unique_lock<std::mutex> mutex_lock(m_implData->handle.net_mutex);

    if (!net->isServer_Connected()) {
        LOG(WARNING) << "Not connected to server";
        return Status::UNREACHABLE;
    }

    net->send_buff[m_sensorIndex].set_func_name("Program");
    net->send_buff[m_sensorIndex].add_func_int32_param(static_cast<::google::int32>(size));
    net->send_buff[m_sensorIndex].add_func_bytes_param(firmware, size);
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
    if (status != Status::OK) {
        LOG(WARNING) << "getFrame() failed on target";
        return status;
    }
    
//when using ITOF camera the data is already deinterleaved, so a simple copy is enough
    memcpy(buffer, net->recv_buff[m_sensorIndex].bytes_payload(0).c_str(), net->recv_buff[m_sensorIndex].bytes_payload(0).length());

    return status;
}

aditof::Status NetworkDepthSensor::readRegisters(const uint16_t *address,
                                                    uint16_t *data,
                                                    size_t length, bool burst /*=true*/) {
    using namespace aditof;

    Network *net = m_implData->handle.net;
    std::unique_lock<std::mutex> mutex_lock(m_implData->handle.net_mutex);

    if (!net->isServer_Connected()) {
        LOG(WARNING) << "Not connected to server";
        return Status::UNREACHABLE;
    }

    net->send_buff[m_sensorIndex].set_func_name("ReadRegisters");
    net->send_buff[m_sensorIndex].add_func_int32_param(static_cast<::google::int32>(length));
    net->send_buff[m_sensorIndex].add_func_int32_param(static_cast<::google::int32>(burst));
    net->send_buff[m_sensorIndex].add_func_bytes_param(address, burst ? 2 : length * sizeof(uint16_t));
    net->send_buff[m_sensorIndex].add_func_bytes_param(data, length * sizeof(uint16_t));
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
        memcpy(data, net->recv_buff[m_sensorIndex].bytes_payload(0).c_str(),
               net->recv_buff[m_sensorIndex].bytes_payload(0).length());
    }

    return status;
}

aditof::Status NetworkDepthSensor::writeRegisters(const uint16_t *address,
                                                     const uint16_t *data,
                                                     size_t length, bool burst /*=true*/) {
    using namespace aditof;

    Network *net = m_implData->handle.net;
    std::unique_lock<std::mutex> mutex_lock(m_implData->handle.net_mutex);

    if (!net->isServer_Connected()) {
        LOG(WARNING) << "Not connected to server";
        return Status::UNREACHABLE;
    }

    net->send_buff[m_sensorIndex].set_func_name("WriteRegisters");
    net->send_buff[m_sensorIndex].add_func_int32_param(static_cast<::google::int32>(length));
    net->send_buff[m_sensorIndex].add_func_int32_param(static_cast<::google::int32>(burst));
    net->send_buff[m_sensorIndex].add_func_bytes_param(address, burst ? 2 : length * sizeof(uint16_t));
    net->send_buff[m_sensorIndex].add_func_bytes_param(data, length * sizeof(uint16_t));
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

aditof::Status NetworkDepthSensor::getAvailableControls(std::vector<std::string> &controls) const
{
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

        for (int i = 0; i < net->recv_buff[m_sensorIndex].strings_payload_size(); i++) {
            std::string controlName = net->recv_buff[m_sensorIndex].strings_payload(i);
            controls.push_back(controlName);
        }
    }

    return status;
}

aditof::Status NetworkDepthSensor::setControl(const std::string &control,
                               const std::string &value)
{
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
                               std::string &value) const
{
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

aditof::Status NetworkDepthSensor::adsd3500_read_cmd(uint16_t cmd, uint16_t *data){
    using namespace aditof;

    Network *net = m_implData->handle.net;
    std::unique_lock<std::mutex> mutex_lock(m_implData->handle.net_mutex);

    if (!net->isServer_Connected()) {
        LOG(WARNING) << "Not connected to server";
        return Status::UNREACHABLE;
    }

    net->send_buff[m_sensorIndex].set_func_name("Adsd3500ReadCmd");
    net->send_buff[m_sensorIndex].add_func_int32_param(static_cast<::google::int32>(cmd));
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

    if(status == Status::OK) {
        *data = static_cast<uint16_t>(net->recv_buff[m_sensorIndex].int32_payload(0));
    }

    return status;
}

aditof::Status NetworkDepthSensor::adsd3500_write_cmd(uint16_t cmd, uint16_t data) {
    using namespace aditof;

    Network *net = m_implData->handle.net;
    std::unique_lock<std::mutex> mutex_lock(m_implData->handle.net_mutex);

    if (!net->isServer_Connected()) {
        LOG(WARNING) << "Not connected to server";
        return Status::UNREACHABLE;
    }

    net->send_buff[m_sensorIndex].set_func_name("Adsd3500WriteCmd");
    net->send_buff[m_sensorIndex].add_func_int32_param(static_cast<::google::int32>(cmd));
    net->send_buff[m_sensorIndex].add_func_int32_param(static_cast<::google::int32>(data));
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

aditof::Status NetworkDepthSensor::adsd3500_read_payload_cmd(uint32_t cmd, uint8_t* readback_data, uint16_t payload_len) {
    using namespace aditof;

    Network *net = m_implData->handle.net;
    std::unique_lock<std::mutex> mutex_lock(m_implData->handle.net_mutex);

    if (!net->isServer_Connected()) {
        LOG(WARNING) << "Not connected to server";
        return Status::UNREACHABLE;
    }

    net->send_buff[m_sensorIndex].set_func_name("Adsd3500ReadPayloadCmd");
    net->send_buff[m_sensorIndex].add_func_int32_param(static_cast<::google::int32>(cmd));
    net->send_buff[m_sensorIndex].add_func_int32_param(static_cast<::google::int32>(payload_len));
    net->send_buff[m_sensorIndex].add_func_bytes_param(readback_data, 4 * sizeof (uint8_t));
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

    if(status == Status::OK) {     
        memcpy(readback_data, net->recv_buff[m_sensorIndex].bytes_payload(0).c_str(), net->recv_buff[m_sensorIndex].bytes_payload(0).length());
    }
    
    return status;
}

aditof::Status NetworkDepthSensor::adsd3500_read_payload(uint8_t* payload, uint16_t payload_len) {
    using namespace aditof;

    Network *net = m_implData->handle.net;
    std::unique_lock<std::mutex> mutex_lock(m_implData->handle.net_mutex);

    if (!net->isServer_Connected()) {
        LOG(WARNING) << "Not connected to server";
        return Status::UNREACHABLE;
    }

    net->send_buff[m_sensorIndex].set_func_name("Adsd3500ReadPayload");
    net->send_buff[m_sensorIndex].add_func_int32_param(static_cast<::google::int32>(payload_len));
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
    
    if(status == Status::OK) {     
        memcpy(payload, net->recv_buff[m_sensorIndex].bytes_payload(0).c_str(), net->recv_buff[m_sensorIndex].bytes_payload(0).length());
    }

    return status;
}

aditof::Status NetworkDepthSensor::adsd3500_write_payload_cmd(uint32_t cmd, uint8_t* payload, uint16_t payload_len) {
    using namespace aditof;

    Network *net = m_implData->handle.net;
    std::unique_lock<std::mutex> mutex_lock(m_implData->handle.net_mutex);

    if (!net->isServer_Connected()) {
        LOG(WARNING) << "Not connected to server";
        return Status::UNREACHABLE;
    }

    net->send_buff[m_sensorIndex].set_func_name("Adsd3500WritePayloadCmd");
    net->send_buff[m_sensorIndex].add_func_int32_param(static_cast<::google::int32>(cmd));
    net->send_buff[m_sensorIndex].add_func_int32_param(static_cast<::google::int32>(payload_len));
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

aditof::Status NetworkDepthSensor::adsd3500_write_payload(uint8_t* payload, uint16_t payload_len) {
    using namespace aditof;

    Network *net = m_implData->handle.net;
    std::unique_lock<std::mutex> mutex_lock(m_implData->handle.net_mutex);

    if (!net->isServer_Connected()) {
        LOG(WARNING) << "Not connected to server";
        return Status::UNREACHABLE;
    }

    net->send_buff[m_sensorIndex].set_func_name("Adsd3500WritePayload");
    net->send_buff[m_sensorIndex].add_func_int32_param(static_cast<::google::int32>(payload_len));
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
