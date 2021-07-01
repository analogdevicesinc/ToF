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
#include "usb_buffer.pb.h"
#include "connections/usb/usb_storage.h"
#include "usb_windows_utils.h"
#include "device_utils.h"

#include <chrono>
#include <glog/logging.h>
#include <string>
#include <thread>

using namespace aditof;

struct UsbStorage::ImplData {
    UsbHandle *handle;
    std::string name;
    unsigned char id;
};

UsbStorage::UsbStorage(const std::string &name, unsigned char id)
    : m_implData(new ImplData) {
    m_implData->name = name;
    m_implData->id = id;
}

UsbStorage::~UsbStorage() = default;

Status UsbStorage::open(void *handle) {
    if (!handle) {
        LOG(ERROR) << "Invalid handle";
        return Status::INVALID_ARGUMENT;
    }
    m_implData->handle = reinterpret_cast<UsbHandle *>(handle);

    using namespace aditof;

    Status status = Status::OK;

    // Construct request message
    usb_payload::ClientRequest requestMsg;
    requestMsg.set_func_name(usb_payload::FunctionName::STORAGE_OPEN);

    // Send request
    std::string requestStr;
    requestMsg.SerializeToString(&requestStr);
    status = UsbWindowsUtils::uvcExUnitSendRequest(m_implData->handle->pVideoInputFilter, requestStr);
    if (status != aditof::Status::OK) {
        LOG(ERROR) << "Request open flash storage failed";
        return status;
    }

    // Read UVC gadget response
    std::string responseStr;
    status = UsbWindowsUtils::uvcExUnitGetResponse(m_implData->handle->pVideoInputFilter, responseStr);
    if (status != aditof::Status::OK) {
        LOG(ERROR) << "Failed to get response of the request to open flash memory";
        return status;
    }
    usb_payload::ServerResponse responseMsg;
    bool parsed = responseMsg.ParseFromString(responseStr);
    if (!parsed) {
        LOG(ERROR) << "Failed to deserialize string containing UVC gadget response";
        return aditof::Status::INVALID_ARGUMENT;
    }

    DLOG(INFO) << "Received the following message: " << responseMsg.DebugString();

    if (responseMsg.status() != usb_payload::Status::OK) {
        LOG(ERROR) << "Open flash memory operation failed on UVC gadget";
        return static_cast<aditof::Status>(responseMsg.status());
    }

    return status;
}

Status UsbStorage::read(const uint32_t address, uint8_t *data,
                        const size_t bytesCount) {
    if (!m_implData->handle) {
        LOG(ERROR) << "Cannot read! EEPROM is not opened.";
        return Status::GENERIC_ERROR;
    }
    if (!data) {
        LOG(ERROR) << "Cannot read! data pointer is invaid.";
        return Status::INVALID_ARGUMENT;
    }

        using namespace aditof;

    Status status = Status::OK;

    // TO DO: is it required to call UvcFindNodeAndGetControl here?

    // Construct request message
    usb_payload::ClientRequest requestMsg;
    requestMsg.set_func_name(usb_payload::FunctionName::STORAGE_READ);
    requestMsg.add_func_int32_param(static_cast<::google::int32>(bytesCount));
    requestMsg.add_func_int32_param(static_cast<::google::int32>(address));
    requestMsg.add_func_bytes_param(data, bytesCount);

    // Send request
    std::string requestStr;
    requestMsg.SerializeToString(&requestStr);
    status = UsbWindowsUtils::uvcExUnitSendRequest(m_implData->handle->pVideoInputFilter, requestStr);
    if (status != aditof::Status::OK) {
        LOG(ERROR) << "Request to read memory failed";
        return status;
    }

    // Read UVC gadget response
    std::string responseStr;
    status = UsbWindowsUtils::uvcExUnitGetResponse(m_implData->handle->pVideoInputFilter, responseStr);
    if (status != aditof::Status::OK) {
        LOG(ERROR) << "Failed to get response of the request to read memory";
        return status;
    }
    usb_payload::ServerResponse responseMsg;
    bool parsed = responseMsg.ParseFromString(responseStr);
    if (!parsed) {
        LOG(ERROR) << "Failed to deserialize string containing UVC gadget response";
        return aditof::Status::INVALID_ARGUMENT;
    }

    DLOG(INFO) << "Received the following message: " << responseMsg.DebugString();

    if (responseMsg.status() != usb_payload::Status::OK) {
        LOG(ERROR) << "Read registers operation failed on UVC gadget";
        return static_cast<aditof::Status>(responseMsg.status());
    }
   
    memcpy(data, responseMsg.bytes_payload(0).c_str(), responseMsg.bytes_payload(0).length());

    return Status::OK;
}

Status UsbStorage::write(const uint32_t address, const uint8_t *data,
                         const size_t bytesCount) {
    if (!m_implData->handle) {
        LOG(ERROR) << "Cannot write! EEPROM is not opened.";
        return Status::GENERIC_ERROR;
    }
    if (!data) {
        LOG(ERROR) << "Cannot write! data pointer is invaid.";
        return Status::INVALID_ARGUMENT;
    }

    using namespace aditof;

    Status status = Status::OK;

    // TO DO: is it required to call UvcFindNodeAndGetControl here?

    // Construct request message
    usb_payload::ClientRequest requestMsg;
    requestMsg.set_func_name(usb_payload::FunctionName::STORAGE_WRITE);
    requestMsg.add_func_int32_param(static_cast<::google::int32>(bytesCount));
    requestMsg.add_func_int32_param(static_cast<::google::int32>(address));
    requestMsg.add_func_bytes_param(data, bytesCount);

    // Send request
    std::string requestStr;
    requestMsg.SerializeToString(&requestStr);
    status = UsbWindowsUtils::uvcExUnitSendRequest(m_implData->handle->pVideoInputFilter, requestStr);
    if (status != aditof::Status::OK) {
        LOG(ERROR) << "Request to write registers failed";
        return status;
    }

    // Read UVC gadget response
    std::string responseStr;
    status = UsbWindowsUtils::uvcExUnitGetResponse(m_implData->handle->pVideoInputFilter, responseStr);
    if (status != aditof::Status::OK) {
        LOG(ERROR) << "Failed to get response of the request to write registers";
        return status;
    }
    usb_payload::ServerResponse responseMsg;
    bool parsed = responseMsg.ParseFromString(responseStr);
    if (!parsed) {
        LOG(ERROR) << "Failed to deserialize string containing UVC gadget response";
        return aditof::Status::INVALID_ARGUMENT;
    }

    DLOG(INFO) << "Received the following message: " << responseMsg.DebugString();

    if (responseMsg.status() != usb_payload::Status::OK) {
        LOG(ERROR) << "Read registers operation failed on UVC gadget";
        return static_cast<aditof::Status>(responseMsg.status());
    }

    return Status::OK;
}

Status UsbStorage::getCapacity(size_t &nbBytes) const
{
    if (!m_implData->handle) {
        LOG(ERROR) << "Cannot read! EEPROM is not opened.";
        return Status::GENERIC_ERROR;
    }

    using namespace aditof;

    Status status = Status::OK;

    // TO DO: is it required to call UvcFindNodeAndGetControl here?

    // Construct request message
    usb_payload::ClientRequest requestMsg;
    requestMsg.set_func_name(usb_payload::FunctionName::STORAGE_READ_CAPACITY);
    requestMsg.add_func_int32_param(static_cast<::google::int32>(nbBytes));

    // Send request
    std::string requestStr;
    requestMsg.SerializeToString(&requestStr);
    status = UsbWindowsUtils::uvcExUnitSendRequest(m_implData->handle->pVideoInputFilter, requestStr);
    if (status != aditof::Status::OK) {
        LOG(ERROR) << "Request to read memory size failed";
        return status;
    }

    // Read UVC gadget response
    std::string responseStr;
    status = UsbWindowsUtils::uvcExUnitGetResponse(m_implData->handle->pVideoInputFilter, responseStr);
    if (status != aditof::Status::OK) {
        LOG(ERROR) << "Failed to get response of the request to read memory size";
        return status;
    }
    usb_payload::ServerResponse responseMsg;
    bool parsed = responseMsg.ParseFromString(responseStr);
    if (!parsed) {
        LOG(ERROR) << "Failed to deserialize string containing UVC gadget response";
        return aditof::Status::INVALID_ARGUMENT;
    }

    DLOG(INFO) << "Received the following message: " << responseMsg.DebugString();

    if (responseMsg.status() != usb_payload::Status::OK) {
        LOG(ERROR) << "Read memory size operation failed on UVC gadget";
        return static_cast<aditof::Status>(responseMsg.status());
    }
   
    memcpy(&nbBytes, responseMsg.bytes_payload(0).c_str(), responseMsg.bytes_payload(0).length());

    return Status::OK;
}

Status UsbStorage::close() {
    using namespace aditof;

    Status status = Status::OK;

    // Construct request message
    usb_payload::ClientRequest requestMsg;
    requestMsg.set_func_name(usb_payload::FunctionName::STORAGE_CLOSE);

        // Send request
    std::string requestStr;
    requestMsg.SerializeToString(&requestStr);
    status = UsbWindowsUtils::uvcExUnitSendRequest(m_implData->handle->pVideoInputFilter, requestStr);
    if (status != aditof::Status::OK) {
        LOG(ERROR) << "Request close flash storage failed";
        return status;
    }

    // Read UVC gadget response
    std::string responseStr;
    status = UsbWindowsUtils::uvcExUnitGetResponse(m_implData->handle->pVideoInputFilter, responseStr);
    if (status != aditof::Status::OK) {
        LOG(ERROR) << "Failed to get response of the request to close flash memory";
        return status;
    }
    usb_payload::ServerResponse responseMsg;
    bool parsed = responseMsg.ParseFromString(responseStr);
    if (!parsed) {
        LOG(ERROR) << "Failed to deserialize string containing UVC gadget response";
        return aditof::Status::INVALID_ARGUMENT;
    }

    DLOG(INFO) << "Received the following message: " << responseMsg.DebugString();

    if (responseMsg.status() != usb_payload::Status::OK) {
        LOG(ERROR) << "Close flash memory operation failed on UVC gadget";
        return static_cast<aditof::Status>(responseMsg.status());
    }

    m_implData->handle = nullptr;

    return Status::OK;
}

Status UsbStorage::getName(std::string &name) const {
    name = m_implData->name;
    return Status::OK;
}
