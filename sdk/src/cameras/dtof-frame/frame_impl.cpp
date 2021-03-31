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
#include "frame_impl.h"
#include <aditof/frame_operations.h>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <fstream>
#include <glog/logging.h>
#include <unordered_map>

struct FrameImpl::ImplData {
    std::unordered_map<std::string, uint16_t *> m_dataLocations;
    std::unique_ptr<uint16_t> m_allData;
    size_t allDataNbBytes;
};

FrameImpl::FrameImpl() : m_implData(new FrameImpl::ImplData) {}

FrameImpl::~FrameImpl() = default;

FrameImpl::FrameImpl(const FrameImpl &op) {
    allocFrameData(op.m_details);
    memcpy(m_implData->m_allData.get(), op.m_implData->m_allData.get(),
           m_implData->allDataNbBytes);
    m_details = op.m_details;
}

FrameImpl &FrameImpl::operator=(const FrameImpl &op) {
    if (this != &op) {
        allocFrameData(op.m_details);
        memcpy(m_implData->m_allData.get(), op.m_implData->m_allData.get(),
               m_implData->allDataNbBytes);
        m_details = op.m_details;
    }

    return *this;
}

aditof::Status FrameImpl::setDetails(const aditof::FrameDetails &details) {
    using namespace aditof;
    Status status = Status::OK;

    if (details == m_details) {
        LOG(INFO) << "Same details provided. Doing nothing.";
        return status;
    }

    allocFrameData(details);
    m_details = details;

    return status;
}

aditof::Status FrameImpl::getDetails(aditof::FrameDetails &details) const {
    details = m_details;

    return aditof::Status::OK;
}

aditof::Status
FrameImpl::getDataDetails(const std::string &dataType,
                          aditof::FrameDataDetails &details) const {
    auto detailsIter =
        std::find_if(m_details.dataDetails.begin(), m_details.dataDetails.end(),
                     [&dataType](const aditof::FrameDataDetails &details) {
                         return dataType == details.type;
                     });
    if (detailsIter == m_details.dataDetails.end()) {
        LOG(WARNING) << "Could not find any details for type: " << dataType;
        return aditof::Status::INVALID_ARGUMENT;
    }

    details = *detailsIter;

    return aditof::Status::OK;
}

aditof::Status FrameImpl::getData(const std::string &dataType,
                                  uint16_t **dataPtr) {
    using namespace aditof;

    if (m_implData->m_dataLocations.count(dataType) > 0) {
        *dataPtr = m_implData->m_dataLocations[dataType];
    } else {
        dataPtr = nullptr;
        LOG(ERROR) << dataType << " is not supported by this frame!";
        return Status::INVALID_ARGUMENT;
    }

    return Status::OK;
}

void FrameImpl::allocFrameData(const aditof::FrameDetails &details) {
    unsigned int totalFrameSize = 0;

    for (const auto &details : details.dataDetails) {
        totalFrameSize += details.width * details.height;
    }

    m_implData->m_allData.reset(new uint16_t[totalFrameSize]);
    m_implData->m_dataLocations.emplace("allData", m_implData->m_allData.get());

    unsigned int pos = 0;
    for (const auto &details : details.dataDetails) {
        m_implData->m_dataLocations.emplace(details.type,
                                            m_implData->m_allData.get() + pos);
        pos += details.width * details.height;
    }
}

aditof::Status FrameImpl::getAvailableAttributes(std::vector<std::string> &
                                                 /*attributes*/) const {
    return aditof::Status::OK;
}
aditof::Status FrameImpl::setAttribute(const std::string & /*attribute*/,
                                       const std::string & /*value*/) {
    return aditof::Status::OK;
}
aditof::Status FrameImpl::getAttribute(const std::string & /*attribute*/,
                                       std::string & /*value*/) const {
    return aditof::Status::OK;
}
