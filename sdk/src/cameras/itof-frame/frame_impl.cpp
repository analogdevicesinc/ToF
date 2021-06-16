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
#include "aditof_internal.h"
#include "aditof/frame_operations.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <fstream>
#include <glog/logging.h>
#include <unordered_map>

const std::vector<std::string> availableAttributes = {
    "mode",
    "width",
    "height",
    "subframes",
    "embed_width", 
    "embed_height", 
    "passive_ir", 
    "total_captures",
    "embed_hdr_length"
};

struct FrameImpl::ImplData {
    std::unordered_map<std::string, uint16_t *> m_dataLocations;
    std::shared_ptr<uint16_t> m_allData;
    size_t allDataNbBytes;
};

FrameImpl::FrameImpl() : m_implData(new FrameImpl::ImplData) {
    for (auto attributeKey : availableAttributes){
        m_attributes.emplace(attributeKey, "");
        if (attributeKey == "embed_hdr_length"){
            m_attributes[attributeKey] = std::to_string(EMBED_HDR_LENGTH);
        }
    }
}

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

aditof::Status FrameImpl::getDataDetails(const std::string &dataType,
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

template<typename IntType> aditof::Status FrameImpl::getIntAttribute(const std::string &attribute_key, IntType &attribute_value) {
    aditof::Status status = aditof::Status::OK;
    std::string attribute_str;
    status = FrameImpl::getAttribute(attribute_key, attribute_str);

    if (status == aditof::Status::INVALID_ARGUMENT) {
        return status;
    }

    attribute_value = std::atoi(attribute_str.c_str());
    return aditof::Status::OK;
}

aditof::FrameDataDetails FrameImpl::getFrameDetailByName(const aditof::FrameDetails &details, const std::string name) {
    auto frame_detail = std::find_if(details.dataDetails.begin(), details.dataDetails.end(),
                                [&name](const aditof::FrameDataDetails frame_detail) {
                                    return frame_detail.type == name;
                                });

    if (frame_detail == details.dataDetails.end()) {
        LOG(WARNING) << "Could not find any attribute with name: " << name;
    }

    return *frame_detail;
}

void FrameImpl::allocFrameData(const aditof::FrameDetails &details) {
    using namespace aditof;
    unsigned int totalSize = 0;
    unsigned int pos = 0;
    uint16_t  embed_hdr_length;
    uint8_t total_captures;

    //get attributes 
    getIntAttribute<uint16_t>("embed_hdr_length", embed_hdr_length);
    getIntAttribute<uint8_t>("total_captures", total_captures);

    auto getSubframeSize = [embed_hdr_length, total_captures](FrameDataDetails frameDetail){
        if (frameDetail.type == "header"){
            return (unsigned long int)(embed_hdr_length / 2) * total_captures;
        }
        else if(frameDetail.type == "xyz"){
            return (unsigned long int)(frameDetail.height * frameDetail.width * sizeof(Point3I));
        }
        else {
            return (unsigned long int)frameDetail.height * frameDetail.width;
        }
    };

    //compute total size TODO this could be precomputed TBD @dNechita
    for (FrameDataDetails frameDetail : details.dataDetails){
        totalSize += getSubframeSize(frameDetail);
    }

    //store pointers to the contents described by FrameDetails
    m_implData->m_allData.reset(new uint16_t[totalSize]);
   
    //TODO wouldn`t it be safer to store relative position to .get() instead of absolute address ? TBD @dNechita
    m_implData->m_dataLocations.emplace("frameData", m_implData->m_allData.get()); //frame data
    for (FrameDataDetails frameDetail : details.dataDetails){
        m_implData->m_dataLocations.emplace(frameDetail.type, m_implData->m_allData.get() + pos); //raw data
        LOG(INFO) << frameDetail.type;

        pos  += getSubframeSize(frameDetail);
    }

}

aditof::Status FrameImpl::getAvailableAttributes(std::vector<std::string> &attributes) const {
    attributes.clear();

    for (auto const &key_value_pair : m_attributes) {
        attributes.emplace_back(key_value_pair.first);
    }
    return aditof::Status::OK;
}

aditof::Status FrameImpl::setAttribute(const std::string &attribute,
                                       const std::string &value) {
    /*if (m_attributes.find(attribute) == m_attributes.end()) {
        LOG(WARNING) << "Could not find any attribute with name: " << attribute;
        return aditof::Status::INVALID_ARGUMENT;
    }*/

    m_attributes[attribute] = value;
    return aditof::Status::OK;
}

aditof::Status FrameImpl::getAttribute(const std::string &attribute,
                                       std::string &value) {
    if (m_attributes.find(attribute) == m_attributes.end()) {
        LOG(WARNING) << "Could not find any attribute with name: " << attribute;
        return aditof::Status::INVALID_ARGUMENT;
    }

    value = m_attributes[attribute];
    return aditof::Status::OK;
}
