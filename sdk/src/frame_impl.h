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
#ifndef FRAME_IMPL
#define FRAME_IMPL

#include <aditof/frame_definitions.h>
#include <aditof/status_definitions.h>

#include <memory>
#include <stdint.h>
#include <vector>
#include <map>

class FrameImpl {
  public:
    FrameImpl();
    //~FrameImpl();
    FrameImpl(const FrameImpl &op);
    FrameImpl &operator=(const FrameImpl &op);
    ~FrameImpl();

  public: // from TofFrame
    aditof::Status setDetails(const aditof::FrameDetails &details);
    aditof::Status getDetails(aditof::FrameDetails &details) const;
    aditof::Status getDataDetails(const std::string &dataType,
                                  aditof::FrameDataDetails &details) const;
    aditof::Status getData(const std::string &dataType, uint16_t **dataPtr);
    aditof::Status
    getAvailableAttributes(std::vector<std::string> &attributes) const;
    aditof::Status setAttribute(const std::string &attribute,
                                const std::string &value);
    aditof::Status getAttribute(const std::string &attribute,
                                std::string &value);

  private:
    void allocFrameData(const aditof::FrameDetails &details);

  private:
    struct ImplData;
    aditof::FrameDetails m_details;
    std::unique_ptr<ImplData> m_implData;
    std::map<std::string, std::string> m_attributes;
    template<typename IntType> aditof::Status getIntAttribute(const std::string &attribute_key, IntType &attribute_value);
    aditof::FrameDataDetails getFrameDetailByName(const aditof::FrameDetails &details, const std::string name);
};

#endif // FRAME_IMPL
