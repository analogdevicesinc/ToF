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

#include "frame_handler_impl.h"
#include <aditof/frame_handler.h>
#ifdef USE_GLOG
#include <glog/logging.h>
#else
#include <aditof/log.h>
#endif
using namespace aditof;

FrameHandler::FrameHandler() : m_impl(new FrameHandlerImpl) {}

FrameHandler::~FrameHandler() = default;

FrameHandler::FrameHandler(FrameHandler &&) noexcept = default;

FrameHandler &FrameHandler::operator=(FrameHandler &&) noexcept = default;

Status FrameHandler::setOutputFilePath(const std::string &filePath) {
    return m_impl->setOutputFilePath(filePath);
}

Status FrameHandler::setInputFileName(const std::string &fullFileName) {
    return m_impl->setInputFileName(fullFileName);
}

Status FrameHandler::saveFrameToFile(aditof::Frame &frame,
                                     const std::string &fileName) {
    return m_impl->saveFrameToFile(frame, fileName);
}

Status FrameHandler::saveFrameToFileMultithread(aditof::Frame &frame,
                                                const std::string &fileName) {

    return m_impl->saveFrameToFileMultithread(frame, fileName);
}

Status FrameHandler::readNextFrame(aditof::Frame &frame,
                                   const std::string &fullFileName) {
    return m_impl->readNextFrame(frame, fullFileName);
}

Status FrameHandler::setCustomFormat(const std::string &format) {
    return m_impl->setCustomFormat(format);
}

Status FrameHandler::storeFramesToSingleFile(bool enable) {
    return m_impl->storeFramesToSingleFile(enable);
}

Status FrameHandler::setFrameContent(const std::string &frameContent) {
    return m_impl->setFrameContent(frameContent);
}
