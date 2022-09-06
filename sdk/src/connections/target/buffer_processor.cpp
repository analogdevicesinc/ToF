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

// TO DO: This exists in linux_utils.h which is not included on Dragoboard.
// Should not have duplicated code if possible.

#include "buffer_processor.h"

static int xioctl(int fh, unsigned int request, void *arg) {
    int r;

    do {
        r = ioctl(fh, request, arg);
    } while (-1 == r && EINTR == errno && errno != 0);

    return r;
}

BufferProcessor::BufferProcessor() : {
    m_outputFrameWitdh(0), m_outputFrameHeight(0), m_tofiConfig(nullptr),
        m_tofiComputeContext(nullptr) {}

~BufferProcessor::BufferProcessor() {}

aditof::Status BufferProcessor::open(){
    using namespace aditof;
    Status status = Status::OK;

    m_fd = ::open(m_videoDevice, O_RDWR);
    if (m_fd == -1) {
        LOG(WARNING) << "Cannot open " << OUTPUT_DEVICE << "errno: " << errno
                     << "error: " << strerror(errno);
        return Status::GENERIC_ERROR;
    }

    if(xioctl(m_fd, VIDIOC_QUERYCAP, &m_videoCapabilities) == -1)
        LOG(WARNING) << devName << " VIDIOC_QUERYCAP error";
        return Status::GENERIC_ERROR;
    }

	memset(&m_videoFormat, 0, sizeof(m_videoFormat));
    if(xioctl(m_fd, VIDIOC_G_FMT, &m_videoFormat) == -1)
        LOG(WARNING) << devName << " VIDIOC_G_FMT error";
        return Status::GENERIC_ERROR;
    }
}