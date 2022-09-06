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

struct VideoDev {
    int fd;
    int sfd;
    struct buffer *videoBuffers;
    unsigned int nVideoBuffers;
    struct v4l2_plane planes[8];
    enum v4l2_buf_type videoBuffersType;
    bool started;

    VideoDev()
        : fd(-1), sfd(-1), videoBuffers(nullptr), nVideoBuffers(0),
          started(false) {}
};

BufferProcessor::BufferProcessor() : {
    m_outputFrameWitdh(0), m_outputFrameHeight(0), m_tofiConfig(nullptr),
        m_tofiComputeContext(nullptr), m_vidPropSet(false),
        m_processorPropSet(false), m_videoDev(nullptr)
}

~BufferProcessor::BufferProcessor() {
    if (NULL != m_tofiComputeContext) {
        LOG(INFO) << "freeComputeLibrary";
        FreeTofiCompute(m_tofiComputeContext);
        m_tofiComputeContext = NULL;
    }

    if (m_tofiConfig != NULL) {
        FreeTofiConfig(m_tofiConfig);
        m_tofiConfig = NULL;
    }

    freeConfigData();

    if (m_fd != 0) {
        if (::close(m_fd) == -1) {
            LOG(ERROR) << "Failed to close " << m_videoDevice
                       << " error: " << strerror(errno);
        }
    }

    return aditof::Status::OK;
}

aditof::Status BufferProcessor::open() {
    using namespace aditof;
    Status status = Status::OK;

    m_videoDev->fd = ::open(m_videoDevice, O_RDWR);
    if (m_videoDev->fd == -1) {
        LOG(ERROR) << "Cannot open " << OUTPUT_DEVICE << "errno: " << errno
                   << "error: " << strerror(errno);
        return Status::GENERIC_ERROR;
    }

    if (xioctl(m_videoDev->fd, VIDIOC_QUERYCAP, &m_videoCapabilities) == -1) {
        LOG(ERROR) << devName << " VIDIOC_QUERYCAP error";
        return Status::GENERIC_ERROR;
    }

    memset(&m_videoFormat, 0, sizeof(m_videoFormat));
    if (xioctl(m_videoDev->fd, VIDIOC_G_FMT, &m_videoFormat) == -1) {
        LOG(ERROR) << devName << " VIDIOC_G_FMT error";
        return Status::GENERIC_ERROR;
    }

    return status;
}

aditof::Status BufferProcessor::setVideoProperties(int frameWidth,
                                                   int frameHeight) {
    using namespace aditof;
    Status status = Status::OK;

    m_outputFrameWidth = frameWidth;
    m_outputFrameHeight = frameHeight;

    m_videoFormat.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
    m_videoFormat.fmt.pix.width = frameWidth;
    m_videoFormat.fmt.pix.height = frameHeight;
    m_videoFormat.fmt.pix.pixelformat = V4L2_PIX_FMT_SBGGR12;
    m_videoFormat.fmt.pix.sizeimage = frameWitdh * frameHeight;
    m_videoFormat.fmt.pix.field = V4L2_FIELD_NONE;
    m_videoFormat.fmt.pix.bytesperline = framewidth;
    m_videoFormat.fmt.pix.colorspace = V4L2_COLORSPACE_SRGB;

    if (xioctl(m_videoDev->fd, VIDIOC_S_FMT, &m_videoFormat) == -1) {
        LOG(ERROR) << "Failed to set format!";
        return Status::GENERIC_ERROR;
    }

    return status;
}

aditof::Status BufferProcessor::setProcessorProperties(
    uint8_t *iniFile, uint16_t iniFileLength, uint8_t *calData,
    uint16_t calDataLength, uint16_t mode, bool ispEnabled) {

    if (ispEnabled) {
        uint32_t status = ADI_TOFI_SUCCESS;

        if (iniFile != nullptr) {
            ConfigFileData depth_ini = {pData, iniFileLength};

            if (ispEnabled) {
                memcpy(m_xyzDealiasData, calData, calDataLength);
                m_tofiConfig =
                    InitTofiConfig_isp((ConfigFileData *)&depth_ini, mode,
                                       &status, m_xyzDealiasData);
            } else {
                ConfigFileData calData = {calData, calDataLength};
                if (calData.p_data != NULL) {
                    m_tofiConfig = InitTofiConfig(&calData, NULL, &depth_ini,
                                                  mode, &status);
                } else {
                    LOG(ERROR) << "Failed to get calibration data";
                }
            }

        } else {
            m_tofiConfig = InitTofiConfig(&calData, NULL, NULL, mode, &status);
        }

        if ((m_tofiConfig == NULL) ||
            (m_tofiConfig->p_tofi_cal_config == NULL) ||
            (status != ADI_TOFI_SUCCESS)) {
            LOG(ERROR) << "InitTofiConfig failed";
            return aditof::Status::GENERIC_ERROR;

        } else {
            m_tofiComputeContext =
                InitTofiCompute(m_tofiConfig->p_tofi_cal_config, &status);
            if (m_tofiComputeContext == NULL || status != ADI_TOFI_SUCCESS) {
                LOG(ERROR) << "InitTofiCompute failed";
                return aditof::Status::GENERIC_ERROR;
            }
        }
    } else {
        LOG(ERROR) << "Could not initialize compute library because config "
                      "data hasn't been loaded";
        return aditof::Status::GENERIC_ERROR;
    }
}

aditof::Status BufferProcessor::processFrame(uint16_t *buffer = nullptr) {

}

aditof::Status BufferProcessor::waitForBufferPrivate(struct VideoDev *dev) {
    fd_set fds;
    struct timeval tv;
    int r;

    if (dev == nullptr)
        dev = &m_implData->videoDevs[0];

    FD_ZERO(&fds);
    FD_SET(dev->fd, &fds);

    tv.tv_sec = 20;
    tv.tv_usec = 0;

    r = select(dev->fd + 1, &fds, NULL, NULL, &tv);

    if (r == -1) {
        LOG(WARNING) << "select error "
                     << "errno: " << errno << " error: " << strerror(errno);
        return aditof::Status::GENERIC_ERROR;
    } else if (r == 0) {
        LOG(WARNING) << "select timeout";
        return aditof::Status::GENERIC_ERROR;
    }
    return aditof ::Status::OK;
}

aditof::Status
BufferProcessor::dequeueInternalBufferPrivate(struct v4l2_buffer &buf,
                                             struct VideoDev *dev) {
    using namespace aditof;
    Status status = Status::OK;

    if (dev == nullptr)
        dev = &m_implData->videoDevs[0];

    CLEAR(buf);
    buf.type = dev->videoBuffersType;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.length = 1;
    buf.m.planes = dev->planes;

    if (xioctl(dev->fd, VIDIOC_DQBUF, &buf) == -1) {
        LOG(WARNING) << "VIDIOC_DQBUF error "
                     << "errno: " << errno << " error: " << strerror(errno);
        switch (errno) {
        case EAGAIN:
        case EIO:
            break;
        default:
            return Status::GENERIC_ERROR;
        }
    }

    if (buf.index >= dev->nVideoBuffers) {
        LOG(WARNING) << "Not enough buffers avaialable";
        return Status::GENERIC_ERROR;
    }

    return status;
}

aditof::Status BufferProcessor::getInternalBufferPrivate(
    uint8_t **buffer, uint32_t &buf_data_len, const struct v4l2_buffer &buf,
    struct VideoDev *dev) {
    if (dev == nullptr)
        dev = &m_implData->videoDevs[0];

    *buffer = static_cast<uint8_t *>(dev->videoBuffers[buf.index].start);
    buf_data_len = buf.bytesused;

    return aditof::Status::OK;
}

aditof::Status
BufferProcessor::enqueueInternalBufferPrivate(struct v4l2_buffer &buf,
                                             struct VideoDev *dev) {
    if (dev == nullptr)
        dev = &m_implData->videoDevs[0];

    if (xioctl(dev->fd, VIDIOC_QBUF, &buf) == -1) {
        LOG(WARNING) << "VIDIOC_QBUF error "
                     << "errno: " << errno << " error: " << strerror(errno);
        return aditof::Status::GENERIC_ERROR;
    }

    return aditof::Status::OK;
}
