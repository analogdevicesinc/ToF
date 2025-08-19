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
#include "connections/usb/usb_depth_sensor.h"
#include "connections/usb/usb_utils.h"
#include "usb_buffer.pb.h"
#include "usb_linux_utils.h"

#include "device_utils.h"

#include <cmath>
#include <fcntl.h>
#include <glog/logging.h>
#include <linux/usb/video.h>
#include <linux/uvcvideo.h>
#include <linux/videodev2.h>
#include <sys/mman.h>
#include <unordered_map>

struct buffer {
    void *start;
    size_t length;
};

struct CalibrationData {
    std::string mode;
    float gain;
    float offset;
    uint16_t *cache;
};

struct UsbDepthSensor::ImplData {
    int fd;
    struct buffer *buffers;
    unsigned int buffersCount;
    struct v4l2_format fmt;
    bool opened;
    bool started;
    std::unordered_map<std::string, CalibrationData> calibration_cache;
};

UsbDepthSensor::UsbDepthSensor(const std::string &name, const std::string &driverPath)
    : m_driverPath(driverPath), m_implData(new UsbDepthSensor::ImplData) {
    m_implData->fd = 0;
    m_implData->opened = false;
    m_implData->started = false;
    m_implData->buffers = nullptr;
    m_implData->buffersCount = 0;
    m_sensorDetails.connectionType = aditof::ConnectionType::USB;
    m_sensorName = name;
}

UsbDepthSensor::~UsbDepthSensor() {
    if (m_implData->started) {
        stop();
    }

    for (auto it = m_implData->calibration_cache.begin();
         it != m_implData->calibration_cache.begin(); ++it) {
        delete[] it->second.cache;
        it->second.cache = nullptr;
    }

    for (unsigned int i = 0; i < m_implData->buffersCount; ++i) {
        if (-1 == munmap(m_implData->buffers[i].start,
                         m_implData->buffers[i].length)) {
            LOG(WARNING) << "munmap, error:" << errno << "(" << strerror(errno)
                         << ")";
            return;
        }
    }
    if (m_implData->buffers)
        free(m_implData->buffers);

    if (m_implData->fd != 0) {
        if (-1 == close(m_implData->fd))
            LOG(WARNING) << "close, error:" << errno << "(" << strerror(errno)
                         << ")";
    }
}

aditof::Status UsbDepthSensor::open() {
    using namespace aditof;
    Status status = Status::OK;
    std::string availableFrameTypesBlob;

    LOG(INFO) << "Opening device";

    m_implData->fd = ::open(m_driverPath.c_str(), O_RDWR | O_NONBLOCK, 0);
    if (-1 == m_implData->fd) {
        LOG(WARNING) << "Cannot open '" << m_driverPath << "' error: " << errno
                     << "(" << strerror(errno) << ")";
        return Status::UNREACHABLE;
    }

    CLEAR(m_implData->fmt);

    m_implData->fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    // Preserve original settings as set by v4l2-ctl for example
    if (-1 ==
        UsbLinuxUtils::xioctl(m_implData->fd, VIDIOC_G_FMT, &m_implData->fmt)) {
        LOG(WARNING) << "VIDIOC_G_FMT, error:" << errno << "("
                     << strerror(errno) << ")";
        return Status::GENERIC_ERROR;
    }

    // Query the target about the frame types that are supported by the depth sensor

    // Send request
    usb_payload::ClientRequest requestMsg;
    requestMsg.set_func_name(
        usb_payload::FunctionName::GET_AVAILABLE_FRAME_TYPES);
    std::string requestStr;
    requestMsg.SerializeToString(&requestStr);
    status = UsbLinuxUtils::uvcExUnitSendRequest(m_implData->fd, requestStr);
    if (status != aditof::Status::OK) {
        LOG(ERROR) << "Request to get available frame types failed";
        return status;
    }

    // Read UVC gadget response
    std::string responseStr;
    status = UsbLinuxUtils::uvcExUnitGetResponse(m_implData->fd, responseStr);
    if (status != aditof::Status::OK) {
        LOG(ERROR) << "Response for get available frame types request failed";
        return status;
    }
    usb_payload::ServerResponse responseMsg;
    bool parsed = responseMsg.ParseFromString(responseStr);
    if (!parsed) {
        LOG(ERROR)
            << "Failed to deserialize string containing UVC gadget response";
        return aditof::Status::INVALID_ARGUMENT;
    }

    if (responseMsg.status() != usb_payload::Status::OK) {
        LOG(ERROR)
            << "Get available frame types operation failed on UVC gadget";
        return static_cast<aditof::Status>(responseMsg.status());
    }

    // If request and response went well, extract data from response
    UsbUtils::protoMsgToDepthSensorFrameTypes(
        m_depthSensorFrameTypes, responseMsg.available_frame_types());

    m_implData->opened = true;

    return status;
}

aditof::Status UsbDepthSensor::start() {
    using namespace aditof;

    if (m_implData->started) {
        LOG(INFO) << "Device already started";
        return Status::BUSY;
    }
    LOG(INFO) << "Starting device";

    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (-1 == UsbLinuxUtils::xioctl(m_implData->fd, VIDIOC_STREAMON, &type)) {
        LOG(WARNING) << "VIDIOC_STREAMON, error:" << errno << "("
                     << strerror(errno) << ")";
        return Status::GENERIC_ERROR;
    }

    m_implData->started = true;

    return Status::OK;
}

aditof::Status UsbDepthSensor::stop() {
    using namespace aditof;

    if (!m_implData->started) {
        LOG(INFO) << "Device already stopped";
        return Status::BUSY;
    }
    LOG(INFO) << "Stopping device";

    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (-1 == UsbLinuxUtils::xioctl(m_implData->fd, VIDIOC_STREAMOFF, &type)) {
        LOG(WARNING) << "VIDIOC_STREAMOFF, error:" << errno << "("
                     << strerror(errno) << ")";
        return Status::GENERIC_ERROR;
    }

    m_implData->started = false;

    return Status::OK;
}

aditof::Status UsbDepthSensor::getAvailableFrameTypes(
    std::vector<aditof::DepthSensorFrameType> &types) {
    using namespace aditof;
    Status status = Status::OK;

    types = m_depthSensorFrameTypes;

    return status;
}

aditof::Status
UsbDepthSensor::setFrameType(const aditof::DepthSensorFrameType &type) {
    using namespace aditof;

    Status status = Status::OK;

    // Send the frame type and all its content all the way to target
    usb_payload::ClientRequest requestMsg;
    auto frameTypeMsg = requestMsg.mutable_frame_type();
    UsbUtils::depthSensorFrameTypeToProtoMsg(type, frameTypeMsg);
    // Send request
    requestMsg.set_func_name(usb_payload::FunctionName::SET_FRAME_TYPE);
    std::string requestStr;
    requestMsg.SerializeToString(&requestStr);
    status = UsbLinuxUtils::uvcExUnitSendRequest(m_implData->fd, requestStr);
    if (status != Status::OK) {
        LOG(ERROR) << "Set frame type operation failed on UVC gadget";
        return status;
    }
    struct v4l2_requestbuffers req;

    req.count = 0;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;
    if (UsbLinuxUtils::xioctl(m_implData->fd, VIDIOC_REQBUFS, &req) == -1) {
        LOG(WARNING) << "VIDIOC_REQBUFS error "
                     << "errno: " << errno << " error: " << strerror(errno);
        return Status::GENERIC_ERROR;
    }

    bool found = false;
    struct v4l2_fmtdesc fmtdesc;
    memset(&fmtdesc,0,sizeof(fmtdesc));
    fmtdesc.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    while (UsbLinuxUtils::xioctl(m_implData->fd, VIDIOC_ENUM_FMT, &fmtdesc) == 0)
    {
        struct v4l2_frmsizeenum frmenum;
        memset(&frmenum, 0, sizeof frmenum);
        frmenum.pixel_format = fmtdesc.pixelformat;
        while (UsbLinuxUtils::xioctl(m_implData->fd, VIDIOC_ENUM_FRAMESIZES, &frmenum) == 0)
        {
            if ((frmenum.discrete.width == type.width) &&
                (frmenum.discrete.height == type.height)) {
                    found = true;
                    break;
                }
            frmenum.index++;
        }
        if (found)
            break;
        else
            fmtdesc.index++;
    }

    if (!found) {
        LOG(WARNING) << "UVC does not support the requested format "
                     << "errno: " << errno << " error: " << strerror(errno);
        return Status::GENERIC_ERROR;
    }

    m_implData->fmt.fmt.pix.width = type.width;
    m_implData->fmt.fmt.pix.height = type.height;
    m_implData->fmt.fmt.pix.pixelformat = fmtdesc.pixelformat;
    m_implData->fmt.fmt.pix.field = V4L2_FIELD_ANY;

    if (-1 ==
        UsbLinuxUtils::xioctl(m_implData->fd, VIDIOC_S_FMT, &m_implData->fmt)) {
        LOG(WARNING) << "Failed to set Pixel Format, error: " << errno << "("
                     << strerror(errno) << ")";
        return Status::GENERIC_ERROR;
    }


    CLEAR(req);
    req.count = 4;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;

    if (-1 == UsbLinuxUtils::xioctl(m_implData->fd, VIDIOC_REQBUFS, &req)) {
        if (EINVAL == errno) {
            LOG(WARNING) << m_driverPath << " does not support memmory mapping";
        } else {
            LOG(WARNING) << "VIDIOC_REQBUFS, error:" << errno << "("
                         << strerror(errno) << ")";
        }
        return Status::GENERIC_ERROR;
    }

    if (req.count < 2) {
        LOG(WARNING) << "Insufficient buffer memory on " << m_driverPath;
        return Status::GENERIC_ERROR;
    }

    if (!m_implData->buffers) {
        m_implData->buffers =
            static_cast<buffer *>(calloc(req.count, sizeof(struct buffer)));
    }

    if (!m_implData->buffers) {
        LOG(WARNING) << "Out of memory";
        return Status::GENERIC_ERROR;
    }

    for (m_implData->buffersCount = 0; m_implData->buffersCount < req.count;
         ++m_implData->buffersCount) {
        struct v4l2_buffer buf;

        CLEAR(buf);

        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = m_implData->buffersCount;

        if (-1 ==
            UsbLinuxUtils::xioctl(m_implData->fd, VIDIOC_QUERYBUF, &buf)) {
            LOG(WARNING) << "VIDIOC_QUERYBUF, error:" << errno << "("
                         << strerror(errno) << ")";
            return Status::GENERIC_ERROR;
        }

        // TO DO: Check if is better to use mremap()

        m_implData->buffers[m_implData->buffersCount].length = buf.length;
        m_implData->buffers[m_implData->buffersCount].start =
            mmap(nullptr, // start anywhere ,
                 buf.length,
                 PROT_READ | PROT_WRITE, // required,
                 MAP_SHARED,             // recommended ,
                 m_implData->fd, buf.m.offset);

        if (MAP_FAILED == m_implData->buffers[m_implData->buffersCount].start) {
            LOG(WARNING) << "mmap, error:" << errno << "(" << strerror(errno)
                         << ")";
            return Status::GENERIC_ERROR;
        }
    }

    for (unsigned int i = 0; i < m_implData->buffersCount; ++i) {
        struct v4l2_buffer buf;

        CLEAR(buf);
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = i;

        if (-1 == UsbLinuxUtils::xioctl(m_implData->fd, VIDIOC_QBUF, &buf)) {
            LOG(WARNING) << "VIDIOC_QBUF, error:" << errno << "("
                         << strerror(errno) << ")";
            return Status::GENERIC_ERROR;
        }
    }

    return status;
}

aditof::Status UsbDepthSensor::program(const uint8_t *firmware, size_t size) {
    using namespace aditof;

    if (!firmware) {
        LOG(WARNING) << "No firmware provided";
        return Status::INVALID_ARGUMENT;
    }

    struct uvc_xu_control_query cq;
    unsigned char buf[MAX_BUF_SIZE];
    size_t written_bytes = 0;
    __useconds_t sleepDuration =
        100000; /* Keep 100 ms delay between 'program' calls */

    while (written_bytes < size) {

        CLEAR(cq);
        cq.query = UVC_SET_CUR; // bRequest
        cq.unit = 0x03;         // wIndex of Extension Unit
        cq.selector = 1;        // WValue for AFE Programming
        cq.data = buf;
        cq.size = MAX_BUF_SIZE;

        usleep(5000);
        if ((size - written_bytes) > MAX_PACKET_SIZE) {
            buf[0] = 0x01;
            buf[1] = MAX_PACKET_SIZE;
            memcpy(&buf[2], firmware + written_bytes, MAX_PACKET_SIZE);

            if (-1 ==
                UsbLinuxUtils::xioctl(m_implData->fd, UVCIOC_CTRL_QUERY, &cq)) {
                LOG(WARNING)
                    << "Programming AFE error "
                    << "errno: " << errno << " error: " << strerror(errno);
            }
            written_bytes += MAX_PACKET_SIZE;
        } else {
            CLEAR(buf);
            buf[0] = 0x02;
            buf[1] = static_cast<unsigned char>(size - written_bytes);
            memcpy(&buf[2], firmware + written_bytes, buf[1]);

            cq.data = buf;
            if (-1 ==
                UsbLinuxUtils::xioctl(m_implData->fd, UVCIOC_CTRL_QUERY, &cq)) {
                LOG(WARNING)
                    << "Programming AFE error "
                    << "errno: " << errno << " error: " << strerror(errno);
            }
            written_bytes = size;
        }
    }

    // TO DO: Check if it is really neccessary or if the delay is not to much
    usleep(sleepDuration);

    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (-1 == UsbLinuxUtils::xioctl(m_implData->fd, VIDIOC_STREAMON, &type)) {
        LOG(WARNING) << "VIDIOC_STREAMON, error:" << errno << "("
                     << strerror(errno) << ")";
        return Status::GENERIC_ERROR;
    }

    m_implData->started = true;

    return Status::OK;
}

aditof::Status UsbDepthSensor::getFrame(uint16_t *buffer) {
    using namespace aditof;
    Status status = Status::OK;

    if (!buffer) {
        LOG(WARNING) << "Invalid adddress to buffer provided";
        return Status::INVALID_ARGUMENT;
    }

    fd_set fds;
    struct timeval tv;
    int r;

    FD_ZERO(&fds);
    FD_SET(m_implData->fd, &fds);

    // Timeout : Ensure this compensates for max delays added for programming
    // cycle defined in 'Device::program'
    tv.tv_sec = 2;
    tv.tv_usec = 0;

    r = select(m_implData->fd + 1, &fds, nullptr, nullptr, &tv);

    if (-1 == r) {
        if (EINTR == errno) {
            LOG(WARNING) << "select, error: " << errno << "(" << strerror(errno)
                         << ")";
            return Status::GENERIC_ERROR;
        }
    }

    if (0 == r) {
        LOG(WARNING) << "select timeout: ";
        return Status::BUSY;
    }

    struct v4l2_buffer buf;

    CLEAR(buf);

    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;

    if (-1 == UsbLinuxUtils::xioctl(m_implData->fd, VIDIOC_DQBUF, &buf)) {
        LOG(WARNING) << "Stream Error";
        switch (errno) {
        case EAGAIN:
            break;

        case EIO:
            // Could ignore EIO, see spec.
            // fall through

        default: {
            LOG(WARNING) << "VIDIOC_DQBUF, error: " << errno << "("
                         << strerror(errno) << ")";
            return Status::GENERIC_ERROR;
        }
        }
    }

    if (buf.index >= m_implData->buffersCount) {
        LOG(WARNING) << "buffer index out of range";
        return Status::INVALID_ARGUMENT;
    }

    const char *pdata =
        static_cast<const char *>(m_implData->buffers[buf.index].start);

    memcpy(buffer, pdata, m_implData->buffers[buf.index].length);

    if (-1 == UsbLinuxUtils::xioctl(m_implData->fd, VIDIOC_QBUF, &buf)) {
        LOG(WARNING) << "VIDIOC_QBUF, error: " << errno << "("
                     << strerror(errno) << ")";
        return Status::GENERIC_ERROR;
    }

    return status;
}

aditof::Status UsbDepthSensor::readRegisters(const uint16_t *address,
                                             uint16_t *data, size_t length,
                                             bool burst) {
    using namespace aditof;
    Status status = Status::OK;

    // Construct request message
    usb_payload::ClientRequest requestMsg;
    requestMsg.set_func_name(usb_payload::FunctionName::READ_REGISTERS);
    requestMsg.add_func_int32_param(static_cast<::google::int32>(length));
    requestMsg.add_func_int32_param(static_cast<::google::int32>(burst));
    requestMsg.add_func_bytes_param(address,
                                    burst ? 2 : length * sizeof(uint16_t));

    // Send request
    std::string requestStr;
    requestMsg.SerializeToString(&requestStr);
    status = UsbLinuxUtils::uvcExUnitSendRequest(m_implData->fd, requestStr);
    if (status != aditof::Status::OK) {
        LOG(ERROR) << "Request to read registers failed";
        return status;
    }

    // Read UVC gadget response
    std::string responseStr;
    status = UsbLinuxUtils::uvcExUnitGetResponse(m_implData->fd, responseStr);
    if (status != aditof::Status::OK) {
        LOG(ERROR) << "Failed to get response of the request to read registers";
        return status;
    }
    usb_payload::ServerResponse responseMsg;
    bool parsed = responseMsg.ParseFromString(responseStr);
    if (!parsed) {
        LOG(ERROR)
            << "Failed to deserialize string containing UVC gadget response";
        return aditof::Status::INVALID_ARGUMENT;
    }

    if (responseMsg.status() != usb_payload::Status::OK) {
        LOG(ERROR) << "Read registers operation failed on UVC gadget";
        return static_cast<aditof::Status>(responseMsg.status());
    }

    // If request and response went well, extract data from response
    memcpy(data, responseMsg.bytes_payload(0).c_str(),
           responseMsg.bytes_payload(0).length());

    return Status::OK;
}

aditof::Status UsbDepthSensor::writeRegisters(const uint16_t *address,
                                              const uint16_t *data,
                                              size_t length, bool burst) {
    using namespace aditof;
    Status status = Status::OK;

    // Construct request message
    usb_payload::ClientRequest requestMsg;
    requestMsg.set_func_name(usb_payload::FunctionName::WRITE_REGISTERS);
    requestMsg.add_func_int32_param(static_cast<::google::int32>(length));
    requestMsg.add_func_int32_param(static_cast<::google::int32>(burst));
    requestMsg.add_func_bytes_param(address,
                                    burst ? 2 : length * sizeof(uint16_t));
    requestMsg.add_func_bytes_param(data, length * sizeof(uint16_t));

    // Send request
    std::string requestStr;
    requestMsg.SerializeToString(&requestStr);
    status = UsbLinuxUtils::uvcExUnitSendRequest(m_implData->fd, requestStr);
    if (status != aditof::Status::OK) {
        LOG(ERROR) << "Request to write registers failed";
        return status;
    }

    // Read UVC gadget response
    std::string responseStr;
    status = UsbLinuxUtils::uvcExUnitGetResponse(m_implData->fd, responseStr);
    if (status != aditof::Status::OK) {
        LOG(ERROR)
            << "Failed to get response of the request to write registers";
        return status;
    }
    usb_payload::ServerResponse responseMsg;
    bool parsed = responseMsg.ParseFromString(responseStr);
    if (!parsed) {
        LOG(ERROR)
            << "Failed to deserialize string containing UVC gadget response";
        return aditof::Status::INVALID_ARGUMENT;
    }

    if (responseMsg.status() != usb_payload::Status::OK) {
        LOG(ERROR) << "Read registers operation failed on UVC gadget";
        return static_cast<aditof::Status>(responseMsg.status());
    }

    return Status::OK;
}

aditof::Status UsbDepthSensor::getAvailableControls(std::vector<std::string> &controls) const {
    using namespace aditof;
    Status status = Status::OK;

    // Construct request message
    usb_payload::ClientRequest requestMsg;
    requestMsg.set_func_name(usb_payload::FunctionName::GET_AVAILABLE_CONTROLS);

    // Send request
    std::string requestStr;
    requestMsg.SerializeToString(&requestStr);
    status = UsbLinuxUtils::uvcExUnitSendRequest(m_implData->fd, requestStr);
    if (status != aditof::Status::OK) {
        LOG(ERROR) << "Request to get available controls failed";
        return status;
    }

    // Read UVC gadget response
    std::string responseStr;
    status = UsbLinuxUtils::uvcExUnitGetResponse(m_implData->fd, responseStr);
    if (status != aditof::Status::OK) {
        LOG(ERROR)
            << "Failed to get response of the request to get controls";
        return status;
    }
    usb_payload::ServerResponse responseMsg;
    bool parsed = responseMsg.ParseFromString(responseStr);
    if (!parsed) {
        LOG(ERROR)
            << "Failed to deserialize string containing UVC gadget response";
        return aditof::Status::INVALID_ARGUMENT;
    }

    if (responseMsg.status() != usb_payload::Status::OK) {
        LOG(ERROR) << "Get available controls operation failed on UVC gadget";
        return static_cast<aditof::Status>(responseMsg.status());
    }

    controls.clear();

    for (int i = 0; i < responseMsg.strings_payload_size(); i++) {
        std::string controlName = responseMsg.strings_payload(i);
        controls.push_back(controlName);
    }

    return Status::OK;
}

aditof::Status UsbDepthSensor::setControl(const std::string &control,
                               const std::string &value) {
    using namespace aditof;
    Status status = Status::OK;

    // Construct request message
    usb_payload::ClientRequest requestMsg;
    requestMsg.set_func_name(usb_payload::FunctionName::SET_CONTROL);
    requestMsg.add_func_strings_param(control);
    requestMsg.add_func_strings_param(value);

    // Send request
    std::string requestStr;
    requestMsg.SerializeToString(&requestStr);
    status = UsbLinuxUtils::uvcExUnitSendRequest(m_implData->fd, requestStr);
    if (status != aditof::Status::OK) {
        LOG(ERROR) << "Request to set control failed: " << control;
        return status;
    }

    // Read UVC gadget response
    std::string responseStr;
    status = UsbLinuxUtils::uvcExUnitGetResponse(m_implData->fd, responseStr);
    if (status != aditof::Status::OK) {
        LOG(ERROR) << "Failed to get response of the request to set control: " << control;
        return status;
    }
    usb_payload::ServerResponse responseMsg;
    bool parsed = responseMsg.ParseFromString(responseStr);
    if (!parsed) {
        LOG(ERROR) << "Failed to deserialize string containing UVC gadget response";
        return aditof::Status::INVALID_ARGUMENT;
    }

    if (responseMsg.status() != usb_payload::Status::OK) {
        LOG(ERROR) << "Set control:" << control << " operation failed on UVC gadget";
        return static_cast<aditof::Status>(responseMsg.status());
    }

    return Status::OK;
}

aditof::Status UsbDepthSensor::getControl(const std::string &control,
                               std::string &value) const {
    using namespace aditof;
    Status status = Status::OK;

    // Construct request message
    usb_payload::ClientRequest requestMsg;
    requestMsg.set_func_name(usb_payload::FunctionName::GET_CONTROL);
    requestMsg.add_func_strings_param(control);

    // Send request
    std::string requestStr;
    requestMsg.SerializeToString(&requestStr);
    status = UsbLinuxUtils::uvcExUnitSendRequest(m_implData->fd, requestStr);
    if (status != aditof::Status::OK) {
        LOG(ERROR) << "Request to get control: " << control << " failed";
        return status;
    }

    // Read UVC gadget response
    std::string responseStr;
    status = UsbLinuxUtils::uvcExUnitGetResponse(m_implData->fd, responseStr);
    if (status != aditof::Status::OK) {
        LOG(ERROR) << "Failed to get response of the request to get control: " << control;
        return status;
    }
    usb_payload::ServerResponse responseMsg;
    bool parsed = responseMsg.ParseFromString(responseStr);
    if (!parsed) {
        LOG(ERROR) << "Failed to deserialize string containing UVC gadget response";
        return aditof::Status::INVALID_ARGUMENT;
    }

    if (responseMsg.status() != usb_payload::Status::OK) {
        LOG(ERROR) << "Get control: " << control << " operation failed on UVC gadget";
        return static_cast<aditof::Status>(responseMsg.status());
    }

    value = responseMsg.strings_payload(0);

    return Status::OK;
}

aditof::Status
UsbDepthSensor::getDetails(aditof::SensorDetails &details) const {
    details = m_sensorDetails;
    return aditof::Status::OK;
}

aditof::Status UsbDepthSensor::getHandle(void **handle) {
    if (m_implData->opened) {
        *handle = &m_implData->fd;
        return aditof::Status::OK;
    } else {
        *handle = nullptr;
        LOG(ERROR) << "Won't return the handle. Device hasn't been opened yet.";
        return aditof::Status::UNAVAILABLE;
    }
}

aditof::Status UsbDepthSensor::getName(std::string &name) const {
    name = m_sensorName;
    return aditof::Status::OK;
}

aditof::Status UsbDepthSensor::adsd3500_read_cmd(uint16_t cmd, uint16_t *data){
    return aditof::Status::OK;
}

aditof::Status UsbDepthSensor::adsd3500_write_cmd(uint16_t cmd, uint16_t data) {
    return aditof::Status::OK;
}

aditof::Status UsbDepthSensor::adsd3500_read_payload_cmd(uint32_t cmd, uint8_t* readback_data, uint16_t payload_len) {
    return aditof::Status::OK;
}

aditof::Status UsbDepthSensor::adsd3500_read_payload(uint8_t* payload, uint16_t payload_len) {
    return aditof::Status::OK;
}

aditof::Status UsbDepthSensor::adsd3500_write_payload_cmd(uint32_t cmd, uint8_t* payload, uint16_t payload_len) {
    return aditof::Status::OK;
}

aditof::Status UsbDepthSensor::adsd3500_write_payload(uint8_t* payload, uint16_t payload_len) {
    return aditof::Status::OK;
}