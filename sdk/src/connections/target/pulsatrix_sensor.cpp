/********************************************************************************/
/*                                                                              */
/* Copyright (c) 2020 Analog Devices, Inc. All Rights Reserved.                 */
/* This software is proprietary to Analog Devices, Inc. and its licensors.      */
/*                                                                              */
/********************************************************************************/
#include "pulsatrix_sensor.h"
#include "aditof/frame_operations.h"
#include "utils.h"

#include "cameras/itof-camera/mode_info.h"
#include <algorithm>
#include <arm_neon.h>
#include <cmath>
#include <fcntl.h>
#include <fstream>
#include <glog/logging.h>
#include <linux/videodev2.h>
#include <sstream>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unordered_map>

#define MAX_SUBFRAMES_COUNT                                                    \
    10 // maximum number of subframes that are used to create a full frame (maximum total_captures of all modes)
#define EXTRA_BUFFERS_COUNT                                                    \
    3 // how many extra buffers are sent to the driver in addition to the total_captures of a mode

#define CLEAR(x) memset(&(x), 0, sizeof(x))

#define V4L2_CID_AD_DEV_CHIP_CONFIG (0x9819e1)
#define CTRL_PACKET_SIZE 65537
#define CTRL_SET_MODE (0x9819e0)
#define PULSATRIX_CTRL_PACKET_SIZE 4115
// Can be moved to target_definitions in "camera"/"platform"
#define TEMP_SENSOR_DEV_PATH "/dev/i2c-1"
#define LASER_TEMP_SENSOR_I2C_ADDR 0x49
#define AFE_TEMP_SENSOR_I2C_ADDR 0x4b

#define ADI_DEBUG 1
#define REQ_COUNT 10
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

struct ConfigurationData {
    uint16_t id;
    uint16_t ver;
    uint32_t size;
    uint16_t burst_layout;
    uint16_t burst_num;
    uint16_t burst_setup[4];
    uint16_t start_address;
    uint16_t rsvd;
    uint32_t values;
};

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
struct PulsatrixSensor::ImplData {
    uint8_t numVideoDevs;
    struct VideoDev *videoDevs;
    aditof::DepthSensorFrameType frameType;
    ImplData() : numVideoDevs(1), videoDevs(nullptr), frameType{"", {}, 0, 0} {}
};

// TO DO: This exists in linux_utils.h which is not included on Dragoboard.
// Should not have duplicated code if possible.
static int xioctl(int fh, unsigned int request, void *arg) {
    int r;

    do {
        r = ioctl(fh, request, arg);
    } while (-1 == r && EINTR == errno && errno != 0);

    return r;
}

PulsatrixSensor::PulsatrixSensor(const std::string &driverPath,
                                 const std::string &driverSubPath,
                                 const std::string &captureDev)
    : m_driverPath(driverPath), m_driverSubPath(driverSubPath),
      m_captureDev(captureDev), m_implData(new PulsatrixSensor::ImplData) {
          m_sensorName = "pulsatrix";
      }

PulsatrixSensor::~PulsatrixSensor() {
    struct VideoDev *dev;

    for (unsigned int i = 0; i < m_implData->numVideoDevs; i++) {
        dev = &m_implData->videoDevs[i];
        if (dev->started) {
            stop();
        }
    }

    for (unsigned int i = 0; i < m_implData->numVideoDevs; i++) {
        dev = &m_implData->videoDevs[i];

        for (unsigned int i = 0; i < dev->nVideoBuffers; i++) {
            if (munmap(dev->videoBuffers[i].start,
                       dev->videoBuffers[i].length) == -1) {
                LOG(WARNING)
                    << "munmap error "
                    << "errno: " << errno << " error: " << strerror(errno);
            }
        }
        free(dev->videoBuffers);

        if (close(dev->fd) == -1) {
            LOG(WARNING) << "close m_implData->fd error "
                         << "errno: " << errno << " error: " << strerror(errno);
        }

        if (close(dev->sfd) == -1) {
            LOG(WARNING) << "close m_implData->sfd error "
                         << "errno: " << errno << " error: " << strerror(errno);
        }
    }
}

aditof::Status PulsatrixSensor::open() { 
    using namespace aditof;
    Status status = Status::OK;

    LOG(INFO) << "Opening device";

    struct stat st;
    struct v4l2_capability cap;
    struct VideoDev *dev;

    const char *devName, *subDevName, *cardName;

    std::vector<std::string> driverPaths;
    Utils::splitIntoTokens(m_driverPath, '|', driverPaths);

    std::vector<std::string> driverSubPaths;
    Utils::splitIntoTokens(m_driverSubPath, '|', driverSubPaths);

    std::vector<std::string> cards;
    std::string captureDeviceName(m_captureDev);
    Utils::splitIntoTokens(captureDeviceName, '|', cards);

    LOG(INFO) << "Looking for the following cards:";
    for (const auto card : cards) {
        LOG(INFO) << card;
    }

    m_implData->numVideoDevs = driverSubPaths.size();
    m_implData->videoDevs = new VideoDev[m_implData->numVideoDevs];

    for (unsigned int i = 0; i < m_implData->numVideoDevs; i++) {
        devName = driverPaths.at(i).c_str();
        subDevName = driverSubPaths.at(i).c_str();
        cardName = cards.at(i).c_str();
        dev = &m_implData->videoDevs[i];

        LOG(INFO) << "device: " << devName << "\tsubdevice: " << subDevName;

        /* Open V4L2 device */
        if (stat(devName, &st) == -1) {
            LOG(WARNING) << "Cannot identify " << devName << "errno: " << errno
                         << "error: " << strerror(errno);
            return Status::GENERIC_ERROR;
        }

        if (!S_ISCHR(st.st_mode)) {
            LOG(WARNING) << devName << " is not a valid device";
            return Status::GENERIC_ERROR;
        }

        dev->fd = ::open(devName, O_RDWR | O_NONBLOCK, 0);
        if (dev->fd == -1) {
            LOG(WARNING) << "Cannot open " << devName << "errno: " << errno
                         << "error: " << strerror(errno);
            return Status::GENERIC_ERROR;
        }

        if (xioctl(dev->fd, VIDIOC_QUERYCAP, &cap) == -1) {
            LOG(WARNING) << devName << " VIDIOC_QUERYCAP error";
            return Status::GENERIC_ERROR;
        }

        if (strcmp((char *)cap.card, cardName)) {
            LOG(WARNING) << "CAPTURE Device " << cap.card;
            LOG(WARNING) << "Read " << cardName;
            return Status::GENERIC_ERROR;
        }

        if (!(cap.capabilities &
              (V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_VIDEO_CAPTURE_MPLANE))) {
            LOG(WARNING) << devName << " is not a video capture device";
            return Status::GENERIC_ERROR;
        }

        if (cap.capabilities & V4L2_CAP_VIDEO_CAPTURE_MPLANE) {
            dev->videoBuffersType = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
        } else {
            dev->videoBuffersType = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        }

        if (!(cap.capabilities & V4L2_CAP_STREAMING)) {
            LOG(WARNING) << devName << " does not support streaming i/o";
            return Status::GENERIC_ERROR;
        }

        /* Open V4L2 subdevice */
        if (stat(subDevName, &st) == -1) {
            LOG(WARNING) << "Cannot identify " << subDevName
                         << " errno: " << errno
                         << " error: " << strerror(errno);
            return Status::GENERIC_ERROR;
        }

        if (!S_ISCHR(st.st_mode)) {
            LOG(WARNING) << subDevName << " is not a valid device";
            return Status::GENERIC_ERROR;
        }

        dev->sfd = ::open(subDevName, O_RDWR | O_NONBLOCK, 0);
        if (dev->sfd == -1) {
            LOG(WARNING) << "Cannot open " << subDevName << " errno: " << errno
                         << " error: " << strerror(errno);
            return Status::GENERIC_ERROR;
        }
    }

    return status;
 }

aditof::Status PulsatrixSensor::start() { 
    using namespace aditof;
    Status status = Status::OK;
    struct VideoDev *dev;
    struct v4l2_buffer buf;

    for (unsigned int i = 0; i < m_implData->numVideoDevs; i++) {
        dev = &m_implData->videoDevs[i];
        if (dev->started) {
            LOG(INFO) << "Device already started";
            return Status::BUSY;
        }
        LOG(INFO) << "Starting device " << i;

        for (unsigned int i = 0; i < dev->nVideoBuffers; i++) {
            CLEAR(buf);
            buf.type = dev->videoBuffersType;
            buf.memory = V4L2_MEMORY_MMAP;
            buf.index = i;
            buf.m.planes = dev->planes;
            buf.length = 1;

            if (xioctl(dev->fd, VIDIOC_QBUF, &buf) == -1) {
                LOG(WARNING)
                    << "mmap error "
                    << "errno: " << errno << " error: " << strerror(errno);
                return Status::GENERIC_ERROR;
            }
        }

        if (xioctl(dev->fd, VIDIOC_STREAMON, &dev->videoBuffersType) != 0) {
            LOG(WARNING) << "VIDIOC_STREAMON error "
                         << "errno: " << errno << " error: " << strerror(errno);
            return Status::GENERIC_ERROR;
        }

        dev->started = true;
    }

    return status;
 }

aditof::Status PulsatrixSensor::stop() { 
    using namespace aditof;
    Status status = Status::OK;
    struct VideoDev *dev;

    for (unsigned int i = 0; i < m_implData->numVideoDevs; i++) {
        dev = &m_implData->videoDevs[i];

        if (!dev->started) {
            LOG(INFO) << "Device " << i << " already stopped";
            return Status::BUSY;
        }
        LOG(INFO) << "Stopping device";

        if (xioctl(dev->fd, VIDIOC_STREAMOFF, &dev->videoBuffersType) != 0) {
            LOG(WARNING) << "VIDIOC_STREAMOFF error "
                         << "errno: " << errno << " error: " << strerror(errno);
            return Status::GENERIC_ERROR;
        }

        dev->started = false;
    }
    return status;
 }

aditof::Status PulsatrixSensor::getAvailableFrameTypes(
    std::vector<aditof::DepthSensorFrameType> &types) {

    types =
        availableFrameTypes; //TBD shall we copy / move vector instead of assign
    return aditof::Status::OK;
}

aditof::Status PulsatrixSensor::setModeByIndex(uint8_t modeIndex) {
    using namespace aditof;
    struct VideoDev *dev = &m_implData->videoDevs[0];
    Status status = Status::OK;

    static struct v4l2_control ctrl;

    memset(&ctrl, 0, sizeof(ctrl));

    ctrl.id = CTRL_SET_MODE;
    ctrl.value = modeIndex;

    if (xioctl(dev->sfd, VIDIOC_S_CTRL, &ctrl) == -1) {
        LOG(WARNING) << "Setting Mode error "
                     << "errno: " << errno << " error: " << strerror(errno);
        status = Status::GENERIC_ERROR;
    }

    return status;
}

aditof::Status PulsatrixSensor::setMode(const std::string &mode) {
    uint8_t modeIndex;
    aditof::Status status = aditof::Status::OK;
    LOG(INFO) << "Setting camera mode to " << mode;
    //get mode index by name
    status = convertCameraMode(mode, modeIndex);
    if (status != aditof::Status::OK) {
        return status;
    }
    //get register value by mode index - nothing to do, the value corresponds to the index
    //set register / control
    status = setModeByIndex(modeIndex);
    if (status != aditof::Status::OK) {
        return status;
    }
    return aditof::Status::OK;
}

aditof::Status
PulsatrixSensor::setFrameType(const aditof::DepthSensorFrameType &type) {

    using namespace aditof;
    Status status = Status::OK;
    struct VideoDev *dev;

    struct v4l2_requestbuffers req;
    struct v4l2_format fmt;
    struct v4l2_buffer buf;
    size_t length, offset;
    size_t pix_fallout, pix_drv;

    status = setMode(type.type);
    if (status != aditof::Status::OK) {
        LOG(INFO) << "Failed to set camera mode";
        return status;
    }

    //We have two resolution domains. First is driver one which is defined in adsd3100_sensor.h
    //The second is the depthcompute input domain defined in mode_info.cpp
    //The total number of pixels between these two should be equal
    //Here compute the number of frames that should be requested to the driver based on the required number of pixels from g_modeInfoData[]
    pix_fallout = ModeInfo::getInstance()->getModeInfo(type.type).width *
                  ModeInfo::getInstance()->getModeInfo(type.type).height *
                  ModeInfo::getInstance()->getModeInfo(type.type).subframes;
    pix_drv = type.width * type.height;
    m_capturesPerFrame = pix_fallout / pix_drv;

    for (unsigned int i = 0; i < m_implData->numVideoDevs; i++) {
        dev = &m_implData->videoDevs[i];
        if (type.type != m_implData->frameType.type) {
            for (unsigned int i = 0; i < dev->nVideoBuffers; i++) {
                if (munmap(dev->videoBuffers[i].start,
                           dev->videoBuffers[i].length) == -1) {
                    LOG(WARNING)
                        << "munmap error "
                        << "errno: " << errno << " error: " << strerror(errno);
                    return Status::GENERIC_ERROR;
                }
            }
            free(dev->videoBuffers);
            dev->nVideoBuffers = 0;
            CLEAR(req);
            req.count = 0;
            req.type = dev->videoBuffersType;
            req.memory = V4L2_MEMORY_MMAP;

            if (xioctl(dev->fd, VIDIOC_REQBUFS, &req) == -1) {
                LOG(WARNING)
                    << "VIDIOC_REQBUFS error "
                    << "errno: " << errno << " error: " << strerror(errno);
                return Status::GENERIC_ERROR;
            }
        } else if (dev->nVideoBuffers) {
            return status;
        }

        /* Set the frame format in the driver */
        CLEAR(fmt);
        fmt.type = dev->videoBuffersType;
        fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_SBGGR8;
        fmt.fmt.pix.width = type.width;
        fmt.fmt.pix.height = type.height;

        if (xioctl(dev->fd, VIDIOC_S_FMT, &fmt) == -1) {
            LOG(WARNING) << "Setting Pixel Format error, errno: " << errno
                         << " error: " << strerror(errno);
            return Status::GENERIC_ERROR;
        }

        /* Allocate the video buffers in the driver */
        CLEAR(req);
        req.count = m_capturesPerFrame + EXTRA_BUFFERS_COUNT;
        req.type = dev->videoBuffersType;
        req.memory = V4L2_MEMORY_MMAP;

        if (xioctl(dev->fd, VIDIOC_REQBUFS, &req) == -1) {
            LOG(WARNING) << "VIDIOC_REQBUFS error "
                         << "errno: " << errno << " error: " << strerror(errno);
            return Status::GENERIC_ERROR;
        }

        dev->videoBuffers =
            (buffer *)calloc(req.count, sizeof(*dev->videoBuffers));
        if (!dev->videoBuffers) {
            LOG(WARNING) << "Failed to allocate video m_implData->videoBuffers";
            return Status::GENERIC_ERROR;
        }

        for (dev->nVideoBuffers = 0; dev->nVideoBuffers < req.count;
             dev->nVideoBuffers++) {
            CLEAR(buf);
            buf.type = dev->videoBuffersType;
            buf.memory = V4L2_MEMORY_MMAP;
            buf.index = dev->nVideoBuffers;
            buf.m.planes = dev->planes;
            buf.length = 1;

            if (xioctl(dev->fd, VIDIOC_QUERYBUF, &buf) == -1) {
                LOG(WARNING)
                    << "VIDIOC_QUERYBUF error "
                    << "errno: " << errno << " error: " << strerror(errno);
                return Status::GENERIC_ERROR;
            }

            if (dev->videoBuffersType == V4L2_BUF_TYPE_VIDEO_CAPTURE) {
                length = buf.length;
                offset = buf.m.offset;
            } else {
                length = buf.m.planes[0].length;
                offset = buf.m.planes[0].m.mem_offset;
            }

            dev->videoBuffers[dev->nVideoBuffers].start =
                mmap(NULL, length, PROT_READ | PROT_WRITE, MAP_SHARED, dev->fd,
                     offset);

            if (dev->videoBuffers[dev->nVideoBuffers].start == MAP_FAILED) {
                LOG(WARNING)
                    << "mmap error "
                    << "errno: " << errno << " error: " << strerror(errno);
                return Status::GENERIC_ERROR;
            }

            dev->videoBuffers[dev->nVideoBuffers].length = length;
        }
    }

    m_implData->frameType = type;

    return status;
}

aditof::Status PulsatrixSensor::program(const uint8_t *firmware, size_t size) {

    return aditof::Status::OK;
}

aditof::Status PulsatrixSensor::getFrame(uint16_t *buffer) {

    using namespace aditof;
    struct v4l2_buffer buf[MAX_SUBFRAMES_COUNT];
    struct VideoDev *dev;
    Status status;
    unsigned int buf_data_len;
    uint8_t *pdata;
    dev = &m_implData->videoDevs[0];
    m_capturesPerFrame = 1;
    for (int idx = 0; idx < m_capturesPerFrame; idx++) {
        status = waitForBufferPrivate(dev);
        if (status != Status::OK) {
            return status;
        }

        status = dequeueInternalBufferPrivate(buf[idx], dev);
        if (status != Status::OK) {
            return status;
        }

        status = getInternalBufferPrivate(&pdata, buf_data_len, buf[idx], dev);
        if (status != Status::OK) {
            return status;
        }

        memcpy(buffer, pdata, buf_data_len);

        status = enqueueInternalBufferPrivate(buf[idx], dev);
        if (status != Status::OK) {
            return status;
        }
    }

    return status;
}

aditof::Status PulsatrixSensor::readRegisters(const uint16_t *address,
                                              uint16_t *data, size_t length,
                                              bool burst /*= true*/) {

    return aditof::Status::OK;
}

aditof::Status PulsatrixSensor::writeRegisters(const uint16_t *address,
                                               const uint16_t *data,
                                               size_t length,
                                               bool burst /*= true*/) {
    return aditof::Status::OK;
}

aditof::Status
PulsatrixSensor::getDetails(aditof::SensorDetails &details) const {

    return aditof::Status::OK;
}

aditof::Status PulsatrixSensor::getHandle(void **handle) {

    return aditof::Status::OK;
}

aditof::Status PulsatrixSensor::getName(std::string &name) const {
    name = m_sensorName;
    
    return aditof::Status::OK;
}

aditof::Status PulsatrixSensor::pulsatrix_read_cmd(uint16_t cmd, uint16_t *data){
    using namespace aditof;
    struct VideoDev *dev = &m_implData->videoDevs[0];
    Status status = Status::OK;

    static struct v4l2_ext_control extCtrl;
    static struct v4l2_ext_controls extCtrls;
    static uint8_t buf[PULSATRIX_CTRL_PACKET_SIZE];

    extCtrl.size = PULSATRIX_CTRL_PACKET_SIZE;
    extCtrl.id = V4L2_CID_AD_DEV_CHIP_CONFIG;
    memset(&extCtrls, 0, sizeof(struct v4l2_ext_controls));
    extCtrls.controls = &extCtrl;
    extCtrls.count = 1;

    buf[0] = 1;
    buf[1] = 0;
    buf[2] = 2;
    buf[3] = uint8_t(cmd >> 8);
    buf[4] = uint8_t(cmd & 0xFF);
    extCtrl.p_u8 = buf;

    if (xioctl(dev->sfd, VIDIOC_S_EXT_CTRLS, &extCtrls) == -1) {
        LOG(WARNING) << "Reading Pulsatrix error "
                         << "errno: " << errno << " error: " << strerror(errno);
            return Status::GENERIC_ERROR;
        }

    buf[0] = 0;
    buf[1] = 0;
    buf[2] = 2;

    extCtrl.p_u8 = buf;

    if (xioctl(dev->sfd, VIDIOC_S_EXT_CTRLS, &extCtrls) == -1){ 
        LOG(WARNING) << "Reading Pulsatrix error "
                         << "errno: " << errno << " error: " << strerror(errno);
        return Status::GENERIC_ERROR;
    }

    if (xioctl(dev->sfd, VIDIOC_G_EXT_CTRLS, &extCtrls) == -1){ 
        LOG(WARNING) << "Reading Pulsatrix error "
                         << "errno: " << errno << " error: " << strerror(errno);
        return Status::GENERIC_ERROR;
    }

    memcpy(data, extCtrl.p_u8 + 3, sizeof(uint16_t));

    return status;
}

aditof::Status PulsatrixSensor::pulsatrix_write_cmd(uint16_t cmd, uint16_t data) {
        using namespace aditof;
    struct VideoDev *dev = &m_implData->videoDevs[0];
    Status status = Status::OK;

    static struct v4l2_ext_control extCtrl;
    static struct v4l2_ext_controls extCtrls;
    static uint8_t buf[PULSATRIX_CTRL_PACKET_SIZE];

    extCtrl.size = PULSATRIX_CTRL_PACKET_SIZE;
    extCtrl.id = V4L2_CID_AD_DEV_CHIP_CONFIG;
    memset(&extCtrls, 0, sizeof(struct v4l2_ext_controls));
    extCtrls.controls = &extCtrl;
    extCtrls.count = 1;

    buf[0] = 1;
    buf[1] = 0;
    buf[2] = 4;
    buf[3] = uint8_t(cmd >> 8);
    buf[4] = uint8_t(cmd & 0xFF);
    buf[5] = uint8_t(data >> 8);
    buf[6] = uint8_t(data & 0xFF);
    extCtrl.p_u8 = buf;

    if (xioctl(dev->sfd, VIDIOC_S_EXT_CTRLS, &extCtrls) == -1) {
        LOG(WARNING) << "Reading Pulsatrix error "
                         << "errno: " << errno << " error: " << strerror(errno);
            return Status::GENERIC_ERROR;
        }
    return status;
}

// TO DO: Verify mechanism for read/write burst

aditof::Status PulsatrixSensor::pulsatrix_read_payload_cmd(uint32_t cmd, uint8_t* readback_data, uint16_t payload_len) {
    using namespace aditof;
    struct VideoDev *dev = &m_implData->videoDevs[0];
    Status status = Status::OK;

    static struct v4l2_ext_control extCtrl;
    static struct v4l2_ext_controls extCtrls;
    static uint8_t buf[PULSATRIX_CTRL_PACKET_SIZE];

    extCtrl.size = PULSATRIX_CTRL_PACKET_SIZE;
    extCtrl.id = V4L2_CID_AD_DEV_CHIP_CONFIG;
    memset(&extCtrls, 0, sizeof(struct v4l2_ext_controls));
    extCtrls.controls = &extCtrl;
    extCtrls.count = 1;

    buf[0] = 0xAD;
    buf[1] = uint8_t(payload_len >> 8);
    buf[2] = uint8_t(payload_len & 0xFF);
    buf[3] = 0;
    buf[4] = uint8_t(cmd >> 24);
    buf[5] = uint8_t((cmd >> 16) & 0xFF);
    buf[6] = uint8_t((cmd >> 8) & 0xFF);
    buf[7] = uint8_t(cmd & 0xFF);

    extCtrl.p_u8 = buf;

    if (xioctl(dev->sfd, VIDIOC_S_EXT_CTRLS, &extCtrls) == -1) {
        LOG(WARNING) << "Reading Pulsatrix error "
                         << "errno: " << errno << " error: " << strerror(errno);
            return Status::GENERIC_ERROR;
        }

    if (xioctl(dev->sfd, VIDIOC_G_EXT_CTRLS, &extCtrls) == -1) {
		LOG(WARNING) << "Failed to get ctrl with id " << extCtrl.id;
			return Status::GENERIC_ERROR;
	}

    memcpy(readback_data, extCtrl.p_u8 + 16, payload_len);

    return status;
}

aditof::Status PulsatrixSensor::pulsatrix_write_payload_cmd(uint32_t cmd, uint8_t* payload, uint16_t payload_len) {
    using namespace aditof;
    struct VideoDev *dev = &m_implData->videoDevs[0];
    Status status = Status::OK;

    static struct v4l2_ext_control extCtrl;
    static struct v4l2_ext_controls extCtrls;
    static uint8_t buf[PULSATRIX_CTRL_PACKET_SIZE];

    extCtrl.size = PULSATRIX_CTRL_PACKET_SIZE;
    extCtrl.id = V4L2_CID_AD_DEV_CHIP_CONFIG;
    memset(&extCtrls, 0, sizeof(struct v4l2_ext_controls));
    extCtrls.controls = &extCtrl;
    extCtrls.count = 1;

    buf[0] = 0xAD;
    buf[1] = uint8_t(payload_len >> 8);
    buf[2] = uint8_t(payload_len & 0xFF);
    buf[3] = 0;
    buf[4] = uint8_t(cmd >> 24);
    buf[5] = uint8_t((cmd >> 16) & 0xFF);
    buf[6] = uint8_t((cmd >> 8) & 0xFF);
    buf[7] = uint8_t(cmd & 0xFF);
    memcpy(buf + 8, payload, payload_len);

    if (xioctl(dev->sfd, VIDIOC_S_EXT_CTRLS, &extCtrls) == -1) {
        LOG(WARNING) << "Reading Pulsatrix error "
                         << "errno: " << errno << " error: " << strerror(errno);
            return Status::GENERIC_ERROR;
        }

    return status;
}

aditof::Status PulsatrixSensor::pulsatrix_write_payload(uint8_t* payload, uint16_t payload_len) {
    using namespace aditof;
    struct VideoDev *dev = &m_implData->videoDevs[0];
    Status status = Status::OK;

    static struct v4l2_ext_control extCtrl;
    static struct v4l2_ext_controls extCtrls;
    static uint8_t buf[PULSATRIX_CTRL_PACKET_SIZE];

    extCtrl.size = PULSATRIX_CTRL_PACKET_SIZE;
    extCtrl.id = V4L2_CID_AD_DEV_CHIP_CONFIG;
    memset(&extCtrls, 0, sizeof(struct v4l2_ext_controls));
    extCtrls.controls = &extCtrl;
    extCtrls.count = 1;

    buf[0] = 1;
    buf[1] = uint8_t(payload_len >> 8);
    buf[2] = uint8_t(payload_len & 0xFF);

    memcpy(buf + 3, payload, payload_len);
    extCtrl.p_u8 = buf;

    if (xioctl(dev->sfd, VIDIOC_S_EXT_CTRLS, &extCtrls) == -1) {
        LOG(WARNING) << "Reading Pulsatrix error "
                         << "errno: " << errno << " error: " << strerror(errno);
            return Status::GENERIC_ERROR;
        }

    return status;
}

aditof::Status PulsatrixSensor::waitForBufferPrivate(struct VideoDev *dev) {
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
PulsatrixSensor::dequeueInternalBufferPrivate(struct v4l2_buffer &buf,
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

aditof::Status PulsatrixSensor::getInternalBufferPrivate(
    uint8_t **buffer, uint32_t &buf_data_len, const struct v4l2_buffer &buf,
    struct VideoDev *dev) {
    if (dev == nullptr)
        dev = &m_implData->videoDevs[0];

    *buffer = static_cast<uint8_t *>(dev->videoBuffers[buf.index].start);
    buf_data_len = buf.bytesused;

    return aditof::Status::OK;
}

aditof::Status
PulsatrixSensor::enqueueInternalBufferPrivate(struct v4l2_buffer &buf,
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

aditof::Status PulsatrixSensor::getDeviceFileDescriptor(int &fileDescriptor) {
    using namespace aditof;
    struct VideoDev *dev = &m_implData->videoDevs[0];

    if (dev->fd != -1) {
        fileDescriptor = dev->fd;
        return Status::OK;
    }

    return Status::INVALID_ARGUMENT;
}

aditof::Status PulsatrixSensor::waitForBuffer() {

    return waitForBufferPrivate();
}

aditof::Status PulsatrixSensor::dequeueInternalBuffer(struct v4l2_buffer &buf) {

    return dequeueInternalBufferPrivate(buf);
}

aditof::Status
PulsatrixSensor::getInternalBuffer(uint8_t **buffer, uint32_t &buf_data_len,
                                   const struct v4l2_buffer &buf) {

    return getInternalBufferPrivate(buffer, buf_data_len, buf);
}

aditof::Status PulsatrixSensor::enqueueInternalBuffer(struct v4l2_buffer &buf) {

    return enqueueInternalBufferPrivate(buf);
}

aditof::Status PulsatrixSensor::writeConfigBlock(const uint32_t offset) {

    return aditof::Status::OK;
}
