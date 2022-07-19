/********************************************************************************/
/*                                                                              */
/* Copyright (c) 2020 Analog Devices, Inc. All Rights Reserved.                 */
/* This software is proprietary to Analog Devices, Inc. and its licensors.      */
/*                                                                              */
/********************************************************************************/
#include "adsd3500_sensor.h"
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
#define CTRL_AB_AVG (0x9819e5)
#define CTRL_DEPTH_EN (0x9819e6)
#define CTRL_PHASE_DEPTH_BITS (0x9819e2)
#define CTRL_AB_BITS (0x9819e3)
#define CTRL_CONFIDENCE_BITS (0x9819e4)
#define ADSD3500_CTRL_PACKET_SIZE 4099
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
struct Adsd3500Sensor::ImplData {
    uint8_t numVideoDevs;
    struct VideoDev *videoDevs;
    aditof::DepthSensorFrameType frameType;
    std::unordered_map<std::string, __u32> controlsCommands;
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

Adsd3500Sensor::Adsd3500Sensor(const std::string &driverPath,
                                 const std::string &driverSubPath,
                                 const std::string &captureDev)
    : m_driverPath(driverPath), m_driverSubPath(driverSubPath),
      m_captureDev(captureDev), m_implData(new Adsd3500Sensor::ImplData) {
          m_sensorName = "adsd3500";

    // Define the controls that this sensor has available
    m_controls.emplace("abAveraging", "0");
    m_controls.emplace("depthEnable", "0");
    m_controls.emplace("phaseDepthBits", "0");
    m_controls.emplace("abBits", "0");
    m_controls.emplace("confidenceBits", "0");
    m_controls.emplace("fps","0");

    // Define the commands that correspond to the sensor controls
    m_implData->controlsCommands["abAveraging"] = 0x9819e5;
    m_implData->controlsCommands["depthEnable"] = 0x9819e6;
    m_implData->controlsCommands["phaseDepthBits"] = 0x9819e2;
    m_implData->controlsCommands["abBits"] = 0x9819e3;
    m_implData->controlsCommands["confidenceBits"] = 0x9819e4;
}

Adsd3500Sensor::~Adsd3500Sensor() {
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

aditof::Status Adsd3500Sensor::open() { 
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
#if 1 //Don't open the video device for UVC context. It is opened in uvc-app/lib/v4l2.c
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
#endif
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

aditof::Status Adsd3500Sensor::start() { 
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

aditof::Status Adsd3500Sensor::stop() { 
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

aditof::Status Adsd3500Sensor::getAvailableFrameTypes(
    std::vector<aditof::DepthSensorFrameType> &types) {

    types =
        availableFrameTypes; //TBD shall we copy / move vector instead of assign
    return aditof::Status::OK;
}

aditof::Status Adsd3500Sensor::setModeByIndex(uint8_t modeIndex) {
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

aditof::Status Adsd3500Sensor::setMode(const std::string &mode) {
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
Adsd3500Sensor::setFrameType(const aditof::DepthSensorFrameType &type) {

    using namespace aditof;
    Status status = Status::OK;
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

    status = open();
    if (status != aditof::Status::OK) {
        LOG(INFO) << "Failed to open sensor!";
        return status;
    }

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

    dev = &m_implData->videoDevs[0];

#if 1 // Don't request buffers & set fromat for UVC context. It is already done in uvc-app/lib/v4l2.c
    m_capturesPerFrame = 1;

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

        __u32 pixelFormat = 0;
        if (type.type == "qmp") {
            pixelFormat = V4L2_PIX_FMT_SBGGR8;
        } else if (type.type == "mp") {
            pixelFormat = V4L2_PIX_FMT_SBGGR12;
        } else {
            LOG(ERROR) << "frame type: " << type.type << " " << "is unhandled";
            return Status::GENERIC_ERROR;
        }

        /* Set the frame format in the driver */
        CLEAR(fmt);
        fmt.type = dev->videoBuffersType;
        fmt.fmt.pix.pixelformat = pixelFormat;
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
#endif

    m_implData->frameType = type;

    return status;
}

aditof::Status Adsd3500Sensor::program(const uint8_t *firmware, size_t size) {

    return aditof::Status::OK;
}

aditof::Status Adsd3500Sensor::getFrame(uint16_t *buffer) {

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

aditof::Status Adsd3500Sensor::readRegisters(const uint16_t *address,
                                              uint16_t *data, size_t length,
                                              bool burst /*= true*/) {

    return aditof::Status::OK;
}

aditof::Status Adsd3500Sensor::writeRegisters(const uint16_t *address,
                                               const uint16_t *data,
                                               size_t length,
                                               bool burst /*= true*/) {
    return aditof::Status::OK;
}

aditof::Status Adsd3500Sensor::getAvailableControls(std::vector<std::string> &controls) const
{
    controls.clear();
    controls.reserve(m_controls.size());
    for (const auto &item : m_controls) {
        controls.emplace_back(item.first);
    }

    return aditof::Status::OK;
}

aditof::Status Adsd3500Sensor::setControl(const std::string &control,
                               const std::string &value)
{
    using namespace aditof;
    Status status = Status::OK;

    if (m_controls.count(control) == 0) {
        LOG(WARNING) << "Unsupported control";
        return Status::INVALID_ARGUMENT;
    }

    struct VideoDev *dev = &m_implData->videoDevs[0];

    if(control!= "fps") {
        // Send the command that sets the control value
        struct v4l2_control ctrl;
        memset(&ctrl, 0, sizeof(ctrl));

        ctrl.id = m_implData->controlsCommands[control];
        ctrl.value = std::stoi(value);

        if (xioctl(dev->sfd, VIDIOC_S_CTRL, &ctrl) == -1) {
            LOG(WARNING) << "Failed to set control: " << control << " "
                         << "errno: " << errno << " error: " << strerror(errno);
            status = Status::GENERIC_ERROR;
        }
    } else {
        struct v4l2_streamparm fpsControl;
        memset(&fpsControl, 0, sizeof(struct v4l2_streamparm));
    
        fpsControl.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        fpsControl.parm.capture.timeperframe.numerator = 1;
        fpsControl.parm.capture.timeperframe.denominator = std::stoi(value);
    
        if (xioctl(dev->fd, VIDIOC_S_PARM, &fpsControl) == -1) {
            LOG(WARNING) << "Failed to set control: " << control << " "
                         << "errno: " << errno << " error: " << strerror(errno);
            status = Status::GENERIC_ERROR;
        }
    }

    return status;
}

aditof::Status Adsd3500Sensor::getControl(const std::string &control,
                               std::string &value) const
{
    using namespace aditof;

    if (m_controls.count(control) > 0) {
        // Send the command that reads the control value
        struct v4l2_control ctrl;
        memset(&ctrl, 0, sizeof(ctrl));

        ctrl.id = m_implData->controlsCommands[control];

	struct VideoDev *dev = &m_implData->videoDevs[0];

        if (xioctl(dev->sfd, VIDIOC_G_CTRL, &ctrl) == -1) {
            LOG(WARNING) << "Failed to get control: " << control << " "
                     << "errno: " << errno << " error: " << strerror(errno);
            return Status::GENERIC_ERROR;
        }
        value = std::to_string(ctrl.value);
    } else {
        LOG(WARNING) << "Unsupported control";
        return Status::INVALID_ARGUMENT;
    }

    return aditof::Status::OK;
}

aditof::Status
Adsd3500Sensor::getDetails(aditof::SensorDetails &details) const {

    return aditof::Status::OK;
}

aditof::Status Adsd3500Sensor::getHandle(void **handle) {

    return aditof::Status::OK;
}

aditof::Status Adsd3500Sensor::getName(std::string &name) const {
    name = m_sensorName;
    
    return aditof::Status::OK;
}

aditof::Status Adsd3500Sensor::adsd3500_read_cmd(uint16_t cmd, uint16_t *data){
    using namespace aditof;
    struct VideoDev *dev = &m_implData->videoDevs[0];
    Status status = Status::OK;

    static struct v4l2_ext_control extCtrl;
    static struct v4l2_ext_controls extCtrls;
    static uint8_t buf[ADSD3500_CTRL_PACKET_SIZE];

    extCtrl.size = ADSD3500_CTRL_PACKET_SIZE;
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
        LOG(WARNING) << "Reading Adsd3500 error "
                         << "errno: " << errno << " error: " << strerror(errno);
            return Status::GENERIC_ERROR;
        }

    buf[0] = 0;
    buf[1] = 0;
    buf[2] = 2;

    extCtrl.p_u8 = buf;

    if (xioctl(dev->sfd, VIDIOC_S_EXT_CTRLS, &extCtrls) == -1){ 
        LOG(WARNING) << "Reading Adsd3500 error "
                         << "errno: " << errno << " error: " << strerror(errno);
        return Status::GENERIC_ERROR;
    }

    if (xioctl(dev->sfd, VIDIOC_G_EXT_CTRLS, &extCtrls) == -1){ 
        LOG(WARNING) << "Reading Adsd3500 error "
                         << "errno: " << errno << " error: " << strerror(errno);
        return Status::GENERIC_ERROR;
    }

    *data = (uint16_t)(extCtrl.p_u8[3] << 8) + (uint16_t)(extCtrl.p_u8[4]);

    return status;
}

aditof::Status Adsd3500Sensor::adsd3500_write_cmd(uint16_t cmd, uint16_t data) {
        using namespace aditof;
    struct VideoDev *dev = &m_implData->videoDevs[0];
    Status status = Status::OK;

    static struct v4l2_ext_control extCtrl;
    static struct v4l2_ext_controls extCtrls;
    static uint8_t buf[ADSD3500_CTRL_PACKET_SIZE];

    extCtrl.size = ADSD3500_CTRL_PACKET_SIZE;
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
        LOG(WARNING) << "Reading Adsd3500 error "
                         << "errno: " << errno << " error: " << strerror(errno);
            return Status::GENERIC_ERROR;
        }
    return status;
}

// TO DO: Verify mechanism for read/write burst

aditof::Status Adsd3500Sensor::adsd3500_read_payload_cmd(uint32_t cmd, uint8_t* readback_data, uint16_t payload_len) {
    using namespace aditof;
    struct VideoDev *dev = &m_implData->videoDevs[0];
    Status status = Status::OK;

    //switch to burst mode
    uint32_t switchCmd = 0x0019;
    uint16_t switchPayload = 0x0000;

    status = adsd3500_write_cmd(switchCmd, switchPayload);
    if(status != Status::OK){
        LOG(INFO) << "Failed to switch to burst mode!";
        return status;
    }

    static struct v4l2_ext_control extCtrl;
    static struct v4l2_ext_controls extCtrls;
    static uint8_t buf[ADSD3500_CTRL_PACKET_SIZE];
    memset(buf, 0, ADSD3500_CTRL_PACKET_SIZE * sizeof(uint8_t));

    extCtrl.size = ADSD3500_CTRL_PACKET_SIZE;
    extCtrl.id = V4L2_CID_AD_DEV_CHIP_CONFIG;

    memset(&extCtrls, 0, sizeof(struct v4l2_ext_controls));
    extCtrls.controls = &extCtrl;
    extCtrls.count = 1;

    buf[0] = 0x01;
    buf[1] = 0x00;
    buf[2] = 0x10;

    buf[3] = 0xAD;
    buf[6] = uint8_t(cmd & 0xFF);   

    uint32_t checksum = 0;
    for (int i = 0; i < 7; i++){
        checksum += buf[i+4];
     }
    memcpy(buf + 11, &checksum, 4);
    memcpy(buf + 15, readback_data, 4);
    extCtrl.p_u8 = buf;

    if (xioctl(dev->sfd, VIDIOC_S_EXT_CTRLS, &extCtrls) == -1) {
        LOG(WARNING) << "Reading Adsd3500 error "
                         << "errno: " << errno << " error: " << strerror(errno);
            return Status::GENERIC_ERROR;
        }

    if(cmd == 0x13){
        usleep(1000);
    }

    memset(&extCtrls, 0, sizeof(struct v4l2_ext_controls));
    extCtrls.controls = &extCtrl;
    extCtrls.count = 1;

    buf[0] = 0x00;
    buf[1] = uint8_t(payload_len >> 8);
    buf[2] = uint8_t(payload_len & 0xFF);

    extCtrl.p_u8 = buf;

    if (xioctl(dev->sfd, VIDIOC_S_EXT_CTRLS, &extCtrls) == -1) {
        LOG(WARNING) << "Reading Adsd3500 error "
                         << "errno: " << errno << " error: " << strerror(errno);
            return Status::GENERIC_ERROR;
        }

    if (xioctl(dev->sfd, VIDIOC_G_EXT_CTRLS, &extCtrls) == -1) {
		LOG(WARNING) << "Failed to get ctrl with id " << extCtrl.id;
			return Status::GENERIC_ERROR;
	}

    memcpy(readback_data, extCtrl.p_u8 + 3, payload_len);

    //If we use the read ccb command we need to keep adsd3500 in burst mode
    if(cmd == 0x13){
        return status;
    }

    //switch to standard mode
    memset(&extCtrls, 0, sizeof(struct v4l2_ext_controls));
    extCtrls.controls = &extCtrl;
    extCtrls.count = 1;

    uint8_t switchBuf[] = {0x01, 0x00, 0x10, 0xAD, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00,
                           0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

    extCtrl.p_u8 = switchBuf;

    if (xioctl(dev->sfd, VIDIOC_S_EXT_CTRLS, &extCtrls) == -1) {
        LOG(WARNING) << "Switch Adsd3500 to standard mode error "
                         << "errno: " << errno << " error: " << strerror(errno);
            return Status::GENERIC_ERROR;
        }

    return status;
}

aditof::Status Adsd3500Sensor::adsd3500_read_payload(uint8_t* payload, uint16_t payload_len) {
    using namespace aditof;
    struct VideoDev *dev = &m_implData->videoDevs[0];
    Status status = Status::OK;

    static struct v4l2_ext_control extCtrl;
    static struct v4l2_ext_controls extCtrls;
    static uint8_t buf[ADSD3500_CTRL_PACKET_SIZE];
    memset(buf, 0, ADSD3500_CTRL_PACKET_SIZE * sizeof(uint8_t));

    extCtrl.size = ADSD3500_CTRL_PACKET_SIZE;
    extCtrl.id = V4L2_CID_AD_DEV_CHIP_CONFIG;

    memset(&extCtrls, 0, sizeof(struct v4l2_ext_controls));
    extCtrls.controls = &extCtrl;
    extCtrls.count = 1;

    buf[0] = 0x00;
    buf[1] = uint8_t(payload_len >> 8);
    buf[2] = uint8_t(payload_len & 0xFF);

    extCtrl.p_u8 = buf;

    usleep(30000);

    if (xioctl(dev->sfd, VIDIOC_S_EXT_CTRLS, &extCtrls) == -1) {
        LOG(WARNING) << "Reading Adsd3500 error "
                         << "errno: " << errno << " error: " << strerror(errno);
            return Status::GENERIC_ERROR;
    }

    if (xioctl(dev->sfd, VIDIOC_G_EXT_CTRLS, &extCtrls) == -1) {
		LOG(WARNING) << "Failed to get ctrl with id " << extCtrl.id;
			return Status::GENERIC_ERROR;
	}

    memcpy(payload, extCtrl.p_u8 + 3, payload_len);

    return status;
}

aditof::Status Adsd3500Sensor::adsd3500_write_payload_cmd(uint32_t cmd, uint8_t* payload, uint16_t payload_len) {
    using namespace aditof;
    struct VideoDev *dev = &m_implData->videoDevs[0];
    Status status = Status::OK;

    //switch to burst mode
    uint32_t switchCmd = 0x0019;
    uint16_t switchPayload = 0x0000;

    status = adsd3500_write_cmd(switchCmd, switchPayload);
    if(status != Status::OK){
        LOG(INFO) << "Failed to switch to burst mode!";
    }

    static struct v4l2_ext_control extCtrl;
    static struct v4l2_ext_controls extCtrls;
    static uint8_t buf[ADSD3500_CTRL_PACKET_SIZE];

    extCtrl.size = ADSD3500_CTRL_PACKET_SIZE;
    extCtrl.id = V4L2_CID_AD_DEV_CHIP_CONFIG;
    memset(&extCtrls, 0, sizeof(struct v4l2_ext_controls));
    extCtrls.controls = &extCtrl;
    extCtrls.count = 1;

    payload_len += 16;
    buf[0] = 0x01;
    buf[1] = uint8_t(payload_len >> 8);
    buf[2] = uint8_t(payload_len & 0xFF);

    payload_len -=16;
    buf[3] = 0xAD;
    buf[4] = uint8_t(payload_len >> 8);
    buf[5] = uint8_t(payload_len & 0xFF);
    buf[6] = uint8_t(cmd & 0xFF);

    uint32_t checksum = 0;
    for (int i = 0; i < 7; i++){
        checksum += buf[i+4];
    }
    memcpy(buf + 11, &checksum, 4);
    memcpy(buf + 15, payload, payload_len);
    extCtrl.p_u8 = buf;

    if (xioctl(dev->sfd, VIDIOC_S_EXT_CTRLS, &extCtrls) == -1) {
        LOG(WARNING) << "Writing Adsd3500 error "
                         << "errno: " << errno << " error: " << strerror(errno);
            return Status::GENERIC_ERROR;
        }

    //switch to standard mode
    memset(&extCtrls, 0, sizeof(struct v4l2_ext_controls));
    extCtrls.controls = &extCtrl;
    extCtrls.count = 1;

    uint8_t switchBuf[] = {0x01, 0x00, 0x10, 0xAD, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00,
                           0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

    extCtrl.p_u8 = switchBuf;

    if (xioctl(dev->sfd, VIDIOC_S_EXT_CTRLS, &extCtrls) == -1) {
        LOG(WARNING) << "Switch Adsd3500 to standard mode error "
                         << "errno: " << errno << " error: " << strerror(errno);
            return Status::GENERIC_ERROR;
        }

    return status;
}

aditof::Status Adsd3500Sensor::adsd3500_write_payload(uint8_t* payload, uint16_t payload_len) {
    using namespace aditof;
    struct VideoDev *dev = &m_implData->videoDevs[0];
    Status status = Status::OK;

    static struct v4l2_ext_control extCtrl;
    static struct v4l2_ext_controls extCtrls;
    static uint8_t buf[ADSD3500_CTRL_PACKET_SIZE];

    extCtrl.size = ADSD3500_CTRL_PACKET_SIZE;
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
        LOG(WARNING) << "Writing Adsd3500 error "
                         << "errno: " << errno << " error: " << strerror(errno);
            return Status::GENERIC_ERROR;
        }
    
    usleep(100000);

    return status;
}

aditof::Status Adsd3500Sensor::waitForBufferPrivate(struct VideoDev *dev) {
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
Adsd3500Sensor::dequeueInternalBufferPrivate(struct v4l2_buffer &buf,
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

aditof::Status Adsd3500Sensor::getInternalBufferPrivate(
    uint8_t **buffer, uint32_t &buf_data_len, const struct v4l2_buffer &buf,
    struct VideoDev *dev) {
    if (dev == nullptr)
        dev = &m_implData->videoDevs[0];

    *buffer = static_cast<uint8_t *>(dev->videoBuffers[buf.index].start);
    buf_data_len = buf.bytesused;

    return aditof::Status::OK;
}

aditof::Status
Adsd3500Sensor::enqueueInternalBufferPrivate(struct v4l2_buffer &buf,
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

aditof::Status Adsd3500Sensor::getDeviceFileDescriptor(int &fileDescriptor) {
    using namespace aditof;
    struct VideoDev *dev = &m_implData->videoDevs[0];

    if (dev->fd != -1) {
        fileDescriptor = dev->fd;
        return Status::OK;
    }

    return Status::INVALID_ARGUMENT;
}

aditof::Status Adsd3500Sensor::waitForBuffer() {

    return waitForBufferPrivate();
}

aditof::Status Adsd3500Sensor::dequeueInternalBuffer(struct v4l2_buffer &buf) {

    return dequeueInternalBufferPrivate(buf);
}

aditof::Status
Adsd3500Sensor::getInternalBuffer(uint8_t **buffer, uint32_t &buf_data_len,
                                   const struct v4l2_buffer &buf) {

    return getInternalBufferPrivate(buffer, buf_data_len, buf);
}

aditof::Status Adsd3500Sensor::enqueueInternalBuffer(struct v4l2_buffer &buf) {

    return enqueueInternalBufferPrivate(buf);
}

aditof::Status Adsd3500Sensor::writeConfigBlock(const uint32_t offset) {

    return aditof::Status::OK;
}
