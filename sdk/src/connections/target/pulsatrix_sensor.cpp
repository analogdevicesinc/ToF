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
      m_captureDev(captureDev), m_implData(new PulsatrixSensor::ImplData) {}

PulsatrixSensor::~PulsatrixSensor() {}

aditof::Status PulsatrixSensor::open() { return aditof::Status::OK; }

aditof::Status PulsatrixSensor::start() { return aditof::Status::OK; }

aditof::Status PulsatrixSensor::stop() { return aditof::Status::OK; }

aditof::Status PulsatrixSensor::getAvailableFrameTypes(
    std::vector<aditof::DepthSensorFrameType> &types) {

    return aditof::Status::OK;
}

aditof::Status PulsatrixSensor::setModeByIndex(uint8_t modeIndex) {

    return aditof::Status::OK;
}

aditof::Status PulsatrixSensor::setMode(const std::string &mode) {

    return aditof::Status::OK;
}

aditof::Status
PulsatrixSensor::setFrameType(const aditof::DepthSensorFrameType &type) {

    return aditof::Status::OK;
}

aditof::Status PulsatrixSensor::program(const uint8_t *firmware, size_t size) {

    return aditof::Status::OK;
}

aditof::Status PulsatrixSensor::getFrame(uint16_t *buffer) {

    return aditof::Status::OK;
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

aditof::Status PulsatrixSensor::waitForBufferPrivate(struct VideoDev *dev) {

    return aditof ::Status::OK;
}

aditof::Status
PulsatrixSensor::dequeueInternalBufferPrivate(struct v4l2_buffer &buf,
                                              struct VideoDev *dev) {
    return aditof::Status::OK;
}

aditof::Status PulsatrixSensor::getInternalBufferPrivate(
    uint8_t **buffer, uint32_t &buf_data_len, const struct v4l2_buffer &buf,
    struct VideoDev *dev) {

    return aditof::Status::OK;
}

aditof::Status
PulsatrixSensor::enqueueInternalBufferPrivate(struct v4l2_buffer &buf,
                                              struct VideoDev *dev) {
    return aditof::Status::OK;
}

aditof::Status PulsatrixSensor::getDeviceFileDescriptor(int &fileDescriptor) {
    return aditof::Status::UNAVAILABLE;
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
