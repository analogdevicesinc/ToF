/********************************************************************************/
/*                                                                              */
/* Copyright (c) 2020 Analog Devices, Inc. All Rights Reserved.                 */
/* This software is proprietary to Analog Devices, Inc. and its licensors.      */
/*                                                                              */
/********************************************************************************/
#include "adsd3100_sensor.h"
#include "aditof/frame_operations.h"
#include "utils.h"

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
#include "cameras/itof-camera/mode_info.h"

#define MAX_SUBFRAMES_COUNT 10 // maximum number of subframes that are used to create a full frame (maximum total_captures of all modes)
#define EXTRA_BUFFERS_COUNT 0  // how many extra buffers are sent to the driver in addition to the total_captures of a mode

#define CLEAR(x) memset(&(x), 0, sizeof(x))

#define V4L2_CID_AD_DEV_SET_CHIP_CONFIG 0xA00B00
#define V4L2_CID_AD_DEV_READ_REG 0xA00B01
#define CTRL_PACKET_SIZE 4096
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
struct Adsd3100Sensor::ImplData {
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

Adsd3100Sensor::Adsd3100Sensor(const std::string &driverPath,
                               const std::string &driverSubPath,
                               const std::string &captureDev)
    : m_driverPath(driverPath), m_driverSubPath(driverSubPath),
      m_captureDev(captureDev), m_implData(new Adsd3100Sensor::ImplData) {}

Adsd3100Sensor::~Adsd3100Sensor() {
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

aditof::Status Adsd3100Sensor::open() {
using namespace aditof;
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

aditof::Status Adsd3100Sensor::start() {
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

aditof::Status Adsd3100Sensor::stop() {
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


aditof::Status
Adsd3100Sensor::getAvailableFrameTypes(
    std::vector<aditof::DepthSensorFrameType> &types) {
    using namespace aditof;
    Status status = Status::OK;

    types = availableFrameTypes; //TBD shall we copy / move vector instead of assign 

    return status;
}

aditof::Status Adsd3100Sensor::setModeByIndex(uint8_t modeIndex){
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

aditof::Status Adsd3100Sensor::setMode(const std::string& mode){
    uint8_t modeIndex;
    aditof::Status status = aditof::Status::OK;
    LOG(INFO) << "Setting camera mode to " << mode;
    //get mode index by name
    status = convertCameraMode(mode, modeIndex);
    if (status != aditof::Status::OK){
        return status;
    }
    //get register value by mode index - nothing to do, the value corresponds to the index
    //set register / control
    status = setModeByIndex(modeIndex);
    if (status != aditof::Status::OK){
        return status;
    }
    return aditof::Status::OK;
}

aditof::Status Adsd3100Sensor::setFrameType(const aditof::DepthSensorFrameType &type) {
    using namespace aditof;
    Status status = Status::OK;
    struct VideoDev *dev;

    struct v4l2_requestbuffers req;
    struct v4l2_format fmt;
    struct v4l2_buffer buf;
    size_t length, offset;

    status = setMode(type.type);
    if (status != aditof::Status::OK){
        LOG(INFO) << "Failed to set camera mode";
        return status;
    }

    m_capturesPerFrame = ModeInfo::getInstance()->getModeInfo(type.type).subframes;

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
        } else if (dev->nVideoBuffers) {
            return status;
        }

        /* Set the frame format in the driver */
        CLEAR(fmt);
        fmt.type = dev->videoBuffersType;
        fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
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

aditof::Status Adsd3100Sensor::program(const uint8_t *firmware, size_t size) {
    using namespace aditof;
    Status status = Status::OK;
//     uint32_t offset;

//     /* Dummy Write To Kick Things off */
//     uint16_t nAddr = 0x112u;
//     uint16_t nData = 0x112u;
//     Adsd3100Sensor::writeAfeRegisters(&nAddr, &nData, 1);

//     m_configuration.getConfigOffset(offset_type_register, &offset);
//     writeConfigBlock(offset);

//     /* Enable All Digital Clocks */
//     nAddr = 0x14u;
//     nData = 0x0u;
//     Adsd3100Sensor::writeAfeRegisters(&nAddr, &nData, 1);

//     m_configuration.getConfigOffset(offset_type_lx5_ram, &offset);
//     writeConfigBlock(offset);

//     m_configuration.getConfigOffset(offset_type_lx5_dram_bank0, &offset);
//     writeConfigBlock(offset);

//     m_configuration.getConfigOffset(offset_type_lx5_dram_bank1, &offset);
//     writeConfigBlock(offset);

//     m_configuration.getConfigOffset(offset_type_lx5_dram_bank2, &offset);
//     writeConfigBlock(offset);

//     m_configuration.getConfigOffset(offset_type_lx5_dram_bank3, &offset);
//     writeConfigBlock(offset);

//     m_configuration.getConfigOffset(offset_type_seqram, &offset);
//     writeConfigBlock(offset);

//     m_configuration.getConfigOffset(offset_type_mapram, &offset);
//     writeConfigBlock(offset);

//     m_configuration.getConfigOffset(offset_type_wavram, &offset);
//     writeConfigBlock(offset);

//     m_configuration.getConfigOffset(offset_type_misc_register, &offset);
//     writeConfigBlock(offset);

//     /* TODO : REMOVE ONCE WE CAN GET THIS FROM EFUSE */
//     uint16_t DataPast = 0x00;
//     uint16_t AddrPast = 0x00;
//     char buf[1024];

//     FILE *fp;
//     fp = fopen("raw_calibration","r");
//     uint32_t nSize = 25754;
//     for (uint32_t i = 0; i < nSize; i++)
//     {
//          fread(buf,4 ,1, fp);
//          buf[4]='\0';
//          nAddr = (uint16_t)strtol(&buf[0], NULL, 16); 

//          fread(buf,1 ,1, fp);
//          fread(buf,4 ,1, fp);
//          buf[4]='\0';
//          nData = (uint16_t)strtol(&buf[0], NULL, 16); 
//          fread(buf,2 ,1, fp);
//         if((nData == DataPast) && (AddrPast == nAddr))
//         {
//             uint16_t dummyRead[2u];
//             dummyRead[0] = 0x112u;
//             dummyRead[1] = 0x0u;
//             Adsd3100Sensor::writeAfeRegisters(&dummyRead[0], &dummyRead[1], 1);
//         }
//          Adsd3100Sensor::writeAfeRegisters(&nAddr, &nData, 1);

//         DataPast = nData;
//         AddrPast = nAddr;
//     }

// #if ADI_DEBUG
//     /* Verify data */
//     fseek(fp, 0, SEEK_SET);
//     for (uint32_t i = 0; i < nSize; i++)
//     {
//          uint16_t Data2;
//          fread(buf,4 ,1, fp);
//          buf[4]='\0';
//          nAddr = (uint16_t)strtol(&buf[0], NULL, 16); 

//          fread(buf,1 ,1, fp);
//          fread(buf,4 ,1, fp);
//          buf[4]='\0';
//          Data2 = (uint16_t)strtol(&buf[0], NULL, 16); 
//          fread(buf,2 ,1, fp);

//          if(nAddr == 0x0)
//          {
//             uint16_t tmp = 4;
//             Adsd3100Sensor::writeAfeRegisters(&tmp, &Data2, 1);
//          }
//          else if(nAddr == 0x500)
//          {
//             Adsd3100Sensor::writeAfeRegisters(&nAddr, &Data2, 1);
//          }
//          else if(nAddr == 0x502)
//          {
//             Adsd3100Sensor::writeAfeRegisters(&nAddr, &Data2, 1);
//          }
//          else if(nAddr == 0xE04)
//          {
//             Adsd3100Sensor::writeAfeRegisters(&nAddr, &Data2, 1);
//          }
//          else if(nAddr == 0xE06)
//          {
//             Adsd3100Sensor::writeAfeRegisters(&nAddr, &Data2, 1);
//          }
//          else if(nAddr == 0x80C)
//          {
//             /* VDMA address read/write offset*/
//             Adsd3100Sensor::writeAfeRegisters(&nAddr, &Data2, 1);
//          }
//          else if(nAddr == 0x14)
//          {
//             /* Skip digital clock gating because this changes throughout */
//             continue;
//          }
//          else if(nAddr == 0x132)
//          {
//             /* Skip latched writes */
//             continue;
//          }
//          else if(nAddr == 0x126)
//          {
//             /* Skip latched writes */
//             continue;
//          }
//          else if(nAddr == 0x528)
//          {
//             /* Changes throughout boot process */
//             continue;
//          }
//          else if(nAddr == 0x4)
//          {
//             /* Skip setup writes for a read */
//             continue;
//          }
//          else
//          {
//             if(nAddr == 0x2)
//             {
//                 /* Read instead of write*/
//                 nAddr = 0x6;
//             }
//             if(nAddr == 0x504)
//             {
//                 /* Read instead of write*/
//                 nAddr = 0x506;
//             }
//             if(nAddr == 0xE08)
//             {
//                 /* Read instead of write*/
//                 nAddr = 0xE0A;
//             }
//             if(AddrPast == nAddr)
//             {
//                 uint16_t dummyRead[2];
//                 dummyRead[0] = 0x112;
//                 Adsd3100Sensor::readAfeRegisters(&dummyRead[0], &dummyRead[1], 1);
//             }
//              Adsd3100Sensor::readAfeRegisters(&nAddr, &DataPast, 1);
//              if(DataPast != Data2)
//              {
//                 printf("FAILURE: Read Data2 %.4X != Expected data %.4X at address %.4X\n", DataPast, Data2, nAddr);
//                 return Status::GENERIC_ERROR;
//              }          
//              DataPast = Data2;
//              AddrPast = nAddr;
//          }
//     }
// #endif
//     fclose(fp);

    return status;
}

void saveFrame(std::string id, char* data, size_t size){
    std::ofstream g(std::string(PROJECT_DIR) + "/build/out" + id + ".bin", std::ios::binary);
    g.write(data, size);
    g.close();
}

aditof::Status Adsd3100Sensor::getFrame(uint16_t *buffer) {
     using namespace aditof;
    struct v4l2_buffer buf[MAX_SUBFRAMES_COUNT];
    struct VideoDev *dev;
    Status status;
    unsigned int buf_data_len;
    uint8_t *pdata;
    
    dev = &m_implData->videoDevs[0];

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
        #ifdef SAVE_RAW_FRAMES
	    saveFrame(std::to_string(idx), (char*)pdata, buf[idx].bytesused);
        #endif

	    memcpy(buffer + (buf_data_len / sizeof(uint16_t)) * idx, pdata, buf[idx].bytesused);

        status = enqueueInternalBufferPrivate(buf[idx], dev);
        if (status != Status::OK) {
            return status;
        }
    }
    
    #ifdef SAVE_RAW_FRAMES
    saveFrame("_full_raw", (char*)buffer, buf_data_len * m_capturesPerFrame);
    #endif
    
    return status;
}



aditof::Status Adsd3100Sensor::readAfeRegisters(const uint16_t *address,
                                                uint16_t *data, size_t length) {
    using namespace aditof;
    struct VideoDev *dev = &m_implData->videoDevs[0];
    Status status = Status::OK;

    static struct v4l2_ext_control extCtrl;
    static struct v4l2_ext_controls extCtrls;

    extCtrl.size = 2048 * sizeof(unsigned short);

    for (size_t i = 0; i < length; i++) {
        uint16_t aux_address = address[i];
        extCtrl.p_u16 = const_cast<uint16_t *>(&aux_address);
        extCtrl.id = V4L2_CID_AD_DEV_READ_REG;
        memset(&extCtrls, 0, sizeof(struct v4l2_ext_controls));
        extCtrls.controls = &extCtrl;
        extCtrls.count = 1;

        if (xioctl(dev->sfd, VIDIOC_S_EXT_CTRLS, &extCtrls) == -1) {
            LOG(WARNING) << "Programming AFE error "
                         << "errno: " << errno << " error: " << strerror(errno);
            return Status::GENERIC_ERROR;
        }
        data[i] = *extCtrl.p_u16;
    }

    return status;
}

aditof::Status Adsd3100Sensor::writeAfeRegisters(const uint16_t *address,
                                                 const uint16_t *data,
                                                 size_t length) {
    using namespace aditof;
    Status status = Status::OK;

    static struct v4l2_ext_control extCtrl;
    static struct v4l2_ext_controls extCtrls;
    struct VideoDev *dev = &m_implData->videoDevs[0];
    static unsigned char buf[CTRL_PACKET_SIZE];
    unsigned short sampleCnt = 0;

    length *= 2 * sizeof(unsigned short);
    while (length) {
        memset(buf, 0, CTRL_PACKET_SIZE);
        size_t maxBytesToSend =
            length > CTRL_PACKET_SIZE ? CTRL_PACKET_SIZE : length;
        for (size_t i = 0; i < maxBytesToSend; i += 4) {
            *(unsigned short *)(buf + i) = address[sampleCnt];
            *(unsigned short *)(buf + i + 2) = data[sampleCnt];
            sampleCnt++;
        }
        length -= maxBytesToSend;

        extCtrl.size = 2048 * sizeof(unsigned short);
        extCtrl.p_u16 = (unsigned short *)buf;
        extCtrl.id = V4L2_CID_AD_DEV_SET_CHIP_CONFIG;
        memset(&extCtrls, 0, sizeof(struct v4l2_ext_controls));
        extCtrls.controls = &extCtrl;
        extCtrls.count = 1;

        if (xioctl(dev->sfd, VIDIOC_S_EXT_CTRLS, &extCtrls) == -1) {
            LOG(WARNING) << "Programming AFE error "
                         << "errno: " << errno << " error: " << strerror(errno);
            return Status::GENERIC_ERROR;
        }
    }

    return status;
}


aditof::Status
Adsd3100Sensor::getDetails(aditof::SensorDetails &details) const {
    details = m_sensorDetails;
    return aditof::Status::OK;
}

aditof::Status Adsd3100Sensor::getHandle(void **handle) {
    *handle = nullptr;
    return aditof::Status::OK;
}

aditof::Status Adsd3100Sensor::waitForBufferPrivate(struct VideoDev *dev) {
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
Adsd3100Sensor::dequeueInternalBufferPrivate(struct v4l2_buffer &buf,
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

aditof::Status Adsd3100Sensor::getInternalBufferPrivate(
    uint8_t **buffer, uint32_t &buf_data_len, const struct v4l2_buffer &buf,
    struct VideoDev *dev) {
    if (dev == nullptr)
        dev = &m_implData->videoDevs[0];

    *buffer = static_cast<uint8_t *>(dev->videoBuffers[buf.index].start);
    buf_data_len = buf.bytesused;

    return aditof::Status::OK;
}

aditof::Status
Adsd3100Sensor::enqueueInternalBufferPrivate(struct v4l2_buffer &buf,
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

aditof::Status Adsd3100Sensor::getDeviceFileDescriptor(int &fileDescriptor) {
    using namespace aditof;
    struct VideoDev *dev = &m_implData->videoDevs[0];

    if (dev->fd != -1) {
        fileDescriptor = dev->fd;
        return Status::OK;
    }

    return Status::INVALID_ARGUMENT;
}

aditof::Status Adsd3100Sensor::waitForBuffer() {
    return waitForBufferPrivate();
}

aditof::Status Adsd3100Sensor::dequeueInternalBuffer(struct v4l2_buffer &buf) {
    return dequeueInternalBufferPrivate(buf);
}

aditof::Status
Adsd3100Sensor::getInternalBuffer(uint8_t **buffer, uint32_t &buf_data_len,
                                  const struct v4l2_buffer &buf) {
    return getInternalBufferPrivate(buffer, buf_data_len, buf);
}

aditof::Status Adsd3100Sensor::enqueueInternalBuffer(struct v4l2_buffer &buf) {
    return enqueueInternalBufferPrivate(buf);
}

#define  DEFAULT_CONFIG_FILE_NAME "TODO"
aditof::Status Adsd3100Sensor::writeConfigBlock(const uint32_t offset){
    FILE *fid;
    ConfigurationData configuration_data;
    static struct v4l2_ext_control extCtrl;
    static struct v4l2_ext_controls extCtrls;
    static uint16_t tempBuf[2];
    
    /* Open configuration file */
    fid = fopen(DEFAULT_CONFIG_FILE_NAME, "r");
    
    fseek(fid, offset, SEEK_SET);

    /* Read blocks*/
    fread(&configuration_data.id, sizeof(uint16_t), 1u, fid);
    fread(&configuration_data.ver, sizeof(uint16_t), 1u, fid);
    fread(&configuration_data.size, sizeof(uint32_t), 1u, fid);
    fread(&configuration_data.burst_layout, sizeof(uint16_t), 1u, fid);
    fread(&configuration_data.burst_num, sizeof(uint16_t), 1u, fid);
    fread(configuration_data.burst_setup, sizeof(uint16_t), 4u, fid);
    /* Write setup values */
    for(uint32_t i = 0u; i < configuration_data.burst_num*2u; i+=2u)
    {
        Adsd3100Sensor::writeAfeRegisters(&configuration_data.burst_setup[i], &configuration_data.burst_setup[i+1u], 1);
    }
    fread(&configuration_data.start_address, sizeof(uint16_t), 1u, fid);
    fread(&configuration_data.rsvd, sizeof(uint16_t), 1u, fid);
    fread(&configuration_data.values, sizeof(uint32_t), 1u, fid);
    static uint16_t Data[16400u];
    if(configuration_data.burst_layout == 1)
    {
        /* Burst Write */
        fread(Data, sizeof(uint16_t), configuration_data.values, fid);
        for(uint32_t i = 0u; i < configuration_data.values; i++)
        {
            Adsd3100Sensor::writeAfeRegisters(&configuration_data.start_address, &Data[i], 1);
            
            tempBuf[0] = 0x112u;
            tempBuf[1] = 0xABCD;
            Adsd3100Sensor::writeAfeRegisters(&tempBuf[0], &tempBuf[1], 1);  
        }
    }
    else
    {
        uint16_t nAddrLast = 0;
        /* Address data pairs */
        for(uint32_t i = 0u; i < configuration_data.values; i+=2)
        {
            uint16_t nAddr;
            uint16_t nData;
            
            fread(&nAddr, sizeof(uint16_t), 1u, fid);
            fread(&nData, sizeof(uint16_t), 1u, fid);
            

            if(nAddrLast == nAddr)
            {
                tempBuf[0] = 0x112u;
                tempBuf[1] = 0xABCD;
                Adsd3100Sensor::writeAfeRegisters(&tempBuf[0], &tempBuf[1], 1);
            }
            tempBuf[0] = nAddr;
            tempBuf[1] = nData;
            Adsd3100Sensor::writeAfeRegisters(&tempBuf[0], &tempBuf[1], 1);
            
            nAddrLast = nAddr;
        }  
    }

    fclose(fid);

#if ADI_DEBUG


    /* Verify writes */
    uint16_t DataPast = 0;
    uint16_t AddrPast = 0;
    uint16_t nAddr;
    uint16_t DataNew;
    fid = fopen(DEFAULT_CONFIG_FILE_NAME, "r");
    fseek(fid, offset, SEEK_SET);

    /* Read blocks*/
    fread(&configuration_data.id, sizeof(uint16_t), 1u, fid);
    fread(&configuration_data.ver, sizeof(uint16_t), 1u, fid);
    fread(&configuration_data.size, sizeof(uint32_t), 1u, fid);
    fread(&configuration_data.burst_layout, sizeof(uint16_t), 1u, fid);
    fread(&configuration_data.burst_num, sizeof(uint16_t), 1u, fid);
    fread(configuration_data.burst_setup, sizeof(uint16_t), 4u, fid);
    /* read setup values */
    for(uint32_t i = 0u; i < configuration_data.burst_num*2u; i+=2u)
    {
         nAddr = configuration_data.burst_setup[i];
         DataNew = configuration_data.burst_setup[i+1u];
         if(nAddr == 0x0)
         {
            uint16_t tmp = 4;
            Adsd3100Sensor::writeAfeRegisters(&tmp, &DataNew, 1);
         }
         else if(nAddr == 0x500)
         {
            Adsd3100Sensor::writeAfeRegisters(&nAddr, &DataNew, 1);
         }
         else if(nAddr == 0x502)
         {
            Adsd3100Sensor::writeAfeRegisters(&nAddr, &DataNew, 1);
         }
         else if(nAddr == 0xE04)
         {
            Adsd3100Sensor::writeAfeRegisters(&nAddr, &DataNew, 1);
         }
         else if(nAddr == 0xE06)
         {
            Adsd3100Sensor::writeAfeRegisters(&nAddr, &DataNew, 1);
         }
         else if(nAddr == 0x80C)
         {
            /* VDMA address read/write offset*/
            Adsd3100Sensor::writeAfeRegisters(&nAddr, &DataNew, 1);
         }
         else if(nAddr == 0x14)
         {
            /* Update digital clock gating because this changes throughout */
            Adsd3100Sensor::writeAfeRegisters(&nAddr, &DataNew, 1);
         }
         else if(nAddr == 0x132)
         {
            /* Skip latched writes */
            continue;
         }
         else if(nAddr == 0x126)
         {
            /* Skip latched writes */
            continue;
         }
         else if(nAddr == 0x528)
         {
            /* Changes throughout boot process */
            continue;
         }
         else if(nAddr == 0x4)
         {
            /* Skip setup writes for a read */
            continue;
         }
         else
         {
            if(nAddr == 0x2)
            {
                /* Read instead of write*/
                nAddr = 0x6;
            }
            if(nAddr == 0x504)
            {
                /* Read instead of write*/
                nAddr = 0x506;
            }
            if(nAddr == 0xE08)
            {
                /* Read instead of write*/
                nAddr = 0xE0A;
            }
            if(AddrPast == nAddr)
            {
                uint16_t dummyRead[2];
                dummyRead[0] = 0x112;
                Adsd3100Sensor::readAfeRegisters(&dummyRead[0], &dummyRead[1], 1);
            }
             Adsd3100Sensor::readAfeRegisters(&nAddr, &DataPast, 1);
             if(DataPast != DataNew)
             {
                printf("FAILURE: Read Data %.4X != Expected data %.4X at address %.4X\n", DataPast, DataNew, nAddr);
                return aditof::Status::GENERIC_ERROR;
             }          
             DataPast = DataNew;
             AddrPast = nAddr;
         }
    }
    fread(&configuration_data.start_address, sizeof(uint16_t), 1u, fid);
    fread(&configuration_data.rsvd, sizeof(uint16_t), 1u, fid);
    fread(&configuration_data.values, sizeof(uint32_t), 1u, fid);
    if(configuration_data.burst_layout == 1)
    {
        /* Burst Write */
        fread(Data, sizeof(uint16_t), configuration_data.values, fid);
        for(uint32_t i = 0u; i < configuration_data.values; i++)
        {
             nAddr = configuration_data.start_address; 
             DataNew = Data[i];
             if(nAddr == 0x0)
             {
                uint16_t tmp = 4;
                Adsd3100Sensor::writeAfeRegisters(&tmp, &DataNew, 1);
             }
             else if(nAddr == 0x500)
             {
                Adsd3100Sensor::writeAfeRegisters(&nAddr, &DataNew, 1);
             }
             else if(nAddr == 0x502)
             {
                Adsd3100Sensor::writeAfeRegisters(&nAddr, &DataNew, 1);
             }
             else if(nAddr == 0xE04)
             {
                Adsd3100Sensor::writeAfeRegisters(&nAddr, &DataNew, 1);
             }
             else if(nAddr == 0xE06)
             {
                Adsd3100Sensor::writeAfeRegisters(&nAddr, &DataNew, 1);
             }
             else if(nAddr == 0x80C)
             {
                /* VDMA address read/write offset*/
                Adsd3100Sensor::writeAfeRegisters(&nAddr, &DataNew, 1);
             }
             else if(nAddr == 0x14)
             {
                /* Update digital clock gating because this changes throughout */
                Adsd3100Sensor::writeAfeRegisters(&nAddr, &DataNew, 1);
             }
             else if(nAddr == 0x132)
             {
                /* Skip latched writes */
                continue;
             }
             else if(nAddr == 0x126)
             {
                /* Skip latched writes */
                continue;
             }
             else if(nAddr == 0x528)
             {
                /* Changes throughout boot process */
                continue;
             }
             else if(nAddr == 0x4)
             {
                /* Skip setup writes for a read */
                continue;
             }
             else
             {
                if(nAddr == 0x2)
                {
                    /* Read instead of write*/
                    nAddr = 0x6;
                }
                if(nAddr == 0x504)
                {
                    /* Read instead of write*/
                    nAddr = 0x506;
                }
                if(nAddr == 0xE08)
                {
                    /* Read instead of write*/
                    nAddr = 0xE0A;
                }
                if(AddrPast == nAddr)
                {
                    uint16_t dummyRead[2];
                    dummyRead[0] = 0x112;
                    Adsd3100Sensor::readAfeRegisters(&dummyRead[0], &dummyRead[1], 1);
                }
                 Adsd3100Sensor::readAfeRegisters(&nAddr, &DataPast, 1);
                 if(DataPast != DataNew)
                 {
                    printf("FAILURE: Read Data %.4X != Expected data %.4X at address %.4X\n", DataPast, DataNew, nAddr);
                    return aditof::Status::GENERIC_ERROR;
                 }          
                 DataPast = DataNew;
                 AddrPast = nAddr;
             }
        }
    }
    else
    {
        /* Address data pairs */
        for(uint32_t i = 0u; i < configuration_data.values; i+=2)
        {
            fread(&nAddr, sizeof(uint16_t), 1u, fid);
            fread(&DataNew, sizeof(uint16_t), 1u, fid);
            
             if(nAddr == 0x0)
             {
                uint16_t tmp = 4;
                Adsd3100Sensor::writeAfeRegisters(&tmp, &DataNew, 1);
             }
             else if(nAddr == 0x500)
             {
                Adsd3100Sensor::writeAfeRegisters(&nAddr, &DataNew, 1);
             }
             else if(nAddr == 0x502)
             {
                Adsd3100Sensor::writeAfeRegisters(&nAddr, &DataNew, 1);
             }
             else if(nAddr == 0xE04)
             {
                Adsd3100Sensor::writeAfeRegisters(&nAddr, &DataNew, 1);
             }
             else if(nAddr == 0xE06)
             {
                Adsd3100Sensor::writeAfeRegisters(&nAddr, &DataNew, 1);
             }
             else if(nAddr == 0x80C)
             {
                /* VDMA address read/write offset*/
                Adsd3100Sensor::writeAfeRegisters(&nAddr, &DataNew, 1);
             }
             else if(nAddr == 0x14)
             {
                /* Update digital clock gating because this changes throughout */
                Adsd3100Sensor::writeAfeRegisters(&nAddr, &DataNew, 1);
             }
             else if(nAddr == 0x132)
             {
                /* Skip latched writes */
                continue;
             }
             else if(nAddr == 0x126)
             {
                /* Skip latched writes */
                continue;
             }
             else if(nAddr == 0x528)
             {
                /* Changes throughout boot process */
                continue;
             }
             else if(nAddr == 0x4)
             {
                /* Skip setup writes for a read */
                continue;
             }
             else
             {
                if(nAddr == 0x2)
                {
                    /* Read instead of write*/
                    nAddr = 0x6;
                }
                if(nAddr == 0x504)
                {
                    /* Read instead of write*/
                    nAddr = 0x506;
                }
                if(nAddr == 0xE08)
                {
                    /* Read instead of write*/
                    nAddr = 0xE0A;
                }
                if(AddrPast == nAddr)
                {
                    uint16_t dummyRead[2];
                    dummyRead[0] = 0x112;
                    Adsd3100Sensor::readAfeRegisters(&dummyRead[0], &dummyRead[1], 1);
                }
                 Adsd3100Sensor::readAfeRegisters(&nAddr, &DataPast, 1);
                 if(DataPast != DataNew)
                 {
                    printf("FAILURE: Read Data %.4X != Expected data %.4X at address %.4X\n", DataPast, DataNew, nAddr);
                    return aditof::Status::GENERIC_ERROR;
                 }          
                 DataPast = DataNew;
                 AddrPast = nAddr;
             }
        }  
    }

    fclose(fid);
#endif
    return aditof::Status::OK;   
}
