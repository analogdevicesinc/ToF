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
#include <glog/logging.h>
#include <linux/videodev2.h>
#include <sstream>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unordered_map>

#define CLEAR(x) memset(&(x), 0, sizeof(x))

#define V4L2_CID_AD_DEV_SET_CHIP_CONFIG 0xA00B00
#define V4L2_CID_AD_DEV_READ_REG 0xA00B01
#define CTRL_PACKET_SIZE 4096
// Can be moved to target_definitions in "camera"/"platform"
#define TEMP_SENSOR_DEV_PATH "/dev/i2c-1"
#define LASER_TEMP_SENSOR_I2C_ADDR 0x49
#define AFE_TEMP_SENSOR_I2C_ADDR 0x4b


#define ADI_DEBUG 1

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

struct Adsd3100Sensor::ImplData {
    int fd;
    int sfd;
    struct buffer *videoBuffers;
    unsigned int nVideoBuffers;
    struct v4l2_plane planes[1];
    aditof::FrameDetails frameDetails;
    bool started;
    enum v4l2_buf_type videoBuffersType;
    std::unordered_map<mode_name_enum, CalibrationData> calibration_cache;

    ImplData()
        : fd(-1), sfd(-1), nVideoBuffers(0),
          videoBuffers(nullptr), frameDetails{0, 0, "", {0.0f, 1.0f}},
          started(false) {}
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
      m_captureDev(captureDev), m_implData(new Addi9036Sensor::ImplData) {
    m_implData->calibration_cache =
        std::unordered_map<mode_name_enum, CalibrationData>();
}

Adsd3100Sensor::~Adsd3100Sensor() {
    if (m_implData->started) {
        stop();
    }

    for (auto it = m_implData->calibration_cache.begin();
         it != m_implData->calibration_cache.begin(); ++it) {
        delete[] it->second.cache;
        it->second.cache = nullptr;
    }

    for (unsigned int i = 0; i < m_implData->nVideoBuffers; i++) {
        if (munmap(m_implData->videoBuffers[i].start,
                   m_implData->videoBuffers[i].length) == -1) {
            LOG(WARNING) << "munmap error "
                         << "errno: " << errno << " error: " << strerror(errno);
        }
    }
    free(m_implData->videoBuffers);

    if (close(m_implData->fd) == -1) {
        LOG(WARNING) << "close m_implData->fd error "
                     << "errno: " << errno << " error: " << strerror(errno);
    }

    if (close(m_implData->sfd) == -1) {
        LOG(WARNING) << "close m_implData->sfd error "
                     << "errno: " << errno << " error: " << strerror(errno);
    }
}

aditof::Status Adsd3100Sensor::open() {
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

    if (m_implData->started) {
        LOG(INFO) << "Device already started";
        return Status::BUSY;
    }
    LOG(INFO) << "Starting device";

    uint16_t nAddr = 0x000C;
    uint16_t nData = 0x00C5;
    Adsd3100Sensor::writeAfeRegisters(&nAddr, &nData, 1);

    /* Verify the Sequencer is waiting for the FSYNC */
    uint32_t nCounter = 0;

    while((nCounter < 100) && (nData != 0x2))
    {
            nAddr = 0x0256;
            Adsd3100Sensor::readAfeRegisters(&nAddr, &nData, 1);
            nCounter++;
    }
    if(nData != 0x2)
    {

        LOG(WARNING) << "Failure: Sequencer was unable to initialize. Stuck in state " << nData;

        if(nData = 0xF)
        {
            nAddr = 0x032;
            uint16_t nErrorCode;
            Adsd3100Sensor::readAfeRegisters(&nAddr, &nErrorCode, 1);
            LOG(WARNING) << "Sequencer error code " << nErrorCode;    
        }
        return Status::GENERIC_ERROR;
    }

    struct v4l2_buffer buf;
    for (unsigned int i = 0; i < m_implData->nVideoBuffers; i++) {
        CLEAR(buf);
        buf.type = m_implData->videoBuffersType;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = i;
        buf.m.planes = m_implData->planes;
        buf.length = 1;

        if (xioctl(m_implData->fd, VIDIOC_QBUF, &buf) == -1) {
            LOG(WARNING) << "mmap error "
                         << "errno: " << errno << " error: " << strerror(errno);
            return Status::GENERIC_ERROR;
        }
    }

    if (xioctl(m_implData->fd, VIDIOC_STREAMON,
               &m_implData->videoBuffersType) == -1) {
        LOG(WARNING) << "VIDIOC_STREAMON error "
                     << "errno: " << errno << " error: " << strerror(errno);
        return Status::GENERIC_ERROR;
    }

    m_implData->started = true;

    return status;
}

aditof::Status Adsd3100Sensor::stop() {
    using namespace aditof;
    Status status = Status::OK;

    if (!m_implData->started) {
        LOG(INFO) << "Device already stopped";
        return Status::BUSY;
    }
    LOG(INFO) << "Stopping device";

    uint16_t nAddr = 0x000C;
    uint16_t nData = 0x00C2;
    Adsd3100Sensor::writeAfeRegisters(&nAddr, &nData, 1);

    if (xioctl(m_implData->fd, VIDIOC_STREAMOFF,
               &m_implData->videoBuffersType) == -1) {
        LOG(WARNING) << "VIDIOC_STREAMOFF error "
                     << "errno: " << errno << " error: " << strerror(errno);
        return Status::GENERIC_ERROR;
    }

    m_implData->started = false;

    return status;
}

aditof::Status
Adsd3100Sensor::getAvailableFrameTypes(std::vector<aditof::FrameDetails> &types) {
    using namespace aditof;
    Status status = Status::OK;

    FrameDetails details;

    details.width = 1024;
    details.height = 1024;
    details.cal_data.offset = 0;
    details.cal_data.gain = 1;
    details.type = "depth_ir";
    types.push_back(details);

    details.width = 1024;
    details.height = 1024;
    details.cal_data.offset = 0;
    details.cal_data.gain = 1;
    details.type = "raw";
    types.push_back(details);

    return status;
}

aditof::Status Adsd3100Sensor::setFrameType(const aditof::FrameDetails &details) {
    using namespace aditof;
    Status status = Status::OK;

    struct v4l2_requestbuffers req;
    struct v4l2_format fmt;
    struct v4l2_buffer buf;
    size_t length, offset;

    if (details != m_implData->frameDetails) {
        for (unsigned int i = 0; i < m_implData->nVideoBuffers; i++) {
            if (munmap(m_implData->videoBuffers[i].start,
                       m_implData->videoBuffers[i].length) == -1) {
                LOG(WARNING)
                    << "munmap error "
                    << "errno: " << errno << " error: " << strerror(errno);
                return Status::GENERIC_ERROR;
            }
        }
        free(m_implData->videoBuffers);
        m_implData->nVideoBuffers = 0;
    } else if (m_implData->nVideoBuffers) {
        return status;
    }

    /* Set the frame format in the driver */
    CLEAR(fmt);
    fmt.type = m_implData->videoBuffersType;
    fmt.fmt.pix.width = details.width;
    fmt.fmt.pix.height = details.height;

    if (xioctl(m_implData->fd, VIDIOC_S_FMT, &fmt) == -1) {
        LOG(WARNING) << "Setting Pixel Format error, errno: " << errno
                     << " error: " << strerror(errno);
        return Status::GENERIC_ERROR;
    }

    /* Allocate the video buffers in the driver */
    CLEAR(req);
    req.count = 2;
    req.type = m_implData->videoBuffersType;
    req.memory = V4L2_MEMORY_MMAP;

    if (xioctl(m_implData->fd, VIDIOC_REQBUFS, &req) == -1) {
        LOG(WARNING) << "VIDIOC_REQBUFS error "
                     << "errno: " << errno << " error: " << strerror(errno);
        return Status::GENERIC_ERROR;
    }

    m_implData->videoBuffers =
        (buffer *)calloc(req.count, sizeof(*m_implData->videoBuffers));
    if (!m_implData->videoBuffers) {
        LOG(WARNING) << "Failed to allocate video m_implData->videoBuffers";
        return Status::GENERIC_ERROR;
    }

    for (m_implData->nVideoBuffers = 0; m_implData->nVideoBuffers < req.count;
         m_implData->nVideoBuffers++) {
        CLEAR(buf);
        buf.type = m_implData->videoBuffersType;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = m_implData->nVideoBuffers;
        buf.m.planes = m_implData->planes;
        buf.length = 1;

        if (xioctl(m_implData->fd, VIDIOC_QUERYBUF, &buf) == -1) {
            LOG(WARNING) << "VIDIOC_QUERYBUF error "
                         << "errno: " << errno << " error: " << strerror(errno);
            return Status::GENERIC_ERROR;
        }

        if (m_implData->videoBuffersType == V4L2_BUF_TYPE_VIDEO_CAPTURE) {
            length = buf.length;
            offset = buf.m.offset;
        } else {
            length = buf.m.planes[0].length;
            offset = buf.m.planes[0].m.mem_offset;
        }

        m_implData->videoBuffers[m_implData->nVideoBuffers].start =
            mmap(NULL, length, PROT_READ | PROT_WRITE, MAP_SHARED,
                 m_implData->fd, offset);

        if (m_implData->videoBuffers[m_implData->nVideoBuffers].start ==
            MAP_FAILED) {
            LOG(WARNING) << "mmap error "
                         << "errno: " << errno << " error: " << strerror(errno);
            return Status::GENERIC_ERROR;
        }

        m_implData->videoBuffers[m_implData->nVideoBuffers].length = length;
    }

    m_implData->frameDetails = details;

    return status;
}

aditof::Status Adsd3100Sensor::program(const uint8_t *firmware, size_t size) {
    using namespace aditof;
    Status status = Status::OK;
    uint32_t offset;

    /* Dummy Write To Kick Things off */
    uint16_t nAddr = 0x112u;
    uint16_t nData = 0x112u;
    Adsd3100Sensor::writeAfeRegisters(&nAddr, &nData, 1);

    m_configuration.getConfigOffset(offset_type_register, &offset);
    writeConfigBlock(offset);

    /* Enable All Digital Clocks */
    nAddr = 0x14u;
    nData = 0x0u;
    Adsd3100Sensor::writeAfeRegisters(&nAddr, &nData, 1);

    m_configuration.getConfigOffset(offset_type_lx5_ram, &offset);
    writeConfigBlock(offset);

    m_configuration.getConfigOffset(offset_type_lx5_dram_bank0, &offset);
    writeConfigBlock(offset);

    m_configuration.getConfigOffset(offset_type_lx5_dram_bank1, &offset);
    writeConfigBlock(offset);

    m_configuration.getConfigOffset(offset_type_lx5_dram_bank2, &offset);
    writeConfigBlock(offset);

    m_configuration.getConfigOffset(offset_type_lx5_dram_bank3, &offset);
    writeConfigBlock(offset);

    m_configuration.getConfigOffset(offset_type_seqram, &offset);
    writeConfigBlock(offset);

    m_configuration.getConfigOffset(offset_type_mapram, &offset);
    writeConfigBlock(offset);

    m_configuration.getConfigOffset(offset_type_wavram, &offset);
    writeConfigBlock(offset);

    m_configuration.getConfigOffset(offset_type_misc_register, &offset);
    writeConfigBlock(offset);

    /* TODO : REMOVE ONCE WE CAN GET THIS FROM EFUSE */
    uint16_t DataPast = 0x00;
    uint16_t AddrPast = 0x00;
    char buf[1024];

    FILE *fp;
    fp = fopen("raw_calibration","r");
    uint32_t nSize = 25754;
    for (uint32_t i = 0; i < nSize; i++)
    {
         fread(buf,4 ,1, fp);
         buf[4]='\0';
         nAddr = (uint16_t)strtol(&buf[0], NULL, 16); 

         fread(buf,1 ,1, fp);
         fread(buf,4 ,1, fp);
         buf[4]='\0';
         nData = (uint16_t)strtol(&buf[0], NULL, 16); 
         fread(buf,2 ,1, fp);
        if((nData == DataPast) && (AddrPast == nAddr))
        {
            uint16_t dummyRead[2u];
            dummyRead[0] = 0x112u;
            dummyRead[1] = 0x0u;
            Adsd3100Sensor::writeAfeRegisters(&dummyRead[0], &dummyRead[1], 1);
        }
         Adsd3100Sensor::writeAfeRegisters(&nAddr, &nData, 1);

        DataPast = nData;
        AddrPast = nAddr;
    }

#if ADI_DEBUG
    /* Verify data */
    fseek(fp, 0, SEEK_SET);
    for (uint32_t i = 0; i < nSize; i++)
    {
         uint16_t Data2;
         fread(buf,4 ,1, fp);
         buf[4]='\0';
         nAddr = (uint16_t)strtol(&buf[0], NULL, 16); 

         fread(buf,1 ,1, fp);
         fread(buf,4 ,1, fp);
         buf[4]='\0';
         Data2 = (uint16_t)strtol(&buf[0], NULL, 16); 
         fread(buf,2 ,1, fp);

         if(nAddr == 0x0)
         {
            uint16_t tmp = 4;
            Adsd3100Sensor::writeAfeRegisters(&tmp, &Data2, 1);
         }
         else if(nAddr == 0x500)
         {
            Adsd3100Sensor::writeAfeRegisters(&nAddr, &Data2, 1);
         }
         else if(nAddr == 0x502)
         {
            Adsd3100Sensor::writeAfeRegisters(&nAddr, &Data2, 1);
         }
         else if(nAddr == 0xE04)
         {
            Adsd3100Sensor::writeAfeRegisters(&nAddr, &Data2, 1);
         }
         else if(nAddr == 0xE06)
         {
            Adsd3100Sensor::writeAfeRegisters(&nAddr, &Data2, 1);
         }
         else if(nAddr == 0x80C)
         {
            /* VDMA address read/write offset*/
            Adsd3100Sensor::writeAfeRegisters(&nAddr, &Data2, 1);
         }
         else if(nAddr == 0x14)
         {
            /* Skip digital clock gating because this changes throughout */
            continue;
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
             if(DataPast != Data2)
             {
                printf("FAILURE: Read Data2 %.4X != Expected data %.4X at address %.4X\n", DataPast, Data2, nAddr);
                return Status::GENERIC_ERROR;
             }          
             DataPast = Data2;
             AddrPast = nAddr;
         }
    }
#endif
    fclose(fp);

    return status;
}

aditof::Status Adsd3100Sensor::getFrame(uint16_t *buffer) {
    using namespace aditof;
    Status status = Status::OK;

    fd_set fds;
    struct timeval tv;
    int r;
    struct v4l2_buffer buf;

    unsigned int width;
    unsigned int height;
    unsigned int offset[2];
    unsigned int offset_idx;

    FD_ZERO(&fds);
    FD_SET(m_implData->fd, &fds);

    tv.tv_sec = 60;
    tv.tv_usec = 0;

    r = select(m_implData->fd + 1, &fds, NULL, NULL, &tv);

    if (r == -1) {
        LOG(WARNING) << "select error "
                     << "errno: " << errno << " error: " << strerror(errno);
        return Status::GENERIC_ERROR;
    } else if (r == 0) {
        LOG(WARNING) << "select timeout";
        return Status::GENERIC_ERROR;
    }

    CLEAR(buf);
    buf.type = m_implData->videoBuffersType;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.length = 1;
    buf.m.planes = m_implData->planes;

    if (xioctl(m_implData->fd, VIDIOC_DQBUF, &buf) == -1) {
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

    if (buf.index >= m_implData->nVideoBuffers) {
        LOG(WARNING) << "Not enough buffers avaialable";
        return Status::GENERIC_ERROR;
    }

    width = m_implData->frameDetails.width;
    height = m_implData->frameDetails.height;
    const uint8_t *pdata =
        static_cast<uint8_t *>(m_implData->videoBuffers[buf.index].start);
    offset[0] = 0;
    offset[1] = height * width / 2;
    if ((width == 668)) {
        unsigned int j = 0;
        for (unsigned int i = 0; i < (height * width * 3 / 2); i += 3) {
            if ((i != 0) && (i % (336 * 3) == 0)) {
                j -= 4;
            }

            buffer[j] = (((unsigned short)*(pdata + i)) << 4) |
                        (((unsigned short)*(pdata + i + 2)) & 0x000F);
            j++;

            buffer[j] = (((unsigned short)*(pdata + i + 1)) << 4) |
                        ((((unsigned short)*(pdata + i + 2)) & 0x00F0) >> 4);
            j++;
        }
    } else {
        // clang-format off
        uint16_t *depthPtr = buffer;
        uint16_t *irPtr = buffer + (width * height) / 2;
        unsigned int j = 0;

        /* The frame is read from the device as an array of uint8_t's where
         * every 3 uint8_t's can produce 2 uint16_t's that have only 12 bits
         * in use.
         * Ex: consider uint8_t a, b, c;
         * We first convert a, b, c to uint16_t
         * We obtain uint16_t f1 = (a << 4) | (c & 0x000F)
         * and uint16_t f2 = (b << 4) | ((c & 0x00F0) >> 4);
         */
        for (unsigned int i = 0; i < (height * width * 3 / 2); i += 24) {
            /* Read 24 bytes from pdata and deinterleave them in 3 separate 8 bytes packs
             *                                   |-> a1 a2 a3 ... a8
             * a1 b1 c1 a2 b2 c2 ... a8 b8 c8  ->|-> b1 b2 b3 ... b8
             *                                   |-> c1 c2 c3 ... c8
             * then convert all the values to uint16_t          
             */
            uint8x8x3_t data = vld3_u8(pdata);
            uint16x8_t aData = vmovl_u8(data.val[0]);
            uint16x8_t bData = vmovl_u8(data.val[1]);
            uint16x8_t cData = vmovl_u8(data.val[2]);

            uint16x8_t lowMask = vdupq_n_u16(0x000F);
            uint16x8_t highMask = vdupq_n_u16(0x00F0);

            /* aBuffer = (a << 4) | (c & 0x000F) for every a and c value*/
            uint16x8_t aBuffer = vorrq_u16(vshlq_n_u16(aData, 4), vandq_u16(cData, lowMask));

            /* bBuffer = (b << 4) | ((c & 0x00F0) >> 4) for every b and c value*/
            uint16x8_t bBuffer = vorrq_u16(vshlq_n_u16(bData, 4), vshrq_n_u16(vandq_u16(cData, highMask), 4));

            uint16x8x2_t toStore;
            toStore.val[0] = aBuffer;
            toStore.val[1] = bBuffer;

            /* Store the 16 frame pixel in the corresponding image */
            if ((j / width) % 2) {
                vst2q_u16(irPtr, toStore);
                irPtr += 16;
            } else {
                vst2q_u16(depthPtr, toStore);
                depthPtr += 16;
            }

            j += 16;
            pdata += 24;
        }
        // clang-format on
    }

    if (xioctl(m_implData->fd, VIDIOC_QBUF, &buf) == -1) {
        LOG(WARNING) << "VIDIOC_QBUF error "
                     << "errno: " << errno << " error: " << strerror(errno);
        return Status::GENERIC_ERROR;
    }

    return status;
}

aditof::Status Adsd3100Sensor::readAfeRegisters(const uint16_t *address,
                                             uint16_t *data, size_t length) {
    using namespace aditof;
    Status status = Status::OK;

    static struct v4l2_ext_control extCtrl;
    static struct v4l2_ext_controls extCtrls;
    static uint16_t readBuf[1];
    

    for (size_t i = 0; i < length; i++) {

        /* The address is 14-bits and should not utilize the upper most bits */
        if((address[i] >> 14u) != 0u)
        {
            return Status::GENERIC_ERROR;
        }

        readBuf[0] = address[i];
        usleep(500);
        extCtrl.size = 8000;
        extCtrl.p_u16 = const_cast<uint16_t *>(&readBuf[0]);
        extCtrl.id = V4L2_CID_AD_DEV_READ_REG;
        memset(&extCtrls, 0, sizeof(struct v4l2_ext_controls));
        extCtrls.controls = &extCtrl;
        extCtrls.count = 1;

        if (xioctl(m_implData->sfd, VIDIOC_S_EXT_CTRLS, &extCtrls) == -1) {
            LOG(WARNING) << "Read AFE Register error "
                         << "errno: " << errno << " error: " << strerror(errno) << " at address " << address[i];
            return Status::GENERIC_ERROR;
        }
        data[i] = *extCtrl.p_u16;
        printf("%.4X %.4X \n", address[i],  data[i]);
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
    static uint16_t writeBuf[2];
    unsigned short sampleCnt = 0;

    for (size_t i = 0; i < length; i++) {
        usleep(100);

        writeBuf[0] = address[i];
        writeBuf[1] = data[i];
        printf("%.4X %.4X \n", address[i],  data[i]);

        extCtrl.size = 8000;
        extCtrl.p_u16 = (uint16_t *)&writeBuf;
        extCtrl.id = V4L2_CID_AD_DEV_SET_CHIP_CONFIG;
        memset(&extCtrls, 0, sizeof(struct v4l2_ext_controls));
        extCtrls.controls = &extCtrl;
        extCtrls.count = 1;

        if (xioctl(m_implData->sfd, VIDIOC_S_EXT_CTRLS, &extCtrls) == -1) {
            LOG(WARNING) << "Programming AFE error "
                         << "errno: " << errno << " error: " << strerror(errno) << " at address " << address[i] << " with data " << data[i];
            return Status::GENERIC_ERROR;
        }
    }

    return status;
}

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
