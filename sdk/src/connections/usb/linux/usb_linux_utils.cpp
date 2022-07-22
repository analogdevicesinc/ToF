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
#include "usb_linux_utils.h"

#include <errno.h>
#include <glog/logging.h>
#include <linux/usb/video.h>
#include <linux/uvcvideo.h>
#include <memory>
#include <sys/ioctl.h>

#ifdef DEBUG_USB
#include <iostream>
#include <linux/uvcvideo.h>
#include <string.h>
#endif

int UsbLinuxUtils::xioctl(int fh, unsigned long request, void *arg) {
    int r;

    do {
        r = ioctl(fh, request, arg);
        // printf("Error=%d\n",errno);
    } while (-1 == r && (EINTR == errno || EIO == errno) && errno != 0);

#ifdef DEBUG_USB
    if (request == (int)UVCIOC_CTRL_QUERY) {
        static int count = 0;
        struct uvc_xu_control_query *cq = (struct uvc_xu_control_query *)arg;
        count++;
        printf("%d Calling IOCTL with bRequest %02x, wValue %04x, wIndex "
               "%04x, "
               "wLength %04x\n",
               count, cq->query, cq->selector, cq->unit, cq->size);
        if (r != 0) {
            printf("Return values: %d \n", r);
            printf("IOCTL failed, error num: %d, %s\n", errno, strerror(errno));
        }
    }
#endif
    return r;
}

int UsbLinuxUtils::uvcExUnitReadOnePacket(int fd, uint8_t selector,
                                          uint8_t *BytesToWrite,
                                          uint8_t nbBytesToWrite, uint8_t *data,
                                          uint8_t nbPacketBytes,
                                          uint8_t nbBytesToRead) {
    int ret = 0;

    if (nbPacketBytes > MAX_BUF_SIZE) {
        LOG(ERROR)
            << nbPacketBytes
            << " is greater than the maximum size for a packet which is: "
            << MAX_BUF_SIZE;
        return -EINVAL;
    }

    if (nbBytesToWrite > MAX_BUF_SIZE) {
        LOG(ERROR)
            << nbBytesToWrite
            << " is greater than the maximum size for a packet which is: "
            << MAX_BUF_SIZE;
        return -EINVAL;
    }

    struct uvc_xu_control_query cq;
    uint8_t packet[nbPacketBytes];

    if (nbBytesToWrite > 0) {
        memcpy(packet, BytesToWrite, nbBytesToWrite);
        packet[nbBytesToWrite] = nbPacketBytes;

        // This set property will send bytes needed by get property
        CLEAR(cq);
        cq.query = UVC_SET_CUR;
        cq.data = static_cast<unsigned char *>(packet);
        cq.size = nbPacketBytes;
        cq.unit = 0x03;
        cq.selector = selector;

        ret = UsbLinuxUtils::xioctl(fd, UVCIOC_CTRL_QUERY, &cq);
        if (ret == -1) {
            LOG(WARNING) << "Error in sending address to device, error: "
                         << errno << "(" << strerror(errno) << ")";
            return ret;
        }
    }

    // This get property will get the bytes
    CLEAR(cq);
    cq.query = UVC_GET_CUR;
    cq.data = static_cast<unsigned char *>(packet);
    cq.size = nbPacketBytes;
    cq.unit = 0x03;
    cq.selector = selector;

    ret = UsbLinuxUtils::xioctl(fd, UVCIOC_CTRL_QUERY, &cq);
    if (ret == -1) {
        LOG(WARNING) << "Error in reading data from device, error: " << errno
                     << "(" << strerror(errno) << ")";
        return ret;
    }

    memcpy(data, packet, nbBytesToRead);

    return ret;
}

int UsbLinuxUtils::uvcExUnitReadBuffer(int fd, uint8_t selector, int16_t id,
                                       uint32_t address, uint8_t *data,
                                       uint32_t bufferLength) {
    int ret = 0;

    if (id < -1 || id > 255) {
        LOG(ERROR)
            << id
            << " is greater than the maximum size (255) accepted for an id";
        return -EINVAL;
    }

    uint8_t nbWrPacketBytes = sizeof(address) + (id > -1 ? 1 : 0);
    uint8_t wrPacket[nbWrPacketBytes];
    uint32_t *crtAddress =
        reinterpret_cast<uint32_t *>(wrPacket + (id > -1 ? 1 : 0));

    uint32_t readBytes = 0;
    uint32_t readLength = 0;

    *crtAddress = address;
    if (id > -1) {
        wrPacket[0] = id;
    }
    while (readBytes < bufferLength) {
        readLength = bufferLength - readBytes < MAX_BUF_SIZE
                         ? bufferLength - readBytes
                         : MAX_BUF_SIZE;
        ret =
            uvcExUnitReadOnePacket(fd, selector, wrPacket, nbWrPacketBytes,
                                   data + readBytes, MAX_BUF_SIZE, readLength);
        if (ret < 0) {
            LOG(WARNING) << "Failed to read a packet via UVC extension unit";
            return ret;
        }
        *crtAddress += readLength;
        readBytes += readLength;
    }

    return ret;
}

int UsbLinuxUtils::uvcExUnitWriteBuffer(int fd, uint8_t selector, int16_t id,
                                        uint32_t address, const uint8_t *data,
                                        uint32_t bufferLength) {
    int ret = 0;

    if (id < -1 || id > 255) {
        LOG(ERROR)
            << id
            << " is greater than the maximum size (255) accepted for an id";
        return -EINVAL;
    }

    uint8_t nbLeadingBytes = sizeof(address) + (id > -1 ? 1 : 0);
    struct uvc_xu_control_query cq;
    uint8_t packet[MAX_BUF_SIZE];
    uint32_t *crtAddress =
        reinterpret_cast<uint32_t *>(packet + (id > -1 ? 1 : 0));
    size_t writeLen = 0;
    size_t writtenBytes = 0;

    *crtAddress = address;
    if (id > -1) {
        packet[0] = static_cast<uint8_t>(id);
    }

    // This set property will send the address and data to be written at the address
    CLEAR(cq);
    cq.query = UVC_SET_CUR;
    cq.data = static_cast<unsigned char *>(packet);
    cq.size = MAX_BUF_SIZE;
    cq.unit = 0x03;
    cq.selector = selector;

    while (writtenBytes < bufferLength) {
        writeLen =
            bufferLength - writtenBytes > MAX_BUF_SIZE - (nbLeadingBytes + 1)
                ? MAX_BUF_SIZE - (nbLeadingBytes + 1)
                : bufferLength - writtenBytes;
        packet[nbLeadingBytes] = writeLen;
        memcpy(&packet[nbLeadingBytes + 1], data + writtenBytes, writeLen);

        ret = UsbLinuxUtils::xioctl(fd, UVCIOC_CTRL_QUERY, &cq);
        if (ret == -1) {
            LOG(WARNING) << "Failed to write a packet via UVC extension unit";
            return ret;
        }
        writtenBytes += writeLen;
        *crtAddress += writeLen;
    }

    return ret;
}

aditof::Status UsbLinuxUtils::uvcExUnitGetString(int fd, int uvcControlId,
                                                 std::string &outStr) {
    uint16_t bufferLength;

    int ret = UsbLinuxUtils::uvcExUnitReadBuffer(
        fd, uvcControlId, -1, 0, reinterpret_cast<uint8_t *>(&bufferLength),
        sizeof(bufferLength));

    if (ret < 0) {
        LOG(WARNING)
            << "Failed to read size of buffer holding sensors info. Error: "
            << ret;
        return aditof::Status::GENERIC_ERROR;
    }

    std::unique_ptr<uint8_t[]> data(new uint8_t[bufferLength + 1]);
    ret = UsbLinuxUtils::uvcExUnitReadBuffer(
        fd, uvcControlId, -1, sizeof(bufferLength), data.get(), bufferLength);
    if (ret < 0) {
        LOG(WARNING) << "Failed to read the content of buffer holding sensors "
                        "info. Error: "
                     << ret;
        return aditof::Status::GENERIC_ERROR;
    }

    data[bufferLength] = '\0';
    outStr = reinterpret_cast<char *>(data.get());

    return aditof::Status::OK;
}

int UsbLinuxUtils::UvcExUnitSetProperty(int fd, uint8_t selector,
                                        const uint8_t *data, size_t nbBytes) {
    struct uvc_xu_control_query cq;
    uint8_t *nonConstBuffer = const_cast<uint8_t *>(data);

    CLEAR(cq);
    cq.query = UVC_SET_CUR;
    cq.data = static_cast<unsigned char *>(nonConstBuffer);
    cq.size = nbBytes;
    cq.unit = 0x03;
    cq.selector = selector;

    return UsbLinuxUtils::xioctl(fd, UVCIOC_CTRL_QUERY, &cq);
}

int UsbLinuxUtils::UvcExUnitGetProperty(int fd, uint8_t selector, uint8_t *data,
                                        size_t nbBytes) {
    struct uvc_xu_control_query cq;

    CLEAR(cq);
    cq.query = UVC_GET_CUR;
    cq.data = static_cast<unsigned char *>(data);
    cq.size = nbBytes;
    cq.unit = 0x03;
    cq.selector = selector;

    return UsbLinuxUtils::xioctl(fd, UVCIOC_CTRL_QUERY, &cq);
}

aditof::Status
UsbLinuxUtils::uvcExUnitSendRequest(int fd, const std::string &requestStr) {
    const uint8_t uvcSendRequestControl = 1;

    // Send the size of the string we're about to send so that UVC gadget knows how many bytes to expect
    size_t stringLength = requestStr.size();
    int ret = UvcExUnitSetProperty(fd, uvcSendRequestControl,
                                   reinterpret_cast<uint8_t *>(&stringLength),
                                   MAX_BUF_SIZE);
    if (ret < 0) {
        LOG(WARNING)
            << "Failed to write the length of the request string. Error: "
            << ret << "(" << strerror(ret) << ")";
        return aditof::Status::GENERIC_ERROR;
    }

    // Send the entire string to the UVC-gadget
    uint8_t packet[MAX_BUF_SIZE];
    size_t writeLen = 0;
    size_t writtenBytes = 0;
    const char *data = requestStr.data();

    while (writtenBytes < stringLength) {
        writeLen = stringLength - writtenBytes > MAX_BUF_SIZE
                       ? MAX_BUF_SIZE
                       : stringLength - writtenBytes;
        memcpy(&packet, data + writtenBytes, writeLen);

        ret = UvcExUnitSetProperty(fd, uvcSendRequestControl, packet,
                                   MAX_BUF_SIZE);
        if (ret < 0) {
            LOG(WARNING) << "Failed to write a packet of the send request "
                            "string. Error: "
                         << ret << "(" << strerror(ret) << ")";
            return aditof::Status::GENERIC_ERROR;
        }
        writtenBytes += writeLen;
    }

    return aditof::Status::OK;
}

aditof::Status UsbLinuxUtils::uvcExUnitGetResponse(int fd,
                                                   std::string &responseStr) {
    const uint8_t uvcGetRequestControl = 2;

    uint8_t packet[MAX_BUF_SIZE];

    // Read the length of the string we're about to read next from the UVC gadget
    int ret =
        UvcExUnitGetProperty(fd, uvcGetRequestControl, packet, MAX_BUF_SIZE);
    if (ret < 0) {
        LOG(WARNING)
            << "Failed to read the length of the response string. Error: "
            << ret << "(" << strerror(ret) << ")";
        return aditof::Status::GENERIC_ERROR;
    }
    size_t stringLength = reinterpret_cast<size_t *>(packet)[0];

    responseStr.reserve(stringLength);

    size_t readBytes = 0;
    size_t readlength = 0;

    while (readBytes < stringLength) {
        readlength = stringLength - readBytes > MAX_BUF_SIZE
                         ? MAX_BUF_SIZE
                         : stringLength - readBytes;

        ret = UvcExUnitGetProperty(fd, uvcGetRequestControl, packet,
                                   MAX_BUF_SIZE);
        if (ret < 0) {
            LOG(WARNING)
                << "Failed to read a packet of the response string. Error: "
                << ret << "(" << strerror(ret) << ")";
            return aditof::Status::GENERIC_ERROR;
        }
        responseStr.append(reinterpret_cast<const char *>(packet), readlength);
        readBytes += readlength;
    }

    return aditof::Status::OK;
}
