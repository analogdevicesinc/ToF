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
#include "gpio.h"
#ifdef USE_GLOG
#include <glog/logging.h>
#else
#include <aditof/log.h>
#endif

#include <errno.h>
#include <fcntl.h>
#include <linux/gpio.h>
#include <sys/ioctl.h>
#include <unistd.h>

using namespace aditof;
Gpio::Gpio(const std::string &charDeviceName, int gpioNumber)
    : m_charDevName{charDeviceName}, m_lineHandle{-1},
      m_gpioNumber(gpioNumber) {}

int Gpio::open(int openType) {
    int ret;
    int fd;

    fd = ::open(m_charDevName.c_str(), O_RDONLY);
    if (fd <= 0) {
        LOG(ERROR) << "Failed to open gpio!";
        return errno;
    }

    struct gpiohandle_request request;
    request.lineoffsets[0] = m_gpioNumber;
    request.flags = openType;
    request.lines = 1;
    ret = ioctl(fd, GPIO_GET_LINEHANDLE_IOCTL, &request);
    ::close(fd);
    if (ret == -1) {
        LOG(ERROR) << "ioctl failed with error: " << errno;
        return errno;
    }
    m_lineHandle = request.fd;

    return ret;
}

int Gpio::openForWrite() { return this->open(GPIOHANDLE_REQUEST_OUTPUT); }

int Gpio::openForRead() { return this->open(GPIOHANDLE_REQUEST_INPUT); }

int Gpio::close() {
    int ret = 0;

    if (m_lineHandle != -1) {
        ret = ::close(m_lineHandle);
        m_lineHandle = -1;
    }

    return ret;
}

int Gpio::readValue(int &value) {
    struct gpiohandle_data data;
    int ret;

    if (m_lineHandle == -1) {
        LOG(ERROR) << "The Gpio object is not initialized!";
        return -EBADFD;
    }

    data.values[0] = value;
    ret = ioctl(m_lineHandle, GPIOHANDLE_GET_LINE_VALUES_IOCTL, &data);
    if (ret == -1) {
        return errno;
    }
    value = data.values[0];

    return ret;
}

int Gpio::writeValue(int value) {
    struct gpiohandle_data data;
    int ret;

    if (m_lineHandle == -1) {
        LOG(ERROR) << "The Gpio object is not initialized!";
        return -EBADFD;
    }

    data.values[0] = value;
    ret = ioctl(m_lineHandle, GPIOHANDLE_SET_LINE_VALUES_IOCTL, &data);
    if (ret == -1) {
        return errno;
    }

    return ret;
}
