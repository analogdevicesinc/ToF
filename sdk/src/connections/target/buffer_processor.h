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

#include "v4l_buffer_access_interface.h"

#include "../../cameras/itof-camera/mode_info.h"
#include "../../cameras/itof-camera/tofi/tofi_compute.h"
#include "../../cameras/itof-camera/tofi/tofi_config.h"
#include "../../cameras/itof-camera/tofi/tofi_util.h"

#define OUTPUT_DEVICE "/dev/video1"

class BufferProcessor : public aditof::V4lBufferAccessInterface {
    public:
    BufferProcessor();
    ~BufferProcessor();

  public:
    aditof::Status open();
    aditof::Status setVideoProperties(int frameWidth, int frameHeight);
    aditof::Status setProcessorProperties(uint8_t *iniFile, uint16_t iniFileLength,
                           uint8_t *calData, uint16_t calDataLength);
    aditof::Status processFrame(uint16_t *buffer = nullptr);

    private:
    bool m_vidPropSet;
    boot m_processorPropSet;

    uint16_t m_outputFrameWidth;
    uint16_t m_outputFrameHeight;

    TofiConfig *m_tofiConfig;
    TofiComputeContext *m_tofiComputeContext;
    TofiXYZDealiasData m_xyzDealiasData[11];

    struct v4l2_capability m_videoCapabilities;
	struct v4l2_format m_videoFormat;
    const char* m_videoDevice = OUTPUT_DEVICE;
    int m_fd = 0;
}
