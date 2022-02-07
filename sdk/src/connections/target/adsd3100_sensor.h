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

#include "aditof/depth_sensor_interface.h"
#include "connections/target/v4l_buffer_access_interface.h"
#include "cameras/itof-camera/mode_info.h"
#include <memory>

class Adsd3100Sensor : public aditof::DepthSensorInterface,
                       public aditof::V4lBufferAccessInterface {
    public:
        Adsd3100Sensor(const std::string &driverPath,
                            const std::string &driverSubPath,
                            const std::string &captureDev);
        ~Adsd3100Sensor();

    public:
        virtual aditof::Status open() override;
        virtual aditof::Status start() override;
        virtual aditof::Status stop() override;
        virtual aditof::Status
        getAvailableFrameTypes(std::vector<aditof::DepthSensorFrameType> &types) override;
        virtual aditof::Status
        setFrameType(const aditof::DepthSensorFrameType &type) override;
        virtual aditof::Status program(const uint8_t *firmware,
                                   size_t size) override;
        virtual aditof::Status getFrame(uint16_t *buffer) override;
        virtual aditof::Status readRegisters(const uint16_t *address,
                                            uint16_t *data,
                                            size_t length, bool burst = true) override;
        virtual aditof::Status writeRegisters(const uint16_t *address,
                                             const uint16_t *data,
                                             size_t length, bool burst = true) override;
        virtual aditof::Status
        getDetails(aditof::SensorDetails &details) const override;
        virtual aditof::Status getHandle(void **handle) override;

    public: // implements V4lBufferAccessInterface
    // Methods that give a finer control than getFrame()
    // And worksif there is only one v4lbuffer (if many, then I don't know, maybe restructure this interface)
    virtual aditof::Status waitForBuffer() override;
    virtual aditof::Status
    dequeueInternalBuffer(struct v4l2_buffer &buf) override;
    virtual aditof::Status
    getInternalBuffer(uint8_t **buffer, uint32_t &buf_data_len,
                      const struct v4l2_buffer &buf) override;
    virtual aditof::Status
    enqueueInternalBuffer(struct v4l2_buffer &buf) override;
    virtual aditof::Status
    getDeviceFileDescriptor(int &fileDescriptor) override;

    private:
        aditof::Status writeConfigBlock(const uint32_t offset);
        aditof::Status waitForBufferPrivate(struct VideoDev *dev = nullptr);
        aditof::Status dequeueInternalBufferPrivate(struct v4l2_buffer &buf,
                                                    struct VideoDev *dev = nullptr);
        aditof::Status getInternalBufferPrivate(uint8_t **buffer,
                                                uint32_t &buf_data_len,
                                                const struct v4l2_buffer &buf,
                                                struct VideoDev *dev = nullptr);
        aditof::Status enqueueInternalBufferPrivate(struct v4l2_buffer &buf,
                                                    struct VideoDev *dev = nullptr);

        aditof::Status setModeByIndex(uint8_t modeIndex);
        aditof::Status setMode(const std::string& mode);

    private:
        struct ImplData;
        aditof::SensorDetails m_sensorDetails;
        std::string m_driverPath;
        std::string m_driverSubPath;
        std::string m_captureDev;
        std::unique_ptr<ImplData> m_implData;
        uint8_t m_capturesPerFrame;

         const std::vector<aditof::DepthSensorFrameType> availableFrameTypes =
        {
            {"lt_bin",
                {
                    {"raw", 320, 2592},
                    {"ir", 320, 288},
                    {"xyz", 320, 288},
                    {"depth", 320, 288},
                    {"embedded_header", 1, 128},
                },
                3840, 216, //TODO header size not counted here
            }, 
            {"pcm",
                {
                    {"ir", 1024, 1024},
                    {"embedded_header", 1, 128}
                },
                4096, 256, //TODO header size not counted here
            },
            {"mp_pcm", 
                {
                    {"raw", 1024, 10240},                    
                    {"ir", 1024, 1024},
                    {"xyz", 1024, 1024},
                    {"depth", 1024, 1024},
                    {"embedded_header", 1, 128},

                },
                4096, 2560, //TODO header size not counted here
            },
            {"qmp", 
                {
                     {"raw", 512, 5120},
                     {"ir", 512, 512},
                     {"xyz", 512, 512},
                     {"depth", 512, 512},
                     {"embedded_header", 1, 128}
                },
                4096, 640,
            },
            {"mp", 
                {
                    {"raw", 1024, 9216},
                    {"ir", 1024, 1024},
                    {"xyz", 1024, 1024},
                    {"depth", 1024, 1024},
                    {"embedded_header", 1, 128}
                },
                4096, 2304,
            },

        };
};
