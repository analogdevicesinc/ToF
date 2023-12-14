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
#include "buffer_processor.h"
#include "connections/target/v4l_buffer_access_interface.h"
#include <memory>
#include <unordered_map>

#include "cameras/itof-camera/mode_info.h"

class Adsd3500Sensor : public aditof::DepthSensorInterface,
                       public aditof::V4lBufferAccessInterface,
                       public std::enable_shared_from_this<Adsd3500Sensor> {
  public:
    Adsd3500Sensor(const std::string &driverPath,
                   const std::string &driverSubPath,
                   const std::string &captureDev);
    ~Adsd3500Sensor();

  public:
    virtual aditof::Status open() override;
    virtual aditof::Status start() override;
    virtual aditof::Status stop() override;
    virtual aditof::Status getAvailableFrameTypes(
        std::vector<aditof::DepthSensorFrameType> &types) override;
    virtual aditof::Status
    setFrameType(const aditof::DepthSensorFrameType &type) override;
    virtual aditof::Status program(const uint8_t *firmware,
                                   size_t size) override;
    virtual aditof::Status getFrame(uint16_t *buffer) override;
    virtual aditof::Status readRegisters(const uint16_t *address,
                                         uint16_t *data, size_t length,
                                         bool burst = true) override;
    virtual aditof::Status writeRegisters(const uint16_t *address,
                                          const uint16_t *data, size_t length,
                                          bool burst = true) override;
    virtual aditof::Status
    getAvailableControls(std::vector<std::string> &controls) const override;
    virtual aditof::Status setControl(const std::string &control,
                                      const std::string &value) override;
    virtual aditof::Status getControl(const std::string &control,
                                      std::string &value) const override;
    virtual aditof::Status
    getDetails(aditof::SensorDetails &details) const override;
    virtual aditof::Status getHandle(void **handle) override;
    virtual aditof::Status getName(std::string &name) const override;
    virtual aditof::Status
    setHostConnectionType(std::string &connectionType) override;

    virtual aditof::Status adsd3500_read_cmd(uint16_t cmd, uint16_t *data,
                                             unsigned int usDelay = 0) override;
    virtual aditof::Status adsd3500_write_cmd(uint16_t cmd,
                                              uint16_t data) override;
    virtual aditof::Status
    adsd3500_read_payload_cmd(uint32_t cmd, uint8_t *readback_data,
                              uint16_t payload_len) override;
    virtual aditof::Status adsd3500_read_payload(uint8_t *payload,
                                                 uint16_t payload_len) override;
    virtual aditof::Status
    adsd3500_write_payload_cmd(uint32_t cmd, uint8_t *payload,
                               uint16_t payload_len) override;
    virtual aditof::Status
    adsd3500_write_payload(uint8_t *payload, uint16_t payload_len) override;
    virtual aditof::Status adsd3500_reset() override;
    virtual aditof::Status adsd3500_register_interrupt_callback(
        aditof::SensorInterruptCallback &cb) override;
    virtual aditof::Status adsd3500_unregister_interrupt_callback(
        aditof::SensorInterruptCallback &cb) override;
    virtual aditof::Status adsd3500_get_status(int &chipStatus,
                                               int &imagerStatus) override;

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
    virtual aditof::Status
    initTargetDepthCompute(uint8_t *iniFile, uint16_t iniFileLength,
                           uint8_t *calData, uint16_t calDataLength) override;

  public:
    aditof::Status adsd3500InterruptHandler(int signalValue);
    virtual aditof::Status
    getIniParams(std::map<std::string, float> &params) override;
    virtual aditof::Status
    setIniParams(const std::map<std::string, float> &params) override;

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
    aditof::Status setMode(const std::string &mode);
    aditof::Status queryAdsd3500();
    aditof::Adsd3500Status convertIdToAdsd3500Status(int status);
    aditof::Status getIniParamsImpl(void *p_config_params, int params_group,
                                    const void *p_tofi_cal_config);
    aditof::Status setIniParamsImpl(void *p_config_params, int params_group,
                                    const void *p_tofi_cal_config);

  private:
    struct ImplData;
    std::string m_sensorName;
    aditof::SensorDetails m_sensorDetails;
    aditof::ConnectionType m_hostConnectionType;
    std::string m_driverPath;
    std::string m_driverSubPath;
    std::string m_captureDev;
    std::unordered_map<std::string, std::string> m_controls;
    std::unique_ptr<ImplData> m_implData;
    uint8_t m_capturesPerFrame;
    bool m_firstRun;
    unsigned int m_sensorFps;
    bool m_adsd3500Queried;
    std::unordered_map<void *, aditof::SensorInterruptCallback>
        m_interruptCallbackMap;
    std::vector<aditof::DepthSensorFrameType> m_availableFrameTypes;
    BufferProcessor *m_bufferProcessor;
    bool m_depthComputeOnTarget;
    int m_chipStatus;
    int m_imagerStatus;
    aditof::Adsd3500Status m_adsd3500Status;
    bool m_chipResetDone;

    const std::vector<aditof::DepthSensorFrameType> availableFrameTypes = {
        {
            "sr-native",
            {{"raw", 1024, 4096},
             {"depth", 1024, 1024},
             {"ab", 1024, 1024},
             {"conf", 1024, 1024},
             {"xyz", 1024, 1024},
             {"metadata", 1, 128}},
            1024,
            4096,
        },
        {
            "lr-native",
            {{"raw", 1024, 4096},
             {"depth", 1024, 1024},
             {"ab", 1024, 1024},
             {"conf", 1024, 1024},
             {"xyz", 1024, 1024},
             {"metadata", 1, 128}},
            1024,
            4096,
        },
        {
            "sr-qnative",
            {{"raw", 2560, 512},
             {"depth", 512, 512},
             {"ab", 512, 512},
             {"conf", 512, 512},
             {"xyz", 512, 512},
             {"metadata", 1, 128}},
            2560,
            512,
        },
        {
            "lr-qnative",
            {{"raw", 2560, 512},
             {"depth", 512, 512},
             {"ab", 512, 512},
             {"conf", 512, 512},
             {"xyz", 512, 512},
             {"metadata", 1, 128}},
            2560,
            512,
        },
        {
            "pcm-native",
            {{"ab", 1024, 1024}},
            1024,
            1024,
        },
        {
            "sr-mixed",
            {{"raw", 2560, 512},
             {"depth", 512, 512},
             {"ab", 512, 512},
             {"conf", 512, 512},
             {"xyz", 512, 512},
             {"metadata", 1, 128}},
            2560,
            512,
        },
        {
            "lr-mixed",
            {{"raw", 2560, 512},
             {"depth", 512, 512},
             {"ab", 512, 512},
             {"conf", 512, 512},
             {"xyz", 512, 512},
             {"metadata", 1, 128}},
            2560,
            512,
        }

    };
    const std::vector<aditof::DepthSensorFrameType>
        availableFrameTypesAdsd3030 = {{
                                           "sr-native",
                                           {{"raw", 2560, 640},
                                            {"depth", 512, 640},
                                            {"ab", 512, 640},
                                            {"conf", 512, 640},
                                            {"xyz", 512, 640},
                                            {"metadata", 1, 128}},
                                           2560,
                                           640,
                                       },
                                       {
                                           "lr-native",
                                           {{"raw", 2560, 640},
                                            {"depth", 512, 640},
                                            {"ab", 512, 640},
                                            {"conf", 512, 640},
                                            {"xyz", 512, 640},
                                            {"metadata", 1, 128}},
                                           2560,
                                           640,
                                       },
                                       {
                                           "sr-qnative",
                                           {{"raw", 1280, 320},
                                            {"depth", 256, 320},
                                            {"ab", 256, 320},
                                            {"conf", 256, 320},
                                            {"xyz", 256, 320},
                                            {"metadata", 1, 128}},
                                           1280,
                                           320,
                                       },
                                       {
                                           "lr-qnative",
                                           {{"raw", 1280, 320},
                                            {"depth", 256, 320},
                                            {"ab", 256, 320},
                                            {"conf", 256, 320},
                                            {"xyz", 256, 320},
                                            {"metadata", 1, 128}},
                                           1280,
                                           320,
                                       },
                                       {
                                           "pcm-native",
                                           {{"ab", 512, 640}},
                                           512,
                                           640,
                                       },
                                       {
                                           "sr-mixed",
                                           {{"raw", 1280, 320},
                                            {"depth", 256, 320},
                                            {"ab", 256, 320},
                                            {"conf", 256, 320},
                                            {"xyz", 256, 320},
                                            {"metadata", 1, 128}},
                                           1280,
                                           320,
                                       },
                                       {
                                           "lr-mixed",
                                           {{"raw", 1280, 320},
                                            {"depth", 256, 320},
                                            {"ab", 256, 320},
                                            {"conf", 256, 320},
                                            {"xyz", 256, 320},
                                            {"metadata", 1, 128}},
                                           1280,
                                           320,
                                       }};
};