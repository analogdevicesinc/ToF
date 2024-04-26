#ifndef OFFLINE_DEPTH_SENSOR_H
#define OFFLINE_DEPTH_SENSOR_H

#include "aditof/depth_sensor_interface.h"
#ifdef TARGET
#include "tofi/tofi_compute.h"
#include "tofi/tofi_config.h"
#include "tofi/tofi_util.h"
#endif

#include <map>
#include <memory>

class OfflineDepthSensor : public aditof::DepthSensorInterface {
  public:
    OfflineDepthSensor(std::string path);
    ~OfflineDepthSensor();

  public: // implements DepthSensorInterface
    virtual aditof::Status open() override;
    virtual aditof::Status start() override;
    virtual aditof::Status stop() override;
    virtual aditof::Status
    getAvailableModes(std::vector<uint8_t> &modes) override;
    virtual aditof::Status
    getModeDetails(const uint8_t &mode,
                   aditof::DepthSensorFrameType &details) override;
    virtual aditof::Status
    setMode(const aditof::DepthSensorFrameType &type) override;
    virtual aditof::Status setMode(const uint8_t &mode) override;
    virtual aditof::Status getFrame(uint16_t *buffer) override;
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
                                             unsigned int usDelay) override;
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
    virtual aditof::Status adsd3500_register_interrupt_callback(
        aditof::SensorInterruptCallback &cb) override;
    virtual aditof::Status adsd3500_unregister_interrupt_callback(
        aditof::SensorInterruptCallback &cb) override;
    virtual aditof::Status adsd3500_reset() override;
    virtual aditof::Status adsd3500_get_status(int &chipStatus,
                                               int &imagerStatus) override;
    virtual aditof::Status
    initTargetDepthCompute(uint8_t *iniFile, uint16_t iniFileLength,
                           uint8_t *calData, uint16_t calDataLength) override;
    virtual aditof::Status
    getIniParams(std::map<std::string, float> &params) override;
    virtual aditof::Status
    setIniParams(const std::map<std::string, float> &params) override;

    virtual aditof::Status
    setSensorConfiguration(const std::string &sensorConf) override;

  private:
    std::string m_connectionType;
    aditof::SensorDetails m_sensorDetails;
    std::vector<aditof::DepthSensorFrameType> m_depthSensorFrameTypes;
    std::string m_path;
    std::string m_frameTypeSelected;
    std::map<std::string, std::pair<std::uint16_t *, std::size_t>> m_frameTypes;
    const std::vector<aditof::DepthSensorFrameType> availableModes = {
        {"sr-native",
         {"raw", "depth", "ab", "conf", "xyz", "metadata"},
         0,
         0,
         0,
         1024,
         1024,
         128},
        {"lr-native",
         {"raw", "depth", "ab", "conf", "xyz", "metadata"},
         0,
         0,
         0,
         1024,
         1024,
         128},
        {"sr-qnative",
         {"raw", "depth", "ab", "conf", "xyz", "metadata"},
         0,
         0,
         0,
         512,
         512,
         128},
        {"lr-qnative",
         {"raw", "depth", "ab", "conf", "xyz", "metadata"},
         0,
         0,
         0,
         512,
         512,
         128},
        {"sr-mixed",
         {"raw", "depth", "ab", "conf", "xyz", "metadata"},
         0,
         0,
         0,
         512,
         512,
         128},
        {"lr-mixed",
         {"raw", "depth", "ab", "conf", "xyz", "metadata"},
         0,
         0,
         0,
         512,
         512,
         128}};
#ifdef TARGET
    TofiConfig *m_tofiConfig;
    TofiComputeContext *m_tofiComputeContext;
    TofiXYZDealiasData m_xyzDealiasData[11];
    uint16_t m_outputFrameWidth;
    uint16_t m_outputFrameHeight;
#endif
};

#endif // OFFLINE_DEPTH_SENSOR_H
