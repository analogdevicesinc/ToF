#ifndef OFFLINE_DEPTH_SENSOR_H
#define OFFLINE_DEPTH_SENSOR_H

#include "aditof/depth_sensor_interface.h"
#include "aditof/sensor_definitions.h"

#include <memory>

class OfflineDepthSensor : public aditof::DepthSensorInterface {
public:
    OfflineDepthSensor(std::string path);
    ~OfflineDepthSensor();

public: // implements DepthSensorInterface
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
  virtual aditof::Status readAfeRegisters(const uint16_t *address,
                                          uint16_t *data,
                                          size_t length) override;
  virtual aditof::Status writeAfeRegisters(const uint16_t *address,
                                           const uint16_t *data,
                                           size_t length) override;
  virtual aditof::Status
  getDetails(aditof::SensorDetails &details) const override;
  virtual aditof::Status getHandle(void **handle) override;

private:
  aditof::SensorDetails m_sensorDetails;
  std::vector<aditof::DepthSensorFrameType> m_depthSensorFrameTypes;
  std::string m_path;
};

#endif // OFFLINE_DEPTH_SENSOR_H
