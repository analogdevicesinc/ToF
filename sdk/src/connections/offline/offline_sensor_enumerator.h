#ifndef OFFLINE_SENSOR_ENUMERATOR_H
#define OFFLINE_SENSOR_ENUMERATOR_H

#include "aditof/sensor_enumerator_interface.h"

class OfflineSensorEnumerator : public aditof::SensorEnumeratorInterface {
public:
    OfflineSensorEnumerator();
    ~OfflineSensorEnumerator() = default;

public:
    virtual aditof::Status searchSensors() override;
    virtual aditof::Status
    getDepthSensors(std::vector<std::shared_ptr<aditof::DepthSensorInterface>>
                        &depthSensors) override;
    virtual aditof::Status getStorages(
        std::vector<std::shared_ptr<aditof::StorageInterface>> &storages)
        override;
    virtual aditof::Status getTemperatureSensors(
        std::vector<std::shared_ptr<aditof::TemperatureSensorInterface>>
            &temperatureSensors) override;

private:
    std::vector<std::string> m_sensorsInfo;
};

#endif // OFFLINE_SENSOR_ENUMERATOR_H
