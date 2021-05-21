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

          const std::vector<aditof::DepthSensorFrameType> availableFrameTypes =
        {
            //frame type
            {"short_throw",  
                {  //type, width, height
                    {"depth", 512, 512}, 
                    {"ir", 333, 333},
                    {"embedded_header", 1, 128}
                },
                //width, height
                120, 130, 
            },
            {"long_throw",  
                { 
                    {"depth", 512, 512}, 
                    {"ir", 333, 333},
                    {"embedded_header", 1, 128}
                },
                120, 130, 
            },
            {"aHat1",  
                { 
                    {"depth", 512, 512}, 
                    {"ir", 333, 333},
                    {"embedded_header", 1, 128}
                },
                120, 130, 
            },
            {"pcm",
                {
                    {"ir", 4096, 256},
                    {"embedded_header", 1, 128}
                },
                4096, 256, //TODO header size not counted here
            },
            {"long_throw_native", 
                {
                    {"ir", 4096, 256},
                    {"embedded_header", 1, 128}
                },
                4096, 256,
            },
            {"mp_pcm", 
                {
                    {"raw", 1024, 10240},                    
                    {"ir", 1024, 1024},
                    {"xyz", 1024, 1024},
                    {"depth", 1024, 1024},
                    {"embedded_header", 1, 128},
                },
                1024, (10240 + 1024 * 3), //TODO header size not counted here
            },
            {"chip_char", 
                {
                    {"ir", 4096, 256},
                    {"embedded_header", 1, 128}
                },
                4096, 256,
            },
            {"qmp", 
                {
                    {"ir", 4096, 256},
                    {"embedded_header", 1, 128}
                },
                4096, 256,
            },
            {"pcm8", 
                {
                    {"ir", 4096, 256},
                    {"embedded_header", 1, 128}
                },
                4096, 256,
            },
            {"ahat2", 
                {
                    {"ir", 4096, 256},
                    {"embedded_header", 1, 128}
                },
                4096, 256,
            },
            {"mp", 
                {
                    {"ir", 4096, 256},
                    {"embedded_header", 1, 128}
                },
                4096, 256,
            },

        };
};

#endif // OFFLINE_DEPTH_SENSOR_H
