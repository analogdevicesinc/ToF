/*
MIT License

Copyright (c) 2021 Analog Devices, Inc.

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/
#ifndef CALIBRATION_ITOF_H
#define CALIBRATION_ITOF_H

#include "adsd3100_configuration.h"

#include <aditof/status_definitions.h>

#include <memory>
#include <string>
#include <vector>

namespace aditof {
class DepthSensorInterface;
} // namespace aditof

class CalibrationItof {
  public:
    CalibrationItof(std::shared_ptr<aditof::DepthSensorInterface> sensor);
    ~CalibrationItof() = default;

  public:
    aditof::Status writeConfiguration(const std::string &configurationFile);
    aditof::Status writeCalibration(const std::string &calibrationFile);
    aditof::Status
    writeSettings(const std::vector<std::pair<std::string, int32_t>> &settings);

  private:
    aditof::Status writeDefaultCalibration();

  private:
    std::shared_ptr<aditof::DepthSensorInterface> m_sensor;
    aditof::ADSD3100Configuration m_configuration;
};

#endif /*CALIBRATION_ITOF_H*/
