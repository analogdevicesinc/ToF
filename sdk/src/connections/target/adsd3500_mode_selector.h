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

#include "sensor-tables/custom_modes.h"
#include "sensor-tables/custom_raw_modes.h"
#include "sensor-tables/standard_modes.h"
#include "sensor-tables/standard_raw_modes.h"

#include <aditof/status_definitions.h>

#include <map>

class Adsd3500ModeSelector {
  public:
    Adsd3500ModeSelector();
    ~Adsd3500ModeSelector() = default;

    //functions to set which table configuration to use
    aditof::Status setConfiguration(std::string &configuration);
    aditof::Status
    getConfigurationTable(aditof::DepthSensorFrameType &configurationTable);

    //Functions used to set mode, number of bits, pixel format, etc
    aditof::Status setControl(std::string &control, std::string &value);
    aditof::Status getControl(std::string &control, std::string &value);

  private:
    std::string m_configuration;
    const std::vector<std::string> m_availableConfigurations;
    std::map<std::string, std::string> m_controls;
    std::vector<aditof::DepthSensorFrameType> m_tableInUse;
};