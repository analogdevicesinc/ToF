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
#include "adsd3500_mode_selector.h"

Adsd3500ModeSelector::Adsd3500ModeSelector() : m_configuration("standard") {

    m_availableConfigurations.emplace_back("standard");
    m_availableConfigurations.emplace_back("standardRaw");
    m_availableConfigurations.emplace_back("custom");
    m_availableConfigurations.emplace_back("customRaw");

    m_controls.emplace("imagerType", "");
    m_controls.emplace("mode", "");

    m_controls.emplace("phaseDepthBits", "");
    m_controls.emplace("abBits", "");
    m_controls.emplace("confidenceBits", "");

    m_controls.emplace("inputFormat", "");
};

//functions to set which table configuration to use
aditof::Status
Adsd3500ModeSelector::setConfiguration(const std::string &configuration) {
    if (std::find(m_availableConfigurations.begin(),
                  m_availableConfigurations.end(),
                  configuration) != m_availableConfigurations.end()) {
        m_configuration = configuration;
        return aditof::Status::OK;
    } else {
        return aditof::Status::INVALID_ARGUMENT;
    }
}

aditof::Status Adsd3500ModeSelector::getConfigurationTable(
    DepthSensorFrameTypeUpdated &configurationTable) {

    if (m_configuration == "standard") {
        if (m_controls["imagerType"] == "adsd3100") {
            for (auto modes : adsd3100_standardModes) {
                if (m_controls["mode"] == modes.mode)
                    configurationTable = modes;
                return aditof::Status::OK;
            }
        } else if (m_controls["imagerType"] == "adsd3030") {
            for (auto modes : adsd3030_standardModes) {
                if (m_controls["mode"] == modes.mode) {
                    configurationTable = modes;
                    return aditof::Status::OK;
                }
            }
        }
    }

    return aditof::Status::INVALID_ARGUMENT;
};

aditof::Status Adsd3500ModeSelector::updateConfigurationTable(
    DepthSensorFrameTypeUpdated &configurationTable) {

    for (auto driverConf : configurationTable.driverConfiguration) {
        if (driverConf.depthBits == m_controls["depthBits"] &&
            driverConf.abBits == m_controls["abBits"] &&
            driverConf.confBits == m_controls["confBits"] &&
            driverConf.pixelFormat == m_controls["inputFormat"]) {
            configurationTable.baseResolutionWidth = driverConf.driverWidth;
            configurationTable.baseResolutionHeight = driverConf.driverHeigth;
            configurationTable.pixelFormatIndex = driverConf.pixelFormatIndex;

            return aditof::Status::OK;
        }
    }

    return aditof::Status::INVALID_ARGUMENT;
}

//Functions used to set mode, number of bits, pixel format, etc
aditof::Status Adsd3500ModeSelector::setControl(const std::string &control,
                                                const std::string &value) {
    using namespace aditof;
    Status status = Status::OK;

    if (m_controls.count(control) == 0) {
        return Status::INVALID_ARGUMENT;
    }

    m_controls[control] = value;

    return status;
};

aditof::Status Adsd3500ModeSelector::getControl(const std::string &control,
                                                std::string &value) {
    using namespace aditof;
    Status status = Status::OK;

    if (m_controls.count(control) == 0) {
        return Status::INVALID_ARGUMENT;
    }

    value = m_controls[control];

    return status;
};
