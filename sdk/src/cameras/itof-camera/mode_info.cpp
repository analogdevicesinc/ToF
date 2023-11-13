/********************************************************************************/
/*                                                                              */
/* Copyright (c) 2020 Analog Devices, Inc. All Rights Reserved. */
/* This software is proprietary to Analog Devices, Inc. and its licensors. */
/*                                                                              */
/********************************************************************************/
#include "mode_info.h"
#include <algorithm>
#ifdef USE_GLOG
#include <glog/logging.h>
#else
#include <aditof/log.h>
#endif

//TO DO: update table with new values
ModeInfo::modeInfo ModeInfo::g_newModesAdsd3500Adsd3100[] = {
    {0, 1024, 1024, 2, 49156, 96, 0, "sr-native"},
    {1, 1024, 1024, 3, 49156, 144, 0, "lr-native"},
    {2, 512, 512, 1, 12292, 96, 0, "sr-qnative"},
    {3, 512, 512, 1, 18438, 96, 0, "lr-qnative"},
    {4, 1024, 1024, 1, 18438, 96, 1, "pcm-native"}};

ModeInfo::modeInfo ModeInfo::g_newModesAdsd3500Adsd3030[] = {
    {0, 512, 640, 1, 1670, 1472, 1, "sr-native"},
    {1, 512, 640, 1, 1670, 1472, 1, "lr-native"},
    {2, 256, 320, 1, 1670, 1472, 1, "sr-qnative"},
    {3, 256, 320, 1, 1670, 1472, 1, "lr-qnative"},
    {4, 512, 640, 1, 18438, 96, 1, "pcm-native"}};

ModeInfo::modeInfo ModeInfo::g_newMixedModesAdsd3500Adsd3100[] = {
    {0, 1024, 1024, 2, 49156, 96, 0, "sr-native"},
    {1, 1024, 1024, 3, 49156, 144, 0, "lr-native"},
    {2, 512, 512, 1, 12292, 96, 0, "sr-qnative"},
    {3, 512, 512, 1, 18438, 96, 0, "lr-qnative"},
    {4, 1024, 1024, 1, 18438, 96, 1, "pcm-native"},
    {6, 512, 512, 1, 12292, 96, 0, "sr-mixed"},
    {5, 512, 512, 1, 18438, 96, 0, "lr-mixed"}};

ModeInfo::modeInfo ModeInfo::g_newMixedModesAdsd3500Adsd3030[] = {
    {0, 512, 640, 1, 1670, 1472, 1, "sr-native"},
    {1, 512, 640, 1, 1670, 1472, 1, "lr-native"},
    {2, 256, 320, 1, 1670, 1472, 1, "sr-qnative"},
    {3, 256, 320, 1, 1670, 1472, 1, "lr-qnative"},
    {4, 512, 640, 1, 18438, 96, 1, "pcm-native"},
    {6, 256, 320, 1, 1670, 1472, 0, "sr-mixed"},
    {5, 256, 320, 1, 1670, 1472, 0, "lr-mixed"}};

int ModeInfo::g_imagerType = 1;
int ModeInfo::g_modeVersion = 2;

std::vector<ModeInfo::modeInfo>
    ModeInfo::g_modeInfoData(std::begin(g_newModesAdsd3500Adsd3100),
                             std::end(g_newModesAdsd3500Adsd3100));

//Define the static Singleton pointer
ModeInfo *ModeInfo::m_instance = NULL;

ModeInfo *ModeInfo::getInstance() {
    if (m_instance == NULL) {
        m_instance = new ModeInfo();
    }
    return (m_instance);
}

ModeInfo::modeInfo ModeInfo::getModeInfo(unsigned int mode) {
    for (unsigned int m = 0; m < g_modeInfoData.size(); ++m) {
        modeInfo modeInfo = g_modeInfoData.at(m);
        if (modeInfo.mode == mode)
            return modeInfo;
    }

    return {0};
}

aditof::Status ModeInfo::convertCameraMode(const std::string &mode,
                                           uint8_t &convertedMode) {
    auto it = std::find_if(g_modeInfoData.begin(), g_modeInfoData.end(),
                           [&mode](const ModeInfo::modeInfo &modeInfo) {
                               return mode == modeInfo.mode_name;
                           });
    if (it == g_modeInfoData.end()) {
        LOG(ERROR) << "Could not find mode: " << mode;
        return aditof::Status::INVALID_ARGUMENT;
    }
    convertedMode = (*it).mode;

    return aditof::Status::OK;
}

ModeInfo::modeInfo ModeInfo::getModeInfo(const std::string &mode) {
    for (unsigned int m = 0; m < g_modeInfoData.size(); ++m) {
        modeInfo modeInfo = g_modeInfoData.at(m);
        if (modeInfo.mode_name == mode)
            return modeInfo;
    }

    return {0};
}

aditof::Status
ModeInfo::populateAvailableModes(const std::vector<ModeInfo::modeInfo> modes) {
    for (ModeInfo::modeInfo mode : modes) {
        m_availableModes.emplace_back(mode.mode_name);
    }

    return (m_availableModes.size() > 0) ? aditof::Status::OK
                                         : aditof::Status::GENERIC_ERROR;
}

aditof::Status ModeInfo::setImagerTypeAndModeVersion(int type, int version) {
    using namespace aditof;
    Status status = Status::OK;

    if (type == 1 || type == 2) {
        g_imagerType = type;
    } else {
        return aditof::Status::INVALID_ARGUMENT;
    }

    if (version >= 0 && version <= 3) {
        g_modeVersion = version;
    } else {
        return Status::INVALID_ARGUMENT;
    }

    switch (g_imagerType) {
    case 1: {
        if (version == 3) {
            g_modeInfoData.assign(std::begin(g_newMixedModesAdsd3500Adsd3100),
                                  std::end(g_newMixedModesAdsd3500Adsd3100));
            LOG(INFO) << "Using new mixed modes table for ADSD3500 w/ADSD3100.";
        } else if (version == 2) {
            g_modeInfoData.assign(std::begin(g_newModesAdsd3500Adsd3100),
                                  std::end(g_newModesAdsd3500Adsd3100));
            LOG(INFO) << "Using new modes table for ADSD3500 w/ADSD3100.";
        } else {
            LOG(INFO) << "Unknow modes table.";
            return Status::GENERIC_ERROR;
        }
        break;
    }
    case 2: {
        if (version == 3) {
            g_modeInfoData.assign(std::begin(g_newMixedModesAdsd3500Adsd3030),
                                  std::end(g_newMixedModesAdsd3500Adsd3030));
            LOG(INFO) << "Using new mixed modes table for ADSD3500 w/ADSD3030.";
        } else if (version == 2) {
            g_modeInfoData.assign(std::begin(g_newModesAdsd3500Adsd3030),
                                  std::end(g_newModesAdsd3500Adsd3030));
            LOG(INFO) << "Using new modes table for ADSD3500 w/ADSD3030.";
        } else {
            LOG(INFO) << "Unknow modes table.";
            return Status::GENERIC_ERROR;
        }
        break;
    }
    default:
        LOG(ERROR) << "Invalid imager type: " << g_imagerType;
        status = Status::GENERIC_ERROR;
        break;
    }

    populateAvailableModes(g_modeInfoData);

    if (m_sensorConfigBits.empty()) {
        m_sensorConfigBits.emplace("bitsInDepth", "");
        m_sensorConfigBits.emplace("bitsInAb", "");
        m_sensorConfigBits.emplace("bitsInConf", "");
        m_sensorConfigBits.emplace("pixelFormat", "");
    }

    return status;
};

aditof::Status ModeInfo::getSensorProperties(const std::string mode,
                                             uint16_t *width, uint16_t *height,
                                             uint8_t *pixelFormatIndex) {
    aditof::Status status;
    uint8_t m_mode;
    status = convertCameraMode(mode, m_mode);
    if (status != aditof::Status::OK) {
        LOG(ERROR) << "Invalid mode!";
        return status;
    }

    float depthBits, abBits, confBits;
    std::string pixelFormat;

    int frameWidth = ModeInfo::getInstance()->getModeInfo(m_mode).width;
    int frameHeight = ModeInfo::getInstance()->getModeInfo(m_mode).height;

    for (auto it = m_sensorConfigBits.begin(); it != m_sensorConfigBits.end();
         it++) {
        if (it->second == "") {
            LOG(ERROR) << "Please set the number of " << it->first
                       << " in ini file!";
            return aditof::Status::INVALID_ARGUMENT;
        }
    }

    //convert values from driver settings to pixel number
    depthBits = std::stoi(m_sensorConfigBits["bitsInDepth"]);
    if (depthBits) {
        depthBits = depthBits * 2 + 4;
    }

    abBits = std::stoi(m_sensorConfigBits["bitsInAb"]);
    if (abBits) {
        abBits = abBits * 2 + 4;
    }

    confBits = std::stoi(m_sensorConfigBits["bitsInConf"]);
    if (confBits == 1) {
        confBits = 4;
    } else if (confBits == 2) {
        confBits = 8;
    }

    pixelFormat = m_sensorConfigBits["pixelFormat"];

    if (mode == "pcm-native") {
        *width = frameWidth;
        *height = frameHeight;
        *pixelFormatIndex = 1;
        return aditof::Status::OK;
    }

    //ADSD3500 + ADSD3100
    if (g_imagerType == 1) {
        if (pixelFormat == "raw16") {
            if (depthBits == 16 && abBits == 16 && confBits == 0) {
                if (mode == "lr-native") {
                    *width = 1024;
                    *height = 4096;
                    *pixelFormatIndex = 1;
                } else if (mode == "sr-native") {
                    *width = 1024;
                    *height = 3072;
                    *pixelFormatIndex = 1;
                } else {
                    LOG(ERROR) << "Invalid configuration!";
                    return aditof::Status::INVALID_ARGUMENT;
                }
            } else {
                LOG(ERROR) << "Invalid configuration!";
                return aditof::Status::INVALID_ARGUMENT;
            }
        } else if (pixelFormat == "raw16_bits12_shift4") {
            if (depthBits == 12 && abBits == 12 && confBits == 0) {
                if (mode == "lr-native") {
                    *width = 1024;
                    *height = 4096;
                    *pixelFormatIndex = 1;
                } else if (mode == "sr-native") {
                    *width = 1024;
                    *height = 3072;
                    *pixelFormatIndex = 1;
                } else {
                    LOG(ERROR) << "Invalid configuration!";
                    return aditof::Status::INVALID_ARGUMENT;
                }
            } else if (depthBits == 12 && !abBits && !confBits) {
                if (mode == "lr-native" || mode == "sr-native") {
                    *width = 1024;
                    *height = 1024;
                    *pixelFormatIndex = 1;
                } else {
                    LOG(ERROR) << "Invalid configuration!";
                    return aditof::Status::INVALID_ARGUMENT;
                }
            } else {
                LOG(ERROR) << "Invalid configuration!";
                return aditof::Status::INVALID_ARGUMENT;
            }
        } else if (pixelFormat == "mipiRaw12_8") {
            if (depthBits == 12 && abBits == 16 && confBits == 0) {
                if (mode == "lr-native") {
                    *width = 2048;
                    *height = 3328;
                    *pixelFormatIndex = 0;
                    return aditof::Status::OK;
                } else if (mode == "sr-native") {
                    *width = 2048;
                    *height = 2560;
                    *pixelFormatIndex = 0;
                } else {
                    LOG(ERROR) << "Invalid configuration!";
                    return aditof::Status::INVALID_ARGUMENT;
                }
            }
        } else if (pixelFormat == "raw8") {
            if (m_mode > 1) {
                float totalBits = depthBits + abBits + confBits;
                *width = frameWidth * totalBits / 8;
                *height = frameHeight;
                *pixelFormatIndex = 0;
            }
        }

        //ADSD3500 + ADSD3030
    } else {
        //TO DO: add raw12 modes for adsd3030
        if (pixelFormat == "raw8") {
            if (m_mode < 2) {
                float totalBits = depthBits + abBits + confBits;
                *width = frameWidth * totalBits / 8;
                *height = frameHeight;
                *pixelFormatIndex = 0;
            } else {
                *width = 1280;
                *height = 320;
                *pixelFormatIndex = 0;
            }
        } else {
            LOG(ERROR) << "Invalid configuration!";
            return aditof::Status::INVALID_ARGUMENT;
        }
    }

    return aditof::Status::OK;
};

aditof::Status ModeInfo::setSensorPixelParam(const std::string &control,
                                             const std::string &value) {
    if (m_sensorConfigBits.count(control) == 0) {
        LOG(WARNING) << "Unsuported sensor configuration!";
        return aditof::Status::INVALID_ARGUMENT;
    }

    m_sensorConfigBits[control] = value;
    return aditof::Status::OK;
};
