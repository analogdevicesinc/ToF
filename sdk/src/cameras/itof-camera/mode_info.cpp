/********************************************************************************/
/*                                                                              */
/* Copyright (c) 2020 Analog Devices, Inc. All Rights Reserved. */
/* This software is proprietary to Analog Devices, Inc. and its licensors. */
/*                                                                              */
/********************************************************************************/
#include "mode_info.h"
#include <algorithm>
#include <iostream>

// TODO: load this information from camera module EEPROM
#ifndef ADSD3030
//use new modes by default
ModeInfo::modeInfo *ModeInfo::g_modeInfoData = ModeInfo::g_newModesAdsd3500;

ModeInfo::modeInfo ModeInfo::g_oldModes[] = {
    {1, 320, 288, 9, 2162, 288, 0},
    {3, 1024, 1024, 1, 12289, 64, 1},
    {7, 512, 512, 10, 2195, 896, 1},
    {10, 1024, 1024, 9, 12289, 576, 0},
};

//TO DO: update table with new values
ModeInfo::modeInfo ModeInfo::g_newModesAdsd3100[] = {
    {0, 1024, 1024, 6, 49156, 96, 0},  // SRMP
    {1, 1024, 1024, 9, 49156, 144, 0}, // LRMP
    {2, 512, 512, 6, 12292, 96, 0},    // SRQMP
    {3, 512, 512, 9, 18438, 96, 0},    // SRQMP
    {4, 1024, 1024, 1, 49156, 16, 1},  // PCMMP
};

ModeInfo::modeInfo ModeInfo::g_newModesAdsd3500[] = {
    {0, 1024, 1024, 2, 49156, 96, 0},  // SRMP
    {1, 1024, 1024, 3, 49156, 144, 0}, // LRMP
    {2, 512, 512, 1, 12292, 96, 0},    // SRQMP
    {3, 512, 512, 1, 18438, 96, 0},    // SRQMP
};
#else
ModeInfo::modeInfo *ModeInfo::g_modeInfoData = ModeInfo::g_adsd3030Modes;

ModeInfo::modeInfo ModeInfo::g_adsd3030Modes[] = {
    {5, 512, 640, 10, 1670, 1472, 1},
};
#endif

//Define the static Singleton pointer
ModeInfo *ModeInfo::m_instance = NULL;

ModeInfo *ModeInfo::getInstance() {
    if (m_instance == NULL) {
        m_instance = new ModeInfo();
    }
    return (m_instance);
}

ModeInfo::modeInfo ModeInfo::getModeInfo(unsigned int mode) {
    for (unsigned int m = 0; m < getNumModes(); ++m) {
        modeInfo modeInfo = g_modeInfoData[m];
        if (modeInfo.mode == mode)
            return modeInfo;
    }

    return {0};
}

unsigned int ModeInfo::getNumModes() {
#ifndef ADSD3030
    int modeVersion = ModeInfo::getInstance()->getModeVersion();
    if (modeVersion == 0) {
        return sizeof(ModeInfo::g_oldModes) / sizeof(ModeInfo::g_oldModes[0]);
    } else if (modeVersion == 1) {
        return sizeof(ModeInfo::g_newModesAdsd3100) /
               sizeof(ModeInfo::g_newModesAdsd3100[0]);
    }
    return sizeof(ModeInfo::g_newModesAdsd3500) /
           sizeof(ModeInfo::g_newModesAdsd3500[0]);
#else
    return sizeof(ModeInfo::g_adsd3030Modes) /
           sizeof(ModeInfo::g_adsd3030Modes[0]);
#endif
}

aditof::Status convertCameraMode(const std::string &mode,
                                 uint8_t &convertedMode) {

#ifndef ADSD3030
    int modeVersion = ModeInfo::getInstance()->getModeVersion();
    if (modeVersion == 1 || modeVersion == 2) {
        if (mode == "srmp") {
            convertedMode = 0;
        } else if (mode == "lrmp") {
            convertedMode = 1;
        } else if (mode == "srqmp") {
            convertedMode = 2;
        } else if (mode == "lrqmp") {
            convertedMode = 3;
        } else if (mode == "pcmmp") {
            convertedMode = 4;
        }
    } else {
        if (mode == "lt_bin") {
            convertedMode = 1;
        } else if (mode == "pcmmp") {
            convertedMode = 3;
        } else if (mode == "lrqmp") {
            convertedMode = 7;
        } else if (mode == "lrmp") {
            convertedMode = 10;
        } else {
            return aditof::Status::INVALID_ARGUMENT;
        }
    }
#else
#ifdef ADSD3030_BOXDIM
    convertedMode = 5;
    
    return aditof::Status::OK;
#endif
    if (mode == "vga") {
        convertedMode = 5;
    } else {
        return aditof::Status::INVALID_ARGUMENT;
    }
#endif

    return aditof::Status::OK;
}

ModeInfo::modeInfo ModeInfo::getModeInfo(const std::string &mode) {
    unsigned char modeIndex;

    convertCameraMode(mode, modeIndex);

    return getModeInfo(modeIndex);
}

aditof::Status ModeInfo::setModeVersion(int version) {
    using namespace aditof;
    Status status = Status::OK;

#ifndef ADSD3030
    if (version == 1) {
        g_modeInfoData = g_newModesAdsd3100;
        LOG(INFO) << "Using new modes table for adsd3100.";
    } else if (version == 2) {
        g_modeInfoData = g_newModesAdsd3500;
        LOG(INFO) << "Using new modes table for adsd3500.";
    } else {
        g_modeInfoData = g_oldModes;
        LOG(INFO) << "Using old modes table.";
    }
#endif

    return status;
};

int ModeInfo::getModeVersion() {
#ifndef ADSD3030
    if (g_modeInfoData == g_newModesAdsd3100) {
        return 1;
    } else if (g_modeInfoData == g_newModesAdsd3500) {
        return 2;
    }
#endif
    return 0;
};
