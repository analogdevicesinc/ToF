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
ModeInfo::modeInfo *ModeInfo::g_modeInfoData = ModeInfo::g_newModes;

ModeInfo::modeInfo ModeInfo::g_oldModes[] = {
    {1, 320, 288, 9, 2162, 288, 0},
    {3, 1024, 1024, 1, 12289, 64, 1},
    {7, 512, 512, 10, 2195, 896, 1},
    {10, 1024, 1024, 9, 12289, 576, 0},
};

//TO DO: update table with new values
ModeInfo::modeInfo ModeInfo::g_newModes[] = {
    {1, 320, 288, 9, 2162, 288, 0},
    {3, 1024, 1024, 1, 12289, 64, 1},
    {7, 512, 512, 10, 2195, 896, 1},
    {10, 1024, 1024, 9, 12289, 576, 0},
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
    return sizeof(ModeInfo::g_modeInfoData) /
           sizeof(ModeInfo::g_modeInfoData[0]);
}

aditof::Status convertCameraMode(const std::string &mode,
                                 uint8_t &convertedMode) {

#ifndef ADSD3030
    int modeVersion = ModeInfo::getInstance()->getModeVersion();
    if (modeVersion) {
        //complete names for each mode
        return aditof::Status::UNAVAILABLE;
    } else {
        if (mode == "lt_bin") {
            convertedMode = 1;
        } else if (mode == "pcm") {
            convertedMode = 3;
        } else if (mode == "qmp") {
            convertedMode = 7;
        } else if (mode == "mp") {
            convertedMode = 10;
        } else {
            return aditof::Status::INVALID_ARGUMENT;
        }
    }
#else
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
    if (version) {
        g_modeInfoData = g_newModes;
    } else {
        g_modeInfoData = g_oldModes;
    }
#endif

    return status;
};

int ModeInfo::getModeVersion() {
#ifndef ADSD3030
    if (g_modeInfoData == g_newModes) {
        return 1;
    }
#endif
    return 0;
};
