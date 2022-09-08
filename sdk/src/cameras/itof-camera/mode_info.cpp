/********************************************************************************/
/*                                                                              */
/* Copyright (c) 2020 Analog Devices, Inc. All Rights Reserved. */
/* This software is proprietary to Analog Devices, Inc. and its licensors. */
/*                                                                              */
/********************************************************************************/
#include "mode_info.h"
#include <iostream>
#include <algorithm>

// TODO: load this information from camera module EEPROM
ModeInfo::modeInfo ModeInfo::g_modeInfoData[] = {
    {1, 320, 288, 9, 2162, 288, 0},
    {2, 512, 512, 3, 2195, 896, 1},
    {3, 1024, 1024, 1, 12289, 64, 1},
    {5, 1024, 1024, 10, 12289, 640, 1},
    {7, 512, 512, 10, 2195, 896, 1},
    {10, 1024, 1024, 9, 12289, 576, 0},
};

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


aditof::Status convertCameraMode(const std::string &mode, uint8_t& convertedMode) {
    if (mode == "lt_bin") {
        convertedMode = 1;
    } else if (mode == "qmp_onephase") {
        convertedMode = 2;
    } else if (mode == "pcm") {
        convertedMode = 3;
    } else if (mode == "mp_pcm") {
        convertedMode = 5;
    } else if (mode == "qmp") {
        convertedMode = 7;
    } else if (mode == "mp") {
        convertedMode = 10;
    } else {
        return aditof::Status::INVALID_ARGUMENT;
    }

    return aditof::Status::OK;
}

ModeInfo::modeInfo ModeInfo::getModeInfo(const std::string& mode){
    unsigned char modeIndex;

    convertCameraMode(mode, modeIndex);
    
    return getModeInfo(modeIndex);
}