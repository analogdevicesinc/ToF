/********************************************************************************/
/*                                                                              */
/* Copyright (c) 2020 Analog Devices, Inc. All Rights Reserved. */
/* This software is proprietary to Analog Devices, Inc. and its licensors. */
/*                                                                              */
/********************************************************************************/
#include "mode_info.h"
#include <iostream>

// TODO: load this information from camera module EEPROM
ModeInfo::modeInfo ModeInfo::g_modeInfoData[] = {
    {0, 512, 512, 6, 1317, 896},     {1, 320, 288, 9, 2162, 288},
    {2, 512, 512, 3, 1317, 448},     {3, 1024, 1024, 1, 12289, 64},
    {4, 640, 576, 10, 4321, 640},    {5, 1024, 1024, 10, 12289, 640},
    {6, 1024, 256, 10, 2240, 878},   {7, 512, 512, 10, 2195, 896},
    {8, 1024, 1024, 1, 12289, 64},   {9, 512, 512, 3, 1317, 448},
    {10, 1024, 1024, 9, 12289, 576},
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
    if (0 <= mode && mode < getNumModes()) {
        return ModeInfo::g_modeInfoData[mode];
    }

    return {0};
}

unsigned int ModeInfo::getNumModes() {
    return sizeof(ModeInfo::g_modeInfoData) /
           sizeof(ModeInfo::g_modeInfoData[0]);
}

