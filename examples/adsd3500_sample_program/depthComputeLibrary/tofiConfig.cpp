// Copyright (c) 2022 Analog Devices, Inc. All Rights Reserved.
// This software is proprietary to Analog Devices, Inc. and its licensors.

#include "tofi_config.h"
#include "tofi_error.h"

#include <sstream>

TofiConfig *InitTofiConfig(ConfigFileData *p_cal_file_data,
                           ConfigFileData *p_config_file_data,
                           ConfigFileData *p_ini_file_data, uint16_t mode,
                           uint32_t *p_status) {
    XYZTable xyzObj;
    xyzObj.p_x_table = 0;
    xyzObj.p_y_table = 0;
    xyzObj.p_z_table = 0;

    TofiConfig *Obj = new TofiConfig;
    Obj->n_cols = 0;
    Obj->n_rows = 0;
    Obj->p_cal_gain_block = 0;
    Obj->p_cal_reg_block = 0;
    Obj->p_camera_intrinsics = 0;
    Obj->p_lsdac_block = 0;
    Obj->p_tofi_cal_config = 0;
    Obj->p_tofi_config_str = 0;
    Obj->xyz_table = xyzObj;
    return Obj;
};

std::string iniFileContentFindKeyAndGetValue(std::istream &iniContent,
                                             const std::string &key) {
    iniContent.clear();
    iniContent.seekg(0, std::ios::beg);

    std::string line;
    while (getline(iniContent, line)) {
        if (line.compare(0, key.length(), key) == 0) {
            size_t equalPos = line.find('=');
            if (equalPos != std::string::npos) {
                return line.substr(equalPos + 1);
            }
        }
    }

    return "";
}

TofiConfig *InitTofiConfig_isp(ConfigFileData *p_ini_file_data, uint16_t mode,
                               uint32_t *p_status,
                               TofiXYZDealiasData *p_xyz_dealias_data) {
    XYZTable xyzObj;
    xyzObj.p_x_table = 0;
    xyzObj.p_y_table = 0;
    xyzObj.p_z_table = 0;

    ConfigFileData *configFileObj = new ConfigFileData;
    *configFileObj = *p_ini_file_data;

    // We create a stream object with the content of the .ini file
    // Then we extract the number of bits for: Depth, AB, Confidence
    std::string s((char *)configFileObj->p_data, configFileObj->size);
    std::istringstream is(s);
    uint16_t nb_depth =
        std::stoi(iniFileContentFindKeyAndGetValue(is, "bitsInPhaseOrDepth"));
    uint16_t nb_ab =
        std::stoi(iniFileContentFindKeyAndGetValue(is, "bitsInAB"));
    uint16_t nb_conf =
        std::stoi(iniFileContentFindKeyAndGetValue(is, "bitsInConf"));

    TofiXYZDealiasData *dealiasDataObj = new TofiXYZDealiasData;
    *dealiasDataObj = p_xyz_dealias_data[mode];

    // We sneak the number of bits for Depth, AB, Conf into this variable that is not going to be used anywhere in tofiCompute.cpp (hopefully)
    dealiasDataObj->Freq[0] = (nb_depth << 0) | (nb_ab << 5) | (nb_conf << 10);

    TofiConfig *Obj = new TofiConfig;
    Obj->n_cols = 0;
    Obj->n_rows = 0;
    Obj->p_cal_gain_block = 0;
    Obj->p_cal_reg_block = 0;
    Obj->p_camera_intrinsics = 0;
    Obj->p_lsdac_block = 0;
    Obj->p_tofi_cal_config = reinterpret_cast<const void *>(
        dealiasDataObj); // Not nice but couldn't find a way to store all content of p_xyz_dealias_data without changing API
    Obj->p_tofi_config_str = reinterpret_cast<const char *>(configFileObj);
    Obj->xyz_table = xyzObj;
    return Obj;
};

uint32_t GetXYZ_DealiasData(ConfigFileData *ccb_data,
                            TofiXYZDealiasData *p_xyz_data) {
    return 0;
};

void FreeTofiConfig(TofiConfig *p_tofi_cal_config) {
    ConfigFileData *configFileObj =
        (ConfigFileData *)p_tofi_cal_config->p_tofi_config_str;
    TofiXYZDealiasData *dealiasDataObj =
        (TofiXYZDealiasData *)p_tofi_cal_config->p_tofi_cal_config;

    delete configFileObj;
    delete dealiasDataObj;
    delete p_tofi_cal_config;
};
