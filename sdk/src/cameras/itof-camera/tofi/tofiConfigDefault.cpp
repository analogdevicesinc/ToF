// Copyright (c) 2020 Analog Devices, Inc. All Rights Reserved.
// This software is proprietary to Analog Devices, Inc. and its licensors.

#include "tofi_config.h"
#include "tofi_error.h"

TofiConfig *InitTofiConfig(ConfigFileData *p_cal_file_data,
                           ConfigFileData *p_config_file_data,
                           ConfigFileData *p_ini_file_data, uint16_t mode,
                           uint32_t *p_status) {
    XYZTable *xyzObj;
    xyzObj->p_x_table = 0;
    xyzObj->p_y_table = 0;
    xyzObj->p_z_table = 0;
    
    TofiConfig *Obj;
    Obj->n_cols=0;
    Obj->n_rows=0;
    Obj->p_cal_gain_block=0;
    Obj->p_cal_reg_block=0;
    Obj->p_camera_intrinsics=0;
    Obj->p_lsdac_block=0;
    Obj->p_tofi_cal_config=0;
    Obj->p_tofi_config_str=0;
    Obj->xyz_table = *xyzObj;
    return Obj;
                           };

void FreeTofiConfig(TofiConfig *p_tofi_cal_config) {};
