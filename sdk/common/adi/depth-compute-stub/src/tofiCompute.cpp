// Copyright (c) 2020 Analog Devices, Inc-> All Rights Reserved.
// This software is proprietary to Analog Devices, Inc. and its licensors.

#include "tofi_compute.h"
#include "tofi_error.h"

TofiComputeContext *InitTofiCompute(
    const void *p_tofi_cal_config, uint32_t *p_status) {
        TofiComputeContext *Obj;
        Obj->n_cols = 0;
        Obj->n_rows = 0;
        Obj->p_ab_frame = 0;
        Obj->p_cal_config = 0;
        Obj->p_conf_frame = 0;
        Obj->p_depth16_frame = 0;
        Obj->p_depth_frame = 0;
        Obj->p_tofi_processor_config = 0;
        Obj->p_xyz_frame = 0;
        return Obj;
    };

 int TofiCompute(
    const uint16_t *const input_frame,
    TofiComputeContext *const p_tofi_compute_context,
    TemperatureInfo *p_temperature) {
        return 0;
    };

void FreeTofiCompute(TofiComputeContext *p_tofi_compute_context) {};
