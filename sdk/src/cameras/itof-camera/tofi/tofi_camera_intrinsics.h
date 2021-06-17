/// Copyright (c) 2021 Analog Devices, Inc. All Rights Reserved.
/// This software is proprietary to Analog Devices, Inc. and its licensors.

#ifndef TOFI_CAMERA_INTRINSICS_H
#define TOFI_CAMERA_INTRINSICS_H

#ifdef __cplusplus
extern "C" {  // only need to export C interface if
              // used by C++ source code
#endif

// Structure for the camera intrinsic data
typedef struct {
  float fx;
  float fy;
  float cx;
  float cy;
  float codx;
  float cody;
  float k1;
  float k2;
  float k3;
  float k4;
  float k5;
  float k6;
  float p2;
  float p1;
} CameraIntrinsics;

#ifdef __cplusplus
}
#endif

#endif  // TOFI_CAMERA_INTRINSICS_H
