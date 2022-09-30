#ifndef OPENCV_UNDISTORT_h
#define OPENCV_UNDISTORT_h

#include "tofi_config.h"

void UndistortPoints(float *_srcx, float *_srcy, float *_dstx, float *_dsty,
                     CameraIntrinsics *_cameraMatrix, int maxcount, int rows,
                     int cols, uint8_t row_bin_factor, uint8_t col_bin_factor);

#endif //OPENCV_UNDISTORT_h