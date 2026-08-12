/****************************************************************************
# Copyright (c) 2024 - Analog Devices Inc. All Rights Reserved.
# This software is proprietary & confidential to Analog Devices, Inc.
# and its licensors.
# *****************************************************************************
# *****************************************************************************/

#ifndef V4L2_COMMON_H
#define V4L2_COMMON_H

#include <string>
#include <stdint.h>

#ifdef NVIDIA
#define V4L2_CID_ADSD3500_DEV_CHIP_CONFIG (0x009819d1)
#endif

#ifdef NXP
#define V4L2_CID_ADSD3500_DEV_CHIP_CONFIG (0x009819e1)
#endif

#ifdef RPI
#define V4L2_CID_ADSD3500_DEV_CHIP_CONFIG (0x009819d1)
#endif

std::string find_media_device_with_entity(const std::string &entity_name);

std::string find_subdev_in_media(const std::string &media_dev,
                                 const std::string &entity_name);

bool findDevicePathsAtVideo(const std::string &video, std::string &subdev_path,
                            std::string &device_name);

int xioctl(int fd, int request, void *arg);

bool v4l2_ctrl_set(int fd, uint32_t id, uint8_t *val);

bool v4l2_ctrl_get(int fd, uint32_t id, uint8_t *val);

#endif /* V4L2_COMMON_H */
