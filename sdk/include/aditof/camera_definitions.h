/*
 * BSD 3-Clause License
 *
 * Copyright (c) 2019, Analog Devices, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#ifndef CAMERA_DEFINITIONS_H
#define CAMERA_DEFINITIONS_H

#include "connections.h"
#include "frame_definitions.h"
#include "status_definitions.h"

#include "../../sdk/src/cameras/itof-camera/tofi/tofi_camera_intrinsics.h"

#include <functional>
#include <memory>
#include <string>
#include <vector>

/**
 * @brief Namespace aditof
 */
namespace aditof {

class Frame;

/**
 * @brief Callback for frame updates
 */
typedef std::function<void(Status, Frame *)> FrameUpdateCallback;

/**
 * @struct CameraDetails
 * @brief Describes the properties of a camera.
 */
struct CameraDetails {
    /**
     * @brief Camera identification
     */
    std::string cameraId;

    /**
     * @brief The mode in which the camera operates
     */
    std::string mode;

    /**
     * @brief Details about the frames that camera is capturing
     */
    FrameDetails frameType;

    /**
     * @brief The type of connection with the camera
     */
    ConnectionType connection;

    /**
     * @brief Structure for the camera intrinsic data.
     */
    CameraIntrinsics cameraIntrinsics;

    /**
     * @brief The maximum distance (in millimeters) the camera can measure in
     * the current operating mode.
     */
    int maxDepth;

    /**
     * @brief The minimum distance (in millimeters) the camera can measure in
     * the current operating mode.
     */
    int minDepth;

    /**
     * @brief The number of bits used for representing one pixel data.
     */
    int bitCount;

    /**
     * @brief The U-Boot version that is installed on the embedded system that the camera is attached to.
     */
    std::string uBootVersion;

    /**
     * @brief The kernel version that is installed on the embedded system that the camera is attached to.
     */
    std::string kernelVersion;

    /**
     * @brief The SD card image version on the embedded system that the camera is attached to.
     */
    std::string sdCardImageVersion;
};

} // namespace aditof

#endif // CAMERA_DEFINITIONS_H
