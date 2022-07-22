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
#ifndef FRAME_DEFINITIONS_H
#define FRAME_DEFINITIONS_H

#include <string>
#include <vector>

/**
 * @brief Namespace aditof
 */
namespace aditof {

/**
 * @struct FrameDataDetails
 * @brief Describes the properties of a data that embedded within the frame
 */
struct FrameDataDetails {
    /**
     * @brief The type of data that can be found in a frame. For example it
     * could be depth data or IR data, etc.
     */
    std::string type;

    /**
     * @brief The width of the frame data
     */
    unsigned int width;

    /**
     * @brief The height of the frame data
     */
    unsigned int height;

    /**
     * @brief The size in bytes of a sub-element.
     * A sub-element is a sub-component of an element. All sub-elements make up
     * an element. For instance a 3D point (which is an element) has X, Y and Z
     * coordinates which are sub-elements of the 3D point.
     */
    unsigned int subelementSize;

    /**
     * @brief The number of sub-elements that an element has.
     * An element is the smallest part of the image (a.k.a. captured scene) that
     * together with other elements make up the entire image.
     */
    unsigned int subelementsPerElement;
};

/**
 * @struct FrameDetails
 * @brief Describes the properties of a frame.
 */
struct FrameDetails {
    /**
     * @brief The type of the frame. Can be one of the types provided by the
     * camera.
     */
    std::string type;

    /**
     * @brief A frame can have multiple types of data. For example it could
     * hold data about depth and/or data about IR.
     */
    std::vector<FrameDataDetails> dataDetails;

    /**
     * @brief The mode the camera was set when the frame was captured.
     */
    std::string cameraMode;

    /**
     * @brief The width of the frame.
     */
    unsigned int width;

    /**
     * @brief The height of the frame.
     */
    unsigned int height;

    /**
     * @brief totalCaptures or subframes in a frame
     */
    uint8_t totalCaptures;

    /**
    * @brief is a passive IR frame appended
    */
    bool passiveIRCaptured;
};

/**
 * @struct Point3I
 * @brief Holds the xyz values of a frame
 */
struct Point3I {
    int16_t a; //!< X Information
    int16_t b; //!< Y Information
    int16_t c; //!< Z Information
};

} // namespace aditof

#endif // FRAME_DEFINITIONS_H
