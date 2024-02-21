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

#include <vector>
#include <aditof/depth_sensor_interface.h>

const std::vector<aditof::DepthSensorFrameType> adsd3100StandardRawFrameTypes = {
        {
            "sr-native",
            {{"raw", 1024, 4096},
             {"depth", 1024, 1024},
             {"ab", 1024, 1024},
             {"conf", 1024, 1024},
             {"xyz", 1024, 1024},
             {"metadata", 1, 128}},
            1024,
            4096,
        },
        {
            "lr-native",
            {{"raw", 1024, 4096},
             {"depth", 1024, 1024},
             {"ab", 1024, 1024},
             {"conf", 1024, 1024},
             {"xyz", 1024, 1024},
             {"metadata", 1, 128}},
            1024,
            4096,
        },
        {
            "sr-qnative",
            {{"raw", 2560, 512},
             {"depth", 512, 512},
             {"ab", 512, 512},
             {"conf", 512, 512},
             {"xyz", 512, 512},
             {"metadata", 1, 128}},
            2560,
            512,
        },
        {
            "lr-qnative",
            {{"raw", 2560, 512},
             {"depth", 512, 512},
             {"ab", 512, 512},
             {"conf", 512, 512},
             {"xyz", 512, 512},
             {"metadata", 1, 128}},
            2560,
            512,
        },
        {
            "pcm-native",
            {{"ab", 1024, 1024}},
            1024,
            1024,
        },
        {
            "sr-mixed",
            {{"raw", 2560, 512},
             {"depth", 512, 512},
             {"ab", 512, 512},
             {"conf", 512, 512},
             {"xyz", 512, 512},
             {"metadata", 1, 128}},
            2560,
            512,
        },
        {
            "lr-mixed",
            {{"raw", 2560, 512},
             {"depth", 512, 512},
             {"ab", 512, 512},
             {"conf", 512, 512},
             {"xyz", 512, 512},
             {"metadata", 1, 128}},
            2560,
            512,
        }

    };
    const std::vector<aditof::DepthSensorFrameType>
        adsd3030StandardRawFrameTypes = {{
                                           "sr-native",
                                           {{"raw", 2560, 640},
                                            {"depth", 512, 640},
                                            {"ab", 512, 640},
                                            {"conf", 512, 640},
                                            {"xyz", 512, 640},
                                            {"metadata", 1, 128}},
                                           2560,
                                           640,
                                       },
                                       {
                                           "lr-native",
                                           {{"raw", 2560, 640},
                                            {"depth", 512, 640},
                                            {"ab", 512, 640},
                                            {"conf", 512, 640},
                                            {"xyz", 512, 640},
                                            {"metadata", 1, 128}},
                                           2560,
                                           640,
                                       },
                                       {
                                           "sr-qnative",
                                           {{"raw", 1280, 320},
                                            {"depth", 256, 320},
                                            {"ab", 256, 320},
                                            {"conf", 256, 320},
                                            {"xyz", 256, 320},
                                            {"metadata", 1, 128}},
                                           1280,
                                           320,
                                       },
                                       {
                                           "lr-qnative",
                                           {{"raw", 1280, 320},
                                            {"depth", 256, 320},
                                            {"ab", 256, 320},
                                            {"conf", 256, 320},
                                            {"xyz", 256, 320},
                                            {"metadata", 1, 128}},
                                           1280,
                                           320,
                                       },
                                       {
                                           "pcm-native",
                                           {{"ab", 512, 640}},
                                           512,
                                           640,
                                       },
                                       {
                                           "sr-mixed",
                                           {{"raw", 1280, 320},
                                            {"depth", 256, 320},
                                            {"ab", 256, 320},
                                            {"conf", 256, 320},
                                            {"xyz", 256, 320},
                                            {"metadata", 1, 128}},
                                           1280,
                                           320,
                                       },
                                       {
                                           "lr-mixed",
                                           {{"raw", 1280, 320},
                                            {"depth", 256, 320},
                                            {"ab", 256, 320},
                                            {"conf", 256, 320},
                                            {"xyz", 256, 320},
                                            {"metadata", 1, 128}},
                                           1280,
                                           320,
                                       }};
};
