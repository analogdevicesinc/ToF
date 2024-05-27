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

#ifndef DRIVER_CONFIGURATION_TABLE
#define DRIVER_CONFIGURATION_TABLE

#include <aditof/depth_sensor_interface.h>
#include <aditof/sensor_definitions.h>
#include <vector>

using namespace aditof;

const std::vector<DriverConfiguration> m_adsd3500standard = {
    /* imagerType  mode depth  ab   conf  pixelF dWidth dHeight pixFIndex
   sr-native */
    {"1024", "1024", "2", "16", "16", "0", "raw16", 2048, 3072, 0},
    {"1024", "1024", "2", "12", "12", "0", "raw16_bits12_shift4", 1024, 3072,
     1},
    {"1024", "1024", "2", "12", "0", "0", "raw16_bits12_shift4", 1024, 1024, 1},
    {"1024", "1024", "2", "12", "16", "0", "mipiRaw12_8", 2048, 2560, 0},
    {"1024", "1024", "3", "16", "16", "0", "raw16", 2048, 3072, 0},
    {"1024", "1024", "3", "12", "12", "0", "raw16_bits12_shift4", 1024, 4096,
     1},
    {"1024", "1024", "3", "12", "0", "0", "raw16_bits12_shift4", 1024, 1024, 1},
    {"1024", "1024", "3", "12", "16", "0", "mipiRaw12_8", 2048, 2560, 0}};

const std::vector<DepthSensorModeDetails> adsd3100_standardModes = {
    {0,
     {"raw", "depth", "ab", "conf", "xyz", "metadata"},
     2,
     0,
     0,
     0,
     1024,
     1024,
     128,
     0,
     DriverConfiguration()},
    {1,
     {"raw", "depth", "ab", "conf", "xyz", "metadata"},
     3,
     0,
     0,
     0,
     1024,
     1024,
     128,
     0,
     DriverConfiguration()},
    {4,
     {"ab", "metadata"},
     0,
     0,
     0,
     0,
     1024,
     1024,
     128,
     1,
     DriverConfiguration()},
    {2,
     {"raw", "depth", "ab", "conf", "xyz", "metadata"},
     2,
     0,
     0,
     0,
     512,
     512,
     128,
     0,
     DriverConfiguration()},
    {3,
     {"raw", "depth", "ab", "conf", "xyz", "metadata"},
     3,
     0,
     0,
     0,
     512,
     512,
     128,
     0,
     DriverConfiguration()},
    {6,
     {"raw", "depth", "ab", "conf", "xyz", "metadata"},
     2,
     0,
     0,
     0,
     512,
     512,
     128,
     0,
     DriverConfiguration()},
    {5,
     {"raw", "depth", "ab", "conf", "xyz", "metadata"},
     3,
     0,
     0,
     0,
     512,
     512,
     128,
     0,
     DriverConfiguration()}};

const std::vector<DepthSensorModeDetails> adsd3030_standardModes = {
    {0,
     {"raw", "depth", "ab", "conf", "xyz", "metadata"},
     2,
     0,
     0,
     0,
     512,
     640,
     128,
     0,
     DriverConfiguration()},
    {1,
     {"raw", "depth", "ab", "conf", "xyz", "metadata"},
     3,
     0,
     0,
     0,
     512,
     640,
     128,
     0,
     DriverConfiguration()},
    {4,
     {"ab", "metadata"},
     0,
     0,
     0,
     0,
     512,
     640,
     128,
     1,
     DriverConfiguration()},
    {2,
     {"raw", "depth", "ab", "conf", "xyz", "metadata"},
     2,
     0,
     0,
     0,
     256,
     320,
     128,
     0,
     DriverConfiguration()},
    {3,
     {"raw", "depth", "ab", "conf", "xyz", "metadata"},
     3,
     0,
     0,
     0,
     256,
     320,
     128,
     0,
     DriverConfiguration()},
    {6,
     {"raw", "depth", "ab", "conf", "xyz", "metadata"},
     2,
     0,
     0,
     0,
     256,
     320,
     128,
     0,
     DriverConfiguration()},
    {5,
     {"raw", "depth", "ab", "conf", "xyz", "metadata"},
     3,
     0,
     0,
     0,
     256,
     320,
     128,
     0,
     DriverConfiguration()}};

#endif