/*****************************************************************************
* Copyright (c) 2024 - Analog Devices Inc. All Rights Reserved.
* This software is proprietary & confidential to Analog Devices, Inc.
* and its licensors.
******************************************************************************
******************************************************************************
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
* http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.*/

class ModeInfo {
    public:
    typedef struct {
        uint8_t mode;
        uint16_t width;
        uint16_t height;
        uint8_t subframes;
        uint16_t embed_width;
        uint16_t embed_height;
        uint8_t passive_ir;
        std::string mode_name;
    } modeInfo;
}

ModeInfo::modeInfo ModeInfo::g_newModesAdsd3500Adsd3100[] = {
    {0, 1024, 1024, 2, 49156, 96, 0, "sr-native"},
    {1, 1024, 1024, 3, 49156, 144, 0, "lr-native"},
    {2, 512, 512, 1, 12292, 96, 0, "sr-qnative"},
    {3, 512, 512, 1, 18438, 96, 0, "lr-qnative"},
    {4, 1024, 1024, 1, 18438, 96, 1, "pcm-native"}};

ModeInfo::modeInfo ModeInfo::g_newModesAdsd3500Adsd3030[] = {
    {0, 512, 640, 1, 1670, 1472, 1, "sr-native"},
    {1, 512, 640, 1, 1670, 1472, 1, "lr-native"},
    {2, 256, 320, 1, 1670, 1472, 1, "sr-qnative"},
    {3, 256, 320, 1, 1670, 1472, 1, "lr-qnative"},
    {4, 512, 640, 1, 18438, 96, 1, "pcm-native"}};

ModeInfo::modeInfo ModeInfo::g_newMixedModesAdsd3500Adsd3100[] = {
    {0, 1024, 1024, 2, 49156, 96, 0, "sr-native"},
    {1, 1024, 1024, 3, 49156, 144, 0, "lr-native"},
    {2, 512, 512, 1, 12292, 96, 0, "sr-qnative"},
    {3, 512, 512, 1, 18438, 96, 0, "lr-qnative"},
    {4, 1024, 1024, 1, 18438, 96, 1, "pcm-native"},
    {6, 512, 512, 1, 12292, 96, 0, "sr-mixed"},
    {5, 512, 512, 1, 18438, 96, 0, "lr-mixed"}};

ModeInfo::modeInfo ModeInfo::g_newMixedModesAdsd3500Adsd3030[] = {
    {0, 512, 640, 1, 1670, 1472, 1, "sr-native"},
    {1, 512, 640, 1, 1670, 1472, 1, "lr-native"},
    {2, 256, 320, 1, 1670, 1472, 1, "sr-qnative"},
    {3, 256, 320, 1, 1670, 1472, 1, "lr-qnative"},
    {4, 512, 640, 1, 18438, 96, 1, "pcm-native"},
    {6, 256, 320, 1, 1670, 1472, 0, "sr-mixed"},
    {5, 256, 320, 1, 1670, 1472, 0, "lr-mixed"}};